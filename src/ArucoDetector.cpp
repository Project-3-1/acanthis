#include "ros/ros.h"

#include <opencv2/opencv.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/highgui.hpp>
#include <string>
#include <chrono>

#include "cv_bridge/cv_bridge.h"

#include "acanthis/ArucoPose.h"
#include "sensor_msgs/Image.h"
#include "geometry_msgs/PoseStamped.h"

using namespace std;
using namespace cv;

static ros::Subscriber pose_sub;
static geometry_msgs::PoseStamped::_pose_type pose;

static int circle_buffer_index = 0;
static int circle_buffer_length = 15;
static std::vector<cv::Vec3f> marker_positions(circle_buffer_length);
static cv::Vec3f std_marker;
static cv::Vec3f mean_marker;

static std::vector<cv::Vec3f> drone_positions(circle_buffer_length);
static std::vector<double> dt(circle_buffer_length);
static cv::Vec3f marker_velocity;
static cv::Vec3f drone_velocity;


static bool read_calibration(const std::string& filename, Mat& camMatrix, Mat& distCoeffs)
{
    FileStorage fs(filename, FileStorage::READ);
    if (!fs.isOpened())
        return false;
    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    cout << "camera_matrix = " << endl << " "  << camMatrix << endl << endl;
    cout << "distortion_coefficients = " << endl << " "  << distCoeffs << endl << endl;
    return true;
}

static int get_dictionary(int value) {
    if(value == 4) {
        return cv::aruco::DICT_4X4_50;
    } else if(value == 5) {
        return cv::aruco::DICT_5X5_50;
    } else if(value == 6) {
        return cv::aruco::DICT_6X6_50;
    } else if(value == 7) {
        return cv::aruco::DICT_7X7_50;
    } else {
        ROS_ERROR("INVALID DICTIONARY SIZE (%d) pick: 4, 5, 6, or 7.", value);
        return -1;
    }
}

static void calculate_avg_std(std::vector<cv::Vec3f>& data, cv::Vec3f& mean, cv::Vec3f& std) {
    cv::Vec3f sum(0, 0, 0);
    for(auto & i : data) {
        sum[0] += i[0];
        sum[1] += i[1];
        sum[2] += i[2];
    }

    mean[0] = sum[0] / data.size();
    mean[1] = sum[1] / data.size();
    mean[2] = sum[2] / data.size();

    for(auto &i : data) {
        std[0] += pow(i[0] - mean[0], 2);
        std[1] += pow(i[1] - mean[1], 2);
        std[2] += pow(i[2] - mean[2], 2);
    }

    std[0] = sqrt(std[0] / data.size());
    std[1] = sqrt(std[1] / data.size());
    std[2] = sqrt(std[2] / data.size());

}

static void _update_pos(const geometry_msgs::PoseStamped &p) {
    pose = p.pose;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "aruco_detector");
    ros::NodeHandle node("~");
    ros::Rate rate(60);

    pose_sub = pose_sub = node.subscribe("/crazyflie/pose", 1, _update_pos); // actually important to keep this reference around...


    // --- config
    string filename_calibration;
    if(!node.getParam("calibration", filename_calibration)) {
        ROS_ERROR("Path to calibration file missing.");
        return 1;
    }

    string filename_calibration_fisheye;
    if(!node.getParam("calibration_fisheye", filename_calibration_fisheye)) {
        ROS_ERROR("Path to fisheye calibration file missing.");
        return 1;
    }

    float marker_size;
    if(!node.getParam("marker_size", marker_size)) {
        ROS_ERROR("Marker size not specified in [m].");
        return 1;
    }

    string camera_name;
    if(!node.getParam("camera", camera_name)) {
        ROS_ERROR("Camera path missing.");
        return 1;
    }

    int dictionary_size;
    if(!node.getParam("dictionary", dictionary_size)) {
        ROS_ERROR("Dictionary id is missing (4, 5, 6, 7).");
        return 1;
    }
    const int DICTIONARY = get_dictionary(dictionary_size);
    if(DICTIONARY == -1) {
        return 1;
    }

    bool publish_debug_image = false;
    node.getParam("publish_debug_image", publish_debug_image);

    // --- publisher
    ros::Publisher pose_pub = node.advertise<acanthis::ArucoPose>("pose", 1);

    ros::Publisher debug_image_pub;
    sensor_msgs::ImagePtr debug_image_msg;
    if(publish_debug_image) {
        debug_image_pub = node.advertise<sensor_msgs::Image>("image/debug", 1);
    }

    // --- message
    acanthis::ArucoPose marker_pose_msg;
    marker_pose_msg.header.seq = 0;
    marker_pose_msg.header.stamp = ros::Time::now();


    // --- TODO
    VideoCapture inputVideo(camera_name, CAP_V4L2);

    if(!inputVideo.isOpened()) {
        ROS_ERROR("Could not open '%s'.", camera_name.c_str());
        return 1;
    }

    Mat cameraMatrixAfterFisheye, distCoeffsAfterFisheye;
    Mat cameraMatrixFisheye, distCoeffsFisheye;
    bool read_calibration_ok = read_calibration(filename_calibration, cameraMatrixAfterFisheye, distCoeffsAfterFisheye);
    bool read_calibration_fisheye_ok = read_calibration(filename_calibration_fisheye, cameraMatrixFisheye, distCoeffsFisheye);
    if (!read_calibration_ok || !read_calibration_fisheye_ok) {
        ROS_ERROR("Failed to read camera calibration file");
        return 1;
    } else {

        Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(DICTIONARY);
        Ptr<aruco::DetectorParameters> params = aruco::DetectorParameters::create();

        bool first_image = true;
        cv::Mat map1, map2;

        //---
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
        clahe->setClipLimit(4);
        cv::Mat dst;

        Mat image;

        while (ros::ok() && inputVideo.grab()) {
            inputVideo.retrieve(image);
            cv::cvtColor(image, image, CV_BGR2GRAY);

            //--- remove distortion from image
            //TODO AFAIK we only need to init the two maps once...
            if(first_image) {
                cv::fisheye::initUndistortRectifyMap(cameraMatrixFisheye, distCoeffsFisheye, cv::Mat::eye(3, 3, CV_32F),
                                                     cameraMatrixFisheye, image.size(), CV_16SC2, map1, map2);
                first_image = false;
            }
            remap(image, image, map1, map2, INTER_LINEAR, BORDER_CONSTANT);

            //--- Adaptive Histogram Normalisation
            //clahe->apply(image, image);
            //---

            std::vector<int> markerIds;
            std::vector<std::vector<Point2f> > markerCorners;
            aruco::detectMarkers(image, dictionary, markerCorners, markerIds, params);

            if(publish_debug_image) {
                cv::cvtColor(image, image, CV_GRAY2BGR);
            }

            double text_x, text_y, text_z;
            bool text_accepted = false;
            if (!markerIds.empty()) {
                std::vector<Vec3d> rvecs, tvecs;
                aruco::estimatePoseSingleMarkers(markerCorners, marker_size, cameraMatrixAfterFisheye, distCoeffsAfterFisheye, rvecs, tvecs);
                if (!rvecs.empty()) {

                    if(publish_debug_image) {
                        for (int i = 0; i < markerIds.size(); i++) {
                            cv::aruco::drawAxis(image, cameraMatrixAfterFisheye, distCoeffsAfterFisheye, rvecs.at(i), tvecs.at(i), marker_size);
                        }
                    }

                    for (int i = 0; i < markerIds.size(); i++) {

                        text_accepted = false;
                        Vec<double,3> tvec = tvecs[i];

                        float x = tvec[0];
                        float y = -tvec[1];
                        float z = -tvec[2];

                        if(marker_positions.size() == circle_buffer_length) {
                            calculate_avg_std(marker_positions, mean_marker, std_marker);

                            // std check - 95% interval
                            const double mul = 2;
                            if(
                                    (x <= (mean_marker[0] + mul * std_marker[0]) && x >= (mean_marker[0] - mul * std_marker[0])) &&
                                    (y <= (mean_marker[1] + mul * std_marker[1]) && y >= (mean_marker[1] - mul * std_marker[1])) &&
                                    (z <= (mean_marker[2] + mul * std_marker[2]) && z >= (mean_marker[2] - mul * std_marker[2]))
                                )   {

                                text_accepted = true;

                                // --- position
                                marker_pose_msg.header.seq++;
                                marker_pose_msg.header.stamp = ros::Time::now();

                                marker_pose_msg.marker_id = markerIds.at(i);

                                marker_pose_msg.position.x = x;
                                marker_pose_msg.position.y = y;
                                marker_pose_msg.position.z = z;

                                // --- velocity
                                for(int j = 1; j < circle_buffer_length; j++) {
                                    marker_velocity = (marker_positions[j] - marker_positions[j - 1]) / (dt[j] - dt[j - 1]);
                                    drone_velocity = (drone_positions[j] - drone_positions[j - 1]) / (dt[j] - dt[j - 1]);
                                }
                                ROS_INFO_STREAM("drone velocity " << drone_velocity << std::endl);
                                marker_velocity /= circle_buffer_length;
                                drone_velocity /= circle_buffer_length;

                                marker_pose_msg.abs_velocity.x = marker_velocity[0];
                                marker_pose_msg.abs_velocity.y = marker_velocity[1];
                                marker_pose_msg.abs_velocity.z = marker_velocity[2];

                                marker_velocity -= drone_velocity;

                                marker_pose_msg.rel_velocity.x = marker_velocity[0];
                                marker_pose_msg.rel_velocity.y = marker_velocity[1];
                                marker_pose_msg.rel_velocity.z = marker_velocity[2];

                                // --- publish
                                pose_pub.publish(marker_pose_msg);
                            }
                        }

                        marker_positions[circle_buffer_index] = cv::Vec3f(x, y, z);
                        dt[circle_buffer_index] = ros::Time::now().toSec();
                        drone_positions.emplace_back(pose.position.x, pose.position.y, pose.position.z);
                        circle_buffer_index = (circle_buffer_index + 1) % circle_buffer_length;


                        // ---
                        text_x = marker_pose_msg.position.x;
                        text_y = marker_pose_msg.position.y;
                        text_z = marker_pose_msg.position.z;
                        // ---

                    }
                }
            }

            if(publish_debug_image) {
                putText(image, format("x: %.2f", text_x), Point(10, 50),
                        FONT_HERSHEY_COMPLEX, 1, CV_RGB(255,0, 0), 3);
                putText(image, format("y: %.2f", text_y), Point(10, 80),
                        FONT_HERSHEY_COMPLEX, 1, CV_RGB(0, 255, 0), 3);
                putText(image, format("z: %.2f", text_z), Point(10, 110),
                        FONT_HERSHEY_COMPLEX, 1, CV_RGB(0,0, 255), 3);
                putText(image, format("%s", text_accepted ? "Accepted" : "Rejected"), Point(10, 130),
                        FONT_HERSHEY_COMPLEX, 0.5, CV_RGB(255 * !text_accepted,255 * text_accepted , 0), 1);

                debug_image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
                debug_image_pub.publish(debug_image_msg);
            }

            ros::spinOnce();
            rate.sleep();
        }
    }

    return 0;
}
