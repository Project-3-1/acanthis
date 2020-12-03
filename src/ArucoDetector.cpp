#include "ros/ros.h"

#include <opencv2/opencv.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/highgui.hpp>
#include <string>
#include <chrono>

#include "cv_bridge/cv_bridge.h"

#include "acanthis/ArucoPose.h"
#include "sensor_msgs/Image.h"

using namespace std;
using namespace cv;
const int DICTIONARY = aruco::DICT_6X6_50;

static bool readCameraParameters(const std::string& filename, Mat& camMatrix, Mat& distCoeffs)
{
    FileStorage fs(filename, FileStorage::READ);
    if (!fs.isOpened())
        return false;
    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "aruco_detector");
    ros::NodeHandle node("~");
    ros::Rate rate(60);

    // --- config
    string filename;
    if(!node.getParam("calibration", filename)) {
        ROS_ERROR("Path to calibration file missing.");
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

    bool publish_debug_image = false;
    node.getParam("publish_debug_image", publish_debug_image);

    // --- publisher
    ros::Publisher pose_pub = node.advertise<acanthis::ArucoPose>("pose", 1);

    ros::Publisher debug_image_pub;
    sensor_msgs::ImagePtr debug_image_msg;
    if(publish_debug_image) {
        debug_image_pub = node.advertise<sensor_msgs::Image>("image", 1);
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

    Mat cameraMatrix, distCoeffs;
    bool readOk = readCameraParameters(filename, cameraMatrix, distCoeffs);
    if (!readOk) {
        ROS_ERROR("Failed to read camera calibration file");
        return 1;
    } else {

        Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(DICTIONARY);
        Ptr<aruco::DetectorParameters> params = aruco::DetectorParameters::create();

        while (ros::ok() && inputVideo.grab()) {
            Mat image;
            inputVideo.retrieve(image);

            std::vector<int> markerIds;
            std::vector<std::vector<Point2f> > markerCorners;
            aruco::detectMarkers(image, dictionary, markerCorners, markerIds, params);

            if (!markerIds.empty()) {
                std::vector<Vec3d> rvecs, tvecs;
                aruco::estimatePoseSingleMarkers(markerCorners, marker_size, cameraMatrix, distCoeffs, rvecs, tvecs);
                if (!rvecs.empty()) {

                    if(publish_debug_image) {
                        for (int i = 0; i < markerIds.size(); i++) {
                            cv::aruco::drawAxis(image, cameraMatrix, distCoeffs, rvecs.at(i), tvecs.at(i), marker_size);
                        }
                    }

                    for (int i = 0; i < markerIds.size(); i++) {
                        //Vec<double,3> rvec = rvecs[i];
                        Vec<double,3> tvec = tvecs[i];

                        marker_pose_msg.header.seq++;
                        marker_pose_msg.header.stamp = ros::Time::now();

                        marker_pose_msg.marker_id = markerIds.at(i);

                        marker_pose_msg.position.x = tvec[0];
                        marker_pose_msg.position.y = tvec[1];
                        marker_pose_msg.position.z = tvec[2];

                        pose_pub.publish(marker_pose_msg);
                    }
                }
            }

            if(publish_debug_image) {
                debug_image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
                debug_image_pub.publish(debug_image_msg);
            }

            ros::spinOnce();
            rate.sleep();
        }
    }

    return 0;
}
