#include "ros/ros.h"

#include <opencv2/opencv.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <string>
#include <chrono>

#include "geometry_msgs/PoseStamped.h"
using namespace std;
using namespace cv;
const int DICTIONARY = aruco::DICT_6X6_50;

static bool readCameraParameters(std::string filename, Mat& camMatrix, Mat& distCoeffs)
{
    FileStorage fs(filename, FileStorage::READ);
    if (!fs.isOpened())
        return false;
    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ArucoDetector", ros::init_options::AnonymousName);
    ros::NodeHandle node("~");
    ros::Rate rate(60);

    // --- publisher
    ros::Publisher pose_pub = node.advertise<geometry_msgs::PoseStamped>("marker_pose", 1);

    // --- message
    geometry_msgs::PoseStamped marker_pose;
    marker_pose.header.seq = 0;
    marker_pose.header.stamp = ros::Time::now();

    // --- TODO
    VideoCapture inputVideo("/dev/video2", CAP_V4L2);
    Mat cameraMatrix, distCoeffs;
    std::string filename;
    node.getParam("calibration", filename);

    bool readOk = readCameraParameters(filename, cameraMatrix, distCoeffs);
    if (!readOk) {
        std::cerr << "Invalid camera file" << std::endl;
        return 1;
    } else {

        Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(DICTIONARY);
        Ptr<aruco::CharucoBoard> board = aruco::CharucoBoard::create(5, 7, 0.04f, 0.02f, dictionary);
        Ptr<aruco::DetectorParameters> params = aruco::DetectorParameters::create();

        while (inputVideo.grab()) {
            Mat image;
            inputVideo.retrieve(image);

            std::vector<int> markerIds;
            std::vector<std::vector<Point2f> > markerCorners;
            aruco::detectMarkers(image, board->dictionary, markerCorners, markerIds, params);

            if (!markerIds.empty()) {
                std::vector<Vec3d> rvecs, tvecs;
                aruco::estimatePoseSingleMarkers(markerCorners, 0.05, cameraMatrix, distCoeffs, rvecs, tvecs);// if at least one charuco corner detected
                if (!rvecs.empty()) {
                    for (int i = 0; i < rvecs.size(); ++i) {
                        auto rvec = rvecs[i];
                        auto tvec = tvecs[i];

                        marker_pose.header.seq++;
                        marker_pose.header.stamp = ros::Time::now();

                        marker_pose.pose.position.x = tvec[0];
                        marker_pose.pose.position.y = tvec[1];
                        marker_pose.pose.position.z = tvec[2];

                        pose_pub.publish(marker_pose);
                    }
                }
            }

            ros::spinOnce();
            rate.sleep();
        }
    }

    return 0;
}
