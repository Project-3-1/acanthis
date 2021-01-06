//
// Created by jan on 05/01/2021.
//

#ifndef SRC_ARUCOEKF_H
#define SRC_ARUCOEKF_H
#include <opencv2/opencv.hpp>

const int stateSize = 4;
const int measSize = 2;

class ArucoEKF {

private:

    bool found = false;
    double ticks = 0;

    cv::KalmanFilter kf;

    cv::Mat state;  // [x,y,v_x,v_y]
    cv::Mat meas;    // [z_x,z_y]


public:
    ArucoEKF();

    /**
     * Updates the Kalman Filter
     * @param x The absolute (!) x position of the Aruco marker.
     * @param y The absolute (!) x position of the Aruco marker.
     */
    void update(double x, double y);

    /**
     * Resets the KF to its inital state. This could be useful if the drone lost track of the marker for whatever
     * reason.
     */
    void reset();

    cv::Vec4d get_position();
    cv::Vec4d get_velocity();
};


#endif //SRC_ARUCOEKF_H