//
// Created by jan on 05/01/2021.
//

#include "ArucoEKF.h"
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

ArucoEKF::ArucoEKF() : state(stateSize, 1, CV_32F), meas(measSize, 1, CV_32F) {
    this->kf = KalmanFilter(stateSize, measSize, 0, CV_32F);
    this->reset();
}

void ArucoEKF::update(double x, double y) {
    double precTick = ticks;
    ticks = (double) getTickCount();
    double dT = (ticks - precTick) / getTickFrequency(); //seconds

    if (found)
    {
        kf.transitionMatrix.at<float>(2) = dT;
        kf.transitionMatrix.at<float>(7) = dT;
        state = kf.predict();
    }

    meas.at<float>(0) = x;
    meas.at<float>(1) = y;

    if (!found)
    {
        kf.errorCovPre.at<float>(0) = 1;
        kf.errorCovPre.at<float>(5) = 1;
        kf.errorCovPre.at<float>(10) = 1;
        kf.errorCovPre.at<float>(15) = 1;

        state.at<float>(0) = meas.at<float>(0);
        state.at<float>(1) = meas.at<float>(1);
        state.at<float>(2) = 0;
        state.at<float>(3) = 0;

        kf.statePost = state;
        found = true;
    }
    else
    {
        kf.correct(meas); // Kalman Correction
    }
    cout << "Measure matrix:" << meas << endl;

}

void ArucoEKF::reset() {
    // Transition State Matrix A
    // Note: set dT at each processing step!
    // [ 1 0 dT 0  ]
    // [ 0 1 0  dT ]
    // [ 0 0 1  0  ]
    // [ 0 0 0  1  ]
    setIdentity(kf.transitionMatrix);

    // Measure Matrix H
    // [ 1 0 0 0 ]
    // [ 0 1 0 0 ]
    kf.measurementMatrix = Mat::zeros(measSize, stateSize, CV_32F);
    kf.measurementMatrix.at<float>(0) = 1.0f;
    kf.measurementMatrix.at<float>(5) = 1.0f;

    // Process Noise Covariance Matrix Q
    // [ Ex   0   0     0     ]
    // [ 0    Ey  0     0     ]
    // [ 0    0   Ev_x  0     ]
    // [ 0    0   0     Ev_y  ]
    setIdentity(kf.processNoiseCov, Scalar(1e-2));

    // Measures Noise Covariance Matrix R
    setIdentity(kf.measurementNoiseCov, Scalar(1e-1));

    this->found = false;
    this->ticks = 0;
}

Vec2d ArucoEKF::get_position() {
    return Vec2d(this->state.at<float>(0), this->state.at<float>(1));
}

cv::Vec2d ArucoEKF::get_velocity() {
    return Vec2d(this->state.at<float>(2), this->state.at<float>(3));
}

double ArucoEKF::get_last_seen() {
    double current_ticks = getTickCount();
    return (current_ticks - this->ticks) / getTickFrequency();
}



