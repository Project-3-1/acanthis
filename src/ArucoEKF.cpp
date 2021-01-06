//
// Created by jan on 05/01/2021.
//

#include "ArucoEKF.h"
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

ArucoEKF::ArucoEKF() : state(stateSize, 1, CV_32F), meas(measSize, 1, CV_32F) {
    this->kf = KalmanFilter(stateSize, measSize, 2, CV_32F);
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

        Mat control = Mat::zeros(4, 1, CV_32F);
        control.at<float>(1) = 0.5 * pow(dT, 2);
        control.at<float>(3) = 0.5 * pow(dT, 2);
        state = kf.predict();
    }

    meas.at<float>(0) = x;
    meas.at<float>(1) = y;

    if (!found)
    {
        setIdentity(kf.errorCovPre);

        state.at<float>(0) = meas.at<float>(0);
        state.at<float>(1) = meas.at<float>(1);
        state.at<float>(2) = 0;
        state.at<float>(3) = 0;

        kf.statePost = state;
        found = true;
    }
    else
    {
        kf.correct(meas);
    }
    //cout << "Measure matrix:" << meas << endl;
    cout << "Error matrix:" << endl << kf.errorCovPost << endl;

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
    kf.measurementMatrix.at<float>(0) = 1;
    kf.measurementMatrix.at<float>(5) = 1;

    // Process Noise Covariance Matrix Q
    // [ Ex   0   0     0     ]
    // [ 0    Ey  0     0     ]
    // [ 0    0   Ev_x  0     ]
    // [ 0    0   0     Ev_y  ]
    setIdentity(kf.processNoiseCov, Scalar(1e-2));

    // Measures Noise Covariance Matrix R
    setIdentity(kf.measurementNoiseCov, Scalar(1e-1));

    // control_vector = numpy.matrix([[0],[0],[0.5*-9.81*timeslice*timeslice],[-9.81*timeslice]])
    kf.controlMatrix = Mat::zeros(4, 4, CV_32F);
    kf.controlMatrix.at<float>(5) = 1;
    kf.controlMatrix.at<float>(15) = 1;

    this->found = false;
    this->ticks = 0;
}

Vec4d ArucoEKF::get_position() {
    return Vec4d(
            this->state.at<float>(0), // x
            this->state.at<float>(1), // y
            2 * sqrt(kf.errorCovPost.at<float>(0)), // x,  2x std
            2 * sqrt(kf.errorCovPost.at<float>(5)) // y, 2x std
    );
}

cv::Vec4d ArucoEKF::get_velocity() {
    return Vec4d(
            this->state.at<float>(2), // v_x
            this->state.at<float>(3), // v_y
            2 * sqrt(kf.errorCovPost.at<float>(10)), // v_x, 2x std
            2 * sqrt(kf.errorCovPost.at<float>(15)) // v_y, 2x std
    );
}


