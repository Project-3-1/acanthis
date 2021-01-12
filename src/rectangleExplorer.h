#ifndef SRC_RECTANGLEEXPLORER_H
#define SRC_RECTANGLEEXPLORER_H

#include <chrono>
#include "flightcontroller.h"
#include "ros/ros.h"
#include "crazyflie_driver/Position.h"
#include "crazyflie_driver/GenericLogData.h"
#include "geometry_msgs/PoseStamped.h"

#include "acanthis/ArucoPose.h"
#include "ArucoEKF.h"

enum State {
    EXPLORATION = 0,
    TRACKING = 1,
    LANDING = 2,
    DONE = 3
};

class RectangleExplorer {
    State state = EXPLORATION;

    ros::NodeHandle& node;
    FlightController controller;

    ros::Subscriber aruco_pose_sub;

    std::chrono::time_point<std::chrono::system_clock> marker_last_seen = std::chrono::system_clock::from_time_t(0);
    int marker_offset_id = 0;
    cv::Vec3f marker_offset;

    acanthis::ArucoPose::ConstPtr aruco_pose;

    ArucoEKF ekf;

    float hoverHeight;
    double minDist;
    double waySize;
    double distMoved;
    bool inFirstLoop;
public:
    RectangleExplorer(ros::NodeHandle& node, double frequency);

    void run();

    void explore();

    void track();
    void track_velocity();

    void land();
    void land_velocity();

    long get_aruco_last_seen();

    void demo();

private:
    bool is_close(double distance);
    bool is_in_Distance(double distance);
    void turn_at_wall(Direction direction);
    void get_relative_left_right(Direction current, Direction& d1, Direction& d2);
    Direction negate_dir(Direction dir);
    void move_in_dir(Direction dir);

    void _update_aruco_pose(const acanthis::ArucoPose::ConstPtr& pose);
};

#endif //SRC_RECTANGLEEXPLORER_H
