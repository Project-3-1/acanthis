#ifndef SRC_Bug_H
#define SRC_Bug_H

#include "flightcontroller.h"
#include "ros/ros.h"
#include "crazyflie_driver/Position.h"
#include "crazyflie_driver/GenericLogData.h"
#include "geometry_msgs/PoseStamped.h"

#include "acanthis/ArucoPose.h"
#include "ArucoEKF.h"

enum State {
    EXPLORATION,
    TRACKING,
    DONE
};

class Bug {
    State state = EXPLORATION;

    ros::NodeHandle& node;
    FlightController controller;

    ros::Subscriber aruco_pose_sub;
    double marker_offset_x;
    double marker_offset_y;
    double marker_offset_z;

    acanthis::ArucoPose::ConstPtr aruco_pose;

    ArucoEKF ekf;

    float hoverHeight;
    double minDist;
    double waySize;
    double distMoved;
    bool inFirstLoop;
public:
    Bug(ros::NodeHandle& node, double frequency);
    void explore();
    void demo();

private:
    void avoid();
    bool is_close(double distance);
    bool is_in_Distance(double distance);
    void turn_at_wall(Direction direction);
    void get_relative_left_right(Direction current, Direction& d1, Direction& d2);
    Direction negate_dir(Direction dir);
    void move_in_dir(Direction dir);

    void _update_aruco_pose(const acanthis::ArucoPose::ConstPtr& pose);
};

#endif //SRC_Bug_H