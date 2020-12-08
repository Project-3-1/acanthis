#ifndef SRC_RECTANGLEEXPLORER_H
#define SRC_RECTANGLEEXPLORER_H

#include "flightcontroller.h"
#include "ros/ros.h"
#include "crazyflie_driver/Position.h"
#include "crazyflie_driver/GenericLogData.h"
#include "geometry_msgs/PoseStamped.h"

#include "acanthis/ArucoPose.h"

enum State {
    EXPLORATION,
    TRACKING,
    DONE
};

class RectangleExplorer {
    State state = EXPLORATION;

    ros::NodeHandle& node;
    FlightController controller;

    ros::Subscriber aruco_pose_sub;
    double marker_x;
    double marker_y;

    acanthis::ArucoPose::ConstPtr aruco_pose;

    float hoverHeight;
    double minDist;
    double waySize;
    double distMoved;
    bool inFirstLoop;
public:
    RectangleExplorer(ros::NodeHandle& node, double frequency);
    void explore();
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
