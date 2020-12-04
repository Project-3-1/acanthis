#ifndef SRC_RECTANGLEEXPLORER_H
#define SRC_RECTANGLEEXPLORER_H

#include "flightcontroller.h"
#include "ros/ros.h"
#include "crazyflie_driver/Position.h"
#include "crazyflie_driver/GenericLogData.h"
#include "geometry_msgs/PoseStamped.h"


class RectangleExplorer {
    double freqency;
    double hoverHeight;
    double minDist;
    double waySize;
public:
    RectangleExplorer(ros::NodeHandle n, double freqency);
    void explore();

private:
    bool is_close(double distance);
    bool is_in_Distance(double distance);
    void turn_at_wall(Direction direction);
    void get_relative_left_right(Direction current, Direction& d1, Direction& d2);
    Direction negate_dir(Direction dir);
};

#endif //SRC_RECTANGLEEXPLORER_H
