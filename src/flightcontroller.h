#ifndef FLIGHTCONTROLLER_H
#define FLIGHTCONTROLLER_H

#include "ros/ros.h"
#include "crazyflie_driver/Position.h"
#include "crazyflie_driver/GenericLogData.h"
#include "geometry_msgs/PoseStamped.h"

const double DEG_TO_RAD = M_PI / 180.0;
const double RAD_TO_DEG = 180.0 / M_PI;

enum Direction {
    FORWARD = 0,
    RIGHT   = 1,
    BACK    = 2,
    LEFT    = 3,
    UP      = 4,
    DOWN    = 5,
};

class FlightController {
    crazyflie_driver::Position position;
    double frequency;
    geometry_msgs::PoseStamped::_pose_type pose;
    ros::Publisher cmd_position_pub, cmd_stop_pub;
    ros::Subscriber crazyflie_pose_sub, crazyflie_ranger_sub;

    double range_measurements[5] {0, 0, 0, 0, 0};

public:
    FlightController(ros::NodeHandle n, double freqency);

    void arm_drone();

    void move_absolute(double x, double y, double z, int yaw);
    void move_relative(double dx, double dy, double dz, int dyaw);

    void move_until_object(Direction direction, double distance);

    double get_distance_measurement(Direction direction);

    void takeoff(float height);
    void land();
    void stop();
    void hover(double time);


private:
    ros::Rate create_rate() const;
    void _update_pos(const geometry_msgs::PoseStamped &p);
    void _update_ranger(const crazyflie_driver::GenericLogData::ConstPtr &ranger);
    void _publish_position(double x, double y, double z, double yaw);
    double _calculate_yaw(geometry_msgs::PoseStamped::_pose_type::_orientation_type orientation);

    void _wait_for_pose_subscription();
    void _wait_for_ranger_subscription();
};

#endif // FLIGHTCONTROLLER_H
