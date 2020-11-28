#ifndef FLIGHTCONTROLLER_H
#define FLIGHTCONTROLLER_H

#include "ros/ros.h"
#include "crazyflie_driver/Position.h"
#include "geometry_msgs/PoseStamped.h"

class FlightController {
    crazyflie_driver::Position position;
    double frequency;
    geometry_msgs::PoseStamped::_pose_type pose;
    ros::Publisher cmd_position_pub, cmd_stop_pub;

public:
    FlightController(ros::NodeHandle n, double freqency);

    void arm_drone();

    void moveTo(double x, double y, double z, double yaw);

    void move(double dx, double dy, double dz, double dyaw);

    void takeoff(float height);

    void land();

    void stop();

    ros::Rate create_rate() const;

private:
    void _updatePos(const geometry_msgs::PoseStamped &p);

};

#endif // FLIGHTCONTROLLER_H
