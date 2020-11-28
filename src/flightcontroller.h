#ifndef FLIGHTCONTROLLER_H
#define FLIGHTCONTROLLER_H

#include "ros/ros.h"
#include "crazyflie_driver/Position.h"
#include "geometry_msgs/PoseStamped.h"

class FlightController {
  crazyflie_driver::Position position;
  int frequency;
  geometry_msgs::PoseStamped pose;
  ros::Publisher cmd_position_pub, cmd_stop_pub;

  public:
      FlightController(ros::NodeHandle n, int freqency, const ros::Publisher& cmd_position_pub, const ros::Publisher& cmd_stop_pub);
      void arm_drone();
      void moveTo(float x, float y, float z, float yaw, float max_time);
      void takeoff(double height);
      void _updatePos(const geometry_msgs::PoseStamped & pos);
      void land();
      void stop();
      ros::Rate create_rate();
};

#endif // FLIGHTCONTROLLER_H
