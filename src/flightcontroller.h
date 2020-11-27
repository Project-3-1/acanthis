#ifndef FLIGHTCONTROLLER_H
#define FLIGHTCONTROLLER_H

#include "ros/ros.h"
#include "crazyflie_driver/Position.h"

class FlightController {
  crazyflie_driver::Position position;
  int frequency;
  ros::Publisher cmd_position_pub, cmd_stop_pub;

  public:
      FlightController(int freqency, ros::Publisher cmd_position_pub, ros::Publisher cmd_stop_pub);
      void arm_drone();
      void moveTo(float x, float y, float z, float yaw, float max_time);
      void takeoff(float height);
      void land();
      void stop();
      ros::Rate create_rate();
};

#endif // FLIGHTCONTROLLER_H
