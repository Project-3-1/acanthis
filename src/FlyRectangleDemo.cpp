
#include "ros/ros.h"
#include "crazyflie_driver/Takeoff.h"
#include "crazyflie_driver/UpdateParams.h"
#include "crazyflie_driver/Hover.h"
#include "crazyflie_driver/Position.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Point.h"
using namespace std;

int main(int argc, char **argv)
{

  ros::init(argc, argv, "FlyRectangleDemo", ros::init_options::AnonymousName);
  ros::NodeHandle n("~");
  ros::Rate rate(10);

  ROS_INFO("Waiting for /crazyflie/takeoff");
  ros::ServiceClient takeoffService = n.serviceClient<crazyflie_driver::Takeoff>("/crazyflie/takeoff");
  takeoffService.waitForExistence();
  ROS_INFO("/crazyflie/takeoff alive");

  ros::Publisher cmd_position_pub = n.advertise<crazyflie_driver::Position>("/crazyflie/cmd_position", 1);

  crazyflie_driver::Position position;
  position.header.seq = 0;
  position.header.stamp = ros::Time::now();

  while(ros::ok()) {
    for(int i = 0; i < 10; i++) {
      position.header.seq++;
      position.header.stamp = ros::Time::now();
      position.x = 0;
      position.y = 0;
      position.z = i / 25.0;
      position.yaw = 0;
      cmd_position_pub.publish(position);
      rate.sleep();
    }
    for(int i = 0; i < 20; i++) {
      position.header.seq++;
      position.header.stamp = ros::Time::now();
      position.x = 0;
      position.y = 0;
      position.z = 0.4;
      position.yaw = 0;
      cmd_position_pub.publish(position);
      rate.sleep();
    }
    break;
  }

  while (ros::ok()) {
      position.header.seq++;
      position.header.stamp = ros::Time::now();
      position.x = 0;
      position.y = 0;
      position.z = 0.4;
      cmd_position_pub.publish(position);
      rate.sleep();
  }

  return 0;
}

