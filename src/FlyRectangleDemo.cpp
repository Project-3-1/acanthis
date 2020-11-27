#include "ros/ros.h"
#include "crazyflie_driver/Takeoff.h"
#include "crazyflie_driver/UpdateParams.h"
#include "crazyflie_driver/Hover.h"
#include "crazyflie_driver/Position.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Point.h"

#include "flightcontroller.h"

using namespace std;


//static void stop() {
//  std_msgs::Empty stop_msg;
//  cmd_stop_pub.publish(stop_msg);
//}



int main(int argc, char **argv)
{


  ros::init(argc, argv, "FlyRectangleDemo", ros::init_options::AnonymousName);
  ros::NodeHandle n("~");

  // --- init static variables
  ros::Rate rate(10);
  ros::Publisher cmd_position_pub = n.advertise<crazyflie_driver::Position>("/crazyflie/cmd_position", 1);
  ros::Publisher cmd_stop_pub = n.advertise<std_msgs::Empty>("/crazyflie/cmd_stop", 1);

  FlightController controller(10, cmd_position_pub, cmd_stop_pub);
  controller.arm_drone();
  controller.takeoff(.4);
  //  for(int i = 0; i < 50; i++) {
//    ROS_INFO("turn: %d", i);
//    controller.moveTo(0, 0, 10, 0, -1);
//    rate.sleep();
//    ros::spinOnce();
//  }

  controller.stop();
//  FlightController controller(10, cmd_stop_pub, cmd_position_pub);
//  controller.moveTo(0, 0, 0, 0);

//  // --- take off
//  while(ros::ok()) {
//    for(int i = 0; i < 10; i++) {
//      moveTo(0, 0, i / 25.0, 0);
//    }
//    for(int i = 0; i < 20; i++) {
//      moveTo(0, 0, 0.4, 0);
//    }
//    break;
//  }

//  moveTo(0, 0, 0.6, 0);
//  stop();
  ros::spin();
  return 0;
}


