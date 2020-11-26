
#include "ros/ros.h"
#include "crazyflie_driver/Takeoff.h"
#include "crazyflie_driver/Hover.h"
#include "crazyflie_driver/Position.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Point.h"
using namespace std;

int main(int argc, char **argv)
{

  ros::init(argc, argv, "FlyRectangleDemo", ros::init_options::AnonymousName);
  ros::NodeHandle n("~");
  ros::Rate rate(2);

//  ROS_INFO("Waiting for /crazyflie/takeoff");
//  ros::ServiceClient takeoffService = n.serviceClient<crazyflie_driver::Takeoff>("/crazyflie/takeoff");
//  takeoffService.waitForExistence();
//  ROS_INFO("/crazyflie/takeoff alive");

//  crazyflie_driver::Takeoff takeoff;
//  takeoff.request.duration = ros::Duration(5);
//  takeoff.request.groupMask = 1;
//  takeoff.request.height = 0.5;

  ros::Publisher cmd_position_pub = n.advertise<crazyflie_driver::Position>("/crazyflie/cmd_position", 1);

  crazyflie_driver::Position position;
  position.header.seq = 0;
  position.header.stamp = ros::Time::now();

  while (ros::ok()) {
    for(int i = 0; i < 3; i++) {
      ROS_INFO("a");
      position.header.seq++;
      position.header.stamp = ros::Time::now();
      position.x = 0;
      position.y = 0;
      position.z += 0.1;
      cmd_position_pub.publish(position);
      ros::Duration(0.1).sleep();
      ROS_INFO("b");
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
      break;
  }




//  ros::Publisher chatter_pub = n.advertise<crazyflie_driver::Hover>("/crazyflie/cmd_hover", 1);

//  crazyflie_driver::Hover hover;
//  hover.header.seq = 0;
//  hover.header.stamp = ros::Time::now();

//  while (ros::ok()) {
//    ROS_INFO("a");

//    hover.vx = 0.001;
//    hover.vy = 0.001;
//    hover.yawrate = 0;
//    hover.zDistance = 0.5;
//    chatter_pub.publish(hover);
//    hover.header.seq++;
//    ros::spinOnce();
//    rate.sleep();
//  }
  //--- call services
  //ros::service::waitForService("takeoff", ros::Duration(2));
  //ros::ServiceClient client = node.serviceClient<crazyflie_driver::Takeoff>("takeoff");
  //crazyflie_driver::Takeoff request;
  //request.request.groupMask = 0;
  //request.request.height = 0.3;
  //request.request.duration = ros::Duration(5, 0);
  //bool x = client.call(request);
  //ROS_INFO("%d", x);


  return 0;
}

