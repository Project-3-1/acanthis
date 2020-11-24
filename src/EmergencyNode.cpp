#include "ros/ros.h"
#include "acanthis/Shutdown.h"
#include "crazyflie_driver/FullState.h"


void imu_callback(const crazyflie_driver::FullState::ConstPtr& msg)
{
  ROS_INFO("I got a message from the IMU");
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "EmergencyNode");
  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("crazyflie/imu", 10, imu_callback);
  ros::spin();
  /*ros::Publisher state_publisher = n.advertise<acanthis::Shutdown>("EmergencyNode", 1);

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {

    acanthis::Shutdown msg;
    msg.crashed = false;


    state_publisher.publish(msg);
    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }*/


  return 0;
}

