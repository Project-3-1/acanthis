#include "ros/ros.h"
#include "acanthis/Shutdown.h"
#

int main(int argc, char **argv)
{

  ros::init(argc, argv, "EmergencyNode");
  ros::NodeHandle node;
  //ros::Subscriber sub = n.subscribe("crazyflie/imu", 10, imu_callback);

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


/*void imu_callback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}*/
