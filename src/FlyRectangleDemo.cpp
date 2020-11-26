
#include "ros/ros.h"
#include "crazyflie_driver/Takeoff.h"
#include "std_msgs/Empty.h"
using namespace std;

int main(int argc, char **argv)
{

  ros::init(argc, argv, "FlyRectangleDemo");
  //ros::NodeHandle node;

  ROS_INFO("DKE");
  //--- call services
  //ros::service::waitForService("takeoff", ros::Duration(2));
  //ros::ServiceClient client = node.serviceClient<crazyflie_driver::Takeoff>("takeoff");
  //crazyflie_driver::Takeoff request;
  //request.request.groupMask = 0;
  //request.request.height = 0.3;
  //request.request.duration = ros::Duration(5, 0);
  //bool x = client.call(request);
  //ROS_INFO("%d", x);

  while(ros::ok()) {
    ROS_INFO("llllllllllllllllHELOOOOOOOOOOOOOOOOOOOOO");
    cout << "Hello World 2!" << endl;
    ros::spinOnce();

  }
  //crayzflie_driver::UpdateParams request;
  //ros::param::set("kalman/resetEstimation", true);
  //request.
  //loop_rate.sleep();
  //ros::param::set("kalman/resetEstimation", false);
  //loop_rate.sleep();



  return 0;
}

