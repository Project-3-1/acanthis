#include "flightcontroller.h"

#include <stdlib.h>
#include <math.h>

#include "ros/ros.h"
#include "std_msgs/Empty.h"

#include "crazyflie_driver/Position.h"


FlightController::FlightController(ros::NodeHandle n, int frequency, const ros::Publisher& cmd_position_pub, const ros::Publisher& cmd_stop_pub)
{
  this->cmd_position_pub = cmd_position_pub;
  this->cmd_stop_pub = cmd_stop_pub;

  this->frequency = frequency;

  this->position.header.seq = 0;
  this->position.header.stamp = ros::Time::now();
  this->position.x = 0;
  this->position.y = 0;
  this->position.z = 0;
  this->position.yaw = 0;
  n.subscribe("/crazyflie/pose", 1,&FlightController::_updatePos,this);
}


void FlightController::arm_drone() {
  ros::Duration(2).sleep();
  while(!ros::ok()) {
    ROS_INFO("Waiting for ros...");
  }
  for(int i = 0; i < 3; i++) {
    this->moveTo(0, 0, 0, 0, .1);
  }
}

void FlightController::takeoff(double height) {
  ROS_ASSERT(height > 0.3);
  while(ros::ok()) {
    for(int i = 0; i < 10; i++) {
      // --- takes us to 30cm in 1sec - this is the min altiude because of the ground effect
      moveTo(0, 0, i / 30.0, 0, 0.1);
    }

    double delta = height - 0.3;
    if(abs(delta) > 0.01) { // --- if delta greater than 1cm
      for(int i = 1; i <= ceil(delta / 0.1); i++) {
        moveTo(0, 0, height - (1 - i * 0.1) * delta, 0, 0.1);
      }
    }
    break;
  }
}

void FlightController::land(){
    ROS_INFO("LANDING ACTIVE");
    ros::Rate rate = create_rate();
    while(ros::ok()){
        ROS_INFO("%f",pose.pose.position.z);
        ros::spinOnce();
        rate.sleep();
    }
}

void FlightController::stop() {
  std_msgs::Empty stop_msg;
  cmd_stop_pub.publish(stop_msg);
}

void FlightController::moveTo(float x, float y, float z, float yaw, float max_time) {
  ros::Time start = ros::Time::now();
  ros::Rate rate = create_rate();
  while (ros::ok()) {


    this->position.header.seq++;
    this->position.header.stamp = ros::Time::now();
    this->position.x = x;
    this->position.y = y;
    this->position.z = z;
    this->position.yaw = yaw;

    this->cmd_position_pub.publish(position);
    rate.sleep();

    if (max_time > 0) {
      if((ros::Time::now() - start).toSec() > max_time) {
        break;
      }
    }
    else {
      break;
    }
  }
}

ros::Rate FlightController::create_rate() {
  ros::Rate rate(this->frequency);
  return rate;
}

void FlightController::_updatePos(const geometry_msgs::PoseStamped& pos) {
    this->pose = pos;
}
