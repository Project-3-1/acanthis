#include "flightcontroller.h"

#include <stdlib.h>
#include <math.h>

#include "ros/ros.h"
#include "std_msgs/Empty.h"

#include "crazyflie_driver/Position.h"


FlightController::FlightController(ros::NodeHandle node, int frequency)
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
  node.subscribe("/crazyflie/pose", 1, &FlightController::_updatePos, this);
  node.advertise<crazyflie_driver::Position>("/crazyflie/cmd_position", 1);
  node.advertise<std_msgs::Empty>("/crazyflie/cmd_stop", 1);
}


void FlightController::arm_drone() {
  float dis = pose.position.x * pose.position.x + pose.position.y * pose.position.y + pose.position.z * pose.position.z;
  while((pose.position.x * pose.position.x + pose.position.y * pose.position.y + pose.position.z * pose.position.z) == 0) {
    ROS_INFO("Waiting for crazyflie to send first /crazyflie/pose packet...");
    ros::Duration(0.1).sleep();
  }
  for(int i = 0; i < 3; i++) {
    this->moveTo(0, 0, 0, 0);
  }
}

void FlightController::takeoff(double height) {
  ROS_ASSERT(height > 0.3);
  ROS_ASSERT(pose.position.z < 0.05); // we are landed currently 

  ros::Rate rate = create_rate();

  while(ros::ok()) {
    // --- takes us to 30cm in 1sec - this is the min altiude because of the ground effect
    for(int i = 1; i <= frequency; i++) {
        this->position.header.seq++;
        this->position.header.stamp = ros::Time::now();
        this->position.x = pose.position.x;
        this->position.y = pose.position.y;
        this->position.z = i / 30.0;
        this->position.yaw = yaw;
        this->cmd_position_pub.publish(position);
        rate.sleep();
    }
  }

  // move to the final height in normal flight mode
  move(0, 0, height - 0.3, 0);
}

void FlightController::land(){
    ROS_INFO("LANDING ACTIVE");
    move(0,0, 0.2-pose.position.z, 0);
    stop();
}

void FlightController::stop() {
  std_msgs::Empty stop_msg;
  cmd_stop_pub.publish(stop_msg);
}

void FlightController::move(float dx, float dy, float dz, float dyaw) {
  moveTo(pose.position.x + dx, pose.position.y + dy, pose.position.z + dz, 0); //TODO what is the yaw field called in pose?
}


void FlightController::moveTo(float x, float y, float z, float yaw) {
  ros::Time start = ros::Time::now();
  ros::Rate rate = create_rate();

  // Note: The idea here is that we first adjust the z axis because going up and down is
  // slow. Furthermore, we make the assumption that z >= 0.3 because of the ground effect.
  // After the drone has reached the desired altiude, we adjust the remaining horizontal axis.
  // TODO: Test if we can just combine all axis (does it even matter)

  // --- max movement speed for each axis
  const float max_x = 0.2;
  const float max_y = 0.2;
  const float max_z = 0.1;

  float cx = pose.position.x;
  float cy = pose.position.y;
  float cz = pose.position.z;

  // --- how much we have to move on each axis
  float dx = x - cx;
  float dy = y - cy;
  float dz = z - cz;

  // --- we adjust the z axis first because it is slow
  if(abs(dz) > 0) {
    float z_speed = dz / max_z;
    while(ros::ok()) {
      for(int i = 1; i <= ceil(frequency * z_speed); i++) {
        this->position.header.seq++;
        this->position.header.stamp = ros::Time::now();
        this->position.x = cx;
        this->position.y = cy;
        this->position.z = cz + (z_speed * i) / frequency;
        this->position.yaw = yaw;
        this->cmd_position_pub.publish(position);

        ros::spinOnce();
        rate.sleep();
      }
      break;
    }
  }
  

  // --- adjust x and z axis
  float max_time = max(abs(dx / max_x), abs(dy / max_y));
  float x_speed = dx / max_time;
  float y_speed = dy / max_time;
  
  while (ros::ok()) {
    for(int i = 1; i <= ceil(frequency * max_time); i++) {
        this->position.header.seq++;
        this->position.header.stamp = ros::Time::now();
        this->position.x = cx + (x_speed * i) / frequency;
        this->position.y = cy + (y_speed * i) / frequency;
        this->position.z = z;
        this->position.yaw = yaw;
        this->cmd_position_pub.publish(position);

        ros::spinOnce();
        rate.sleep();
      }
  }

  // --- TODO: figure out if yaw is in degrees or radians and add it 
}

ros::Rate FlightController::create_rate() {
  ros::Rate rate(this->frequency);
  return rate;
}

void FlightController::_updatePos(const geometry_msgs::PoseStamped& pose) {
    this->pose = pose.pose;
}
