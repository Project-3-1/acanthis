#include "flightcontroller.h"

#include <cstdlib>
#include <cmath>

#include "ros/ros.h"
#include "std_msgs/Empty.h"

#include "crazyflie_driver/Position.h"


/**
 * Creates a new flight controller to control the Crazyflie.
 * @param node
 * @param frequency
 */
FlightController::FlightController(ros::NodeHandle node, double frequency) {
    this->frequency = frequency;

    this->position.header.seq = 0;
    this->position.header.stamp = ros::Time::now();
    this->position.x = 0;
    this->position.y = 0;
    this->position.z = 0;
    this->position.yaw = 0;
    node.subscribe("/crazyflie/pose", 1, &FlightController::_updatePos, this);
    cmd_position_pub = node.advertise<crazyflie_driver::Position>("/crazyflie/cmd_position", 1);
    cmd_stop_pub = node.advertise<std_msgs::Empty>("/crazyflie/cmd_stop", 1);
}

/**
 * In order to arm the drone, we have to send a few messages.
 */
void FlightController::arm_drone() {
    while ((pose.position.x * pose.position.x + pose.position.y * pose.position.y +
            pose.position.z * pose.position.z) == 0) {
        ROS_INFO("Waiting for crazyflie to send first /crazyflie/pose packet...");
        ros::Duration(0.1).sleep();
    }
    for (int i = 0; i < 3; i++) {
        this->moveTo(0, 0, 0, 0);
    }
}

/**
 * This method takes off iff the drone is on the ground. This is checked by checking if pos(z) <= 5cm. The takeoff height
 * also has to be >= 30cm, otherwise the ground effect might cause the drone to crash. This is enforced with assertions.
 * @param height
 */
void FlightController::takeoff(float height) {
    ROS_ASSERT(height >= 0.3);
    ROS_ASSERT(pose.position.z <= 0.05); // we are landed currently

    ros::Rate rate = create_rate();

    while (ros::ok()) {
        // --- takes us to 30cm in 1sec - this is the min altiude because of the ground effect
        for (int i = 1; i <= frequency; i++) {
            this->position.header.seq++;
            this->position.header.stamp = ros::Time::now();
            this->position.x = pose.position.x;
            this->position.y = pose.position.y;
            this->position.z = i / 30.0;
            this->position.yaw = 0; //TODO calculate yaw
            this->cmd_position_pub.publish(position);
            rate.sleep();
        }
    }

    // move to the final height in normal flight mode
    move(0, 0, height - 0.3f, 0);
}

void FlightController::land() {
    ROS_INFO("LANDING ACTIVE");
    move(0, 0, 0.2 - pose.position.z, 0);
    stop();
}

void FlightController::stop() {
    std_msgs::Empty stop_msg;
    cmd_stop_pub.publish(stop_msg);
}

/**
 * Moves the drone relative to its current position.
 * @param dx delta x
 * @param dy delta x
 * @param dz delta z
 * @param dyaw delta yaw
 */
void FlightController::move(double dx, double dy, double dz, double dyaw) {
    moveTo(pose.position.x + dx, pose.position.y + dy, pose.position.z + dz,
           0); //TODO what is the yaw field called in pose?
}

/**
 * Moves the drone in an __absolute__ manner.
 * @param x new x position
 * @param y new y position
 * @param z new z position
 * @param yaw new yaw rotation
 */
void FlightController::moveTo(double x, double y, double z, double yaw) {
    ros::Rate rate = create_rate();

    // Note: The idea here is that we first adjust the z axis because going up and down is
    // slow. Furthermore, we make the assumption that z >= 0.3 because of the ground effect.
    // After the drone has reached the desired altitude, we adjust the remaining horizontal axis.
    // TODO: Test if we can just combine all axis (does it even matter)

    // --- max movement speed for each axis in m/s
    const double max_x = 0.2;
    const double max_y = 0.2;
    const double max_z = 0.1;

    double cx = pose.position.x;
    double cy = pose.position.y;
    double cz = pose.position.z;

    // --- how much we have to move on each axis
    double dx = x - cx;
    double dy = y - cy;
    double dz = z - cz;

    // --- we adjust the z axis first because it is slow
    if (abs(dz) > 0) {
        double z_speed = dz / max_z;
        while (ros::ok()) {
            for (int i = 1; i <= ceil(frequency * z_speed); i++) {
                position.header.seq++;
                position.header.stamp = ros::Time::now();
                position.x = cx;
                position.y = cy;
                position.z = cz + (z_speed * i) / frequency;
                position.yaw = yaw;
                cmd_position_pub.publish(position);

                ros::spinOnce();
                rate.sleep();
            }
            break;
        }
    }


    // --- adjust x and z axis
    double max_time = std::max(abs(dx / max_x), abs(dy / max_y));
    double x_speed = dx / max_time;
    double y_speed = dy / max_time;

    while (ros::ok()) {
        for (int i = 1; i <= ceil(frequency * max_time); i++) {
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

ros::Rate FlightController::create_rate() const {
    return {frequency};
}

/**
 * Called by the /crazyflie/pose subscriber to update the pose of the drone.
 * @param p
 */
void FlightController::_updatePos(const geometry_msgs::PoseStamped &p) {
    this->pose = p.pose;
}
