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

    // ---
    this->crazyflie_pose_sub = node.subscribe("/crazyflie/pose", 1, &FlightController::_updatePos, this);
    ros::Rate rate = create_rate();
    while (ros::ok()) {
        if(this->crazyflie_pose_sub.getNumPublishers() == 0) {
            ROS_WARN("FlightController: Waiting for /crazyflie/pose...");
        } else {
            ROS_INFO("FlightController: /crazyflie/pose is now publishing...");
            break;
        }
        ros::spinOnce();
        rate.sleep();
    }

    // ---
    this->cmd_position_pub = node.advertise<crazyflie_driver::Position>("/crazyflie/cmd_position", 1);
    this->cmd_stop_pub = node.advertise<std_msgs::Empty>("/crazyflie/cmd_stop", 1);
}

/**
 * In order to arm the drone, we have to send a few messages.
 */
void FlightController::arm_drone() {
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
        // --- takes us to 30cm in 1sec - this is the min altitude because of the ground effect
        for (int i = 1; i <= frequency; i++) {
            _publish_position(pose.position.x, pose.position.y, i / 30.0, _calculate_yaw(pose.orientation));
            ros::spinOnce();
            rate.sleep();
        }
        break;
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
void FlightController::move(double dx, double dy, double dz, int dyaw) {
    moveTo(pose.position.x + dx, pose.position.y + dy, pose.position.z + dz,
           _calculate_yaw(pose.orientation) * RAD_TO_DEG + dyaw);
}

/**
 * Moves the drone in an __absolute__ manner.
 * @param x new x position
 * @param y new y position
 * @param z new z position
 * @param yaw new yaw rotation
 */
void FlightController::moveTo(double x, double y, double z, int yaw) {
    ros::Rate rate = create_rate();

    // Note: The idea here is that we first adjust the z axis because going up and down is
    // slow. Furthermore, we make the assumption that z >= 0.3 because of the ground effect.
    // After the drone has reached the desired altitude, we adjust the remaining horizontal axis.
    // TODO: Test if we can just combine all axis (does it even matter)

    // --- max movement speed for each axis in m/s
    const double max_x = 0.2; // in m/s
    const double max_y = 0.2; // in m/s
    const double max_z = 0.1; // in m/s
    const double max_yaw = 20; // in deg/s

    // --- current values
    double cx = pose.position.x;
    double cy = pose.position.y;
    double cz = pose.position.z;
    int cyaw = _calculate_yaw(pose.orientation) * RAD_TO_DEG;

    // --- how much we have to move on each axis
    double dx = x - cx;
    double dy = y - cy;
    double dz = z - cz;
    int dyaw = (yaw-cyaw + 540) % 360 - 180;

    // --- we adjust the z axis first because it is slow
    if (abs(dz) > 0) {
        double z_steps = abs(dz / max_z);
        double z_speed = dz / z_steps;
        while (ros::ok()) {
            for (int i = 1; i <= floor(frequency * z_speed); i++) {
                _publish_position(cx, cy, cz + (z_speed * i) / frequency, cyaw);
                ros::spinOnce();
                rate.sleep();
            }
            break;
        }
    }

    // --- adjust x and z axis
    double max_steps = std::max(abs(dx / max_x), abs(dy / max_y));

    if(max_steps > 0) {
        double x_speed = dx / max_steps;
        double y_speed = dy / max_steps;

        while (ros::ok()) {
            for (int i = 1; i <= floor(frequency * max_steps); i++) {
                _publish_position(cx + (x_speed * i) / frequency, cy + (y_speed * i) / frequency, z, cyaw);
                ros::spinOnce();
                rate.sleep();
            }
        }
    }

    // --- adjust yaw
    if (abs(dyaw) > 0) {
        double yaw_steps = abs(dyaw / max_yaw);
        double yaw_speed = dyaw / yaw_steps;
        while (ros::ok()) {
            for (int i = 1; i <= floor(frequency * yaw_speed); i++) {
                _publish_position(x, y, z,cyaw + (yaw_speed * i) / frequency);
                ros::spinOnce();
                rate.sleep();
            }
            break;
        }
    }

    // --- final adjustment
    while (ros::ok()) {
        _publish_position(x, y, z, yaw);
        break;
    }
}

ros::Rate FlightController::create_rate() const {
    return {frequency};
}

/**
 * Called by the /crazyflie/pose subscriber to update the pose of the drone.
 * @param p
 */
void FlightController::_updatePos(const geometry_msgs::PoseStamped &p) {
    ROS_WARN("update pose");
    this->pose = p.pose;
}

void FlightController::_publish_position(double x, double y, double z, double yaw) {
    ROS_INFO("_publish_position -> x: %.2f, y: %.2f, z: %.2f, yaw:  %.2f", x, y, z, yaw);
    /*position.header.seq++;
    position.header.stamp = ros::Time::now();
    position.x = x;
    position.y = y;
    position.z = z;
    position.yaw = yaw;
    cmd_position_pub.publish(position);*/
}

double FlightController::_calculate_yaw(geometry_msgs::PoseStamped::_pose_type::_orientation_type q) {
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}
