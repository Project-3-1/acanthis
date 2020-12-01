#include "flightcontroller.h"

#include <cstdlib>
#include <cmath>

#include "ros/ros.h"
#include "std_msgs/Empty.h"

#include "crazyflie_driver/Position.h"
#include "crazyflie_driver/GenericLogData.h"

const bool ENABLE_YAW_CONTROL = true;

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

    this->crazyflie_pose_sub = node.subscribe("/crazyflie/pose", 1, &FlightController::_updatePos, this); // actually important to keep this reference around...
    this->_wait_for_pose_subscription();

    this->crazyflie_ranger_sub = node.subscribe("/crazyflie/ranger_deck", 1, &FlightController::_updateRanger, this); // actually important to keep this reference around...
    this->_wait_for_ranger_subscription();

    // ---
    this->cmd_position_pub = node.advertise<crazyflie_driver::Position>("/crazyflie/cmd_position", 1);
    this->cmd_stop_pub = node.advertise<std_msgs::Empty>("/crazyflie/cmd_stop", 1);
}

/**
 * In order to arm the drone, we have to send a few messages.
 */
void FlightController::arm_drone() {
    for (int i = 0; i < 3; i++) {
        this->moveRelative(0, 0, 0, 0);
    }
}

/**
 * This method takes off iff the drone is on the ground. This is checked by checking if pos(z) <= 5cm. The takeoff height
 * also has to be >= 30cm, otherwise the ground effect might cause the drone to crash. This is enforced with assertions.
 * @param height
 */
void FlightController::takeoff(float height) {
    ROS_ASSERT(height >= 0.3);
    //ROS_ASSERT(pose.position.z <= 0.05);

    ros::Rate rate(10);

    while (ros::ok()) {
        // --- takes us to 30cm in 1sec - this is the min altitude because of the ground effect
        for (int i = 1; i <= 10; i++) {
            _publish_position(pose.position.x, pose.position.y, i / 25.0, _calculate_yaw(pose.orientation));
            ros::spinOnce();
            rate.sleep();
        }
        break;
    }

    // move to the final height in normal flight mode
    moveAbsolute(0, 0, height, 0);
}

void FlightController::land() {
    ROS_INFO("LANDING ACTIVE");
    moveRelative(0, 0, 0.2 - pose.position.z, 0);
    stop();
}

void FlightController::stop() {
    std_msgs::Empty stop_msg;
    cmd_stop_pub.publish(stop_msg);
}

/**
 * Hovers in place for the specified amount of time
 * @param time in seconds
 */
void FlightController::hover(double time) {
    geometry_msgs::PoseStamped::_pose_type cp = this->pose;
    ros::Time start = ros::Time::now();
    while (true) {
        ROS_INFO("yaw-copy:  %.2f", _calculate_yaw(cp.orientation));
        moveAbsolute(cp.position.x, cp.position.y, cp.position.z, _calculate_yaw(cp.orientation)); //todo fix

        ros::Time now = ros::Time::now();
        if((now - start).toSec() >= 5) {
            break;
        }
    }
}

/**
 * Moves the drone relative to its current position.
 * @param dx delta x
 * @param dy delta x
 * @param dz delta z
 * @param dyaw delta yaw
 */
void FlightController::moveRelative(double dx, double dy, double dz, int dyaw) {
    moveAbsolute(pose.position.x + dx, pose.position.y + dy, pose.position.z + dz,
                 _calculate_yaw(pose.orientation) + dyaw);
}

/**
 * Moves the drone in an __absolute__ manner.
 * @param x new x position
 * @param y new y position
 * @param z new z position
 * @param yaw new yaw rotation
 */
void FlightController::moveAbsolute(double x, double y, double z, int yaw) {
    ros::Rate rate = create_rate();

    // --- max movement speed for each axis in m/s
    const double max_x = 0.6; // in m/s
    const double max_y = 0.6; // in m/s
    const double max_z = 0.2; // in m/s
    const double max_yaw = 45; // in deg/s

    // --- current values
    double cx = pose.position.x;
    double cy = pose.position.y;
    double cz = pose.position.z;
    double cyaw = _calculate_yaw(pose.orientation);

    // --- how much we have to move on each axis
    double dx = x - cx;
    double dy = y - cy;
    double dz = z - cz;
    double dyaw = (yaw-cyaw);// + 3 * M_PI) % 2 * M_PI - M_PI_2;
    ROS_INFO("delta yaw: %.2f", dyaw);

    // --- we adjust the z axis first because it is slow
    if (abs(dz) > 0) {
        double z_steps = abs(dz / max_z);
        double z_speed = dz / z_steps;
        while (ros::ok()) {
            ROS_INFO(" dz: %f, max_z: %f, z steps: %f", dz, max_z, ceil(frequency * z_steps));
            for (int i = 1; i <= abs(ceil(frequency * z_steps)); i++) {
                _publish_position(cx, cy, cz + (z_speed * i) / frequency, cyaw);
                ros::spinOnce();
                rate.sleep();
            }
            break;
        }
    }

    // --- adjust x and z axis
    double max_steps = std::max(abs(dx / max_x), abs(dy / max_y));
    ROS_INFO("max steps for x and z axis: %f", max_steps);

    if(max_steps > 0) {
        double x_speed = dx / max_steps;
        double y_speed = dy / max_steps;

        while (ros::ok()) {
            for (int i = 1; i <= floor(frequency * max_steps); i++) {
                _publish_position(cx + (x_speed * i) / frequency, cy + (y_speed * i) / frequency, z, cyaw);
                ros::spinOnce();
                rate.sleep();
            }
            break;
        }
    }

    // --- adjust yaw
    if (abs(dyaw) > 0 && ENABLE_YAW_CONTROL) {
        double yaw_steps = abs(dyaw / max_yaw);
        double yaw_speed = dyaw / yaw_steps;
        while (ros::ok()) {
            for (int i = 1; i <= floor(frequency * yaw_steps); i++) {
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
        ros::spinOnce();
        double error = std::sqrt((std::pow(pose.position.x - x, 2) + std::pow(pose.position.y - y, 2)
                                  + std::pow(pose.position.z - z, 2)));

        double yaw_error = std::fmod(std::abs(_calculate_yaw(pose.orientation) - yaw), 360);

        if(error < 0.05 && yaw_error <= 10) {
            break;
        }
        ROS_WARN("Error still too big -> dis_err: %.2f, yaw_err: %.2fdeg", error, yaw_error);
        rate.sleep();
    }
}

ros::Rate FlightController::create_rate() const {
    return {frequency};
}

void FlightController::_publish_position(double x, double y, double z, double yaw) {
    ///ROS_INFO("_publish_position -> x: %.2f, y: %.2f, z: %.2f, yaw:  %.2f", x, y, z, yaw);
    position.header.seq++;
    position.header.stamp = ros::Time::now();
    position.x = x;
    position.y = y;
    position.z = z;
    if(ENABLE_YAW_CONTROL) {
        position.yaw = yaw;
    }
    cmd_position_pub.publish(position);
}

double FlightController::_calculate_yaw(geometry_msgs::PoseStamped::_pose_type::_orientation_type q) {
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp) * RAD_TO_DEG;
}

// --- Subscribers

/**
 * Called by the /crazyflie/pose subscriber to update the pose of the drone.
 * @param p
 */
void FlightController::_updatePos(const geometry_msgs::PoseStamped &p) {
    this->pose = p.pose;
}


void FlightController::_updateRanger(const crazyflie_driver::GenericLogData::ConstPtr ranger) {
    this->range_front = ranger->values[Direction::FORWARD];
    this->range_right = ranger->values[Direction::RIGHT];
    this->range_back = ranger->values[Direction::BACK];
    this->range_left = ranger->values[Direction::LEFT];
    this->range_up = ranger->values[Direction::UP];
}

void FlightController::_wait_for_pose_subscription() {
    // ---
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
}

void FlightController::_wait_for_ranger_subscription() {
    ros::Rate rate = create_rate();
    while (ros::ok()) {
        if(this->crazyflie_ranger_sub.getNumPublishers() == 0) {
            ROS_WARN("FlightController: Waiting for /crazyflie/ranger_deck...");
        } else {
            ROS_INFO("FlightController: /crazyflie/ranger_deck is now publishing...");
            double start_left = this->range_left, start_right = this->range_right, start_up = this->range_up,
                    start_back = this->range_back, start_front = this->range_down;

            double change = 0;
            for(int i = 0; i < 10; i++)  {
                change += std::sqrt(std::pow(start_left - this->range_left, 2) + std::pow(start_right - this->range_right, 2) +
                                    std::pow(start_up - this->range_up, 2) + std::pow(start_back- this->range_back, 2) +
                                    std::pow(start_front - this->range_front, 2));
                ros::spinOnce();
                rate.sleep();
            }

            if(change == 0) {
                ROS_ERROR("FlightController: /crazyflie/ranger_deck not updating");
                ROS_ASSERT(false);
            } else {
                ROS_INFO("FlightController: /crazyflie/ranger_deck seems to be okay (change: %.2f)", change);
            }
            break;
        }
        ros::spinOnce();
        rate.sleep();
    }
}
