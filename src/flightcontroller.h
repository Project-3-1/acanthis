#ifndef FLIGHTCONTROLLER_H
#define FLIGHTCONTROLLER_H

#include "ros/ros.h"

#include "crazyflie_driver/Position.h"
#include "acanthis/CmdVelocity.h"
#include "crazyflie_driver/GenericLogData.h"
#include "geometry_msgs/PoseStamped.h"
#include "crazyflie_driver/VelocityWorld.h"
#include <opencv4/opencv2/opencv.hpp>

const double DEG_TO_RAD = M_PI / 180.0;
const double RAD_TO_DEG = 180.0 / M_PI;
const double MM_TO_M = 1.0 / 1000;
const double M_TO_MM = 1000;

enum Direction {
    FORWARD = 0,
    RIGHT   = 1,
    BACK    = 2,
    LEFT    = 3,
    UP      = 4,
    DOWN    = 5,
};

const double LANDING_HEIGHT = 0.1; // [m] - At which point the drone should turn of its motors

class FlightController {

    crazyflie_driver::Position position;
    crazyflie_driver::VelocityWorld velocity;
    geometry_msgs::PoseStamped::_pose_type pose;
    ros::Publisher cmd_position_pub, cmd_stop_pub, cmd_velocity_pub;
    ros::Subscriber crazyflie_pose_sub, crazyflie_ranger_sub;

    bool cancelled = false;
    double frequency;
    double range_measurements[5] {0, 0, 0, 0, 0};

public:
    FlightController(ros::NodeHandle n, double freqency);

    bool is_move_cancelled();
    void cancel_movement();

    void arm_drone();

    void cmd_velocity(double x, double y, double z);

    void move_absolute(double x, double y, double z, int yaw, bool error_correction);
    void move_relative(double x, double y, double z, int yaw, bool error_correction);

    void move_absolute(double x, double y, double z, int yaw);
    void move_relative(double x, double y, double z, int yaw);

    void move_until_object(Direction direction, double min_distance);
    void move_in_direction(Direction direction, double distance);
    void turn_left();
    void turn_right();

    cv::Vec3f get_position();
    double get_x();
    double get_y();
    double get_z();
    double get_distance_measurement(Direction direction);
    Direction get_closest_direction(const Direction directions []);

    void takeoff(float height);
    void land();
    void stop();
    void hover(double time);


private:
    void _reset_move_cancelled();

    void _update_pos(const geometry_msgs::PoseStamped &p);
    void _update_ranger(const crazyflie_driver::GenericLogData::ConstPtr &ranger);
    void _publish_position(double x, double y, double z, double yaw);
    double _calculate_yaw(geometry_msgs::PoseStamped::_pose_type::_orientation_type orientation);

    void _wait_for_pose_subscription();
    void _wait_for_ranger_subscription();

    ros::Rate _create_rate() const;
};

#endif // FLIGHTCONTROLLER_H
