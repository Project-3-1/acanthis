#include "rectangleExplorer.h"

#include "flightcontroller.h"
#include "ros/node_handle.h"

#include "acanthis/ArucoPose.h"

RectangleExplorer::RectangleExplorer(ros::NodeHandle& node, double frequency)
    : controller(FlightController(node, frequency)), node(node) {

    // ---
    ros::Rate rate(2);
    this->aruco_pose_sub = node.subscribe("/acanthis/aruco_detector/pose", 1, &RectangleExplorer::_update_aruco_pose, this);
    while (this->aruco_pose_sub.getNumPublishers() == 0) {
        rate.sleep();
    }
    // ---

    this->hoverHeight = 0.9;
    this->minDist = 0.5;
    this->waySize = 0.5;
    this->distMoved = 0;
    this->inFirstLoop = true;
}
bool RectangleExplorer::is_close(double distance){
    double min = 0.3;
    return (distance < min);
}
bool RectangleExplorer::is_in_Distance(double distance) {
    double min = 0.75;
    return (distance < min);
}

void RectangleExplorer::run() {
    // Start and hover at height
    controller.arm_drone();
    controller.takeoff(hoverHeight);

    while (ros::ok() && state != DONE) {
        //ROS_INFO("transition state: %d", state);

        switch (state) {
            case EXPLORATION: explore();
                continue;
            case TRACKING: track();
                continue;
            case LANDING: land();
                continue;
            case DONE:
                break;
        }
    }
}

void RectangleExplorer::explore() {
    controller.hover(2.5);
    controller.move_relative(0, 0, hoverHeight - controller.get_z(), 0, true);

    // Find Closest Wall
    Direction directions[] {LEFT,RIGHT,FORWARD,BACK};
    Direction closest = controller.get_closest_direction(directions);
    //Move To It
    ROS_INFO("move %d", closest);
    controller.move_until_object(closest,minDist);
    ROS_INFO("move done");
    //Get Directions between which we move
    Direction dir1 = LEFT;
    Direction dir2 = RIGHT;
    get_relative_left_right(closest,dir1,dir2);
    // angle changes
    double dist1 = controller.get_distance_measurement(dir1);
    double dist2 = controller.get_distance_measurement(dir2);

    if(dist1 < dist2){
        if(dist1 < 1){
            double angle = (atan(dist1/minDist))*RAD_TO_DEG;
            ROS_INFO("Angle1%f",angle);
            controller.move_relative(.0,.0,.0,angle, true);
        }
    }else{
        if(dist2 < 1){
            double angle = (atan(dist2/minDist))*RAD_TO_DEG;
            ROS_INFO("Angle2%f",angle);
            controller.move_relative(.0,.0,.0,angle, true);
        }
    }

    Direction goTo = negate_dir(closest);
    // Do Exploration
    while (ros::ok() && inFirstLoop && state == EXPLORATION){
        controller.move_until_object(dir1,minDist);
        if(state == TRACKING) break;
        move_in_dir(goTo);
        if(state == TRACKING) break;
        if(!inFirstLoop){
            return;
        }
        controller.move_until_object(dir2,minDist);
        if(state == TRACKING) break;
        move_in_dir(goTo);
        if(state == TRACKING) break;
        ros::spinOnce();
        if(state == TRACKING) break;
    }

}

void RectangleExplorer::track_velocity() {

    controller.move_absolute(controller.get_x(), controller.get_y(), controller.get_z(), 0);
    /*cv::Vec3f platform_velocity(aruco_pose->velocity.x, aruco_pose->velocity.y, aruco_pose->velocity.z);
    cv::Vec3f platform_position(aruco_pose->position.x, aruco_pose->position.y, aruco_pose->position.z);
    cv::Vec3f drone_speed = controller.get_max_speed();

    double dt = 100;
    double last_valid_dt = dt;
    cv::Vec3f intersection = platform_position + platform_velocity * dt;
    cv::Vec3f trajectory = intersection / dt;
    ROS_INFO_STREAM("intersection >> " << intersection);
    ROS_INFO_STREAM("position >> " << controller.get_position());
    ROS_INFO_STREAM("platform_velocity >> " << platform_velocity);

    while (true) {
        dt -= 0.25;
        intersection = platform_position + platform_velocity * dt;
        trajectory = intersection / dt;
        ROS_INFO_STREAM("dt: " << intersection << " - " << trajectory);
        if(trajectory[0] < drone_speed[0] && trajectory[1] < drone_speed[1]) {
            last_valid_dt = dt;
        } else {
            break;
        }
    }*/



    const double transition_distance = 0.5; // [m]
    double error = sqrt(pow(marker_offset[0], 2) + pow(marker_offset[1], 2));

    cv::Vec3f platform_position;
    const cv::Vec3f drone_speed = controller.get_max_speed();
    ros::Rate rate(10);
    double now;

    while (error >= 0.05) {

        platform_position = cv::Vec3f(aruco_pose->position.x, aruco_pose->position.y, 0);
        double dt_x = platform_position[0] / drone_speed[0];
        double dt_y = platform_position[1] / drone_speed[1];

        double max_dt = std::max(dt_x, dt_y);
        double v_x = platform_position[0] / max_dt;
        double v_y = platform_position[1] / max_dt;

        now = ros::Time::now().toSec();
        ROS_INFO("max_dt: %.2f", max_dt);
        while (ros::Time::now().toSec() - now <= max_dt) {
            if(controller.get_z() < hoverHeight) {
                controller.cmd_velocity(v_x, v_y, 0);
            } else {
                controller.cmd_velocity(v_x, v_y, 0);
            }
            ros::spinOnce();
            rate.sleep();
        }

        ros::spinOnce();

        error = sqrt(pow(marker_offset[0], 2) + pow(marker_offset[1], 2));
    }
    controller.cmd_velocity(0, 0, 0);
    controller.land();
    state = TRACKING;

}

void RectangleExplorer::track() {
    const double transition_distance = 1; // [m]

    int last_id = -1;
    // euclidian distance to marker
    cv::Vec3f dir = marker_offset;
    double error = sqrt(pow(dir[0], 2) + pow(dir[1], 2));
    while (ros::ok() && error > transition_distance && state == TRACKING) {
        ROS_INFO("error %.2f -> %.2f %.2f %.2f", error, dir[0], dir[1], dir[2]);
        controller.move_relative(dir[0], dir[1], 0, 0, true);

        ros::spinOnce();
        if(last_id != marker_offset_id) {
            last_id = marker_offset_id;
            dir = marker_offset;
        }

        last_id = marker_offset_id;
        error = sqrt(pow(dir[0], 2) + pow(dir[1], 2));

        if(get_aruco_last_seen() > 2) {
            state = EXPLORATION;
            break;
        }
    }
    if(state == TRACKING) {
        state = LANDING;
    }
}

void RectangleExplorer::land_velocity() {

}

void RectangleExplorer::land() {
    const double decent_speed = 0.15;
    const double drop_height = 0.3;

    ros::spinOnce();
    int last_id = -1;

    while (ros::ok() ) {
        // if new marker since last movement...
        if(last_id != this->marker_offset_id) {
            double z_offset = -decent_speed;
            if(controller.get_z() + z_offset <= drop_height) {
                z_offset = decent_speed  - (controller.get_z() + z_offset);
            }
            controller.move_relative(marker_offset[0], marker_offset[1], z_offset, 0, true);
            last_id = this->marker_offset_id;
        } else if(get_aruco_last_seen() <= 5 && controller.get_z() <= hoverHeight) { // [s] && [m]
            controller.move_relative(0, 0, 0, 0, true);
            //controller.move_relative(0, 0, 0.10, 0, true);
        } else {
            ROS_INFO("EXPLORATION");
            state = EXPLORATION;
            break;
        }

        ros::spinOnce();

        if(controller.get_z() <= drop_height) {
            state = DONE;
            break;
        }

        /*if(get_aruco_last_seen() <= 1) {
            if(controller.get_z() <= 0.15 && error <= 0.05) {
                state = DONE;
                ROS_INFO("DONE");
                break;
            }
        }*/

        /*cv::Vec4d platform_velocity = ekf.get_velocity();
        ROS_INFO("error %.2f [m], v_x=%.2f ±%.2f [m/s], v_y=%.2f ±%.2f [m/s]", error, platform_velocity[0],
                 platform_velocity[2], platform_velocity[1], platform_velocity[3]);*/

    }

    controller.stop();

}

void RectangleExplorer::demo() {

    ros::Rate rate(60);
    while (ros::ok()) {
        //cv::Vec4d platform_velocity = ekf.get_velocity();
        //ROS_INFO("EKF -> v_x=%.2f (%.2f) [m/s], v_y=%.2f (%.2f) [m/s]", platform_velocity[0], platform_velocity[2], platform_velocity[1], platform_velocity[3]);
        ros::spinOnce();
        rate.sleep();
    }

    /*controller.arm_drone();
    controller.takeoff(0.5);

    ros::Rate rate(1);
    int i = 0;
    while (ros::ok() && i < 8) {
        controller.hover(0.5);
        i++;
        rate.sleep();
        ros::spinOnce();
    }

    controller.land();*/

}

void RectangleExplorer::get_relative_left_right(Direction lastDir, Direction& d1, Direction& d2){
    switch (lastDir) {
        case LEFT:
        case RIGHT: d1 = FORWARD;d2 = BACK;break;
        case FORWARD:
        case BACK: d1 = LEFT; d2 = RIGHT;break;
    }
}
Direction RectangleExplorer::negate_dir(Direction dir){
    switch (dir) {
        case LEFT: return RIGHT;
        case RIGHT: return LEFT;
        case FORWARD: return BACK;
        case BACK: return FORWARD;
        case UP: return DOWN;
        case DOWN: return UP;
    }
    ROS_WARN("UNKNOWN DIRECTION :: in negate_dir");
    return LEFT;
}

void RectangleExplorer::move_in_dir(Direction dir) {
    double d = controller.get_distance_measurement(dir);
    if(d < (minDist+waySize) && false){
        ROS_INFO("AT_WALL");
        controller.move_absolute(0, 0, controller.get_distance_measurement(Direction::DOWN), 0);
        controller.land();
        inFirstLoop = false;
        return;
    }
    controller.move_in_direction(dir,waySize);
    distMoved = distMoved + waySize;
}

void RectangleExplorer::_update_aruco_pose(const acanthis::ArucoPose::ConstPtr& pose) {
    if(this->state == EXPLORATION) {
        this->state = TRACKING;
        this->controller.cancel_movement();
    }

    this->aruco_pose = pose;
    marker_offset = cv::Vec3f(pose->position.x,
                              pose->position.y,
                              pose->position.z);
    this->marker_offset_id += 1;
    this->marker_last_seen = std::chrono::system_clock::now();
}

long RectangleExplorer::get_aruco_last_seen() {
    return std::chrono::duration_cast<std::chrono::seconds>((std::chrono::system_clock::now() - marker_last_seen))
            .count();
}

