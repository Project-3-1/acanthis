#include "rectangleExplorer.h"

#include "flightcontroller.h"
#include "ros/node_handle.h"
#include "math.h"

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

    this->hoverHeight = 0.5;
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
                continue;
        }
    }
}

void RectangleExplorer::explore() {
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

void RectangleExplorer::track() {
    const double transition_distance = 0.05; // [m]

    int last_id = -1;
    // euclidian distance to marker
    cv::Vec3f dir = marker_offset;
    double error = sqrt(pow(dir[0], 2) + pow(dir[1], 2));
    while (ros::ok() && error > transition_distance && state == TRACKING) {
        ROS_INFO("error %.2f -> %.2f %.2f %.2f", error, dir[0], dir[1], dir[2]);
        controller.move_relative(dir[0], dir[1], 0, 0, false);

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

void RectangleExplorer::land() {
    // now we do translation for x,y (5 [cm]), and z until it is less than 15 [cm]
    // TODO An issue with this might be that we can no longer do the x,y error ECo any weird filtering for the marker in marker thing,

    ros::spinOnce();
    double error = sqrt(pow(marker_offset[0], 2) + pow(marker_offset[1], 2));
    int last_id = -1;

    while (ros::ok() && error > 0.05 ) {
        error = sqrt(pow(marker_offset[0], 2) + pow(marker_offset[1], 2));
        // if new marker since last movement...
        if(last_id != this->marker_offset_id) {
            double z_offset = -0.05;
            if(controller.get_z() + z_offset <= 0.15) {
                z_offset = 0.15 - (controller.get_z() + z_offset);
            }
            if(z_offset < -.1) {
                z_offset = -.1;
            }
            controller.move_relative(marker_offset[0], marker_offset[1], z_offset, 0, false);
            last_id = this->marker_offset_id;
        } else if(get_aruco_last_seen() < 2 && controller.get_z() <= 1.8) { // [s] && [m]
            ROS_ERROR("what");
            controller.hover(1);
            //controller.move_relative(0, 0, 0.10, 0, true);
        } else {
            ROS_INFO("EXPLORATION");
            state = EXPLORATION;
            break;
        }

        ros::spinOnce();

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

    ros::Rate rate(2);
    while (ros::ok()) {
        ROS_INFO("    %.2f %.2f %.2f", controller.get_x(), controller.get_y(), controller.get_z());
        if(marker_offset_id != -1) {
            ROS_INFO("+   %.2f %.2f %.2f", marker_offset[0], marker_offset[1], marker_offset[2]);
        }
        ros::spinOnce();
        rate.sleep();
    }

    /*controller.arm_drone();
    controller.takeoff(0.5);

    int i = 30;
    while (ros::ok() && i> 0) {
        controller.hover(1);
        cv::Vec4d platform_velocity = ekf.get_velocity();
        ROS_INFO("EKF -> v_x=%.2f (%.2f) [m/s], v_y=%.2f (%.2f) [m/s]", platform_velocity[0],
                 platform_velocity[2], platform_velocity[1], platform_velocity[3]);
        ros::spinOnce();
        i--;
    }

    if (ros::ok()) {
        controller.land();
    }*/
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
                              -pose->position.z);
    this->marker_offset_id += 1;
    this->marker_last_seen = std::chrono::system_clock::now();

    // --- update ekf
    /*if(this->ekf.get_last_seen() >= 10) {
        ROS_WARN("Reset EKF because the marker was out of sight for >= 10 [s]");
        this->ekf.reset();
    }
    this->ekf.update(controller.get_x() + pose->position.x, controller.get_y() + pose->position.y);*/
}

long RectangleExplorer::get_aruco_last_seen() {
    return std::chrono::duration_cast<std::chrono::seconds>((std::chrono::system_clock::now() - marker_last_seen))
            .count();
}
