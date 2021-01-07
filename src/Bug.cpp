#include "Bug.h"
#include "flightcontroller.h"
#include "ros/node_handle.h"
#include "math.h"

#include "acanthis/ArucoPose.h"

Bug::Bug(ros::NodeHandle& node, double frequency)
    : controller(FlightController(node, frequency)), node(node) {

    // ---
    ros::Rate rate(2);
    this->aruco_pose_sub = node.subscribe("/acanthis/aruco_detector/pose", 1, &Bug::_update_aruco_pose, this);
    while (this->aruco_pose_sub.getNumPublishers() == 0) {
        ROS_INFO("wait for aruco pose");
        rate.sleep();
    }
    ROS_INFO("pose arrived");
    // ---

    this->hoverHeight = 0.4;
    this->minDist = 0.2;
    this->waySize = 0.5;
    this->distMoved = 0;
    this->inFirstLoop = true;
}
bool Bug::is_close(double distance){
    double min = 0.3;
    return (distance < min);
}
bool Bug::is_in_Distance(double distance) {
    double min = 0.75;
    return (distance < min);
}
void Bug::turn_at_wall(Direction direction) {

}

void Bug::explore() {
    // Start and hover at height
    controller.arm_drone();
    ROS_INFO("takeoff");
    controller.takeoff(hoverHeight);
    ROS_INFO("takeoff done");
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

    avoid();

    ROS_WARN("MARKER DETECTED!!!!!!");

    if(state == TRACKING) {
        // euclidian distance to marker
        double error = sqrt(pow(marker_offset_x, 2) + pow(marker_offset_y, 2));
        while (ros::ok() && error > 0.1) {
            ROS_INFO("Tracking mode %.2f", error);
            controller.move_relative(marker_offset_x, marker_offset_y, 0, 0, false);
            error = sqrt(pow(marker_offset_x, 2) + pow(marker_offset_y, 2));
            ros::spinOnce();
        }

        // now we do translation for x,y (5 [cm]), and z until it is less than 15 [cm]
        // TODO An issue with this might be that we can no longer do the x,y error correction if we are already too close
        //  to the marker, and we can no longer see it.
        //   a) We can either try to make again the marker with a marker inside, or
        //   b) for now, we can just figure out what height and x,y error is acceptable before we are no longer able to see it, or
        //   c) we can try to tell the Aruco detector to start looking for a 2x2 marker instead of a 3x3 marker if that is only
        //      visible, because then we don't need to do any weird filtering for the marker in marker thing,
        while (ros::ok() && (error > 0.05 && controller.get_z() < 0.15)) {
            cv::Vec4d platform_velocity = ekf.get_velocity();
            double height = marker_offset_z + 0.15;
            controller.move_relative(marker_offset_x, marker_offset_y, height, 0, false);
            ros::spinOnce();
            error = sqrt(pow(marker_offset_x, 2) + pow(marker_offset_y, 2));
            ROS_INFO("error %.2f [m], v_x=%.2f ±%.2f [m/s], v_y=%.2f ±%.2f [m/s]", error, platform_velocity[0],
                     platform_velocity[2], platform_velocity[1], platform_velocity[3]);
        }

        controller.stop();
    }


}

void Bug::avoid(){
    auto step = 0.05;
    auto x = controller.get_x();
    auto y = controller.get_y();
    Direction directions[] {LEFT,RIGHT,FORWARD,BACK};
    Direction closest = controller.get_closest_direction(directions);
    ROS_INFO(" closest", closest);
    if(closest == 3 || closest == 4){
        controller.move_relative(.0,.0,.0,90);
    }
    closest = controller.get_closest_direction(directions);
    ROS_INFO(" #2 closest ", directions[closest]);
    while( ros::ok() && state == EXPLORATION /*|| (x!=controller.get_x() && y!=controller.get_y()*/){
        if(state == TRACKING) break;
        // TODO target check
        while(closest == controller.get_closest_direction(directions)){
            if(state == TRACKING) break;
            controller.move_in_direction(FORWARD,step);
            if(controller.get_distance_measurement(closest) > minDist){
                controller.move_in_direction(closest,step);
            }
        }
        if(closest != controller.get_closest_direction(directions)){
            auto dir = 1;
            if(closest == 2){
                dir = -1;
            }
            controller.move_relative(.0,.0,.0,dir*90);
            closest = controller.get_closest_direction(directions);
        }

        ros::spinOnce();
    }
}

void Bug::demo() {

    controller.arm_drone();
    controller.takeoff(0.5);

    while (ros::ok() && state == EXPLORATION) {
        if(controller.get_distance_measurement(FORWARD) < 0.2) {
            controller.land();
            ROS_WARN("land because wall");
            return;
        }
        controller.move_relative(0.1, 0, 0, 0);
        ros::spinOnce();
    }

    if(state == TRACKING) {
        double error = sqrt(pow(marker_offset_x, 2) + pow(marker_offset_y, 2));

        // first we do some crude translation correction for x and y until the error is less than 20 [cm]
        while (ros::ok() && error > 0.2) {
            controller.move_relative(marker_offset_x, marker_offset_y, 0, 0);
            ros::spinOnce();
            error = sqrt(pow(marker_offset_x, 2) + pow(marker_offset_y, 2));
            ROS_INFO("Target area error %.2fm", error);
        }


    }

    if (ros::ok()) {
        controller.land();
    }
}

void Bug::get_relative_left_right(Direction lastDir, Direction& d1, Direction& d2){
    switch (lastDir) {
        case LEFT:
        case RIGHT: d1 = FORWARD;d2 = BACK;break;
        case FORWARD:
        case BACK: d1 = LEFT; d2 = RIGHT;break;
    }
}
Direction Bug::negate_dir(Direction dir){
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

void Bug::move_in_dir(Direction dir) {
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

void Bug::_update_aruco_pose(const acanthis::ArucoPose::ConstPtr& pose) {
    ROS_INFO("marker detected");
    if(this->state == EXPLORATION) {
        this->state = TRACKING;
        this->controller.cancel_movement();
    }
    if(this->state == TRACKING) {
        this->aruco_pose = pose;
        marker_offset_x = -pose->position.x;
        marker_offset_y = -pose->position.y;
        marker_offset_z = -pose->position.z;

        // --- update ekf
        if(this->ekf.get_last_seen() >= 10) {
            this->ekf.reset();
        }

        this->ekf.update(controller.get_x() + pose->position.x, controller.get_y() + pose->position.y);
    }
}
