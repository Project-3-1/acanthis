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
        ROS_INFO("wait for aruco pose");
        rate.sleep();
    }
    ROS_INFO("pose arrived");
    // ---

    this->hoverHeight = 0.35;
    this->minDist = 0.3;
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
void RectangleExplorer::turn_at_wall(Direction direction) {

}

void RectangleExplorer::explore() {
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
    Direction goTo = negate_dir(closest);
    // Do Exploration
    while (ros::ok() && inFirstLoop && state == EXPLORATION){
        controller.move_until_object(dir1,minDist);
        if(state == TRACKING) {
            break;
        }
        move_in_dir(goTo);
        if(state == TRACKING) {
            break;
        }
        if(!inFirstLoop){
            return;
        }
        controller.move_until_object(dir2,minDist);
        if(state == TRACKING) {
            break;
        }
        move_in_dir(goTo);
        if(state == TRACKING) {
            break;
        }
        ros::spinOnce();
    }
    ROS_WARN("MARKER DETECTED!!!!!!");

    // euclidian distance to marker
    double error = sqrt(pow(controller.get_x() - marker_x, 2)
            + pow(controller.get_y() - marker_y, 2));
    while (ros::ok() && state == TRACKING) {
        ROS_INFO("Tracking mode %.2f", error);
        if(error > 0.1) { //10[cm]
            controller.move_absolute(marker_x, marker_y, controller.get_z(),  0);
            error = sqrt(pow(controller.get_x() - marker_x, 2)
                         + pow(controller.get_y() - marker_y, 2));
        } else {
            state = DONE;
            controller.land();
            break;
        }

        ros::spinOnce();
    }
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
    if(d < (minDist+waySize)){
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
    ROS_WARN("update pose");
    if(this->state == EXPLORATION) {
        this->state = TRACKING;
    }
    if(this->state == TRACKING) {
        this->aruco_pose = pose;
        marker_x = aruco_pose->position.x + pose->position.x;
        marker_y = aruco_pose->position.y + pose->position.y;
    }
}
