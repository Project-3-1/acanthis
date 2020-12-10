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
        ROS_INFO("wait for aruco pose");
        rate.sleep();
    }
    ROS_INFO("pose arrived");
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
    Direction closest = FORWARD;//controller.get_closest_direction(directions);
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
            controller.move_relative(.0,.0,.0,angle);
        }
    }else{
        if(dist2 < 1){
            double angle = (atan(dist2/minDist))*RAD_TO_DEG;
            ROS_INFO("Angle2%f",angle);
            controller.move_relative(.0,.0,.0,angle);
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
    ROS_WARN("MARKER DETECTED!!!!!!");

    // euclidian distance to marker
    double error = sqrt(pow(marker_x, 2) + pow( marker_y, 2));
    while (ros::ok() && state == TRACKING) {
        ROS_INFO("Tracking mode %.2f", error);
        if(error > 0.05) { //10[cm]
            controller.move_relative(marker_x, marker_y, 0,  0);
            error = sqrt(pow(marker_x, 2) + pow( marker_y, 2));
        } else {
            state = DONE;
            controller.land();
            break;
        }

        ros::spinOnce();
    }
}

void RectangleExplorer::demo() {

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
        if (ros::ok()) {
            double error = sqrt(pow(marker_x, 2) + pow(marker_y, 2));
            while (error > 0.05) {
                controller.move_relative(marker_x, marker_y, 0, 0);
                ros::spinOnce();
                error = sqrt(pow(marker_x, 2) + pow(marker_y, 2));
                ROS_INFO("Target area error %.2fm", error);
            }
        }
    }

    if (ros::ok()) {
        controller.land();
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
        this->controller.cancel_movement();
    }
    if(this->state == TRACKING) {
        this->aruco_pose = pose;
        marker_x = -pose->position.x;
        marker_y = -pose->position.y;
    }
}
