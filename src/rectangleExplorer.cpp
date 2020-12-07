#include "rectangleExplorer.h"

#include <utility>
#include "flightcontroller.h"
#include "ros/node_handle.h"

RectangleExplorer::RectangleExplorer(FlightController& controller)
        : controller(controller) {
    this->hoverHeight = 0.35;
    this->minDist = 0.2;
    this->waySize = 0.3;
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
    while (ros::ok() && inFirstLoop){
        controller.move_until_object(dir1,minDist);
        move_in_dir(goTo);
        controller.move_until_object(dir2,minDist);
        move_in_dir(goTo);
    }
    controller.land();
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
        controller.move_in_direction(negate_dir(dir),distMoved);
        inFirstLoop = false;
        return;
    }
    controller.move_in_direction(dir,waySize);
    distMoved = distMoved + waySize;
}