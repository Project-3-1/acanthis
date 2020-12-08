#include "rectangleExplorer.h"

#include <utility>
#include "flightcontroller.h"
#include "ros/node_handle.h"
#include "math.h"

RectangleExplorer::RectangleExplorer(FlightController& controller)
        : controller(controller) {
    this->hoverHeight = 0.35;
    this->minDist = 0.4;
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
            double angle = atan(dist1/minDist)*RAD_TO_DEG;
            ROS_INFO("Angle1%f",angle);
            controller.move_relative(.0,.0,.0,angle);
        }
    }else{
        if(dist2 < 1){
            double angle = atan(dist2/minDist)*RAD_TO_DEG;
            ROS_INFO("Angle2%f",angle);
            controller.move_relative(.0,.0,.0,angle);
        }
    }
    controller.hover(3);
    controller.land();
    return;
    Direction goTo = negate_dir(closest);
    // Do Exploration
    while (ros::ok() && inFirstLoop){
        controller.move_until_object(dir1,minDist);
        move_in_dir(goTo);
        if(!inFirstLoop){
            return;
        }
        controller.move_until_object(dir2,minDist);
        move_in_dir(goTo);
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