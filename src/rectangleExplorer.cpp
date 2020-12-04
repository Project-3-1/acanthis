#include "rectangleExplorer.h"
#include "flightcontroller.h"
#include "ros/node_handle.h"

ros::NodeHandle node;

RectangleExplorer::RectangleExplorer(ros::NodeHandle n, double freqency) {
    node = n;
    this->hoverHeight = 0.5;
    this->minDist = 0.3;
    this->waySize = 0.3;
    this->freqency = freqency;
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
    FlightController controller(node, freqency);
    // Start and hover at height
    controller.arm_drone();
    controller.takeoff(hoverHeight);
    // Find Closest Wall
    Direction directions[] {LEFT,RIGHT,FORWARD,BACK};
    Direction closest = controller.get_closest_direction(directions);
    //Move To It
    controller.move_until_object(closest,minDist);
    //Get Directions between which we move
    Direction dir1 = LEFT;
    Direction dir2 = RIGHT;
    get_relative_left_right(closest,dir1,dir2);
    Direction goTo = negate_dir(closest);
    // Do Exploration
    while (ros::ok()){
        controller.move_until_object(dir1,minDist);
        controller.move_in_direction(goTo,waySize);
        controller.move_until_object(dir2,minDist);
        controller.move_in_direction(goTo,waySize);
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