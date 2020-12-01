#include "ros/ros.h"

#include "flightcontroller.h"

using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "FlyRectangleDemo", ros::init_options::AnonymousName);
    ros::NodeHandle node("~");

    FlightController controller(node, 10);

    //controller.arm_drone();
    ROS_WARN("takeooff!!!");
    controller.takeoff(0.5);
    ROS_WARN("takeoff done!!!");
    ROS_WARN("move!!!");
    //controller.move(0.5, 0, 0, 0);
    ROS_WARN("move done!!!");
    controller.moveRelative(0, 1, 0, 0);
    controller.moveRelative(1, 0, 0, 0);
    controller.moveRelative(0, -1, 0, 0);
    controller.moveRelative(-1, 0, 0, 0);
    ROS_WARN("HOVER Done!");
    ROS_WARN("LANDING REQUESTED!!!");
    controller.land();
    ROS_WARN("LANDING done!!!");

    ros::spin();
    return 0;
}
