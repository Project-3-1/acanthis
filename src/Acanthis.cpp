#include "ros/ros.h"

#include "flightcontroller.h"
#include "rectangleExplorer.h"
#include "acanthis/ArucoPose.h"

using namespace std;

acanthis::ArucoPose pose;
static void update_marker_pos(const acanthis::ArucoPose& p) {
    pose = p;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "FlyRectangleDemo", ros::init_options::AnonymousName);
    ros::NodeHandle node("~");

    //FlightController controller(node, 100);
    //RectangleExplorer rectangleExplorer(controller);
    //
    //
    /*FlightController controller(node, 100);
    ros::Subscriber subscriber = node.subscribe("/acanthis/aruco_detector/pose", 1, update_marker_pos);
    while (subscriber.getNumPublishers() == 0) {
        ROS_INFO("wait for aruco pose");
    }
    ROS_INFO("pose arrived");

    while (ros::ok()) {
        ROS_INFO("x: %f", pose.position.x);
    }*/

    RectangleExplorer rectangleExplorer(node, 100);
    rectangleExplorer.demo();

    /*FlightController controller(node, 100);
    controller.arm_drone();
    controller.takeoff(1.2);
    controller.hover(2);
    controller.move_relative(2.5, 0, 0, 0);
    controller.land();*/

    /*FlightController controller(node, 100);
    controller.arm_drone();
    controller.takeoff(1.5);
    controller.move_relative(1.5, 0, 0, 0);
    controller.move_relative(0, 1.5, 0, 0);
    controller.move_relative(-1.5, 0, 0, 0);
    controller.move_relative(0, -1.5, 0, 0);
    controller.land();*/

    ros::spin();
    return 0;
}
