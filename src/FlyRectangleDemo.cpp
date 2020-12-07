#include "ros/ros.h"

#include "flightcontroller.h"
#include "rectangleExplorer.h"

using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "FlyRectangleDemo", ros::init_options::AnonymousName);
    ros::NodeHandle node("~");

    FlightController controller(node, 100);
    RectangleExplorer rectangleExplorer(controller);
    rectangleExplorer.explore();
    /*FlightController controller(node, 100);

    controller.arm_drone();
    controller.takeoff(0.4);
    controller.hover(5);*/
    /*controller.move_relative(1, 0, 0, 0);
    controller.move_relative(0, 0, 0, 90);
    controller.move_relative(0, 1, 0, 0);
    controller.move_relative(0, 0, 0, 90);
    controller.move_relative(-1, 0, 0, 0);
    controller.move_relative(0, 0, 0, 90);
    controller.move_relative(0, -1, 0, 0);
    controller.move_relative(0, 0, 0, 90);*/
    //controller.land();

    ros::spin();
    return 0;
}
