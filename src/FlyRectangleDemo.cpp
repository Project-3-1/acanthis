#include "ros/ros.h"

#include "flightcontroller.h"

using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "FlyRectangleDemo", ros::init_options::AnonymousName);
    ros::NodeHandle node("~");

    FlightController controller(node, 100);

    controller.arm_drone();
    controller.takeoff(0.3);

    controller.move_until_object(Direction::FORWARD, 1);
    /*controller.move_relative(1, 0, 0, 0);
    controller.move_relative(0, 0, 0, 90);
    controller.move_relative(0, 1, 0, 0);
    controller.move_relative(0, 0, 0, 90);
    controller.move_relative(-1, 0, 0, 0);
    controller.move_relative(0, 0, 0, 90);
    controller.move_relative(0, -1, 0, 0);
    controller.move_relative(0, 0, 0, 90);*/
    //controller.moveRelative(0.5, 0.5, 0.2, 90);
    controller.land();

    ros::spin();
    return 0;
}
