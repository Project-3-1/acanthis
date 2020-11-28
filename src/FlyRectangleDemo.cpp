#include "ros/ros.h"
#include "crazyflie_driver/Takeoff.h"
#include "crazyflie_driver/UpdateParams.h"
#include "crazyflie_driver/Hover.h"
#include "crazyflie_driver/Position.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Point.h"

#include "flightcontroller.h"

using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "FlyRectangleDemo", ros::init_options::AnonymousName);
    ros::NodeHandle node("~");

    FlightController controller(node, 10);

    controller.arm_drone();
    controller.takeoff(.5);
    controller.move(0.1, 0.1, 0, 0);
    controller.land();
    ros::spin();
    return 0;
}
