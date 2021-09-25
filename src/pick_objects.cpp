#include "home_service/HomeService.hpp"

using namespace home_service;

int main(int argc, char **argv) {
    ros::init(argc, argv, "pick_objects_node");

    HomeService homeService {};

    float x { 4.0 }, y { 0.0 };
    ROS_INFO("Robot is travelling to the pick up zone");
    if (homeService.goTo(x, y)) {
        ROS_INFO("Robot picked up the virtual object");
    } else {
        ROS_INFO("The robot failed to pick up the virtual object");
        ROS_INFO("%s", homeService.getState().c_str());
        return -1;
    }

    ros::Duration(5.0).sleep();

    x = 4.0;
    y = 2.5;
    int yaw { 180 };
    ROS_INFO("Robot is travelling to the drop off zone");
    if (homeService.goTo(x, y, yaw)) {
        ROS_INFO("Robot dropped the virtual object");
    } else {
        ROS_INFO("The robot failed to drop the virtual object");
        ROS_INFO("%s", homeService.getState().c_str());
        return -1;
    }

    return 0;
}
