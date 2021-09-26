#include <thread>
#include "home_service/GoalSupervisor.hpp"

using namespace home_service;

int main(int argc, char **argv) {
    ros::init(argc, argv, "home_service_node");
    ros::NodeHandle nh {};
    GoalSupervisor goalSupervisor { nh };

    goalSupervisor.start();

    ros::spin();

    return 0;
}
