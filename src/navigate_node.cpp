#include "home_service/NavigationClient.hpp"

using namespace home_service;

int main(int argc, char **argv) {
    ros::init(argc, argv, "navigate_node");

    ros::NodeHandle nh {};
    NavigationClient navigationClient { nh };

    ros::spin();

    return 0;
}
