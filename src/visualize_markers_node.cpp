#include "home_service/MarkerVisualizer.hpp"

using namespace home_service;

int main(int argc, char** argv) {
    ros::init(argc, argv, "visualize_markers_node");
    ros::NodeHandle nh {};

    MarkerVisualizer markerVisualizer { nh };

    ros::spin();

    return 0;
}
