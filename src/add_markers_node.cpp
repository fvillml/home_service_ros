#include "home_service/MarkerVisualizer.hpp"

using namespace home_service;

int main(int argc, char** argv) {
    ros::init(argc, argv, "add_markers_node");
    ros::NodeHandle nh {};

    MarkerVisualizer markerVisualizer { nh };
    // Wait for subscriber to subscribe
    ros::Duration(5.0).sleep();

    // Publish the marker at the pickup zone
    float x { 4.0 }, y { 0.0 };
    markerVisualizer.visualize(x, y);

    // Pause 5 seconds
    ros::Duration(5.0).sleep();

    // Hide the marker
    markerVisualizer.remove();

    // Pause 5 seconds
    ros::Duration(5.0).sleep();

    // Publish the marker at the drop off zone
    x = 4.0;
    y = 2.5;
    markerVisualizer.visualize(x, y);

    return 0;
}
