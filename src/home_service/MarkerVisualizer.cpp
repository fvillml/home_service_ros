#include "home_service/MarkerVisualizer.hpp"

using namespace home_service;

MarkerVisualizer::MarkerVisualizer(ros::NodeHandle& nodeHandler)
    : mShape(Shape::CUBE)
    , mMarker() {
    // Initialize publisher
    mMarkerPublisher = nodeHandler.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    // Initialize services
    mVisualizeService = nodeHandler.advertiseService("marker_service", &MarkerVisualizer::visualizeCb, this);

    // Set the marker type.
    mMarker.type = toMarker[mShape];

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the
    // header
    mMarker.pose.position.x = 0;
    mMarker.pose.position.y = 0;
    mMarker.pose.position.z = 0;
    mMarker.pose.orientation.x = 0.0;
    mMarker.pose.orientation.y = 0.0;
    mMarker.pose.orientation.z = 0.0;
    mMarker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    mMarker.scale.x = 0.5;
    mMarker.scale.y = 0.5;
    mMarker.scale.z = 0.5;

    // Set the color -- be sure to set alpha to something non-zero!
    mMarker.color.r = 0.97f;
    mMarker.color.g = 0.58f;
    mMarker.color.b = 0.03f;
    mMarker.color.a = 1.0;

    mMarker.lifetime = ros::Duration();

    // Wait for anyone to be subscribed
    if (!hasSubscriber()) {
        ROS_WARN("MarkerVisualizer started without subscriber, some markers messages could be lost.");
    }
}

void MarkerVisualizer::visualize(const float& x, const float& y) {
    if (!hasSubscriber()) {
        ROS_WARN("No subscriber detected.");
    }
    ROS_INFO("Going to publish marker at (%f, %f)", x, y);
    publishMarker(x, y, "map", "goal", 0, visualization_msgs::Marker::ADD);
}

void MarkerVisualizer::remove() {
    if (!hasSubscriber()) {
        ROS_WARN("No subscriber detected.");
    }
    ROS_INFO("Going to remove marker at (%f, %f)", mMarker.pose.position.x, mMarker.pose.position.y);
    publishMarker(mMarker.pose.position.x, mMarker.pose.position.y, "map", "goal", 0,
        visualization_msgs::Marker::DELETE);
}

bool MarkerVisualizer::hasSubscriber() {
    return mMarkerPublisher.getNumSubscribers() > 0;
}

void MarkerVisualizer::publishMarker(const float& x, const float& y, const std::string& frame_id,
    const std::string& ns, const int& id, const uint& action) {
    std::lock_guard<std::mutex> lk(mMarkerMutex);
    mMarker.header.frame_id = frame_id;
    mMarker.ns = ns;
    mMarker.id = id;
    mMarker.header.stamp = ros::Time::now();
    mMarker.action = action;
    mMarker.pose.position.x = x;
    mMarker.pose.position.y = y;
    mMarkerPublisher.publish(mMarker);
}

bool MarkerVisualizer::visualizeCb(
    home_service::Visualize::Request& req, home_service::Visualize::Response& res) {
    ROS_INFO("requested to visualize object : id=%d, ns=%s, f_id=%s, at x=%f, y=%f", req.id, req.ns.c_str(),
        req.frame_id.c_str(), req.x, req.y);
    if (!hasSubscriber()) {
        res.result = false;
        res.msg = "No subscriber detected.";
    } else {
        uint action = toAction(req.action);
        if (action == 99u) {
            res.result = false;
            res.msg = "Action not suported. Please select one from: add, modify, delete, delete_all.";
        } else {
            publishMarker(req.x, req.y, req.frame_id, req.ns, req.id, action);
            res.result = true;
            res.msg = "Done";
        }
    }
    ROS_INFO("sending back response: [%s] - %s", res.result ? "true" : "false", res.msg.c_str());
    return true;
}

uint MarkerVisualizer::toAction(const std::string& actionStr) {
    if (actionStr == "add") {
        return visualization_msgs::Marker::ADD;
    } else if (actionStr == "modify") {
        return visualization_msgs::Marker::MODIFY;
    } else if (actionStr == "delete") {
        return visualization_msgs::Marker::DELETE;
    } else if (actionStr == "delete_all") {
        return visualization_msgs::Marker::DELETEALL;
    } else {
        // to be changed by a custom enum but meh
        return 99u;
    }
}