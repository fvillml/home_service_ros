#include "home_service/NavigationClient.hpp"

using namespace home_service;

NavigationClient::NavigationClient(ros::NodeHandle& nodeHandler)
    : mClient("move_base", true)
    , mGoal() {
    // Wait 5 sec for move_base action server to come up
    ROS_INFO("Waiting for the move_base action server to come up");
    mIsReady = mClient.waitForServer(ros::Duration(5.0));
    ROS_INFO("Server Ok.");

    // set up the frame parameters
    mGoal.target_pose.header.frame_id = "map";

    // Start subscriber
    mGoalSubscriber = nodeHandler.subscribe("goal", 1, &NavigationClient::goalCallback, this);
}

bool NavigationClient::navigate(const float& x, const float& y, const int& yaw) {
    std::lock_guard<std::mutex> lx(mNavigateMutex);
    if (!mIsReady) {
        return false;
    }
    mIsNavigating = true;

    // Set the goal
    setCurrentGoal(x, y, yaw);

    // Send the goal position and orientation for the robot to reach
    mClient.sendGoal(mGoal);

    // Wait an infinite time for the results
    mClient.waitForResult();
    mIsNavigating = false;

    // Check if the robot reached its goal
    return mClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
}

std::string NavigationClient::getState() const {
    return mClient.getState().getText();
}

void NavigationClient::setCurrentGoal(const float& x, const float& y, const int& yaw) {
    // Create this quaternion from roll/pitch/yaw (in radians)
    tf2::Quaternion orientation {};
    orientation.setRPY(0, 0, (yaw * 3.14f) / (180.0f));
    orientation.normalize();
    // Define a position and orientation for the robot to reach
    mGoal.target_pose.header.stamp = ros::Time::now();
    mGoal.target_pose.pose.position.x = x;
    mGoal.target_pose.pose.position.y = y;
    mGoal.target_pose.pose.orientation.x = orientation[0];
    mGoal.target_pose.pose.orientation.y = orientation[1];
    mGoal.target_pose.pose.orientation.z = orientation[2];
    mGoal.target_pose.pose.orientation.w = orientation[3];
}

void NavigationClient::goalCallback(const home_service::Goal::ConstPtr& msg) {
    ROS_INFO("Requested to navigate to (%f, %f)", msg->x, msg->y);
    if (mIsNavigating.load()) {
        ROS_INFO("Robot is already navigating. Waiting...");
    }
    navigate(msg->x, msg->y);
}