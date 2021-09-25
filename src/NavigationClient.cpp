#include "home_service/NavigationClient.hpp"

using namespace home_service;

NavigationClient::NavigationClient()
    : mClient("move_base", true)
    , mGoal() {
    // Wait 5 sec for move_base action server to come up
    ROS_INFO("Waiting for the move_base action server to come up");
    mIsReady = mClient.waitForServer(ros::Duration(5.0));

    // set up the frame parameters
    mGoal.target_pose.header.frame_id = "map";
}

bool NavigationClient::navigate(const float& x, const float& y, const int& yaw) {
    if (!mIsReady) {
        return false;
    }

    // Set the goal
    setCurrentGoal(x, y, yaw);

    // Send the goal position and orientation for the robot to reach
    mClient.sendGoal(mGoal);

    // Wait an infinite time for the results
    mClient.waitForResult();

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