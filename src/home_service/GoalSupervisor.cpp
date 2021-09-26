#include "home_service/GoalSupervisor.hpp"

using namespace home_service;

GoalSupervisor::GoalSupervisor(ros::NodeHandle& nodeHandler) {
    mMarkersServiceClient = nodeHandler.serviceClient<home_service::Visualize>("marker_service");
    mGoalPublisher = nodeHandler.advertise<home_service::Goal>("goal", 1000);
    mOdomSubscriber = nodeHandler.subscribe("odom", 1, &GoalSupervisor::odomCallback, this);
    while (mGoalPublisher.getNumSubscribers() < 1) {
        if (!ros::ok()) {
            return;
        }
        ROS_WARN_ONCE("Waiting for /goal publisher to have a subscriber");
        sleep(1);
    }
    ROS_INFO("Subscriber found");
}

GoalSupervisor::~GoalSupervisor() {
    mMainThread.join();
}

void GoalSupervisor::start() {
    mMainThread = std::thread { [this]() { execute(); } };
}

bool GoalSupervisor::execute() {
    // Visualize marker
    mGoalX = 4.0f;
    mGoalY = 0.0f;
    if (!requestVisualization("add")) {
        return false;
    }

    // Publish a new goal
    ROS_INFO("Robot is travelling to the pick up zone");
    publishGoal();

    // Wait for the robot to arrive to the goal
    {
        std::unique_lock<std::mutex> lk(mMutex);
        mArrived = false;
        mCv.wait_for(lk, 60s, [this]() { return mArrived.load() == true; });
    }
    if (!mArrived.load()) {
        ROS_ERROR("Timeout ! Robot didn't manage to pick up the virtual object");
        return false;
    }

    // Remove marker
    ROS_INFO("Robot picked up the virtual object");
    if (!requestVisualization("delete")) {
        return false;
    }

    // Wait 5 seconds
    ros::Duration(5.0).sleep();

    // Publish new goal
    mGoalX = 4.0f;
    mGoalY = 2.5f;
    ROS_INFO("Robot is travelling to the drop off zone");
    publishGoal();

    // wait for the robot to arrive to the goal
    {
        std::unique_lock<std::mutex> lk(mMutex);
        mArrived = false;
        mCv.wait_for(lk, 60s, [this]() { return mArrived.load() == true; });
    }
    if (!mArrived.load()) {
        ROS_ERROR("Timeout ! Robot didn't manage to drop the virtual object");
        return false;
    }

    // visualize marker
    ROS_INFO("Robot dropped the virtual object");
    if (!requestVisualization("add")) {
        return false;
    }
    return true;
}

bool GoalSupervisor::requestVisualization(const std::string& action) {
    home_service::Visualize srv {};
    srv.request.frame_id = "map";
    srv.request.ns = "goal";
    srv.request.id = 0;
    srv.request.action = action;
    srv.request.x = mGoalX;
    srv.request.y = mGoalY;

    if (mMarkersServiceClient.call(srv)) {
        if (srv.response.result) {
            return true;
        } else {
            ROS_ERROR("Failed to publish marker: %s", srv.response.msg.c_str());
            return false;
        }
    } else {
        ROS_ERROR("Failed to call service marker_service");
        return false;
    }
}

void GoalSupervisor::publishGoal() {
    home_service::Goal goal {};
    goal.x = mGoalX;
    goal.y = mGoalY;
    mGoalPublisher.publish(goal);
}

void GoalSupervisor::odomCallback(const nav_msgs::OdometryPtr& msg) {
    float currentX = msg->pose.pose.position.x;
    float currentY = msg->pose.pose.position.y;
    if ((std::fabs(currentX - mGoalX) < mPosTolerance) && (std::fabs(currentY - mGoalY) < mPosTolerance)) {
        mArrived = true;
        mCv.notify_one();
    }
}
