#pragma once

#include <mutex>
#include <atomic>
#include <condition_variable>
#include <chrono>
#include <thread>

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>

#include "home_service/Visualize.h"
#include "home_service/Goal.h"

using namespace std::chrono_literals;

namespace home_service {

/**
 * @brief This class will trigger a sequence of pick up and drop off.
 * @TODO: this could be improved by listening a home_service which will indicate the pick up point and the
 * drop off point.
 */
class GoalSupervisor {
public:
    /**
     * @brief Construct a new NavigationClient object.
     *
     * @param nodeHandler Node handler used to create a subcriber for goal_msgs.
     */
    GoalSupervisor(ros::NodeHandle& nodeHandler);

    ~GoalSupervisor();

    /**
     * @brief Starts the pickup-dropoff thread.
     */
    void start();

private:
    /**
     * @brief Executes the actuall pickup-dropoff sequence.
     * 
     * @return true if successfully picked up and droped off the marker. False otherwise.
     */
    bool execute();

    /**
     * @brief Request an action to the MarkerVisualizer.
     * 
     * @param action String indicating what to do with the marker. Either "add", "delete", "modify" or "delete_all".
     * @return true is successfully visualized the marker. False otherwise.
     */
    bool requestVisualization(const std::string& action);

    /**
     * @brief Publish the current goal to the goal topic.
     * 
     */
    void publishGoal();

    /**
     * @brief Callback for Odometry. Monitors if the robot has arrived to the current goal.
     * 
     * @param msg Odometry message.
     */
    void odomCallback(const nav_msgs::OdometryPtr& msg);

    /** Service client for marker_service */
    ros::ServiceClient mMarkersServiceClient;

    /** ROS publisher of goal messages */
    ros::Publisher mGoalPublisher;

    /** ROS subscriber of odometry messages*/
    ros::Subscriber mOdomSubscriber;

    /** Current goal x coordinate */
    float mGoalX { 0.0f };

    /** Current goal y coordinate */
    float mGoalY { 0.0f };

    /** Conditiona variable used to notify when odometry has reached the goal */
    std::condition_variable mCv {};

    /** Mutex to protect the condition variable */
    std::mutex mMutex {};

    /** Indicates whether the robot is on the goal or not */
    std::atomic_bool mArrived { false };

    /** main thread running the pickup-dropoff sequence*/
    std::thread mMainThread;

    /** Tolerance in x and y to accept that robot's on the goal */
    const float mPosTolerance { 0.2f };
};

} // namespace home_service