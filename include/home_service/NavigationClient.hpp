#pragma once

#include <mutex>

#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>

#include <move_base_msgs/MoveBaseAction.h>

#include <tf2/LinearMath/Quaternion.h>

#include "home_service/Goal.h"

namespace home_service {

/**
 * @brief Class charged of requesting new goals to the move_base node.
 *
 */
class NavigationClient {
public:
    /**
     * @brief Construct a new NavigationClient object.
     *
     * @param nodeHandler Node handler used to create a subcriber for goal_msgs.
     */
    NavigationClient(ros::NodeHandle& nodeHandler);

    ~NavigationClient() = default;

    /**
     * @brief Request the move_base to go to a position in the 2D plane
     * The move_base will accept the position and orientation of the goal based on the parameters
     * xy_goal_tolerance and yaw_goal_tolerance of the trajectory planner.
     *
     * @param x x coordinate of the new goal to reach.
     * @param y y coordinate of the new goal to reach.
     * @param yaw optional yaw for the goal position. Default value is zero.
     * @return true
     * @return false
     */
    bool navigate(const float& x, const float& y, const int& yaw = 0);

    /**
     * @brief Get the state of the last goTo request.
     *
     * @return a string with the state of the last request.
     */
    std::string getState() const;

private:
    /**
     * @brief Set the goal for the current request.
     *
     * @param x x coordinate of the new request.
     * @param y y coordinate of the new request.
     * @param yaw yaw angle of the new request. In degrees.
     */
    void setCurrentGoal(const float& x, const float& y, const int& yaw);

    void goalCallback(const home_service::Goal::ConstPtr& msg);

    /** Client type*/
    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

    /** Client used to send goal requests to the move_base server through a
     * SimpleAction Client */
    MoveBaseClient mClient;

    /** Current goal to reach */
    move_base_msgs::MoveBaseGoal mGoal;

    /** Indicates whether the service is ready and connected */
    bool mIsReady { false };

    ros::Subscriber mGoalSubscriber;

    std::mutex mNavigateMutex {};

    std::atomic_bool mIsNavigating { false };
};

} // namespace home_service