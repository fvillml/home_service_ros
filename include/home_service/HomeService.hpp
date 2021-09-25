#pragma once

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2/LinearMath/Quaternion.h>

namespace home_service {

/**
 * @brief Class charged of requesting new goals to the move_base node.
 * 
 */
class HomeService {
public:
    /**
     * @brief Construct a new HomeService object.
     */
    HomeService();
    ~HomeService() = default;

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
    bool goTo(const float& x, const float& y, const int& yaw = 0);

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

    /** Client type*/
    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

    /** Client used to send goal requests to the move_base server through a
     * SimpleAction Client */
    MoveBaseClient mClient;

    /** Current goal to reach */
    move_base_msgs::MoveBaseGoal mGoal;

    /** Indicates whether the service is ready and connected */
    bool mIsReady { false };
};

} // namespace home_service