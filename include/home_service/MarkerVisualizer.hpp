#pragma once

#include <mutex>

#include <ros/ros.h>

#include <visualization_msgs/Marker.h>

#include <home_service/Visualize.h>

namespace home_service {

/**
 * @brief This class is charged of publishing markers to a visualization_marker as long as there is at least
 * one subscriber to this topic.
 */
class MarkerVisualizer {
public:
    /** Custom enum for shapes of the marker*/
    enum class Shape { CUBE, SPHERE, ARROW, CYLINDER };

    /**
     * @brief Construct a new MarkerVisualizer object.
     *
     * @param nodeHandler NodeHandle used to create the Publisher and the ServiceServer.
     */
    MarkerVisualizer(ros::NodeHandle& nodeHandler);

    ~MarkerVisualizer() = default;

    /**
     * @brief Adds a predefined marker to a position.
     *
     * @param x x coordinate of the position of the marker.
     * @param y y coordinate of the position of the marker.
     */
    void visualize(const float& x, const float& y);

    /**
     * @brief Deletes last predefined marker.
     */
    void remove();

private:
    /**
     * @brief Indicates whether the marker Publisher has a subscriber or not.
     *
     * @return true if the marker Publisher has a subscriber, false otherwise.
     */
    bool hasSubscriber();

    /**
     * @brief Function used to publish a visualization_msgs::Marker message to the visualization_marker topic.
     *
     * @param x x coordinate of the position of the marker.
     * @param y y coordinate of the position of the marker.
     * @param frame_id frame id of the marker.
     * @param ns namespace of the marker.
     * @param id id of the marker.
     * @param action Action indicating what to do with the marker. Either ADD, DELETE, MODIFY or DELETEALL.
     */
    void publishMarker(const float& x, const float& y, const std::string& frame_id, const std::string& ns,
        const int& id, const uint& action);

    /**
     * @brief Callback for the marker_service.
     *
     * @param req Service Request.
     * @param res Service Response.
     * @return true always true.
     */
    bool visualizeCb(home_service::Visualize::Request& req, home_service::Visualize::Response& res);

    /**
     * @brief From string to an uint representing the action.
     *
     * @param actionStr String representation of the action.
     * @return an uint with the action. Either ADD, DELETE, MODIFY or DELETEALL. 99 is returned if no
     * convertion found.
     */
    uint toAction(const std::string& actionStr);

    /** ROS Publisher */
    ros::Publisher mMarkerPublisher;

    /** ROS Service Server*/
    ros::ServiceServer mVisualizeService;

    /** Message to be published into the visualization_marker topic */
    visualization_msgs::Marker mMarker;

    /** Enum object containing the current shape of the marker*/
    Shape mShape;

    /** Map helping to convert from Shape to visualization_msgs shape */
    std::map<Shape, uint> toMarker { { Shape::CUBE, visualization_msgs::Marker::CUBE },
        { Shape::SPHERE, visualization_msgs::Marker::SPHERE },
        { Shape::ARROW, visualization_msgs::Marker::ARROW },
        { Shape::CYLINDER, visualization_msgs::Marker::CYLINDER } };

    /** Mutex to protect the message and the Publisher */
    std::mutex mMarkerMutex {};
};

} // namespace home_service