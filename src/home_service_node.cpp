
// Rename GoalPublisher for MarkerVisualizer. Make it listen to two services called (visualize and remove).

// Create a class called GoalSupervisor. This class will request a visualize to MarkerVisualizer, publish a new goal to a "/goal" topic,  listen to odom, wait for 
//      the robot to arrive to the desired position, request a remove the marker, wait 5 seconds and define a new goal.
// Create a new node called home_service_node running the GoalSupervisor.