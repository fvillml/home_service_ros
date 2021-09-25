#!/bin/bash
# Roscore
xterm  -e  " source ~/ROS/home_service_ws/devel/setup.bash; roscore" & 
sleep 5

# Robot simulation
xterm  -e " roslaunch diff_robot_gazebo diff_robot.launch " &
sleep 5

# amcl demo launch
xterm -e " roslaunch home_service navigation.launch " &
sleep 5

# View Navigation
xterm -e " rosrun rviz rviz -d `rospack find diff_robot_description`/rviz/diff_robot_nav.rviz" &
sleep 5
