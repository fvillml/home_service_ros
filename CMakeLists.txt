cmake_minimum_required(VERSION 3.0.2)
project(home_service)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  move_base_msgs
  actionlib
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Goal.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  Visualize.srv
)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs 
)

## The catkin_package macro generates cmake config files for your package
catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS roscpp message_runtime
)

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare the home_servic library
add_library(${PROJECT_NAME}_lib
  src/${PROJECT_NAME}/NavigationClient.cpp
  src/${PROJECT_NAME}/MarkerVisualizer.cpp
  src/${PROJECT_NAME}/GoalSupervisor.cpp
)
add_dependencies(${PROJECT_NAME}_lib home_service_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_lib ${catkin_LIBRARIES})

## Add the pick_objects node
add_executable(pick_objects_node src/pick_objects_node.cpp)
add_dependencies(pick_objects_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pick_objects_node ${catkin_LIBRARIES} ${PROJECT_NAME}_lib)

## Add the add_markers node
add_executable(add_markers_node src/add_markers_node.cpp)
add_dependencies(add_markers_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(add_markers_node ${catkin_LIBRARIES} ${PROJECT_NAME}_lib)

## Add the navigate node
add_executable(navigate_node src/navigate_node.cpp)
add_dependencies(navigate_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(navigate_node ${catkin_LIBRARIES} ${PROJECT_NAME}_lib)

## Add the visualize_markers node
add_executable(visualize_markers_node src/visualize_markers_node.cpp)
add_dependencies(visualize_markers_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(visualize_markers_node ${catkin_LIBRARIES} ${PROJECT_NAME}_lib)

## Add the home_service node
add_executable(home_service_node src/home_service_node.cpp)
add_dependencies(home_service_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(home_service_node ${catkin_LIBRARIES} ${PROJECT_NAME}_lib)
