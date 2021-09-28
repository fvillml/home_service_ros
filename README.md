# Home Service Robot

Home Service package. This package contains five nodes: 

* **pick_objects_node**: This node will make the robot travel to two position in the map: (4.0, 0.0) and (4.0, 2.5).
* **add_markers_node**: This node will publish orange cube marker messages to the /visualization_marker node. It will first publish one at (4.0, 0.0), then wait 5 seconds, remove it, wait five seconds, and then publish one again at (4.0, 2.5).
* **navigate_node**: This node will listen to the goal topic and make a robot using the [move_base](http://wiki.ros.org/move_base) node, move to the specified position. Every position sent while the robot is moving is dropped.
* **visualize_markers_node**: This node is a server to the markers_service service. When received a request, it will add or remove an orange cube marker to a specified position, with an specified id, frame_id and namespace.
* **home_service_node**: This node will execute a complete pickup-dropoff sequence. A sequence is described as:
    - Publish a marker to a pick up position. (For the moment this is hardcoded to x=4.0 and y-0.0).
    - Make the move_base node move to that desired position.
    - Listen to the /odom topic and check when the robot has achieved the desired pick up position.
    - Once it happens, make the marker disappear.
    - Wait 5 seconds.
    - Request the move_base to go to the drop off zone.
    - Drop the object, i.e. add the marker at this place.

Each node requires the [diff_robot](https://github.com/fvillml/diff_robot) package.

## Dependencies
* These nodes requires the amcl package to be able to localize itself.

```bash
sudo apt install ros-<yourDistro>-amcl
```

* These nodes requires the ros navigation stack to execute path planning.
```bash
sudo apt install ros-<yourDistro>-navigation
```

* xterm

```bash
sudo apt install xterm
```

* [diff_robot](https://github.com/fvillml/diff_robot): clone it in yout workspace

## How to use

1. Clone the diff_robot package (or use your custom package providing the [move_base](http://wiki.ros.org/move_base) node)
```bash
cd ~/catkin_ws/src/
git clone https://github.com/fvillml/diff_robot.git
```
2. Compile using either catkin build or catkin_make.
```bash
cd ~/catkin_ws/
catkin init
catkin build
source devel/setup.bash
```
3. Run one of the shell scripts in the [scripts](scripts/) folder