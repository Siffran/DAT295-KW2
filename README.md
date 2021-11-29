# How to start Gazebo Ros Example

## Helpful Links
- https://wiki.ros.org/urdf/Tutorials
- http://gazebosim.org/tutorials?tut=ros_control&cat=connect_ros
- https://www.youtube.com/watch?v=mtSpqObg9X4

## Copy folders in your catkin_ws/src/

## Dependencies
Make sure that you have installed all dependencies.

## Compiled
Make sure that everything is compiled by catkin_make.

## Startup
### * Using start.sh:

chmod +x start.sh 

./start.sh

### * Manually

### roscore
- source your ROS
- start roscore
### start Gazebo
- in catkin_ws folder
- source devel/setup.bash
- roslaunch my_robot joints.launch
### start Controller
- roslaunch my_robot_control my_robot_control.launch
- rosrun rqt-gui rqt-gui
  - select topic: /my_robot/my_robot_x_postition_controller/command

## Move
- in rqt-gui
- change expression on right side
- click checkbox on left side
