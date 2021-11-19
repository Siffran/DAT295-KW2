# How to start Gazebo Ros Example

## Copy folders in your catkin_ws/src/

## Dependencies
Make sure that you have installed all dependencies.

## Compiled
Make sure that everything is compiled by catkin_make.

## Startup
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
