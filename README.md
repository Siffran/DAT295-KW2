# How to start Gazebo Ros Example

## Helpful Links
- https://wiki.ros.org/urdf/Tutorials
- http://gazebosim.org/tutorials?tut=ros_control&cat=connect_ros
- https://www.youtube.com/watch?v=mtSpqObg9X4

## Required Packages
- sudo apt-get install ros-noetic-hector-gazebo-plugins

## Copy folders to your catkin_ws/src/
## Copy models to your gazebo root
cd to my_robot/worlds

cp models  ~/.gazebo/models -r

## OPTIONAL - create a soft link instead
- 1 remove folder /home/<user>/.gazebo/models
- 2 Create a softlink from our models to gazebo. For me the command looks like this: ln -s /home/dabe/catkin_ws/src/DAT295-KW2/my_robot/worlds/models /home/dabe/.gazebo

## Dependencies
Make sure that you have installed all dependencies. *(you might be missing gazebo_ros_control)*

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
- rosrun rqt_gui rqt_gui
  - select topic: /my_robot/my_robot_x_postition_controller/command

## Move
- in rqt-gui
- change expression on right side
- click checkbox on left side
