# How to start Gazebo Ros Example

## Helpful Links
- https://wiki.ros.org/urdf/Tutorials
- http://gazebosim.org/tutorials?tut=ros_control&cat=connect_ros
- https://www.youtube.com/watch?v=mtSpqObg9X4

## Required Packages
- sudo apt install python3-pip
- pip install haversine
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
(maybe adjust your terminal e.g. from "genome-terminal" to "mate-terminal")

### * Manually

### roscore
- source your ROS
- start roscore

### start Gazebo
- in catkin_ws folder
- source devel/setup.bash
- roslaunch my_robot my_robot.launch
  
### start Controller
- in catkin_ws folder
- source devel/setup.bash
- roslaunch my_robot_control my_robot_control.launch

## Move
- execute one of the main files in my_robot_control/scripts/system
