gnome-terminal -- /bin/bash -c 'source ../../devel/setup.bash; roslaunch my_robot joints.launch'
gnome-terminal -- /bin/bash -c 'source ../../devel/setup.bash; roslaunch my_robot_control my_robot_control.launch'
gnome-terminal -- /bin/bash -c 'source ../../devel/setup.bash; cd my_robot_control/scripts/'

