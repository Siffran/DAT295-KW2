mate-terminal -- /bin/bash -c 'source ../devel/setup.bash; roslaunch my_robot joints.launch'
mate-terminal -- /bin/bash -c 'source ../devel/setup.bash; roslaunch my_robot_control my_robot_control.launch'
mate-terminal -- /bin/bash -c 'source ../devel/setup.bash; cd my_robot_control/scripts/'
