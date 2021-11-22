#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float64

import sys # for passing arguments through console

def control_x(value):
    pub = rospy.Publisher('/my_robot/my_robot_x_position_controller/command', Float64, queue_size=10)
    rospy.init_node('control', anonymous=True)

    rate = rospy.Rate(2) # 10hz
    rate.sleep()
    x_move = value
    rospy.loginfo(x_move)
    pub.publish(x_move)


def control_y(value):
    pub = rospy.Publisher('/my_robot/my_robot_y_position_controller/command', Float64, queue_size=10)
    rospy.init_node('control', anonymous=True)

    rate = rospy.Rate(2) # 10hz
    rate.sleep()
    y_move = value
    rospy.loginfo(y_move)
    pub.publish(y_move)


if __name__ == '__main__':
    try:
        control_x(int(sys.argv[1]))
        control_y(int(sys.argv[2]))
    except rospy.ROSInterruptException:
        pass
    except IndexError:
        print("IndexError")
        control_x(1)
        control_y(1)
