#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float64

import sys # for passing arguments through console

def control_wheels(value):
    pub1 = rospy.Publisher('/my_robot/joint_wheel_1_controller/command', Float64, queue_size=10)
    pub2 = rospy.Publisher('/my_robot/joint_wheel_2_controller/command', Float64, queue_size=10)

    rospy.init_node('control', anonymous=True)

    rate = rospy.Rate(2) # 10hz
    rate.sleep()
    move = -value
    rospy.loginfo(move)
    pub1.publish(move)
    pub2.publish(move)


if __name__ == '__main__':
    try:
        control_wheels(int(sys.argv[1]))
    except rospy.ROSInterruptException:
        pass
    except IndexError:
        print("IndexError")
        control_wheels(1)
