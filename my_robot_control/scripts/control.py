#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float64

import sys # for passing arguments through console

def control(value):
    pub = rospy.Publisher('/my_robot/my_robot_y_position_controller/command', Float64, queue_size=10)
    rospy.init_node('control', anonymous=True)

    rate = rospy.Rate(2) # 10hz
    rate.sleep()
    move = value
    rospy.loginfo(move)
    pub.publish(move)


if __name__ == '__main__':
    try:
        control(int(sys.argv[1]))
    except rospy.ROSInterruptException:
        pass
    except IndexError:
        print("IndexError")
        control(1)
