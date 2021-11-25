#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float64, String
from control_msgs.msg import JointControllerState

import sys # for passing arguments through console

def control_pos(x, y, rate):
    pub = rospy.Publisher('/my_robot/my_robot_x_position_controller/command', Float64, queue_size=10)
    pub2 = rospy.Publisher('/my_robot/my_robot_y_position_controller/command', Float64, queue_size=10)
    rospy.loginfo("Moving to: ({},{})".format(x,y))
    rate = rospy.Rate(5)

    rate.sleep()
    x_move = x
    y_move = y
    pub.publish(x_move)
    pub2.publish(y_move)

def read_pos_y(data):

    rospy.loginfo("Y: {}".format(data.process_value))
    rate = rospy.Rate(5)

    rate.sleep()

def read_pos_x(data):

    rospy.loginfo("X: {}".format(data.process_value))
    rate = rospy.Rate(5)
    rate.sleep()



if __name__ == '__main__':
    try:
        rospy.init_node('control', anonymous=True)
        rate = rospy.Rate(5)
        rate.sleep()
        rospy.loginfo("test")
        control_pos(int(sys.argv[1]), int(sys.argv[2]), rate)
        rospy.Subscriber("my_robot/my_robot_y_position_controller/state", JointControllerState, read_pos_y)
        rospy.Subscriber("my_robot/my_robot_x_position_controller/state", JointControllerState, read_pos_x)

    except rospy.ROSInterruptException:
        pass
    except IndexError:
        print("IndexError")
        #control_x(1)
        #control_y(1)
