#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float64, String
from control_msgs.msg import JointControllerState
import gazebo_msgs.msg
from sensor_msgs.msg import Imu

import sys # for passing arguments through console

def read_pos_y(data):
    rospy.loginfo("Y: {}".format(data.pose[data.name.index('robot')].position.y))

def read_IMU_acc(data):
    rospy.loginfo("Yacc: {}".format(data.linear_acceleration.y))

def read_Odo_velo(data):
    

if __name__ == '__main__':
    #try:
        rospy.init_node('listener', anonymous=True)
        #rate = rospy.Rate(2) # 2hz
        #rate.sleep()
        #rospy.Subscriber("/gazebo/model_states", gazebo_msgs.msg.ModelStates, read_pos_y)
        rospy.Subscriber("/imu", Imu, read_IMU)
        rospy.spin()

    #except rospy.ROSInterruptException:
    #    print("pass")
    #    pass
    #except IndexError:
    #    print("IndexError")
