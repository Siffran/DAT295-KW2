import rospy
from std_msgs.msg import Float64

def set_wheel_velocity(velocity):
    pub1 = rospy.Publisher('/my_robot/joint_wheel_1_controller/command', Float64, queue_size=10)
    pub2 = rospy.Publisher('/my_robot/joint_wheel_2_controller/command', Float64, queue_size=10)

    rate = rospy.Rate(2) # 2hz
    rate.sleep()

    rospy.loginfo(velocity)
    pub1.publish(-velocity)
    pub2.publish(velocity)
