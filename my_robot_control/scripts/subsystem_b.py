import rospy
import sys
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, JointState
import math
import gazebo_msgs.msg
import tf


class Subsystem_B:
    WHEEL_RADIUS = 0.12
    def __init__(self):
        self.wheel1_turns = -0.5
        self.wheel1_new_turn = True
        self.wheel2_turns = 0
        self.distance_wheel_encoder = 0
        self.distance = 0


    def control_wheels(self, velocity):
        pub1 = rospy.Publisher('/my_robot/joint_wheel_1_controller/command', Float64, queue_size=10)
        pub2 = rospy.Publisher('/my_robot/joint_wheel_2_controller/command', Float64, queue_size=10)

        rate = rospy.Rate(2) # 2hz
        rate.sleep()
        move = -velocity
        #rospy.loginfo(move)
        pub1.publish(move)
        pub2.publish(move)

    def read_IMU_acc(self, data):
        rospy.loginfo("Yacc: {}".format(data.linear_acceleration.y))

    def update_wheel_turns(self, data):
        pose = data.pose[data.name.index('robot::wheel_1')]
        quaternion = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        pitch = euler[1]
        if pitch < 0 and self.wheel1_new_turn:
            self.wheel1_turns += 1
            self.wheel1_new_turn = False
        elif pitch >= 0:
            self.wheel1_new_turn = True
        
        self.distance_wheel_encoder = 2*math.pi*self.WHEEL_RADIUS*self.wheel1_turns


    def update_position(self, data):
        self.distance = data.pose[data.name.index('robot')].position.y
        #print(data.name)
        #rospy.loginfo("Y: {}".format(data.pose[data.name.index('robot')]))
        #print("Actual dist:{}".format(-data.pose[data.name.index('robot')].position.y))
        pass


            
if __name__ == '__main__':
    try:
        subsystem_b = Subsystem_B()
        rospy.init_node('subsystem_b', anonymous=True)
        subsystem_b.control_wheels(float(sys.argv[1]))
        #rospy.Subscriber("/imu", Imu, subsystem_b.read_IMU_acc)
        #rospy.Subscriber("/joint_states", JointState, subsystem_b.update_wheel_positions)
        rospy.Subscriber("/gazebo/link_states", gazebo_msgs.msg.LinkStates, subsystem_b.update_wheel_turns)
        rospy.Subscriber("/gazebo/model_states", gazebo_msgs.msg.ModelStates, subsystem_b.update_position)
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            print("Actual:{}".format(subsystem_b.distance))
            print("Wheel encoder:{}".format(subsystem_b.distance_wheel_encoder))
            rate.sleep()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except IndexError:
        print("IndexError")
        subsystem_b.control_wheels_twist(1)
