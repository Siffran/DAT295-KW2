from re import sub
import rospy
import sys
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, JointState
import math
from math import sqrt
import gazebo_msgs.msg
import tf
from filterpy.kalman import ExtendedKalmanFilter
from filterpy.common import Q_discrete_white_noise
from numpy import eye, array
import numpy as np
import matplotlib.pyplot as plt

class Subsystem_B:
    WHEEL_RADIUS = 0.12
    def __init__(self):
        self.direction = 0
        self.rate = rospy.Rate(10)
        self.wheel1_turns = 0#-0.5    # 
        self.wheel1_new_turn = True
        self.wheel2_turns = 0
        self.pos_y_wheel_encoder = 0 
        self.pos_x_wheel_encoder = 0
        self.pos_y_actual = 0
        self.acc_x_imu = 0
        self.acc_y_imu = 0
        self.vel_x_imu = 0
        self.pos_x_imu = 0
        self.vel_y_imu = 0
        self.pos_y_imu = 0
        self.prev_time = 0
        self.prev_degree = 180
        self.degree = 0
        self.quadrant = 0
        self.part_circ = 0
        self.prev_part_circ = 0
        self.pos_y_ekf = 0
        self.ekf = ExtendedKalmanFilter(dim_x=2,dim_z=2)
        self.dt = 0.1
        self.ekf.x = [0,0]
        self.ekf.F = eye(2) + array([
                                    [0, 1],
                                    [0, 0]]) * self.dt
        self.ekf.R = eye(2)*5
        self.ekf.Q = 0.1
        self.ekf.P *= 50


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
        quaternion = (
            data.orientation.x,
            data.orientation.y,
            data.orientation.z,
            data.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        roll = euler [0]
        pitch = euler[1]

        self.acc_x_imu = (data.linear_acceleration.x + 9.81 * math.sin(pitch)) * math.cos(pitch);
        self.acc_y_imu = (data.linear_acceleration.y - 9.81 * math.sin(roll))  * math.cos(roll);

        time = rospy.get_time()
        dt = time - self.prev_time
        self.prev_time = time

        Velocity_old_x = self.vel_x_imu
        self.vel_x_imu = Velocity_old_x+self.acc_x_imu*dt
        self.pos_x_imu += self.vel_x_imu*dt

        Velocity_old_y = self.vel_y_imu
        self.vel_y_imu = Velocity_old_y+self.acc_y_imu*dt
        self.pos_y_imu += self.vel_y_imu*dt

    def pitch_to_degree(self, pitch):
        t_degree = (pitch*180)/math.pi

        if t_degree >= 0 and t_degree <= self.prev_degree:
            self.quadrant = 2
            self.degree = 180 - t_degree
        elif t_degree < 0 and t_degree <= self.prev_degree:
            self.quadrant = 3
            self.degree = 180 - t_degree
        elif t_degree < 0 and t_degree > self.prev_degree:
            self.quadrant = 4
            self.degree = 360 + t_degree
        elif t_degree >= 0 and t_degree > self.prev_degree:
            self.quadrant = 1
            self.degree = t_degree

        self.prev_degree = t_degree

    def update_wheel_turns(self, data):
        pose = data.pose[data.name.index('robot::wheel_1')]
        quaternion = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        pitch = euler[1]
        self.pitch_to_degree(pitch)
        self.part_circ = 2*math.pi*self.WHEEL_RADIUS* (self.degree/360)

        if self.prev_part_circ > self.part_circ:
            self.wheel1_turns += 1

        self.prev_part_circ = self.part_circ

        dist = (2*math.pi*self.WHEEL_RADIUS*self.wheel1_turns \
                    + self.part_circ \
                        -2*math.pi*self.WHEEL_RADIUS/4)
        self.pos_y_wheel_encoder =  dist * math.sin(self.direction) # correction
        self.pos_x_wheel_encoder =  dist * math.cos(self.direction) # correction


    def update_position(self, data):
        self.pos_y_actual = data.pose[data.name.index('robot')].position.y

    def update_direction(self, data):
        pose = data.pose[data.name.index('robot::base_link')]
        quaternion = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        yaw = euler[2]
        self.direction = (yaw*180)/math.pi
    def HJacobian_at(self, x):
        """ compute Jacobian of H matrix at x """
        return array([[1, self.dt],
                    [1,self.dt]])
    def hx(self, x):
        """measurement function"""
        return x[0] + x[1]*self.dt
if __name__ == '__main__':
    try:
        rospy.init_node('subsystem_b', anonymous=True)
        subsystem_b = Subsystem_B()
        subsystem_b.control_wheels(float(sys.argv[1]))
        subsystem_b.prev_time = rospy.get_time()
        rospy.Subscriber("/imu", Imu, subsystem_b.read_IMU_acc)
        #rospy.Subscriber("/joint_states", JointState, subsystem_b.update_wheel_positions)
        rospy.Subscriber("/gazebo/link_states", gazebo_msgs.msg.LinkStates, subsystem_b.update_wheel_turns)
        rospy.Subscriber("/gazebo/link_states", gazebo_msgs.msg.LinkStates, subsystem_b.update_direction)
        rospy.Subscriber("/gazebo/model_states", gazebo_msgs.msg.ModelStates, subsystem_b.update_position)

        steps = 100 #10seconds
        #while not rospy.is_shutdown():
        i = 0
        recorded_positions = {
            "EKF":[],
            "IMU":[],
            "Wheel":[],
            "Actual":[]
        }
        while i < steps:
            print("Actual:{}".format(subsystem_b.pos_y_actual))
            print("Wheel encoder Y:{}".format(subsystem_b.pos_y_wheel_encoder))
            print("Wheel encoder X:{}".format(subsystem_b.pos_x_wheel_encoder))
            #print("Vel Y:{}".format(subsystem_b.vel_y))
            print("IMU:{}".format(subsystem_b.pos_y_imu))
            z = array([subsystem_b.pos_y_wheel_encoder, subsystem_b.pos_y_imu]) 
            subsystem_b.ekf.update(z, subsystem_b.HJacobian_at, subsystem_b.hx)
            print("EKF:{}".format(subsystem_b.ekf.x[0]))
            subsystem_b.ekf.predict()
            subsystem_b.rate.sleep()

            recorded_positions["EKF"].append(subsystem_b.ekf.x[0])
            recorded_positions["IMU"].append(subsystem_b.pos_y_imu)
            recorded_positions["Wheel"].append(subsystem_b.pos_y_wheel_encoder)
            recorded_positions["Actual"].append(subsystem_b.pos_y_actual)
            i += 1
        
        subsystem_b.control_wheels(0)
        plt.plot(range(0,100),recorded_positions["EKF"], label="EKF")
        plt.plot(range(0,100),recorded_positions["IMU"], label="IMU")
        plt.plot(range(0,100),recorded_positions["Wheel"], label="Wheel")
        plt.plot(range(0,100),recorded_positions["Actual"], label="Actual")
        plt.legend()
        plt.show()

    except rospy.ROSInterruptException:
        pass
    except IndexError:
        print("IndexError")
        #subsystem_b.control_wheels(1)
