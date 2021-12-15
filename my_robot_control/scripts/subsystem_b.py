# basic libs
import sys
import math
from math import sqrt
from numpy import eye, array
import numpy as np
import matplotlib.pyplot as plt

# rospy related
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, JointState
import gazebo_msgs.msg
import tf

# kalman filter related
from filterpy.kalman import ExtendedKalmanFilter
from filterpy.common import Q_discrete_white_noise

class Subsystem_B:
    # static
    WHEEL_RADIUS = 0.12
    RATE_Hz = 10

    def __init__(self):
        #
        self.rate = rospy.Rate(self.RATE_Hz)

        # Wheel encoder
        self.wheel1_turns = 0
        self.wheel1_new_turn = True
        self.wheel2_turns = 0
        self.pos_y_wheel_encoder = 0
        self.prev_degree = 180
        self.degree = 0
        self.quadrant = 0
        self.part_circ = 0
        self.prev_part_circ = 0

        self.mu, self.sigma = 0, 0  #0.05, 0.02 # mean and standard deviation

        # Actual Position
        self.pos_y_actual = 0

        #IMU
        self.acc_x_imu = 0
        self.acc_y_imu = 0
        self.vel_x_imu = 0
        self.pos_x_imu = 0
        self.vel_y_imu = 0
        self.pos_y_imu = 0
        self.prev_time = 0
        
        # EKF
        self.pos_y_ekf = 0
        self.ekf = ExtendedKalmanFilter(dim_x=3,dim_z=2) # x = [pos_y, vel_y, acc_y], z = [pos_y_wheel_encoder, acc_y_imu]
        self.dt = 1/self.RATE_Hz
        self.ekf.x = [0,0,0]  # start values of x = [pos, vel, acc]
        self.ekf.F = eye(3) + array([
                                    [0, 1, 0],
                                    [0, 0, 1],
                                    [0, 0, 0]]) * self.dt   # State Transition Function, F @ x = x_new
        self.ekf.R = eye(2)*5   # Measurement Noise Matrix, dim = dim_z x dim_z
        self.ekf.Q = Q_discrete_white_noise(3, var=.1)   # Process Noise Matrix, dim = dim_x x dim_x
                                    # Engineering Parameter:
                                    # small Q: the filter will be overconfident in its prediction model and will diverge from the actual solution
                                    # large Q: the filter will be unduly influenced by the noise in the measurements and perform sub-optimally
        self.ekf.P = eye(3) * 50    # Posterior Covariance, dim = dim_x x dim_x       
        


    def control_wheels(self, velocity):
        pub1 = rospy.Publisher('/my_robot/joint_wheel_1_controller/command', Float64, queue_size=10)
        pub2 = rospy.Publisher('/my_robot/joint_wheel_2_controller/command', Float64, queue_size=10)

        # wait 500ms, otherwise the command is not reaching the subscriber
        rate = rospy.Rate(2) # 2hz
        rate.sleep()

        # publish to subscriber of each wheel
        move = -velocity
        pub1.publish(move)
        pub2.publish(move)

    def read_IMU_acc(self, data):
        # transform from quaternion to euler angles
        quaternion = (
            data.orientation.x,
            data.orientation.y,
            data.orientation.z,
            data.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        roll = euler [0]
        pitch = euler[1]

        # calculate actual acceleration from linear_acc by considering gravity
        self.acc_x_imu = (data.linear_acceleration.x + 9.81 * math.sin(pitch)) * math.cos(pitch);
        self.acc_y_imu = (data.linear_acceleration.y - 9.81 * math.sin(roll))  * math.cos(roll);

        # Calculate time step since last measurement
        time = rospy.get_time()
        dt = time - self.prev_time
        self.prev_time = time

        # Calculate vel and pos by integrating acc
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
        # transform from quaternion to euler angles
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

        # increment if the wheel did a full turn
        if self.prev_part_circ > self.part_circ:
            self.wheel1_turns += 1

        self.prev_part_circ = self.part_circ

        # add partial wheel turn and noise to full wheel turns
        self.pos_y_wheel_encoder = 2*math.pi*self.WHEEL_RADIUS*self.wheel1_turns \
                    + self.part_circ \
                        -2*math.pi*self.WHEEL_RADIUS/4  + np.random.randn() * self.sigma + self.mu# correction

    def actual_position(self, data):
        # actual position from gazebo model
        self.pos_y_actual = data.pose[data.name.index('robot')].position.y

    def HJacobian_at(self, x):
        """ compute Jacobian of H matrix at x for EKF"""
        return array([[1, self.dt, 0.5*self.dt**2],
                      [0, 0, 0.5*self.dt**2]])
        #return array([[1, self.dt],
        #            [1,self.dt]])

    def hx(self, x):
        """measurement function"""
        return x[0] + x[1]*self.dt + 0.5*x[2]*self.dt**2

if __name__ == '__main__':
    try:
        rospy.init_node('subsystem_b', anonymous=True)

        subsystem_b = Subsystem_B()
        subsystem_b.control_wheels(float(sys.argv[1]))  # move robot
        subsystem_b.prev_time = rospy.get_time()

        # start threads for sensor values
        rospy.Subscriber("/imu", Imu, subsystem_b.read_IMU_acc)
        rospy.Subscriber("/gazebo/link_states", gazebo_msgs.msg.LinkStates, subsystem_b.update_wheel_turns)
        rospy.Subscriber("/gazebo/model_states", gazebo_msgs.msg.ModelStates, subsystem_b.actual_position)

        # record values
        steps = 100 # 100*self.dt seconds
        i = 0
        recorded_positions = {
            "EKF":[],
            "IMU":[],
            "Wheel":[],
            "Actual":[]
        }
        while i < steps:
            print("Actual:{}".format(subsystem_b.pos_y_actual))
            print("Wheel encoder:{}".format(subsystem_b.pos_y_wheel_encoder))
            print("IMU:{}".format(subsystem_b.acc_y_imu))
            
            # new measurement values
            z = array([subsystem_b.pos_y_wheel_encoder, subsystem_b.acc_y_imu]) 

            subsystem_b.ekf.update(z, subsystem_b.HJacobian_at, subsystem_b.hx)

            print("EKF:{}".format(subsystem_b.ekf.x[0]))

            recorded_positions["EKF"].append(subsystem_b.ekf.x[0])
            recorded_positions["IMU"].append(subsystem_b.acc_y_imu)
            recorded_positions["Wheel"].append(subsystem_b.pos_y_wheel_encoder)
            recorded_positions["Actual"].append(subsystem_b.pos_y_actual)

            subsystem_b.ekf.predict()

            subsystem_b.rate.sleep()
            i += 1
        
        subsystem_b.control_wheels(0)

        plt.plot(range(0,steps),recorded_positions["EKF"], label="EKF")
        plt.plot(range(0,steps),recorded_positions["IMU"], label="IMUacc")
        plt.plot(range(0,steps),recorded_positions["Wheel"], label="Wheel")
        plt.plot(range(0,steps),recorded_positions["Actual"], label="Actual")
        plt.legend()
        plt.show()

    except rospy.ROSInterruptException:
        pass
    except IndexError:
        print("IndexError")
