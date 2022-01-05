# basic libs
import sys
import math
from math import sqrt
from numpy import eye, array, zeros
import numpy as np
import matplotlib.pyplot as plt

# rospy related
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, JointState, NavSatFix
from gazebo_msgs.srv import GetModelState
import gazebo_msgs.msg
import tf

import message_filters

import haversine as hs
from haversine import Unit

# kalman filter related
from filterpy.kalman import ExtendedKalmanFilter
from filterpy.common import Q_discrete_white_noise

class Subsystem_B:
    # static
    WHEEL_RADIUS = 0.12
    RATE_Hz = 100

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
        self.pos_x_actual = 0
        self.pos_y_actual = 0

        #IMU
        self.acc_x_imu = 0
        self.acc_y_imu = 0
        self.vel_x_imu = 0
        self.pos_x_imu = 0
        self.vel_y_imu = 0
        self.pos_y_imu = 0
        self.prev_time = 0

        #GPS
        self.gps_distance_x = 0
        self.gps_distance_y = 0
        
        # EKF
        self.pos_y_ekf = 0
        self.ekf = ExtendedKalmanFilter(dim_x=9,dim_z=4) # x = [pos_y, vel_y, acc_y], z = [pos_y_wheel_encoder, acc_y_imu]
        self.dt = 1/self.RATE_Hz
        # start values of x = [pos_x, vel_x, acc_x, pos_y, vel_y, acc_y, theta, theta1, theta2]
        self.ekf.x = [0, 0, 0, 0, 0, 0, 0, 0, 0]  
        # Measurement vector z = [pos_x_gps, pos_y_gps, acc_x_imu, acc_y_imu] #, dist_ls, bear_ls]
        self.z = [0, 0, 0, 0]
        # State Transition Function, F @ x = x_new
        self.ekf.F = eye(9) + array([
                                    [0, 1, 0.5 * self.dt, 0, 0, 0, 0, 0, 0],
                                    [0, 0,             1, 0, 0, 0, 0, 0, 0],
                                    [0, 0,             0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 1, 0.5 * self.dt, 0, 0, 0],
                                    [0, 0, 0, 0, 0,             1, 0, 0, 0],
                                    [0, 0, 0, 0, 0,             0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0,             0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0,             0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0,             0],
                                    ]) * self.dt   
        # Measurement Noise Matrix, dim = dim_z x dim_z
        self.ekf.R = eye(4)*5   
        # Process Noise Matrix, dim = dim_x x dim_x
        zero_mat = zeros((3,3))
        q = Q_discrete_white_noise(3, var=.01) 
        q1 = np.concatenate([q, zero_mat, zero_mat], axis=1)
        q2 = np.concatenate([zero_mat, q, zero_mat], axis=1)
        q3 = np.concatenate([zero_mat, zero_mat, q], axis=1)
        self.ekf.Q = np.concatenate([q1,q2,q3])
                                    # Engineering Parameter:
                                    # small Q: the filter will be overconfident in its prediction model and will diverge from the actual solution
                                    # large Q: the filter will be unduly influenced by the noise in the measurements and perform sub-optimally
        self.ekf.P = eye(9) * 50    # Posterior Covariance, dim = dim_x x dim_x       
        


    def control_wheels(self, velocity):
        pub1 = rospy.Publisher('/my_robot/joint_wheel_1_controller/command', Float64, queue_size=10)
        pub2 = rospy.Publisher('/my_robot/joint_wheel_2_controller/command', Float64, queue_size=10)

        # wait 500ms, otherwise the command is not reaching the subscriber
        rate = rospy.Rate(2) # 2hz
        rate.sleep()

        # publish to subscriber of each wheel
        move = -velocity
        pub1.publish(move)
        pub2.publish(-move)

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
        self.acc_x_imu = (data.linear_acceleration.x + 9.81 * math.sin(pitch)) * math.cos(pitch)
        self.acc_y_imu = (data.linear_acceleration.y - 9.81 * math.sin(roll))  * math.cos(roll)

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

    def gps_position(self, gps):
        
        origin = (0, 0)
        current_location_x = (gps.latitude, 0)
        
        self.gps_distance_x = hs.haversine(origin, current_location_x, unit=Unit.METERS)

        current_location_y = (0, gps.longitude)
        
        self.gps_distance_y = hs.haversine(origin, current_location_y, unit=Unit.METERS)


    def actual_position(self, data):
        # actual position from gazebo model
        self.pos_x_actual = data.pose[data.name.index('robot')].position.x
        self.pos_y_actual = data.pose[data.name.index('robot')].position.y

        #model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        #robot_coordinates = model_state('robot', '')
        #self.pos_x_actual = robot_coordinates.pose.position.x
        #self.pos_y_actual = robot_coordinates.pose.position.y

    def callback(self, gps, imu, actual):
        self.gps_position(gps)
        self.read_IMU_acc(imu)
        self.actual_position(actual)

    def HJacobian_at(self, x):
        """ compute Jacobian of H matrix at x for EKF"""
        return array([
                     [1, self.dt, 0.5*self.dt**2, 0, 0, 0, 0, 0, 0],
                     [0, 0, 0, 1, self.dt, 0.5*self.dt**2, 0, 0, 0],
                     [0, 0, 1, 0, 0, 0, 0, 0, 0],
                     [0, 0, 0, 0, 0, 1, 0, 0, 0]
                     ])              # dim = dim_x x dim_z

    def hx(self, x):
        """measurement function"""
        return array([x[0]+0.5*x[2]*self.dt**2, x[3]+0.5*x[5]*self.dt**2, x[2], x[5]])     # dim = dim_z

if __name__ == '__main__':
    try:
        rospy.init_node('subsystem_b', anonymous=True)

        subsystem_b = Subsystem_B()
        subsystem_b.control_wheels(float(sys.argv[1]))  # move robot
        subsystem_b.prev_time = rospy.get_time()

        # start threads for sensor values
        #rospy.Subscriber("/imu", Imu, subsystem_b.read_IMU_acc)
        #rospy.Subscriber("/gazebo/link_states", gazebo_msgs.msg.LinkStates, subsystem_b.update_wheel_turns)
        #rospy.Subscriber("/gazebo/model_states", gazebo_msgs.msg.ModelStates, subsystem_b.actual_position)
        #rospy.Subscriber("/gps", NavSatFix, subsystem_b.gps_position) 

        
        gps_sub = message_filters.Subscriber('/gps', NavSatFix)
        imu_sub = message_filters.Subscriber('/imu', Imu)
        actual_sub = message_filters.Subscriber('/gazebo/model_states', gazebo_msgs.msg.ModelStates)
        
        #ts = message_filters.TimeSynchronizer([gps_sub, imu_sub], 10)
        ts = message_filters.ApproximateTimeSynchronizer([gps_sub, imu_sub, actual_sub], 10, 0.02, allow_headerless=True)
        ts.registerCallback(subsystem_b.callback)

        # record values
        steps = 10 * subsystem_b.RATE_Hz # 100*self.dt seconds
        i = 0
        recorded_positions = {
            "EKF":[],
            "IMU":[],
            "Wheel":[],
            "Actual":[]
        }

        recorded_ekf = {
            "EKF_pos":[],
            "EKF_vel":[],
            "EKF_acc":[],
            "Actual":[]
        }

        recorded_ekf_new = {
            "EKF_pos_x":[],
            "EKF_pos_y":[],
            "Actual x":[],
            "Actual y":[]
        }
        while i < steps:
            print("Actual:{}".format(subsystem_b.pos_y_actual))
            #print("Wheel encoder:{}".format(subsystem_b.pos_y_wheel_encoder))

            print("IMU x acc:{}".format(subsystem_b.acc_x_imu))
            print("IMU y acc:{}".format(subsystem_b.acc_y_imu))
            print("GPS x:{}".format(subsystem_b.gps_distance_x))
            print("GPS y:{}".format(subsystem_b.gps_distance_y))
            
            # new measurement values
            subsystem_b.z = array([subsystem_b.gps_distance_x, subsystem_b.gps_distance_y , subsystem_b.acc_x_imu, subsystem_b.acc_y_imu]) 

            subsystem_b.ekf.update(subsystem_b.z, subsystem_b.HJacobian_at, subsystem_b.hx)

            print("EKF x pos:{}".format(subsystem_b.ekf.x[0]))
            print("EKF y pos:{}".format(subsystem_b.ekf.x[3]))

            #recorded_positions["EKF"].append(subsystem_b.ekf.x[0])
            #recorded_positions["IMU"].append(subsystem_b.acc_y_imu)
            #recorded_positions["Wheel"].append(subsystem_b.pos_y_wheel_encoder)
            #recorded_positions["Actual"].append(subsystem_b.pos_y_actual)

            #recorded_ekf["EKF_pos"].append(subsystem_b.ekf.x[0])
            #recorded_ekf["EKF_vel"].append(subsystem_b.ekf.x[1])
            #recorded_ekf["EKF_acc"].append(subsystem_b.ekf.x[2])
            #recorded_ekf["Actual"].append(subsystem_b.pos_y_actual)

            recorded_ekf_new["EKF_pos_x"].append(subsystem_b.ekf.x[0])
            recorded_ekf_new["EKF_pos_y"].append(subsystem_b.ekf.x[3])
            recorded_ekf_new["Actual x"].append(subsystem_b.pos_x_actual)
            recorded_ekf_new["Actual y"].append(subsystem_b.pos_y_actual)

            subsystem_b.ekf.predict()

            subsystem_b.rate.sleep()
            i += 1
        
        subsystem_b.control_wheels(0)

        #plt.plot(range(0,steps),recorded_positions["EKF"], label="EKF")
        #plt.plot(range(0,steps),recorded_positions["IMU"], label="IMUacc")
        #plt.plot(range(0,steps),recorded_positions["Wheel"], label="Wheel")
        #plt.plot(range(0,steps),recorded_positions["Actual"], label="Actual")
        #plt.plot(range(0,steps),recorded_ekf["EKF_pos"], label="EKF_pos")
        #plt.plot(range(0,steps),recorded_ekf["EKF_vel"], label="EKF_vel")
        #plt.plot(range(0,steps),recorded_ekf["EKF_acc"], label="EKF_acc")
        #plt.plot(range(0,steps),recorded_ekf["Actual"], label="Actual")
        #plt.plot(range(0,steps),recorded_ekf_new["EKF_pos_x"], label="EKF_pos_x")
        #plt.plot(range(0,steps),recorded_ekf_new["EKF_pos_y"], label="EKF_pos_y")
        #plt.plot(range(0,steps),recorded_ekf_new["Actual x"], label="Actual x")
        #plt.plot(range(0,steps),recorded_ekf_new["Actual y"], label="Actual y")
        plt.plot(recorded_ekf_new["EKF_pos_x"],recorded_ekf_new["EKF_pos_y"], label="EKF_pos")
        plt.plot(recorded_ekf_new["Actual x"],recorded_ekf_new["Actual y"], label="Actual x")
        #plt.xlabel("timesteps")
        #plt.ylabel("position")
        plt.xlim([-1, 1])
        plt.xlabel("x")
        plt.ylabel("y")
        plt.title("EKF values")
        #plt.title("Sensor + EKF values")
        plt.legend()
        plt.show()

    except rospy.ROSInterruptException:
        pass
    except IndexError:
        print("IndexError")
