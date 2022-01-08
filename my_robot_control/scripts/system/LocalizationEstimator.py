from filterpy.kalman import ExtendedKalmanFilter
from filterpy.common import Q_discrete_white_noise
from numpy import eye, array, zeros
import numpy as np


class LocalizationEstimator:
    """
    Estimates the current position of the robot.

    :class: `LocalizationEstimator` initializes the desired Localization Estimator depending on given string in `__init__`.

    """
    def __init__(self, sensors, rate):
        """
        Initializes the desired Localization Estimator.

        Input:
        string sensors, defines sensor configuration
        int rate, defines update frequency

        """
        self.RATE_Hz = rate
        if sensors == "imu+wheel":
            self.setup_imu_wheel_EKF()
        elif sensors == "all":
            self.setup_all_EKF()

    def setup_imu_wheel_EKF(self):
        """
        Initializes an EKF for an IMU and a Wheel Encoder Sensor.
        HINT: Pay attention of the matrix dimensions, they are always noted in a comment !!!

        """
        self.ekf = ExtendedKalmanFilter(dim_x=9,dim_z=4)
        self.dt = 1/self.RATE_Hz

        # start values of x = [x_pos, x_vel, x_acc, y_pos, y_vel, y_acc, theta_pos, theta_vel, theta_acc]
        self.ekf.x = [0, 0, 0, 0, 0, 0, 0, 0, 0]  
        # Measurement vector z = [wheel_x, wheel_y, acc_x_imu, acc_y_imu]
        self.z = [0, 0, 0, 0]
        # State Transition Function, F @ x = x_new, dim_F = dim_x x dim_x
        # TODO: add angle theta as shown here: https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/11-Extended-Kalman-Filters.ipynb
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
        # Measurement Noise Matrix, dim_R = dim_z x dim_z
        self.ekf.R = eye(4)*5   

        # Create measurement noise matrix. It is assembled by 9 3x3 matrices cause `Q_discrete_white_noise` can't handle 9x9.
        # dim_Q = dim_x x dim_x 
        q = Q_discrete_white_noise(3, var=.1) 
        zero_mat = zeros((3,3))
        q1 = np.concatenate([q, zero_mat, zero_mat], axis=1)
        q2 = np.concatenate([zero_mat, q, zero_mat], axis=1)
        q3 = np.concatenate([zero_mat, zero_mat, q], axis=1)
        self.ekf.Q = np.concatenate([q1,q2,q3])
                                    # Engineering Parameter:
                                    # small Q: the filter will be overconfident in its prediction model and will diverge from the actual solution
                                    # large Q: the filter will be unduly influenced by the noise in the measurements and perform sub-optimally
        # Posterior Covariance, dim_P = dim_x x dim_x   
        self.ekf.P = eye(9) * 50        
        
        # HJacobian creates the H matrix for the update step.
        # Explained: https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/11-Extended-Kalman-Filters.ipynb
        self.HJacobian_at = lambda x: array([
                     [1, self.dt, 0.5*self.dt**2, 0, 0, 0, 0, 0, 0],
                     [0, 0, 0, 1, self.dt, 0.5*self.dt**2, 0, 0, 0],
                     [0, 0, 1, 0, 0, 0, 0, 0, 0],
                     [0, 0, 0, 0, 0, 1, 0, 0, 0]
                     ])      

        # hx is needed for the measurement correction in the update step. 
        self.hx = lambda x: array([x[0] + x[1]*self.dt + 0.5*x[2]*self.dt**2, x[3] + x[4]*self.dt + 0.5*x[5]*self.dt**2, x[2], x[5]])

    # def imu_wheel_HJacobian_at(self, x):
    #     """ compute Jacobian of H matrix at x for EKF"""
    #     return array([
    #                  [1, self.dt, 0.5*self.dt**2, 0, 0, 0, 0, 0, 0],
    #                  [0, 0, 0, 1, self.dt, 0.5*self.dt**2, 0, 0, 0],
    #                  [0, 0, 1, 0, 0, 0, 0, 0, 0],
    #                  [0, 0, 0, 0, 0, 1, 0, 0, 0]
    #                  ])              # dim = dim_x x dim_z

    # def hx(self, x):
    #     """measurement function"""
    #     return array([x[0] + x[1]*self.dt + 0.5*x[2]*self.dt**2, x[3] + x[4]*self.dt + 0.5*x[5]*self.dt**2, x[2], x[5]])     # dim = dim_z

    def setup_all_EKF(self):
        self.ekf = ExtendedKalmanFilter(dim_x=9,dim_z=4) # x = [pos_y, vel_y, acc_y], z = [pos_y_wheel_encoder, acc_y_imu]
        self.dt = 1/self.RATE_Hz
        # start values of x = [pos_x, vel_x, acc_x, pos_y, vel_y, acc_y, theta, theta_vel, theta_acc]
        self.ekf.x = [0, 0, 0, 0, 0, 0, 0, 0, 0]  
        # Measurement vector z = [wheel_x, wheel, y, acc_x_imu, acc_y_imu]
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
        q = Q_discrete_white_noise(3, var=.1) 
        q1 = np.concatenate([q, zero_mat, zero_mat], axis=1)
        q2 = np.concatenate([zero_mat, q, zero_mat], axis=1)
        q3 = np.concatenate([zero_mat, zero_mat, q], axis=1)
        self.ekf.Q = np.concatenate([q1,q2,q3])
                                    # Engineering Parameter:
                                    # small Q: the filter will be overconfident in its prediction model and will diverge from the actual solution
                                    # large Q: the filter will be unduly influenced by the noise in the measurements and perform sub-optimally
        self.ekf.P = eye(9) * 50    # Posterior Covariance, dim = dim_x x dim_x       
        
        self.HJacobian_at = lambda x: array([
                     [1, self.dt, 0.5*self.dt**2, 0, 0, 0, 0, 0, 0],
                     [0, 0, 0, 1, self.dt, 0.5*self.dt**2, 0, 0, 0],
                     [0, 0, 1, 0, 0, 0, 0, 0, 0],
                     [0, 0, 0, 0, 0, 1, 0, 0, 0]
                     ])      
        self.hx = lambda x: array([x[0] + x[1]*self.dt + 0.5*x[2]*self.dt**2, x[3] + x[4]*self.dt + 0.5*x[5]*self.dt**2, x[2], x[5]])
        
    def update(self, z):
        """
        Calls update step.
        Input:
        ndarray z, measurement vector, pay attention for same dimension as configured.
        """
        self.ekf.update(z, self.HJacobian_at, self.hx)
        
    def predict(self):
        """
        Calls prediction step.
        """
        self.ekf.predict()

    def get_state_vector(self):
        """
        Return current EKF state vector.
        """
        return self.ekf.x