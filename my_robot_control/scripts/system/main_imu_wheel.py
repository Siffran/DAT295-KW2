# other software parts
from control_robot import set_wheel_velocity
from LocalizationEstimator import LocalizationEstimator
from read_sensors import read_actual_pos, WheelEncoder, IMU

# rospy related
import rospy
from sensor_msgs.msg import Imu
import message_filters
import gazebo_msgs.msg

# basic libs
import numpy as np
from matplotlib import pyplot as plt


wheel_encoder = WheelEncoder()
imu= IMU()

# MODIFY: add variable for your sensor, keep pattern "sensor_direction" i.e. wheel_x
imu_x_pos, imu_y_pos, \
    imu_x_acc, imu_y_acc, \
    wheel_x, wheel_y, \
    actual_x, actual_y = 0, 0, 0, 0, 0, 0, 0, 0

def callback(imu_data, wheel_data, actual_data):
    """
    Executes the sensor functions to get current values.
    Called by the TimeSynchonizer.
    
    """

    # MODIFY: Add get function of each sensor in the configuration.
    global imu_x_acc, imu_y_acc, wheel_x, wheel_y, actual_x, actual_y, imu_x_pos, imu_y_pos

    imu.update(imu_data, rospy.get_time())
    imu_x_pos, imu_y_pos = imu.get_pos()
    wheel_encoder.update(wheel_data)
    wheel_x, wheel_y = wheel_encoder.get_pos()
    actual_x, actual_y = read_actual_pos(actual_data)
    
if __name__ == "__main__":
    """
    Initializes rospy and the Subscribers.
    Controls robot velocity.
    Initializes LocalizationEstimator.
    Record and plot result.

    """

    # frequency in which the Estimator is called.
    RATE_Hz = 50 

    # Initialize rospy node and frequency
    rospy.init_node('control', anonymous=True)
    rate = rospy.Rate(RATE_Hz)

    # TODO: dynamically changeable velocity via the terminal
    velocity = int(input("Input robot velocity\n"))
    print("Setting velocity to {}".format(velocity))
    set_wheel_velocity(velocity)

    # MODIFY: Subscribe all needed sensors
    #         Add them to the ApproximateTimeSynchronizer
    #         Adjust time intervall in which two values belong to each other (Third value in ApproximateTimeSynchronizer)
    imu_sub = message_filters.Subscriber('/imu', Imu)
    wheel_sub = message_filters.Subscriber('/gazebo/link_states', gazebo_msgs.msg.LinkStates)
    actual_sub = message_filters.Subscriber('/gazebo/model_states', gazebo_msgs.msg.ModelStates)

    ts = message_filters.ApproximateTimeSynchronizer([imu_sub, wheel_sub, actual_sub], 10, 0.02, True)
    ts.registerCallback(callback)

    # MODIFY: Create instance of Localization estimater.
    #         The setup function has to be created in the LocalizationEstimator.py file
    loc_est = LocalizationEstimator("imupos+wheel", RATE_Hz)

    # MODIFY: Amount of steps the while loop records values.
    steps = 200
    i = 0

    # MODIFY: Values to record
    recorded_positions = {
        "EKF x":[],
        "EKF y":[],
        "Wheel x":[],
        "Wheel y":[],
        "Actual x":[],
        "Actual y":[]
    }

    imu.set_prev_time(rospy.get_time())

    while i <= steps:

        # MODIFY: Measurement vector with your sensor values
        z = np.array([wheel_x, wheel_y, imu_x_pos, imu_y_pos])

        loc_est.update(z)
        loc_est.predict()
        state_vector = loc_est.get_state_vector()

        # MODIFY: Append values to record.
        recorded_positions["EKF x"].append(state_vector[0])
        recorded_positions["EKF y"].append(state_vector[3])
        recorded_positions["Wheel x"].append(wheel_x)
        recorded_positions["Wheel y"].append(wheel_y)
        recorded_positions["Actual x"].append(actual_x)
        recorded_positions["Actual y"].append(actual_y)

        rate.sleep()
        i += 1

    # stop robot
    set_wheel_velocity(0)

    # MODIFY: Values to plot.
    plt.plot(recorded_positions["EKF x"], recorded_positions["EKF y"], label="EKF pos")
    plt.plot(recorded_positions["Wheel x"], recorded_positions["Wheel y"], label="Wheel pos")
    plt.plot(recorded_positions["Actual x"], recorded_positions["Actual y"], label="Actual pos")
    plt.xlabel("x")
    plt.ylabel("y")
    plt.title("EKF values")
    plt.legend()
    plt.show()
        
    
