# other software parts
from control_robot import set_wheel_velocity
from LocalizationEstimator import LocalizationEstimator
from read_sensors import read_actual_pos, read_gps_pos, WheelEncoder, IMU

# rospy related
import rospy
import message_filters
import gazebo_msgs.msg
from gazebo_msgs.srv import GetModelState
from sensor_msgs.msg import LaserScan, Imu, NavSatFix

# basic libs
import numpy as np
from matplotlib import pyplot as plt

'''
How to solve the problem :)

get all sensor data:
 - Wheelencoder and IMU
 - GPS
 - Laser
test it

Solve problem of sensors sending data att different frequency

Add sensor data to EKF

Record position, plot estimated vs actual (over time?)

Add (more) noise to sensors

test again

Profit????
'''

wheel_encoder = WheelEncoder()

# MODIFY: add variable for your sensor, keep pattern "sensor_direction" i.e. wheel_x
imu_x_acc = 0.0
imu_y_acc = 0.0
wheel_x = 0.0
wheel_y = 0.0
actual_x = 0.0
actual_y = 0.0
gps_x = 0.0
gps_y = 0.0

#def callback(imu_data, wheel_data, actual_data):
def callback(gps_data, wheel_data, actual_data):
    """
    Executes the sensor functions to get current values.
    Called by the TimeSynchonizer.
    
    """

    # MODIFY: Add get function of each sensor in the configuration.
    
    #global imu_x_acc, imu_y_acc, wheel_x, wheel_y, actual_x, actual_y
    global gps_x, gps_y, wheel_x, wheel_y, actual_x, actual_y

    #imu_x_acc, imu_y_acc = read_imu_acc(imu_data)
    wheel_encoder.update(wheel_data)
    wheel_x, wheel_y = wheel_encoder.get_pos()
    gps_x, gps_y = read_gps_pos(gps_data)
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
    
    #imu_sub = message_filters.Subscriber('/imu', Imu)
    wheel_sub = message_filters.Subscriber('/gazebo/link_states', gazebo_msgs.msg.LinkStates)
    gps_sub = message_filters.Subscriber('/gps', NavSatFix)
    actual_sub = message_filters.Subscriber('/gazebo/model_states', gazebo_msgs.msg.ModelStates)

    #ts = message_filters.ApproximateTimeSynchronizer([imu_sub, wheel_sub, actual_sub], 10, 0.02, True)
    ts = message_filters.ApproximateTimeSynchronizer([gps_sub, wheel_sub, actual_sub], 10, 0.02, True)
    ts.registerCallback(callback)

    # MODIFY: Create instance of Localization estimater.
    #         The setup function has to be created in the LocalizationEstimator.py file
    loc_est = LocalizationEstimator("all", RATE_Hz)

    # MODIFY: Amount of steps the while loop records values.
    steps = 500
    i = 0

    # MODIFY: Values to record
    recorded_positions = {
        "EKF x":[],
        "EKF y":[],
        "GPS x":[],
        "GPS y":[],
        "Wheel x":[],
        "Wheel y":[],
        "Actual x":[],
        "Actual y":[]
    }


    while i <= steps:

        # MODIFY: Measurement vector with your sensor values
        z = np.array([gps_x, gps_y, wheel_x, wheel_y])

        loc_est.update(z)
        loc_est.predict()
        state_vector = loc_est.get_state_vector()

        # MODIFY: Append values to record.
        recorded_positions["EKF x"].append(state_vector[0])
        recorded_positions["EKF y"].append(state_vector[1])
        recorded_positions["GPS x"].append(gps_x)
        recorded_positions["GPS y"].append(gps_y)
        recorded_positions["Wheel x"].append(wheel_x)
        recorded_positions["Wheel y"].append(wheel_y)
        recorded_positions["Actual x"].append(actual_x)
        recorded_positions["Actual y"].append(actual_y)

        rate.sleep()
        i += 1

    # stop robot
    set_wheel_velocity(0)

    # MODIFY: Values to plot.

    plt.subplot(1, 2, 1) # row 1, col 2 index 1
    plt.plot(range(0,steps+1), recorded_positions["EKF x"], label="EKF pos")
    plt.plot(range(0,steps+1), recorded_positions["GPS x"], color="red", label="GPS pos")
    plt.plot(range(0,steps+1), recorded_positions["Wheel x"], color="green", label="Wheel pos")
    plt.plot(range(0,steps+1), recorded_positions["Actual x"], color="black", label="Actual pos")
    plt.title("My first plot!")
    plt.xlabel('steps ')
    plt.ylabel('Y-Distance ')
    plt.legend()

    plt.subplot(1, 2, 2) # index 2
    plt.plot(range(0, steps+1), recorded_positions["EKF y"], label="EKF pos")
    plt.plot(range(0, steps+1), recorded_positions["GPS y"], color="red", label="GPS pos")
    plt.plot(range(0, steps+1), recorded_positions["Wheel y"], color="green", label="Wheel pos")
    plt.plot(range(0, steps+1), recorded_positions["Actual y"], color="black", label="Actual pos")
    plt.title("My second plot!")
    plt.xlabel('steps ')
    plt.ylabel('X-Distance ')
    plt.legend()

    plt.show()