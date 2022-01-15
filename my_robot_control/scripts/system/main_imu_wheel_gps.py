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
import csv
import pandas as pd  

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
imu = IMU()

# MODIFY: add variable for your sensor, keep pattern "sensor_direction" i.e. wheel_x
gps_x = 0.0
gps_y = 0.0
wheel_x = 0.0
wheel_y = 0.0
imu_x_pos = 0.0
imu_y_pos = 0.0
actual_x = 0.0
actual_y = 0.0

#def callback(imu_data, wheel_data, actual_data):
def callback(gps_data, wheel_data, imu_data):
    """
    Executes the sensor functions to get current values.
    Called by the TimeSynchonizer.
    
    """

    # MODIFY: Add get function of each sensor in the configuration.
    
    #global imu_x_acc, imu_y_acc, wheel_x, wheel_y, actual_x, actual_y
    global gps_x, gps_y, wheel_x, wheel_y, actual_x, actual_y, imu_x_pos, imu_y_pos

    imu.update(imu_data,rospy.get_time())
    imu_x_pos, imu_y_pos = imu.get_pos()
    wheel_encoder.update(wheel_data)
    wheel_x, wheel_y = wheel_encoder.get_pos()
    gps_x, gps_y = read_gps_pos(gps_data)
    
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
    gps_sub = message_filters.Subscriber('/gps', NavSatFix)
    #actual_sub = message_filters.Subscriber('/gazebo/model_states', gazebo_msgs.msg.ModelStates)

    #ts = message_filters.ApproximateTimeSynchronizer([imu_sub, wheel_sub, actual_sub], 10, 0.02, True)
    ts = message_filters.ApproximateTimeSynchronizer([gps_sub, wheel_sub, imu_sub], 10, 0.02, True)
    ts.registerCallback(callback)

    # MODIFY: Create instance of Localization estimater.
    #         The setup function has to be created in the LocalizationEstimator.py file
    loc_est = LocalizationEstimator("imu+wheel+gps", RATE_Hz)

    # MODIFY: Values to record
    recorded_positions = {
        "EKF x":[],
        "EKF y":[],
        "GPS x":[],
        "GPS y":[],
        "Wheel x":[],
        "Wheel y":[],
        "IMU x":[],
        "IMU y":[],
        "Actual x service":[],
        "Actual y service":[],
        "Time":[]
    }

    # MODIFY: How far should the robot travel (in y direction) before stopping
    distance = 20
    i = 0

    start_time = rospy.get_time()
    imu.set_prev_time(rospy.get_time())

    while True:

        # MODIFY: Measurement vector with your sensor values
        z = np.array([gps_x, gps_y, wheel_x, wheel_y, imu_x_pos, imu_y_pos])

        loc_est.update(z)
        loc_est.predict()
        state_vector = loc_est.get_state_vector()

        model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        robot_coordinates = model_state('robot', '')

        # MODIFY: Append values to record.
        recorded_positions["EKF x"].append(state_vector[0])
        recorded_positions["EKF y"].append(state_vector[1])
        recorded_positions["GPS x"].append(gps_x)
        recorded_positions["GPS y"].append(gps_y)
        recorded_positions["Wheel x"].append(wheel_x)
        recorded_positions["Wheel y"].append(wheel_y)
        recorded_positions["IMU x"].append(imu_x_pos)
        recorded_positions["IMU y"].append(imu_y_pos)
        recorded_positions["Actual x service"].append(robot_coordinates.pose.position.x)
        recorded_positions["Actual y service"].append(robot_coordinates.pose.position.y)
        recorded_positions["Time"].append(rospy.get_time()-start_time)

        rate.sleep()
        i += 1

        if (robot_coordinates.pose.position.y > distance):
            break

    # stop robot
    set_wheel_velocity(0)

    
    # save all the data into a csv file
    dict = {'Time': recorded_positions["Time"], 'EKF_x': recorded_positions["EKF x"], 'EKF_y': recorded_positions["EKF y"], 'Actual_x': recorded_positions["Actual x service"],'Actual_y': recorded_positions["Actual y service"]}  
    df = pd.DataFrame(dict) 
    df.to_csv('main_imu_wheel_gps.csv') 
    
    plt.figure(figsize=(14, 6), dpi=120)

    # MODIFY: Values to plot.
    plt.subplot(1, 2, 1) # row 1, col 2 index 1
    plt.plot(recorded_positions["Time"], recorded_positions["EKF x"], label="EKF pos")
    plt.plot(recorded_positions["Time"], recorded_positions["GPS x"], color="red", alpha=0.7, label="GPS pos")
    plt.plot(recorded_positions["Time"], recorded_positions["Wheel x"], color="green", alpha=0.7, label="Wheel pos")
    plt.plot(recorded_positions["Time"], recorded_positions["IMU x"], color="orange", alpha=0.7, label="IMU pos")
    plt.plot(recorded_positions["Time"], recorded_positions["Actual x service"], color="black", label="Actual pos")
    plt.title("X-Distance")
    plt.ylabel("X pos (m)")
    plt.xlabel('Time (sec)')
    plt.legend()

    plt.subplot(1, 2, 2) # index 2
    plt.plot(recorded_positions["Time"], recorded_positions["EKF y"], label="EKF pos")
    plt.plot(recorded_positions["Time"], recorded_positions["GPS y"], color="red", alpha=0.7, label="GPS pos")
    plt.plot(recorded_positions["Time"], recorded_positions["Wheel y"], color="green", alpha=0.7, label="Wheel pos")
    plt.plot(recorded_positions["Time"], recorded_positions["IMU y"], color="orange", alpha=0.7, label="IMU pos")
    plt.plot(recorded_positions["Time"], recorded_positions["Actual y service"], color="black", label="Actual pos")
    plt.title("Y-Distance")
    plt.ylabel("Y pos (m)")
    plt.xlabel('Time (sec)')
    plt.legend()

    plt.show()
