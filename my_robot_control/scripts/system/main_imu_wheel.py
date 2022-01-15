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
import csv
import pandas as pd  

wheel_encoder = WheelEncoder()
imu = IMU()

# MODIFY: add variable for your sensor, keep pattern "sensor_direction" i.e. wheel_x
#Sys.1
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

    # MODIFY: How far should the robot travel (in y direction) before stopping
    distance = 20
    i = 0

    # MODIFY: Values to record
    recorded_positions = {
        "EKF x":[],
        "EKF y":[],
        "Wheel x":[],
        "Wheel y":[],
        "IMU x":[],
        "IMU y":[],
        "Actual x":[],
        "Actual y":[],
        "Time":[]
    }
    start_time = rospy.get_time()
    imu.set_prev_time(rospy.get_time())

    while True:

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
        recorded_positions["IMU x"].append(imu_x_pos)
        recorded_positions["IMU y"].append(imu_y_pos)
        recorded_positions["Actual x"].append(actual_x)
        recorded_positions["Actual y"].append(actual_y)
        recorded_positions["Time"].append(rospy.get_time()-start_time)

        rate.sleep()
        i += 1
        if (actual_y > distance):
            break
    # stop robot
    set_wheel_velocity(0)
    
    # save all the data into a csv file
    dict = {'Time': recorded_positions["Time"], 'EKF_x': recorded_positions["EKF x"], 'EKF_y': recorded_positions["EKF y"], 'Actual_x': recorded_positions["Actual x"],'Actual_y': recorded_positions["Actual y"]}  
    df = pd.DataFrame(dict) 
    df.to_csv('main_imu_wheel.csv') 

    # MODIFY: Values to plot.
    
    fig = plt.figure(figsize=(14, 6), dpi=120)

    plt.subplot(1, 2, 1) # row 1, col 2 index 1
    plt.plot(recorded_positions["Time"], recorded_positions["EKF x"], label="EKF pos")
    plt.plot(recorded_positions["Time"], recorded_positions["Wheel x"], color="green",alpha=0.7,  label="Wheel pos")
    plt.plot(recorded_positions["Time"], recorded_positions["IMU x"], color="orange", alpha=0.7, label="IMU pos")
    plt.plot(recorded_positions["Time"], recorded_positions["Actual x"], color="black", label="Actual pos")
    plt.title("X-Distance")
    plt.ylabel("X pos (m)")
    plt.xlabel('Time (sec)')
    plt.legend()

    plt.subplot(1, 2, 2) # index 2
    plt.plot(recorded_positions["Time"], recorded_positions["EKF y"], label="EKF pos")
    plt.plot(recorded_positions["Time"], recorded_positions["Wheel y"], color="green", alpha=0.7, label="Wheel pos")
    plt.plot(recorded_positions["Time"], recorded_positions["IMU y"], color="orange", alpha=0.7, label="IMU pos")
    plt.plot(recorded_positions["Time"], recorded_positions["Actual y"], color="black", label="Actual pos")
    plt.title("Y-Distance")
    plt.ylabel("Y pos (m)")
    plt.xlabel('Time (sec)')
    plt.legend()
    


    # the error between the estimated position 

    fig2 = plt.figure(figsize=(14, 6), dpi=120)
    plt.subplot(1, 2, 1) # row 1, col 2 index 1
    c1 = [recorded_positions["EKF x"][i] - recorded_positions["Actual x"][i] for i in range(len(recorded_positions["EKF x"]))]
    c2 = [recorded_positions["IMU x"][i] - recorded_positions["Actual x"][i] for i in range(len(recorded_positions["IMU x"]))]
    c3 = [recorded_positions["Wheel x"][i] - recorded_positions["Actual x"][i] for i in range(len(recorded_positions["Wheel x"]))]
    plt.plot(recorded_positions["Time"], c1, label="EKF")
    plt.plot(recorded_positions["Time"], c3, color="green",alpha=0.7,  label="Wheel")
    plt.plot(recorded_positions["Time"], c2, color="orange", alpha=0.7, label="IMU")
    plt.title("the error for x")
    plt.xlabel('Time (sec) ')
    plt.ylabel('X-Error')
    
    plt.subplot(1, 2, 2) # index 2
    c1 = [recorded_positions["EKF y"][i] - recorded_positions["Actual y"][i] for i in range(len(recorded_positions["EKF y"]))]
    c2 = [recorded_positions["IMU y"][i] - recorded_positions["Actual y"][i] for i in range(len(recorded_positions["IMU y"]))]
    c3 = [recorded_positions["Wheel y"][i] - recorded_positions["Actual y"][i] for i in range(len(recorded_positions["Wheel y"]))]
    plt.plot(recorded_positions["Time"], c1, label="EKF")
    plt.plot(recorded_positions["Time"], c3, color="green",alpha=0.7,  label="Wheel")
    plt.plot(recorded_positions["Time"], c2, color="orange", alpha=0.7, label="IMU")

    plt.title("the error for y")
    plt.xlabel('Time (sec) ')
    plt.ylabel('Y-Error')
    plt.legend()

    plt.show()
        
    
