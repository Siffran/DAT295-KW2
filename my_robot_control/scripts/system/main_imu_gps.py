# other software parts
from control_robot import set_wheel_velocity
from LocalizationEstimator import LocalizationEstimator
from read_sensors import read_actual_pos, IMU, read_gps_pos

# rospy related
import rospy
from sensor_msgs.msg import Imu, NavSatFix
import message_filters
import gazebo_msgs.msg

# basic libs
import numpy as np
from matplotlib import pyplot as plt


imu= IMU()

# MODIFY: add variable for your sensor, keep pattern "sensor_direction" i.e. wheel_x
imu_x_pos, imu_y_pos, \
    imu_x_acc, imu_y_acc, \
    gps_x, gps_y, \
    actual_x, actual_y = 0, 0, 0, 0, 0, 0, 0, 0

def callback(gps_data, imu_data, actual_data):
    """
    Executes the sensor functions to get current values.
    Called by the TimeSynchonizer.
    
    """

    # MODIFY: Add get function of each sensor in the configuration.
    global imu_x_acc, imu_y_acc, actual_x, actual_y, imu_x_pos, imu_y_pos, gps_x, gps_y

    imu.update(imu_data, rospy.get_time())
    imu_x_pos, imu_y_pos = imu.get_pos()
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
    imu_sub = message_filters.Subscriber('/imu', Imu)
    gps_sub = message_filters.Subscriber('/gps', NavSatFix)
    actual_sub = message_filters.Subscriber('/gazebo/model_states', gazebo_msgs.msg.ModelStates)

    ts = message_filters.ApproximateTimeSynchronizer([gps_sub, imu_sub, actual_sub], 10, 0.02, True)
    ts.registerCallback(callback)

    # MODIFY: Create instance of Localization estimater.
    #         The setup function has to be created in the LocalizationEstimator.py file
    loc_est = LocalizationEstimator("imupos+gps", RATE_Hz)

    # MODIFY: Amount of steps the while loop records values.
    steps = 400
    i = 0

    # MODIFY: Values to record
    recorded_positions = {
        "EKF x":[],
        "EKF y":[],
        "gps x":[],
        "gps y":[],
        "imu x":[],
        "imu y":[],
        "Actual x":[],
        "Actual y":[]
    }

    imu.set_prev_time(rospy.get_time())

    while i <= steps:
    	
        # MODIFY: Measurement vector with your sensor values
        z = np.array([gps_x, gps_y, imu_x_pos, imu_y_pos])

        loc_est.update(z)
        loc_est.predict()
        state_vector = loc_est.get_state_vector()

        # MODIFY: Append values to record.
        recorded_positions["EKF x"].append(state_vector[0])
        recorded_positions["EKF y"].append(state_vector[2])
        recorded_positions["gps x"].append(gps_x)
        recorded_positions["gps y"].append(gps_y)
        recorded_positions["imu x"].append(imu_x_pos)
        recorded_positions["imu y"].append(imu_y_pos)
        recorded_positions["Actual x"].append(actual_x)
        recorded_positions["Actual y"].append(actual_y)

        rate.sleep()
        i += 1

    # stop robot
    set_wheel_velocity(0)

    # MODIFY: Values to plot.
    fig = plt.figure()
    plt.subplot(1, 2, 1) # row 1, col 2 index 1
    plt.plot(range(0,steps+1), recorded_positions["EKF x"], label="EKF pos")
    plt.plot(range(0,steps+1), recorded_positions["gps x"], color="red", label="GPS pos")
    plt.plot(range(0,steps+1), recorded_positions["imu x"], color="green", label="imu pos")
    plt.plot(range(0,steps+1), recorded_positions["Actual x"], color="black", label="Actual pos")
    plt.title("x distance")
    plt.xlabel('steps ')
    plt.ylabel('X-Distance ')
    plt.legend()
    
    plt.subplot(1, 2, 2) # row 1, col 2 index 1
    plt.plot(range(0,steps+1), recorded_positions["EKF y"], label="EKF pos")
    plt.plot(range(0,steps+1), recorded_positions["gps y"], color="red", label="GPS pos")
    plt.plot(range(0,steps+1), recorded_positions["imu y"], color="green", label="imu pos")
    plt.plot(range(0,steps+1), recorded_positions["Actual y"], color="black", label="Actual pos")
    plt.title("y distance")
    plt.xlabel('steps ')
    plt.ylabel('Y-Distance ')
    plt.legend()
    
    # the error between the estimated position 
    fig2 = plt.figure()
    plt.subplot(1, 2, 1) # index 2
    cc1 = [recorded_positions["EKF x"][i] - recorded_positions["Actual x"][i] for i in range(len(recorded_positions["EKF x"]))]
    cc2 = [recorded_positions["gps x"][i] - recorded_positions["Actual x"][i] for i in range(len(recorded_positions["gps x"]))]
    cc3 = [recorded_positions["imu x"][i] - recorded_positions["Actual x"][i] for i in range(len(recorded_positions["imu x"]))]
    plt.plot(range(0,steps+1), cc1, color="blue", label="EKF error")
    plt.plot(range(0,steps+1), cc2, color="red", label="gps error")
    plt.plot(range(0,steps+1), cc3, color="green", label="imu error")
    plt.title("the error for x")
    plt.xlabel('steps ')
    plt.ylabel('X-Distance ')
    
    plt.subplot(1, 2, 2) # row 1, col 2 index 1
    c1 = [recorded_positions["EKF y"][i] - recorded_positions["Actual y"][i] for i in range(len(recorded_positions["EKF y"]))]
    c2 = [recorded_positions["gps y"][i] - recorded_positions["Actual y"][i] for i in range(len(recorded_positions["gps y"]))]
    c3 = [recorded_positions["imu y"][i] - recorded_positions["Actual y"][i] for i in range(len(recorded_positions["imu y"]))]
    plt.plot(range(0,steps+1), c1, color="blue", label="EKF error")
    plt.plot(range(0,steps+1), c2, color="red", label="gps error")
    plt.plot(range(0,steps+1), c3, color="green", label="imu error")
    plt.title("the error for y")
    plt.xlabel('steps ')
    plt.ylabel('Y-Distance ')
    plt.legend()

    plt.show()
        
    
