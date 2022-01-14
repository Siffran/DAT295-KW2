from sensor_msgs.msg import LaserScan, Imu, NavSatFix
from gazebo_msgs.srv import GetModelState
import rospy
import message_filters
import gazebo_msgs.msg

import sys
sys.path.append('..')
from system.read_sensors import read_actual_pos, read_gps_pos
from system.control_robot import set_wheel_velocity
#from kf_book import book_plots as bp

import haversine as hs
from haversine import Unit

import numpy as np
from matplotlib import pyplot as plt

#Running in parallel
import multiprocessing as mp
import time

gps_x = 0.0
gps_y = 0.0
wheel_x = 0.0
wheel_y = 0.0
imu_x_pos = 0.0
imu_y_pos = 0.0
actual_x = 0.0
actual_y = 0.0

def callback(gps_data):

    global gps_x, gps_y, actual_x, actual_y
    gps_x, gps_y = read_gps_pos(gps_data)


if __name__ == '__main__':
    rospy.init_node('sensor_listener', anonymous=True)

    # frequency in which the Estimator is called.
    RATE_Hz = 50 

    # Initialize rospy node and frequency
    rate = rospy.Rate(RATE_Hz)

    # TODO: dynamically changeable velocity via the terminal
    velocity = int(input("Input robot velocity\n"))
    print("Setting velocity to {}".format(velocity))
    #Start the test speed pattern
    set_wheel_velocity(velocity)

    gps_sub = message_filters.Subscriber('/gps', NavSatFix)

    ts = message_filters.ApproximateTimeSynchronizer([gps_sub], 10, 0.02, True)
    ts.registerCallback(callback)

    # Get initial position (use this as offset later?)
    model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    robot_coordinates = model_state('robot', '')


    print("Starting Position Y: ", robot_coordinates.pose.position.y)
    print("Estimated Position Y: TODO")
    print()

    # MODIFY: Values to record
    recorded_positions = {
        "GPS x":[],
        "GPS y":[],
        "Actual x service":[],
        "Actual y service":[],
        "Time":[]
    }

    start_time = rospy.get_time()

    i = 0

    while True:

        model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        robot_coordinates = model_state('robot', '')

        # MODIFY: Append values to record.
        recorded_positions["GPS x"].append(gps_x)
        recorded_positions["GPS y"].append(gps_y)
        recorded_positions["Actual x service"].append(robot_coordinates.pose.position.x)
        recorded_positions["Actual y service"].append(robot_coordinates.pose.position.y)
        recorded_positions["Time"].append(rospy.get_time()-start_time)

        rate.sleep()
        i = i + 1

        if (robot_coordinates.pose.position.y > 120):
            break

    #Stop the test speed pattern
    set_wheel_velocity(0)

    plt.figure(figsize=(14, 6), dpi=120)

    # MODIFY: Values to plot.
    plt.subplot(1, 2, 1) # row 1, col 2 index 1
    plt.plot(recorded_positions["Time"], recorded_positions["GPS x"], color="red", alpha=0.7, label="GPS pos")
    plt.plot(recorded_positions["Time"], recorded_positions["Actual x service"], color="black", label="Actual pos")
    plt.title("X-Distance")
    plt.ylabel("X pos (m)")
    plt.xlabel('Time (sec)')
    plt.legend()

    plt.subplot(1, 2, 2) # index 2
    plt.plot(recorded_positions["Time"], recorded_positions["GPS y"], color="red", alpha=0.7, label="GPS pos")
    plt.plot(recorded_positions["Time"], recorded_positions["Actual y service"], color="black", label="Actual pos")
    plt.title("Y-Distance")
    plt.ylabel("Y pos (m)")
    plt.xlabel('Time (sec)')
    plt.legend()

    plt.show()