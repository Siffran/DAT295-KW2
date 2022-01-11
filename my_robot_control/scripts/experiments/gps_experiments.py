from sensor_msgs.msg import LaserScan, Imu, NavSatFix
from gazebo_msgs.srv import GetModelState
import rospy
import message_filters
import gazebo_msgs.msg

import sys
sys.path.append('../system')
from read_sensors import read_actual_pos, read_gps_pos
from control_robot import set_wheel_velocity

import haversine as hs
from haversine import Unit

import numpy as np
from matplotlib import pyplot as plt

def callback(gps_data, actual_data):

    '''
    # Get gazebo (real) position
    model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    robot_coordinates = model_state('robot', '')

    print('GPS latitude: ', gps.latitude)
    print('GPS longitude: ', gps.longitude)
    
    origin = (0, 0)
    current_location = (gps.latitude, gps.longitude)
    
    distance = hs.haversine(origin, current_location, unit=Unit.METERS)

    print("Estimated distance : ", distance)
    print("Actual Position Y: ", robot_coordinates.pose.position.y)
    print()
    '''

    global gps_x, gps_y, actual_x, actual_y

    gps_x, gps_y = read_gps_pos(gps_data)
    actual_x, actual_y = read_actual_pos(actual_data)



if __name__ == '__main__':
    rospy.init_node('sensor_listener', anonymous=True)

    # frequency in which the Estimator is called.
    RATE_Hz = 50 

    # Initialize rospy node and frequency
    #rospy.init_node('control', anonymous=True)
    rate = rospy.Rate(RATE_Hz)

    # TODO: dynamically changeable velocity via the terminal
    velocity = int(input("Input robot velocity\n"))
    print("Setting velocity to {}".format(velocity))
    set_wheel_velocity(velocity)

    '''
    #laser_sub = message_filters.Subscriber('/rrbot/laser/scan', LaserScan)
    #imu_sub = message_filters.Subscriber('/imu', Imu)
    gps_sub = message_filters.Subscriber('/gps', NavSatFix)

    ts = message_filters.TimeSynchronizer([gps_sub], 10)
    ts.registerCallback(callback)
    '''

    gps_sub = message_filters.Subscriber('/gps', NavSatFix)
    actual_sub = message_filters.Subscriber('/gazebo/model_states', gazebo_msgs.msg.ModelStates)

    ts = message_filters.ApproximateTimeSynchronizer([gps_sub, actual_sub], 10, 0.02, True)
    ts.registerCallback(callback)

    # Get initial position (use this as offset later?)
    model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    robot_coordinates = model_state('robot', '')


    print("Starting Position Y: ", robot_coordinates.pose.position.y)
    print("Estimated Position Y: TODO")
    print()

    #rospy.spin()

    steps = 500
    i = 0

    # MODIFY: Values to record
    recorded_positions = {
        "GPS x":[],
        "GPS y":[],
        "Actual x":[],
        "Actual y":[],
        "Actual x service":[],
        "Actual y service":[]
    }

    while i <= steps:

        model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        robot_coordinates = model_state('robot', '')

        # MODIFY: Append values to record.
        recorded_positions["GPS x"].append(gps_x)
        recorded_positions["GPS y"].append(gps_y)
        recorded_positions["Actual x"].append(actual_x)
        recorded_positions["Actual y"].append(actual_y)
        recorded_positions["Actual x service"].append(robot_coordinates.pose.position.x)
        recorded_positions["Actual y service"].append(robot_coordinates.pose.position.y)

        rate.sleep()
        i += 1

    set_wheel_velocity(0)

    # MODIFY: Values to plot.
    plt.subplot(1, 2, 1) # row 1, col 2 index 1
    plt.plot(range(0,steps+1), recorded_positions["GPS x"], color="red", label="GPS pos")
    plt.plot(range(0,steps+1), recorded_positions["Actual x"], color="black", label="Actual pos")
    plt.plot(range(0,steps+1), recorded_positions["Actual x service"], color="orange", label="Actual pos service")
    plt.title("X-Distance")
    plt.xlabel('steps')
    plt.legend()

    plt.subplot(1, 2, 2) # index 2
    plt.plot(range(0, steps+1), recorded_positions["GPS y"], color="red", label="GPS pos")
    plt.plot(range(0, steps+1), recorded_positions["Actual y"], color="black", label="Actual pos")
    plt.plot(range(0,steps+1), recorded_positions["Actual y service"], color="orange", label="Actual pos service")
    plt.title("Y-Distance")
    plt.xlabel('steps')
    plt.legend()

    plt.show()