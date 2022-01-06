from sensor_msgs.msg import LaserScan, Imu, NavSatFix
from gazebo_msgs.srv import GetModelState
import rospy
import message_filters

import haversine as hs
from haversine import Unit

def callback(gps):
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

if __name__ == '__main__':
    rospy.init_node('sensor_listener', anonymous=True)

    #laser_sub = message_filters.Subscriber('/rrbot/laser/scan', LaserScan)
    #imu_sub = message_filters.Subscriber('/imu', Imu)
    gps_sub = message_filters.Subscriber('/gps', NavSatFix)

    ts = message_filters.TimeSynchronizer([gps_sub], 10)
    ts.registerCallback(callback)

    # Get initial position (use this as offset later?)
    model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    robot_coordinates = model_state('robot', '')

    print("Starting Position Y: ", robot_coordinates.pose.position.y)
    print("Estimated Position Y: TODO")
    print()

    rospy.spin()