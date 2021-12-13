#!/usr/bin/env python
# -*- coding: utf-8 -*-

from sensor_msgs.msg import LaserScan, Imu
from gazebo_msgs.srv import GetModelState
import rospy
import message_filters


def callback(laser, imu):
	
	global n, flag
	
	'''
	### Possible extension: 

	Look at 3 angles instead of 1. Lets call them 1 2 3.
	2 is the angle we currently use at angle 0.
	1 would be to the left of 2.
	3 would be to the right of 2.

	This way we could adapt to directional changes of the robot.
	if we "trigger" in order 1 2 3. We can to the counter.
	If we "trigger" in order 3 2 1. We cant subtract from the counter.

	When we want to know how many poles we've passed from our starting position we simply take the abs-value of our counter :D

	### Possible extenstion:

	Incorperate IMU and odometry sensors to "map" or provide further accuracy of the pole positions.

	Example 
		-> laser scan trigger (pole detection)
		-> Where are we? -> Look at IMU (and) Odometry sensor data

	'''

	# Read the range matrix of the laser scan sensor. The range matrix contains 720 samples distributed in 
	# the angle range of -pi/2 to pi/2. The range[360] corresponds to the distance of the obstacle in the 
	# scan angle 0, which is x direction. Therefore, when range[360] is lower than 5, the robot is passing a pole.
	distance = laser.ranges[360]

	if (distance<5) & (flag):

		# Get gazebo (real) position
		model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
		robot_coordinates = model_state('robot', '')

		n = n+1
		flag = False

		polesPassed(n) # Output the pole number that the robot has passed
		print("Acceleration from IMY Y: ", imu.linear_acceleration.y)
		print("Estimated Position Y: TODO")
		print("Actual Position Y: ", robot_coordinates.pose.position.y)
		print()
		
	if (flag==False) & (distance>5):
		flag = True
		
def polesPassed(poles):
	print("Number of poles passed: ", poles)

if __name__ == '__main__':
	global n,flag
	n = 0
	flag = True

	rospy.init_node('sensor_listener', anonymous=True)

	laser_sub = message_filters.Subscriber('/rrbot/laser/scan', LaserScan)
	imu_sub = message_filters.Subscriber('/imu', Imu)

	ts = message_filters.TimeSynchronizer([laser_sub, imu_sub], 10)
	ts.registerCallback(callback)

	# Get initial position (use this as offset later?)
	model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
	robot_coordinates = model_state('robot', '')

	polesPassed(n)
	print("Starting Position Y: ", robot_coordinates.pose.position.y)
	print("Estimated Position Y: TODO")
	print()
	
	rospy.spin()

