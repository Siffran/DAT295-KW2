#!/usr/bin/env python
# -*- coding: utf-8 -*-

from sensor_msgs.msg import LaserScan, Imu
from gazebo_msgs.srv import GetModelState
import rospy
import message_filters


#def callback(laser, imu):
def callback(laser):	
	global n,flag,front_flag,back_flag,direction,last_pole_info
	
	'''
	### Possible extenstion:

	Incorperate IMU and odometry sensors to "map" or provide further accuracy of the pole positions.

	Example 
		-> laser scan trigger (pole detection)
		-> Where are we? -> Look at IMU (and) Odometry sensor data

	'''

	# Read the range matrix of the laser scan sensor. The range matrix contains 720 samples distributed in 
	# the angle range of -pi/2 to pi/2. The range[360] corresponds to the distance of the obstacle in the 
	# scan angle 0, which is x direction. Therefore, when range[360] is lower than 5, the robot is passing a pole.
	front_distance = min(laser.ranges[310:330]) # since there is more of an angle compared to the middle, sometimes the polling is out of sync and we miss the pole.
	middle_distance = min(laser.ranges[350:370]) # bigger range in case of big speed
	back_distance = min(laser.ranges[390:410]) # look at a few rays as to not miss the pole when going fast
	pole_distance = 5.0 # pole at 90 degrees
	side_threshhold = 5.1 # pole at 90 +- 40 degrees
	min_distance = 2 # anything closer than 2m is discarded. Sometimes the robot and the sensor freak out and this is mostly to filter out that.

	#if front_distance != float('inf'):
	#	print(f"Front distance: {front_distance}")
	#if middle_distance != float('inf'):
	#	print(f"Middle distance: {middle_distance}")
	#if back_distance != float('inf'):
	#	print(f"Back distance: {back_distance}")

	if (front_distance > min_distance) and (front_distance < side_threshhold) and front_flag:
		print(f"Front trigger: {front_distance}")
		print(f"Pole in front: LPINFO: {last_pole_info}.")
		front_flag = False
		if last_pole_info == None:
			pass
		elif last_pole_info == "middle":
			direction = "reverse"
			print(f"Pole in front: LPINFO: {last_pole_info}. DIRECTION NOW: {direction}")
		elif last_pole_info == "back":
			direction == "forward" ## Last pole seen was the one behind when leaving last pole
			print(f"Pole in front: LPINFO: {last_pole_info}. DIRECTION NOW: {direction}")
		elif last_pole_info == "front": ## if robot switch directions between poles
			if direction == "forward":
				direction = "reverse"
			else: 
				direction = "forward"
			print(f"Pole in front: LPINFO: {last_pole_info}. DIRECTION NOW: {direction}")
		last_pole_info = "front"
	
	if (back_distance > min_distance) and (back_distance < side_threshhold) and back_flag:
		print(f"Back trigger: {back_distance}")
		print(f"Pole in back: LPINFO: {last_pole_info}. DIRECTION NOW: {direction}")
		back_flag = False
		if last_pole_info == None:
			pass
		elif last_pole_info == "middle":
			direction = "forward"
			print(f"Pole in back: LPINFO: {last_pole_info}. DIRECTION NOW: {direction}")
		elif last_pole_info == "front":
			direction == "reverse" ## Last pole seen was the one in front when leaving last pole
			print(f"Pole in back: LPINFO: {last_pole_info}. DIRECTION NOW: {direction}")
		elif last_pole_info == "back": ## if robot switch directions between poles
			if direction == "forward":
				direction = "reverse"
			else: 
				direction = "forward"
			print(f"Pole in back: LPINFO: {last_pole_info}. DIRECTION NOW: {direction}")
		last_pole_info = "back"
		
	if (middle_distance > min_distance) and (middle_distance < pole_distance) & (flag):
		print(f"Middle trigger: {middle_distance}")
		print(f"Pole in middle: LPINFO: {last_pole_info}. DIRECTION NOW: {direction}")
		flag = False
		if last_pole_info == None:
			# This is a bit of an edge-case. This should only happen if you start next to a pole.
			# Lets assume you always start by going forwards.
			direction = "forward"
			print(f"EDGE CASE: Pole in middle: LPINFO: {last_pole_info}. DIRECTION NOW: {direction}")
		elif last_pole_info == "front":
			direction = "forward"
			print(f"Pole in middle: LPINFO: {last_pole_info}. DIRECTION NOW: {direction}")
		elif last_pole_info == "back":
			direction = "reverse"
			print(f"Pole in middle: LPINFO: {last_pole_info}. DIRECTION NOW: {direction}")
		elif last_pole_info == "middle":
			if direction == "forward":
				direction = "reverse"
			else: 
				direction = "forward"
			print(f"Pole in middle: LPINFO: {last_pole_info}. DIRECTION NOW: {direction}")
		last_pole_info = "middle"
		
		# Get gazebo (real) position
		model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
		robot_coordinates = model_state('robot', '')
		if direction == "forward":
			n = n+1
		elif direction == "reverse": 
			n = n-1
		else:
			print("Could not figure out the direction")
			pass # Maybe tie into the IMU here to get the direction
		polesPassed(n) # Output the pole number that the robot has passed
		#print("Acceleration from IMY Y: ", imu.linear_acceleration.y)
		print("Estimated Position Y: TODO")
		print("Actual Position Y: ", robot_coordinates.pose.position.y)
		print(f"Direction: {direction}")
		print()
		
	if (not flag) and (middle_distance > pole_distance):
		print(f"Flag released: {middle_distance}")
		flag = True

	if (not front_flag) and (front_distance > side_threshhold):
		print(f"front flag released: {front_distance}")
		front_flag = True
	
	if (not back_flag) and (back_distance > side_threshhold):
		print(f"back flag released: {back_distance}")
		back_flag = True
		
def polesPassed(poles):
	print("Number of poles passed: ", poles)

if __name__ == '__main__':
	global n,flag,front_flag,back_flag,direction,last_pole_info
	n = 0
	flag = True
	front_flag = True
	back_flag = True
	direction = None
	last_pole_info = None

	rospy.init_node('sensor_listener', anonymous=True)
	rospy.Subscriber('/rrbot/laser/scan', LaserScan, callback, queue_size = 1024)

	#laser_sub = message_filters.Subscriber('/rrbot/laser/scan', LaserScan)
	#imu_sub = message_filters.Subscriber('/imu', Imu)

	#ts = message_filters.TimeSynchronizer([laser_sub, imu_sub], 10)
	#ts.registerCallback(callback)



	# Get initial position (use this as offset later?)
	model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
	robot_coordinates = model_state('robot', '')

	polesPassed(n)
	print("Starting Position Y: ", robot_coordinates.pose.position.y)
	print("Estimated Position Y: TODO")
	print(f"Direction: {direction}")
	print()
	
	rospy.spin()

