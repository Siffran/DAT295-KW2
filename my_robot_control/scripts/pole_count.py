#!/usr/bin/env python
# -*- coding: utf-8 -*-

from sensor_msgs.msg import LaserScan
import rospy
def callback(data):
	global n,flag
	# output the pole number that the robot has passed
	print(n)
	#read the range matrix of the laser scan sensor. The range matrix contains 720 samples distributed in the angle range of -pi/2 to pi/2. The range[360] corresponds to the distance of the obstacle in the scan angle 0, which is x direction. Therefore, when range[360] is lower than 5, the robot is passing a pole.
	
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

	distance = data.ranges[360]
	if (distance<5) & (flag):
		n = n+1
		flag = False
	if (flag==False) & (distance>5):
		flag = True
		

if __name__ == '__main__':
	global n,flag
	n = 0
	flag = True
	rospy.init_node('laser_listener', anonymous=True)
	rospy.Subscriber('/rrbot/laser/scan', LaserScan, callback)
	rospy.spin()

