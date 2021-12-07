#!/usr/bin/env python
# -*- coding: utf-8 -*-

from sensor_msgs.msg import LaserScan
import rospy
def callback(data):
	global n,flag
	# output the pole number that the robot has passed
	print(n)
	#read the range matrix of the laser scan sensor. The range matrix contains 720 samples distributed in the angle range of -pi/2 to pi/2. The range[360] corresponds to the distance of the obstacle in the scan angle 0, which is x direction. Therefore, when range[360] is lower than 5, the robot is passing a pole.
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

