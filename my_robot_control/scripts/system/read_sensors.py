import tf
import math
import haversine as hs
from haversine import Unit
#import traceback

# MODIFY: add your new sensor here. Either as function or class. Depending whats possible.


class Laser:
	def __init__(self):
		#Hardcoded pole map
		#Clean world
		self.pole_pos = [(-5,0), (-5,60), (-5,120), (-5,180), (-5, 240), (-5,300), (-5,360), (-5,420), (-5,480), (-5,540), (-5,600)]
		#Noisy world
		#Pole 1: x-pos: , y-pos:
		self.pole_pos_noise = []
		self.poles_passed = -1
		self.laser_x_pos = 0
		self.laser_y_pos = 0
		self.middle_flag = True
		self.back_flag = True
		self.front_flag = True
		self.direction = None
		self.last_pole_info = None
	
	def get_passed_poles(self):
		return self.poles_passed

	def get_pos(self):
		return self.laser_x_pos, self.laser_y_pos

	def update(self, data):
		# laser.ranges[720]
		# laser.ranges[0]
		# float('inf') 
		####
		intensities = data.intensities
		data = data.ranges
		# Read the range matrix of the laser scan sensor. The range matrix contains 720 samples distributed in 
		# the angle range of -pi/2 to pi/2. The range[360] corresponds to the distance of the obstacle in the 
		# scan angle 0, which is x direction. Therefore, when range[360] is lower than 5, the robot is passing a pole.
		front_distance = min(data[310:330]) # since there is more of an angle compared to the middle, sometimes the polling is out of sync and we miss the pole.
		middle_distance = min(data[350:370]) # bigger range in case of big speed
		back_distance = min(data[390:410]) # look at a few rays as to not miss the pole when going fast
		pole_distance = 5 # pole at 90 degrees
		side_threshhold = 5.6 # pole at 90 +- 40 degrees
		min_distance = 2 # anything closer than 2m is discarded. Sometimes the robot and the sensor freak out and this is mostly to filter out that.
		min_width = 7
		max_width = 10 # One standard pole covers the angle of average 8 laser beams. Considering the detect angle and distance, we assume 7~10 is a proper range for a pole

		if (front_distance > min_distance) and (front_distance < side_threshhold) and self.front_flag:
			self.front_flag = False
		if self.last_pole_info == None:
			pass
		elif self.last_pole_info == "middle":
			self.direction = "reverse"	
		elif self.last_pole_info == "back":
			self.direction == "forward" ## Last pole seen was the one behind when leaving last pole
		elif self.last_pole_info == "front": ## if robot switch directions between poles
			if self.direction == "forward":
				self.direction = "reverse"
			else: 
				self.direction = "forward"
		self.last_pole_info = "front"
		
		if (back_distance > min_distance) and (back_distance < side_threshhold) and self.back_flag:
			self.back_flag = False
			if self.last_pole_info == None:
				pass
			elif self.last_pole_info == "middle":
				self.direction = "forward"
			elif self.last_pole_info == "front":
				self.direction == "reverse" ## Last pole seen was the one in front when leaving last pole
			elif self.last_pole_info == "back": ## if robot switch directions between poles
				if self.direction == "forward":
					self.direction = "reverse"
				else: 
					self.direction = "forward"
			self.last_pole_info = "back"
			
		if (middle_distance > min_distance) and (middle_distance < pole_distance) & (self.middle_flag):
			self.middle_flag = False
			if self.last_pole_info == None:
				# This is a bit of an edge-case. This should only happen if you start next to a pole.
				# Lets assume you always start by going forwards.
				self.direction = "forward"
			elif self.last_pole_info == "front":
				self.direction = "forward"
			elif self.last_pole_info == "back":
				self.direction = "reverse"
			elif self.last_pole_info == "middle":
				if self.direction == "forward":
					self.direction = "reverse"
				else: 
					self.direction = "forward"
			self.last_pole_info = "middle"
			
			# detect whether the detected object is a tree or not: Approximate the width of the detected object according to the laser angle it covers at a time. Use this to rule out trees and tunnel
			count = False
			width = 0
			detected_distance = 6 # we define 6 as the minimum distance that the laser scan approaches the pole
			for a in data[330:380]:
				if a < detected_distance :
					count = True
					width = width + 1
				if (a > detected_distance) and (count):
					break
			if (width >= min_width) and (width <= max_width):

				#print("Acceleration from IMY Y: ", imu.linear_acceleration.y)
				if self.direction == "forward":
					self.poles_passed = self.poles_passed+1
				elif self.direction == "reverse": 
					self.poles_passed = self.poles_passed-1
				else:
					pass # Maybe tie into the IMU here to get the direction

		if (not self.middle_flag) and (middle_distance > pole_distance):
			self.middle_flag = True

		if (not self.front_flag) and (front_distance > side_threshhold):
			self.front_flag = True
		
		if (not self.back_flag) and (back_distance > side_threshhold):
			self.back_flag = True 
		####
		# Tuples of visible poles, (angle, distance)
		# Angle 0 is back, 360 is left, 720 is forward as seen if the cart is traveling forwards
		dist, angle = 0, 0
		found = False
		if self.direction == "forward":
			for i, ray in enumerate(data): #i = degree, ray = length/distance
				if intensities[i] == 200: #poles have intensity of 200 (hardcoded in model)
					found = True
					dist, angle = ray, (i/4) - 90 # to center the robot on the range, 90 directly to left (realistic)
		elif self.direction == "reverse":
			for i, ray in enumerate(data[::-1]): #i = degree, ray = length/distance
				if intensities[i] == 200: #poles have intensity of 200 (hardcoded in model)
					found = True
					dist, angle = ray, (i/4) - 90 # to center the robot on the range, 90 directly to left (realistic)
		else:
			print("Should never happen lmao xD")
		for i, ray in enumerate(data): #i = degree, ray = length/distance
			if intensities[i] == 200: #poles have intensity of 200 (hardcoded in model)
				found = True
				dist, angle = ray, (i/4) - 90 # to center the robot on the range, 90 directly to left (realistic)
		
		if angle < 0: # pole we see is the next pole (we have not passed it yet so +1 and then -dist)
			laser_y_pos = self.pole_pos[self.poles_passed+1][1] + (math.cos(math.radians(90-angle))*dist)
			laser_x_pos = self.pole_pos[self.poles_passed+1][1] + (math.sin(math.radians(90-angle))*dist)


		else:
			laser_y_pos = self.pole_pos[self.poles_passed][1] + (math.cos(math.radians(90-angle))*dist)
			laser_x_pos = self.pole_pos[self.poles_passed][1] + (math.sin(math.radians(90-angle))*dist)

		if not found:
			print("Could not find a pole.")
		
		#self.laser_x_pos = laser_x_pos Changes pos in x,to do
		if (abs(laser_y_pos - self.laser_y_pos) < 5): #incase of a major change in pos, discard it (sensor error or similar)
			self.laser_y_pos = laser_y_pos
		if (abs(laser_x_pos - self.laser_x_pos) < 5): #incase of a major change in pos, discard it (sensor error or similar)
			self.laser_x_pos = laser_x_pos - pole_distance


class IMU:

	def __init__(self):
		self.prev_time = 0
		self.imu_x_vel = 0
		self.imu_x_pos = 0
		self.imu_x_acc = 0
		self.imu_y_vel = 0
		self.imu_y_pos = 0
		self.imu_y_acc = 0

	def update(self, data, time):
		"""
		IMU value conversion from quaternion to accelertion in x or y direction.

		Input:
		Imu data, datatype of ros, get by Subscriber
		"""
		# transform from quaternion to euler angles
		quaternion = (
			data.orientation.x,
			data.orientation.y,
			data.orientation.z,
			data.orientation.w)
		euler = tf.transformations.euler_from_quaternion(quaternion)
		roll = euler [0]
		pitch = euler[1]

		# calculate actual acceleration from linear_acc by considering gravity
		self.imu_x_acc = (data.linear_acceleration.x + 9.81 * math.sin(pitch)) * math.cos(pitch)
		self.imu_y_acc = (data.linear_acceleration.y - 9.81 * math.sin(roll))  * math.cos(roll)

		# Calculate time step since last measurement
		dt = time - self.prev_time
		self.prev_time = time

		# Calculate vel and pos by integrating acc
		Velocity_old_x = self.imu_x_vel
		self.imu_x_vel = Velocity_old_x+self.imu_x_acc*dt
		self.imu_x_pos += self.imu_x_vel*dt

		Velocity_old_y = self.imu_y_vel
		self.imu_y_vel = Velocity_old_y+self.imu_y_acc*dt
		self.imu_y_pos += self.imu_y_vel*dt

	def set_prev_time(self, time):
		self.prev_time = time

	def get_pos(self):
		return self.imu_x_pos, self.imu_y_pos

	def get_acc(self):
		return self.imu_x_acc, self.imu_y_acc

def read_actual_pos(data):
	"""
	Actual position value extraction from data.

	Input:
	ModelStates data, datatype of ros, get by Subscriber
	Return:
	int pos_x_actual, pos_y_actual
	"""
	pos_x_actual = data.pose[data.name.index('robot')].position.x
	pos_y_actual = data.pose[data.name.index('robot')].position.y
	return pos_x_actual, pos_y_actual



def read_gps_pos(data):
	"""
	GPS position value extraction from data.

	Input:
	NavSatFix data, datatype of ros, get by Subscriber
	Return:
	int pos_x_gps, pos_y_gps
	"""

	origin = (57.70887, 11.97456)	#coordinates of GBG
	current_location_x = (data.latitude, 11.97456)
	pos_x_gps = hs.haversine(origin, current_location_x, unit=Unit.METERS)

	current_location_y = (57.70887, data.longitude)
	pos_y_gps = hs.haversine(origin, current_location_y, unit=Unit.METERS)

	if data.latitude < 57.70887:
		pos_x_gps = -pos_x_gps
	
	if data.longitude > 11.97456:
		pos_y_gps = -pos_y_gps


	return pos_x_gps, pos_y_gps

class WheelEncoder:
	"""
	Creates instance of Wheel Encoder.
	
	:class: 
	"""    

	def __init__(self):
		self.x = 0
		self.y = 0
		self.direction = 0 # Direction of travel (radians)
		self.wheel_radius = 0.1 # Radius at center of the cone
		self.degree = 270 # Wheel starts with -90 degrees offset
		self.prev_degree = 270  
		self.prev_tmp_degree = 270  
		self.part_circ = 0
		self.circumfence = 2*math.pi*self.wheel_radius
		#self.prev_part_circ = 0

	def update(self, data):
		# transform from quaternion to euler angles
		self.update_direction(data)
		pose = data.pose[data.name.index('robot::wheel_1')]
		quaternion = (
			pose.orientation.x,
			pose.orientation.y,
			pose.orientation.z,
			pose.orientation.w)
		euler = tf.transformations.euler_from_quaternion(quaternion)
		pitch = euler[1]
		
		self.pitch_to_degree(pitch)

		# TODO: This only works forwards.
		#       Find parameter to differentiate between forward and backward.
		if self.degree > self.prev_degree:
			dist = self.circumfence * (self.degree/360)\
				- self.circumfence * (self.prev_degree/360)
		elif self.degree < self.prev_degree:
			dist = self.circumfence * (self.degree/360)\
				+ self.circumfence * ((360 - self.prev_degree)/360)
		else:
			dist = 0
		
		self.prev_degree = self.degree
		# # if full wheel turn
		# if self.prev_part_circ > self.part_circ:
		#     dist = 2*math.pi*self.wheel_radius \
		#             + self.part_circ \
		#                 -2*math.pi*self.wheel_radius/4
			
		self.x += dist * math.sin(-self.direction)
		self.y += dist * math.cos(self.direction)

		
	def update_direction(self, data):
		pose = data.pose[data.name.index('robot::base_link')]
		quaternion = (
			pose.orientation.x,
			pose.orientation.y,
			pose.orientation.z,
			pose.orientation.w)
		euler = tf.transformations.euler_from_quaternion(quaternion)
		self.direction = euler[2]

	def get_pos(self):
		return self.x, self.y

	def pitch_to_degree(self, pitch):
		"""
		Conversion of of pitch into degrees.
		"""
		tmp_degree = (pitch*180)/math.pi

		if tmp_degree >= 0 and tmp_degree <= self.prev_tmp_degree:  # 2nd quadrant
			self.degree = 180 - tmp_degree
		elif tmp_degree < 0 and tmp_degree <= self.prev_tmp_degree: # 3rd quadrant
			self.degree = 180 - tmp_degree
		elif tmp_degree < 0 and tmp_degree > self.prev_tmp_degree:  # 4th quadrant
			self.degree = 360 + tmp_degree
		elif tmp_degree >= 0 and tmp_degree > self.prev_tmp_degree: # 1st quadrant
			self.degree = tmp_degree

		self.prev_tmp_degree = tmp_degree