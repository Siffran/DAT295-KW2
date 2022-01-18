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
		#self.pole_pos = [(-5,0), (-5,60), (-5,120), (-5,180), (-5, 240), (-5,300), (-5,360), (-5,420), (-5,480), (-5,540), (-5,600)]
		#Noisy world
		self.pole_pos = [(-5,0), (-5,60), (11,120), (-5,180), (-5, 240), (-5,300), (-5,360), (-5,420), (-5,480), (-5,540), (-5,600)]
		#Pole 1: x-pos: , y-pos:
		self.poles_passed = 0
		self.laser_x_pos = 0
		self.laser_y_pos = 0
		self.middle_flag = True
		self.back_flag = True
		self.front_flag = True
		self.direction = "forward"
		self.flag = False
		self.steering_angle = 0
		self.pole_detected = False
	def update_steering_angle(self, data):
		pose = data.pose[data.name.index('robot::base_link')]
		quaternion = (
			pose.orientation.x,
			pose.orientation.y,
			pose.orientation.z,
			pose.orientation.w)
		euler = tf.transformations.euler_from_quaternion(quaternion)
		self.steering_angle = euler[2]

	def get_passed_poles(self):
		return self.poles_passed

	def get_pos(self):
		return self.laser_x_pos, self.laser_y_pos

	def update(self, data, vehicle_data, velocity):
		self.direction = "forward" if velocity > 0 else "reverse" 

		self.update_steering_angle(vehicle_data)
		intensities = data.intensities
		data = data.ranges

		# Pole counting
		front = 360-4*int(math.degrees(self.steering_angle))
		back = 400-4*int(math.degrees(self.steering_angle))
		ray_range = enumerate(data[front:back])
		for i, _ in ray_range:
			if intensities[i+front] == 200:
				if self.direction == "forward" and not self.pole_detected:
					self.pole_detected = True
					self.poles_passed += 1
				elif self.direction == "reverse" and not self.pole_detected:
					self.pole_detected = True
					self.poles_passed -= 1
				break
			elif intensities[i+back] == 200:
				if self.direction == "forward" and self.pole_detected:
					self.pole_detected = False
				elif self.direction == "reverse" and self.pole_detected:
					self.pole_detected = False
				break

		# Angle 0 is forward, 360 is left, 720 is back as seen if the cart is traveling forwards
		dist, angle = 0, 0
		found = False
		if self.direction == "forward":
			for i, ray in enumerate(data): #i = degree, ray = length/distance
				if intensities[i] == 200: #poles have intensity of 200 (hardcoded in model)
					found = True
					dist, angle = ray, (i-360)/4 # to center the robot on the range
		elif self.direction == "reverse":
			for i, ray in enumerate(data[::-1]): #i = degree, ray = length/distance
				if intensities[i] == 200: #poles have intensity of 200 (hardcoded in model)
					found = True
					dist, angle = ray, (i-360)/4  # to center the robot on the range

		angle = angle+math.degrees(self.steering_angle)
		if angle <= 0: # pole we see is the next pole (we have not passed it yet so +1)
			laser_y_pos = self.pole_pos[self.poles_passed][1] + (math.sin(math.radians(angle))*dist)
			laser_x_pos = self.pole_pos[self.poles_passed][0] + (math.cos(math.radians(angle))*dist)
			print("pole:{}, Y:{}".format(self.poles_passed, self.laser_y_pos, laser_y_pos))

		else:
			laser_y_pos = self.pole_pos[self.poles_passed-1][1] + (math.sin(math.radians(angle))*dist)
			laser_x_pos = self.pole_pos[self.poles_passed-1][0] + (math.cos(math.radians(angle))*dist)
			print("pole:{}, Y:{}".format(self.poles_passed, self.laser_y_pos, laser_y_pos))
		if not found:
			print("Could not find a pole.")
			
		
		if (abs(laser_y_pos - self.laser_y_pos) < 5): #incase of a major change in pos, discard it (sensor error or similar)
			self.laser_y_pos = laser_y_pos
		if (abs(laser_x_pos - self.laser_x_pos) < 5): #incase of a major change in pos, discard it (sensor error or similar)
			self.laser_x_pos = laser_x_pos
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
