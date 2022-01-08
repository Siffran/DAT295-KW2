import tf
import math
import haversine as hs
from haversine import Unit

# MODIFY: add your new sensor here. Either as function or class. Depending whats possible.

def read_imu_acc(data):
    """
    IMU value conversion from quaternion to accelertion in x or y direction.

    Input:
    Imu data, datatype of ros, get by Subscriber
    Return:
    int acc_x, acc_y
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
    acc_x = (data.linear_acceleration.x + 9.81 * math.sin(pitch)) * math.cos(pitch)
    acc_y = (data.linear_acceleration.y - 9.81 * math.sin(roll))  * math.cos(roll)

    return acc_x, acc_y

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
    origin = (0, 0)
    current_location_x = (data.latitude, 0)
    pos_x_gps = hs.haversine(origin, current_location_x, unit=Unit.METERS)

    current_location_y = (0, data.longitude)
    pos_y_gps = hs.haversine(origin, current_location_y, unit=Unit.METERS)

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

        if tmp_degree >= 0 and tmp_degree <= self.prev_degree:  # 2nd quadrant
            self.degree = 180 - tmp_degree
        elif tmp_degree < 0 and tmp_degree <= self.prev_degree: # 3rd quadrant
            self.degree = 180 - tmp_degree
        elif tmp_degree < 0 and tmp_degree > self.prev_degree:  # 4th quadrant
            self.degree = 360 + tmp_degree
        elif tmp_degree >= 0 and tmp_degree > self.prev_degree: # 1st quadrant
            self.degree = tmp_degree

        self.prev_degree = tmp_degree