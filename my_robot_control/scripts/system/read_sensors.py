import tf
import math

def read_imu_acc(data):
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
    pos_x_actual = data.pose[data.name.index('robot')].position.x
    pos_y_actual = data.pose[data.name.index('robot')].position.y
    return pos_x_actual, pos_y_actual

class WheelEncoder:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.direction = 0 # Directinon of travel (radians)
        self.wheel_radius = 0.1 # Radius at center of the cone
        self.degree = 0
        self.prev_degree = 180  
        self.part_circ = 0
        self.prev_part_circ = 0

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
        self.part_circ = 2*math.pi*self.wheel_radius* (self.degree/360)

        # if full wheel turn
        if self.prev_part_circ > self.part_circ:
            dist = 2*math.pi*self.wheel_radius \
                    + self.part_circ \
                        -2*math.pi*self.wheel_radius/4
            
            self.x += dist * math.sin(-self.direction)
            self.y += dist * math.cos(self.direction)

        self.prev_part_circ = self.part_circ
        
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
        tmp_degree = (pitch*180)/math.pi

        if tmp_degree >= 0 and tmp_degree <= self.prev_degree:
            self.degree = 180 - tmp_degree
        elif tmp_degree < 0 and tmp_degree <= self.prev_degree:
            self.degree = 180 - tmp_degree
        elif tmp_degree < 0 and tmp_degree > self.prev_degree:
            self.degree = 360 + tmp_degree
        elif tmp_degree >= 0 and tmp_degree > self.prev_degree:
            self.degree = tmp_degree

        self.prev_degree = tmp_degree