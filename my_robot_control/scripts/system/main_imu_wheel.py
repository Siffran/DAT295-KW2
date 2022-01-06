from control_robot import set_wheel_velocity
from LocalizationEstimator import LocalizationEstimator
from read_sensors import read_imu_acc, read_actual_pos, WheelEncoder
from sensor_msgs.msg import Imu
import rospy
import message_filters
import gazebo_msgs.msg
import numpy as np
from matplotlib import pyplot as plt


wheel_encoder = WheelEncoder()
imu_x_acc, imu_y_acc, \
    wheel_x, wheel_y, \
    actual_x, actual_y = 0, 0, 0, 0, 0, 0
def callback(imu_data, wheel_data, actual_data):
    global imu_x_acc, imu_y_acc, wheel_x, wheel_y, actual_x, actual_y
    imu_x_acc, imu_y_acc = read_imu_acc(imu_data)
    wheel_encoder.update(wheel_data)
    wheel_x, wheel_y = wheel_encoder.get_pos()
    actual_x, actual_y = read_actual_pos(actual_data)
    
if __name__ == "__main__":
    ### Setup
    RATE_Hz = 50
    rospy.init_node('control', anonymous=True)
    rate = rospy.Rate(RATE_Hz)
    velocity = int(input("Input robot velocity\n"))
    print("Setting velocity to {}".format(velocity))
    set_wheel_velocity(velocity)

    imu_sub = message_filters.Subscriber('/imu', Imu)
    wheel_sub = message_filters.Subscriber('/gazebo/link_states', gazebo_msgs.msg.LinkStates)
    actual_sub = message_filters.Subscriber('/gazebo/model_states', gazebo_msgs.msg.ModelStates)

    ts = message_filters.ApproximateTimeSynchronizer([imu_sub, wheel_sub, actual_sub], 10, 0.02, True)
    ts.registerCallback(callback)

    loc_est = LocalizationEstimator("imu+wheel", RATE_Hz)
    steps = 100
    i = 0

    recorded_positions = {
        "EKF x":[],
        "EKF y":[],
        "Wheel x":[],
        "Wheel y":[],
        "Actual x":[],
        "Actual y":[]
    }


    while i <= steps:

        z = np.array([wheel_x, wheel_y, imu_x_acc, imu_y_acc])
        loc_est.update(z)
        loc_est.predict()
        state_vector = loc_est.get_state_vector()
        recorded_positions["EKF x"].append(state_vector[0])
        recorded_positions["EKF y"].append(state_vector[3])
        recorded_positions["Wheel x"].append(wheel_x)
        recorded_positions["Wheel y"].append(wheel_y)
        recorded_positions["Actual x"].append(actual_x)
        recorded_positions["Actual y"].append(actual_y)
        rate.sleep()
        i += 1

    # stop robot
    set_wheel_velocity(0)
    plt.plot(recorded_positions["EKF x"], recorded_positions["EKF y"], label="EKF pos")
    plt.plot(recorded_positions["Wheel x"], recorded_positions["Wheel y"], label="Wheel pos")
    plt.plot(recorded_positions["Actual x"], recorded_positions["Actual y"], label="Actual pos")
    plt.xlabel("x")
    plt.ylabel("y")
    plt.title("EKF values")
    plt.legend()
    plt.show()
        
    
