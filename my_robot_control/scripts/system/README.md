
# Overview
**control_robot.py**

self explanatory

**read_sensors.py**

Collection of functions to read from sensors

**main_imu_wheel.py**

We figured it would be most simple to have one main file for each system configuration. This one uses imu acceleration and wheel encoder.

**LocationEstimator.py**

Setup of your EKF.



# How to create a system configuration
We have created a template that can be adjusted for each sensor configuration. It's not overengineered to be very dynamic and requires the author to write new code for each configuration. 

1. Create a new main file, i.e. *main_wheel_gps.py*
2. Ensure you have the correct rostopic subscribers i.e. *imu_sub = message_filters.Subscriber('/imu', Imu)* and pass them to the TimeSynchronizer.
3. In the *Callback* function, use the correct sensor reading functions from *read_sensors.py* (you might have to make them if they're not present in there)
4. Create a new "setup-function" such as the *setup_imu_wheel_EKF* function in *LocationEstimator.py*. This is the tricky part. It will need a bunch of arrays, jacobian and measurement functions that suit the selected sensor configuration.
5. Collect the correct data into the *recorded_positions* dictionary and plot the correct values.
6. Adjust the z array in the while-loop to match with your *LocalizationEstimator*.


# Tips
Modify the "steps" variable to increase/decrease the run-time when testing your configuration