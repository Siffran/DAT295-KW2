# basic libs
import numpy as np
from matplotlib import pyplot as plt
import csv
import pandas as pd  
from sklearn.metrics import mean_squared_error
import plotly.graph_objects as go



'''
import the data for different system from csv file
output the dataframe for mse of different systems
plot the estimation error of different systems

Before using this script, you need to run all the system script once and got the csv file
'''    
if __name__ == "__main__":

    # import all the csv file
    data = pd.read_csv("main_imu_gps.csv")
    Time_imu_gps = data.Time
    EKF_x_imu_gps = data.EKF_x
    EKF_y_imu_gps = data.EKF_y
    Actual_x_imu_gps  = data.Actual_x
    Actual_y_imu_gps  = data.Actual_y
    Error_x_imu_gps = [EKF_x_imu_gps[i] - Actual_x_imu_gps[i] for i in range(len(EKF_x_imu_gps))]
    Error_y_imu_gps = [EKF_y_imu_gps[i] - Actual_y_imu_gps[i] for i in range(len(EKF_y_imu_gps))]
    MSE_x_imu_gps = mean_squared_error(Actual_x_imu_gps, EKF_x_imu_gps)
    MSE_y_imu_gps = mean_squared_error(Actual_y_imu_gps, EKF_y_imu_gps)

        
    data = pd.read_csv("main_gps_wheel.csv")
    Time_gps_wheel = data.Time
    EKF_x_gps_wheel = data.EKF_x
    EKF_y_gps_wheel = data.EKF_y
    Actual_x_gps_wheel  = data.Actual_x
    Actual_y_gps_wheel  = data.Actual_y
    Error_x_gps_wheel = [EKF_x_gps_wheel[i] - Actual_x_gps_wheel[i] for i in range(len(EKF_x_gps_wheel))]
    Error_y_gps_wheel = [EKF_y_gps_wheel[i] - Actual_y_gps_wheel[i] for i in range(len(EKF_y_gps_wheel))]  
    MSE_x_gps_wheel = mean_squared_error(Actual_x_gps_wheel, EKF_x_gps_wheel)
    MSE_y_gps_wheel = mean_squared_error(Actual_y_gps_wheel, EKF_y_gps_wheel)  

    data = pd.read_csv("main_imu_wheel.csv")
    Time_imu_wheel = data.Time
    EKF_x_imu_wheel = data.EKF_x
    EKF_y_imu_wheel = data.EKF_y
    Actual_x_imu_wheel = data.Actual_x
    Actual_y_imu_wheel = data.Actual_y
    Error_x_imu_wheel = [EKF_x_imu_wheel[i] - Actual_x_imu_wheel[i] for i in range(len(EKF_x_imu_wheel))]
    Error_y_imu_wheel = [EKF_y_imu_wheel[i] - Actual_y_imu_wheel[i] for i in range(len(EKF_y_imu_wheel))]
    MSE_x_imu_wheel = mean_squared_error(Actual_x_imu_wheel, EKF_x_imu_wheel)
    MSE_y_imu_wheel= mean_squared_error(Actual_y_imu_wheel, EKF_y_imu_wheel)
        
    data = pd.read_csv("main_imu_wheel_gps.csv")
    Time_imu_wheel_gps = data.Time
    EKF_x_imu_wheel_gps = data.EKF_x
    EKF_y_imu_wheel_gps = data.EKF_y
    Actual_x_imu_wheel_gps = data.Actual_x
    Actual_y_imu_wheel_gps = data.Actual_y
    Error_x_imu_wheel_gps = [EKF_x_imu_wheel_gps[i] - Actual_x_imu_wheel_gps[i] for i in range(len(EKF_x_imu_wheel_gps))]
    Error_y_imu_wheel_gps = [EKF_y_imu_wheel_gps[i] - Actual_y_imu_wheel_gps[i] for i in range(len(EKF_y_imu_wheel_gps))]
    MSE_x_imu_wheel_gps = mean_squared_error(Actual_x_imu_wheel_gps, EKF_x_imu_wheel_gps)
    MSE_y_imu_wheel_gps = mean_squared_error(Actual_y_imu_wheel_gps, EKF_y_imu_wheel_gps)
            
    data = pd.read_csv("main_imu_wheel_gps_laser.csv")
    Time_imu_wheel_gps_laser = data.Time
    EKF_x_imu_wheel_gps_laser = data.EKF_x
    EKF_y_imu_wheel_gps_laser = data.EKF_y
    Actual_x_imu_wheel_gps_laser = data.Actual_x
    Actual_y_imu_wheel_gps_laser = data.Actual_y
    Error_x_imu_wheel_gps_laser = [EKF_x_imu_wheel_gps_laser[i] - Actual_x_imu_wheel_gps_laser[i] for i in range(len(EKF_x_imu_wheel_gps_laser))]
    Error_y_imu_wheel_gps_laser = [EKF_y_imu_wheel_gps_laser[i] - Actual_y_imu_wheel_gps_laser[i] for i in range(len(EKF_y_imu_wheel_gps_laser))]
    MSE_x_imu_wheel_gps_laser = mean_squared_error(Actual_x_imu_wheel_gps_laser, EKF_x_imu_wheel_gps_laser)
    MSE_y_imu_wheel_gps_laser = mean_squared_error(Actual_y_imu_wheel_gps_laser, EKF_y_imu_wheel_gps_laser)
    
    # output the mse for different systems
    mse = {
    'system':['imu_gps','gps_wheel','imu_wheel','imu_wheel_gps','imu_wheel_gps_laser'],
    'mse_x':[MSE_x_imu_gps,MSE_x_gps_wheel, MSE_x_imu_wheel, MSE_x_imu_wheel_gps, MSE_x_imu_wheel_gps_laser],
    'mse_y':[MSE_y_imu_gps,MSE_y_gps_wheel, MSE_y_imu_wheel, MSE_y_imu_wheel_gps, MSE_y_imu_wheel_gps_laser]}
    df = pd.DataFrame(data = mse)
    print(df)
    
    # plot the estimation error figure for different systems
    fig = plt.figure(figsize=(14, 6), dpi=120)   
    
    plt.subplot(1, 2, 1) # row 1, col 2 index 1    
    plt.plot(Time_imu_gps, Error_x_imu_gps, color="red", alpha=0.7, label="imu_gps")
    plt.plot(Time_gps_wheel, Error_x_gps_wheel, color="green", alpha=0.7, label="gps_wheel")
    plt.plot(Time_imu_wheel, Error_x_imu_wheel, color="black", alpha=0.7, label="imu_wheel") 
    plt.plot(Time_imu_wheel_gps, Error_x_imu_wheel_gps, color="purple", alpha=0.7, label="imu_wheel_gps")    
    plt.plot(Time_imu_wheel_gps_laser, Error_x_imu_wheel_gps_laser, color="orange", alpha=0.7, label="imu_wheel_gps_laser")
    plt.title("X-Estimation error")
    plt.ylabel("X error (m)")
    plt.xlabel('Time (sec)')
    plt.legend()

    plt.subplot(1, 2, 2) # index 2
    plt.plot(Time_imu_gps, Error_y_imu_gps, color="red", alpha=0.7, label="imu_gps")
    plt.plot(Time_gps_wheel, Error_y_gps_wheel, color="green", alpha=0.7, label="gps_wheel")
    plt.plot(Time_imu_wheel, Error_y_imu_wheel, color="black", alpha=0.7, label="imu_wheel") 
    plt.plot(Time_imu_wheel_gps, Error_y_imu_wheel_gps, color="purple", alpha=0.7, label="imu_wheel_gps") 
    plt.plot(Time_imu_wheel_gps_laser, Error_y_imu_wheel_gps_laser, color="orange", alpha=0.7, label="imu_wheel_gps_laser")
    plt.title("Y-Estimation error")
    plt.ylabel("Y error (m)")
    plt.xlabel('Time (sec)')
    plt.legend()

   
    plt.show()

                     
    

	

        
    
