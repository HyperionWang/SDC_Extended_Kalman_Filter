
# Extended Kalman Filter Project Readme

In this project I created a Kalman filter and Extended Kalman filter to predict and estimate the state of a moving object of interest with noisy lidar and radar measurements for the sensor fusion in self-driving car.

[//]: # (Image References)
[image1]: ./Docs/Dataset1_EKF_Results.png
[image2]: ./Docs/Dataset2_EKF_Results.png
[image3]: ./Docs/ProcessFlow.png
[image4]: ./Docs/JacobianMatrix.png
The c++ code is sucessfully built and run on "Bash on Ubuntun on Window" CLI environment. The created EntedecKF program is able to connected with Term 2 Simulator. 

The RMS for px, py, vx and vy for the dataset 1 is: 0.0955, 0.0850, 0.4293, 0.4184.

![alt text][image1]

The RMS for px, py, vx and vy for the dataset 2 is: 0.0745, 0.0981, 0.3955, 0.4714.

![alt text][image2]

## Command to use the code
Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. Open the Unity Simulator, choose the project 1
6. run './ExtendedKF' command in CLI


## The code structure
The programs that need to be written to accomplish the project include src/FusionEKF.cpp, src/FusionEKF.h, kalman_filter.cpp, kalman_filter.h, tools.cpp, and tools.h.

The program main.cpp has already been filled out. The main.cpp is used to setup the communication between the program and the Unity Simulator, read the measurement data from the data txt file in ./Data folder, send the measurement data to the Extended Kalman Filter, get the output of the predict and update results from the Kalman filter, and finally compare the update results from Kalman filter with the expected results using the RMS estimation function for the accuracy metric. 

### main.cpp & Simulation Communication:
Here is the main protcol that main.cpp uses for uWebSocketIO in communicating with the simulator.

INPUT: values provided by the simulator to the c++ program
["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)

OUTPUT: values provided by the c++ program to the simulator
["estimate_x"] <= kalman filter estimated position x
["estimate_y"] <= kalman filter estimated position y
["rmse_x"]
["rmse_y"]
["rmse_vx"]
["rmse_vy"]

### Extended Kalman Filter Design:

The major implementation of Kalman filter is in FusionEKF.cpp and kalman_filter.cpp. 

![ProcessFlowChart][image3]

#### FusionEKF class -- Process Measurement Work Flow Class
In FusionEKF.cpp, the implementation is for the process work-flow, including Initialization of Kalman Filter with intial value of the measurement noise R, measurement function H, and Jacobian Function Hj_ (from line 14 to 50). 

In the function 'ProcessMeasurement' in class FusionEKF, the first measurement update (Line 63 to Line 114), prediction (Line 127 to Line 146), finding out either Laser Measurement or Radar Measurement, and update the measurement by calling the kalman_filter's update() (Line 158 to Line 163) or updateEKF() functions (Line 165 to Line 170).

For the first measurement initialization, the Kalman filter's initial value x = [px, py, vx, vy] and uncertainty covariance P are set. Since they are intial value, the P is set to be really large, especially for the velocity covariance. The time is set to be 0. As the first measurement come, the time is set to be the time of the first measurement, and update the Kalman filter's R value according to the measurement type (Laser: R_Laser; Radar: R_radar_), and call the correponding Update functions.

After the first measurement with intialization, the Kalman filter Process goes into a loop with prediction and update with measurement. As the new measurement comes, get the time of the measurment, find the dt between measurements, update the Velocity variation covariance Q, and State transition machine F based on the dt, and call the predict() function from Kalman Filter.

After the prediction, update the measurement based on the incoming measurement type (Laser or Radar), and call the corresponding update() function to update the Kalman Filter's state.

#### KalmanFilter class -- The implementation of Extended/Linear Kalman Filter Class

KalmanFilter Class has three major functions (methods) implemented in the projects: Predict(), Update(), and UpdateEKF(). Both Laser and Radar measurement would share the same Predict() function. 
 
The prediction method Predict() would predict positions and velocity info in VectorXd x, and update the covariance P based on the updated State Transition Machine F, and velocity variation Q.  (Line 25 to Line 38 in kalman_filter.cpp).

```C++
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
```

Two update methods are included in the kalman_filter, Update(measurement z) is for Laser measurement update. Since the Laser measurement only includes the position info px, py, the measurement function H is in linear Matrix format. The x and P are updated using the following formula:

```C++
  y = z - H_ * x_;
  S = H_ * P_ * H_.transpose() + R_;
  K = P_ * H_.transpose() * S.inverse();

  x_ = x_ + K * y;
  P_ = (I - K * H_) * P_;
```

UpdateEKF(measurement z) is for Radar Measurement update. Radar measurement is in polar coordinate, with distance ro, angle theta, and polar velocity ro_dot. Therefore, the first step is to convert the previous state x from Cartesian to Polar coordinate ([px,py,vx,vy] -> [ro, theta, ro_dot]). The function called car2pol(x) (Line 104 to Line 126 in kalman_filter.cpp) is implemented as a method for Kalman filter for this coordinate conversion. Also, since the transition for Covariance P in Radar measurement update is not linear, the Jacobian matrix is used to 'Linearize' the transition on P and x. The CalculateJacobian(x) function is implemented as a method included in the Tools class. The following is the update operation for x and P from Radar Measurement. Also, the angle wrapping is used to prevent the angle update out of the range [-PI, PI]. (Line 72 to Line 82)
```C++
y = z - car2pol(x);
JH = tools.CalculateJacobian(x_);
S = JH * P_ * JHT + R_;
K = P_ * JH.transpose() * S.inverse();

x_ = x_ + K * y;
P_ = (I - K * JH) * P_;
```

The Jacobian Matrix calculation function is implemented in Line 45 to Line 87 in tools.cpp. The formula used is as following:

![alt text][image4]

###Summary

The Kalman filter program ./ExtendedKF is a C++ implementation for object tracking on position and velocity using Extended Kalman Filter. The designed Kalman filter is able to precisely predict the moving object's position and velocity based on the incoming measurement updates using LIDAR and RADAR in real-time. As shown in the previous section, the RMS of prediction error is less than 0.1 for position and less than 0.5 for velocity. 