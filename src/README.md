# **The Extended Kalman Filter**

***

## **Disclaimer**
All files found in this repository were cloned from the creators at Udacity. As per a project submission in the Self-Driving Car Engineer Nanodegree, certain files were filled in to demonstrate working knowledge of Kalman filters and extended Kalman filters.

## **Basic Function**
This project builds an extended Kalman filter to estimate the location of a simulated vehicle in Cartesian space, based on LIDAR and radar measurements of the vehicle.

## **Modified Files**
- `FusionEKF.cpp`
- `kalman_filter.cpp`
- `kalman_filter.h`
- `tools.cpp`
- `tools.h`

## **Code**
The incoming vehicle position and velocity data from the simulation is handled in `main.cpp`. 
#### `FusionEKF.cpp`
A `FusionEKF` object will call a KalmanFilter object to predict and update states, depending on what type of sensor has made the incoming measurement. First, it initializes the process noise and measurement noise covariance matrices, as well as the measurement transformation matrix. The initial state is set to the first measurement. If the measurement comes from the LIDAR, the x and y position are directly copied into the first two states, leaving the last two states set to 0. If the first measurement comes from the radar, whose measurements are polar coordinates, these values are converted to Cartesian, including an approximation of the velocity, so that all four states are initialized. 

#### `kalman_filter.cpp`
The `KalmanFilter` object will handle the actual state predictions and updates. The method `Update()` will update the states using traditional Kalman filter equations, and is used to update the states after a LIDAR measurement. The method `UpdateEKF()` will update the states using the extended Kalman filter equations, and is used to update the states after a radar measurement, which is non-linear. A measurement transformation function is used to convert the predicted states from Cartesian to polar coordinates in order to compute their difference from the current radar measurement.

#### `tools.cpp`
Helper methods to calculate root mean square error (RMSE) and a Jacobian for the extended Kalamn filter equations can be found in `tools.cpp`. In addition, to clean up the readability of the code, two other methods were added: `Cart2Poly()` and `Normpi()`. The former performs the transformation from Cartesian to polar coordinates that is required by the Extended Kalman Filter equations, and the latter then normalizes the resulting phi angle to `[-pi,pi]`.

## **Results**
The filter performed well for both of the provided datasets, Dataset 1 and Dataset 2. The final RMSE values are shown below for each dataset. 

|    | Dataset 1 | Dataset 2 |
|----|:---------:|:---------:|
|  x |    0.0973 |    0.0726 |
|  y |    0.0855 |    0.0965 |
| vx |    0.4513 |    0.4216 |
| vy |    0.4399 |    0.4932 |

It was found that intializing the velocity states to a non-zero value (that of the first converted radar measurement) improved the RMSEs of Dataset 2, which likely lists the radar measurement first, unlike Dataset 1, whose RMSEs remained unchanged because it likely lists the LIDAR measurement first. 

## **Notes**
Please see Section "`tools.cpp`" for additions that were not required by the project. The project was completed using the Udacity workspace, and should compile with `cmake` and `make`.

**Author: Ayeda Sayeed, 2019**