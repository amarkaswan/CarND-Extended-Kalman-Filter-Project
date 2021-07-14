# Extended Kalman Filter Project

#### This project aims to develop a sensor fusion software pipeline for tracking the position and velocity of a moving object around an autonomous vehicle using [Extended Kalman Filter (EKF)](https://en.wikipedia.org/wiki/Extended_Kalman_filter) in C++. In particular, the EKF uses noisy lidar and radar measurements generated by [Udacity's Self-Driving Car Simulator](https://github.com/udacity/self-driving-car-sim/releases) to estimate the state of the object of interest. The pipeline needs to estimate the state of the moving object within a given tolerance of the root mean squared error (RMSE).   
<p> </p>

## Prerequisites
- To begin with, Udacity provides a [starter code](https://github.com/udacity/CarND-Extended-Kalman-Filter-Project/) for this project. This code uses WebSocket communication protocol to establish the communication between the sensor fusion software pipeline, i.e., the server-side, and the simulator, i.e., the client-side. 
- This project requires some tools to successfully compile and run the said software pipeline, and the readers can find their details at [README.md](https://github.com/udacity/CarND-Extended-Kalman-Filter-Project/blob/master/README.md).
- The main program can be built and run by executing the following commands from the project top directory.
```
mkdir build
cd build
cmake ..
make
/ExtendedKF
```
## EKF Source Files
The EKF is implemented based on the starter code provided by the Udacity. The soucre files of the EKF are included in `src` folder of this repository, which are listed below.
```
src
│───kalman_filter.h
│───kalman_filter.cpp
│───tools.h
│───tools.cpp
│───measurement_package.h
│───FusionEKF.h
│───FusionEKF.cpp
│───main.cpp
│───json.hpp
```

A brief overview of these files is provided as follows.
- `kalman_filter.h`: It contains the declaration of the `KalmanFilter` class. This class includes the state (i.e., the position and velocity) vector `x`, state transition matrix `F`, state covariance matrix `P`, process noise covariance matrix `Q`, measurement noise covariance matrix `R`, and measurement transition matrix `H`. Moreover, it also incorporates the member functions, namely `Predict()`, `Update()`,  and `UpdateEKF()`.  
- `kalman_filter.cpp`: It provides the definition of `Predict()`, `Update()`, and `UpdateEKF()` member functions of the `KalmanFilter` class. These functions are utilized to predict the object's new state and update the state based on the lidar and radar measurements, respectively.
- `tools.h`: It comprises the declaration of `Tools` class. This class includes the member functions `CalculateRMSE()` and `CalculateJacobian()` to calcuate the RMSE between the estimated state and the ground truth (i.e., object's actual state) and the Jacobian matrix (i.e., measurement transition matrix) for the radar measurements, respectively.
- `tools.cpp`: It encompases the implementation of `CalculateRMSE()` and `CalculateJacobian()` member functions of `Tools` class.
- `measurement_package.h`: It includes the declaration of the `MeasurementPackage` class that temporarily stores the sensor measurement data with a timestamp. 
- `FusionEKF.h`: It comprises the declaration of `FusionEKF` class. This class creates the objects of `KalmanFilter` and `Tools` classes and includes a member function `ProcessMeasurement()`. 
- `FusionEKF.cpp`: It implements the definition of the `ProcessMeasurement()` member function of the `FusionEKF` class. This function initializes the object's state based on the first sensor measurement. It then uses the subsequent measurements to predict and update the new state of the object. In particular, it first updates the state transition matrix `F` and process noise covariance matrix `Q` based on the time elapsed between the previous and new measurements. Next, it predicts the object's new state using the previous state vector `x`, state covariance matrix `P`, updated state transition matrix `F`, and updated process noise covariance matrix `Q`. After that, it updates the predicted state based on the type of measurement it has received from the simulator. Suppose the new measurement is from the lidar sensor. In that case, it uses measurement transition matrix `H` defined for the linear model since the lidar provides the measurements in the cartesian coordinate system only. Otherwise, it calculates the Jacobian matrix by calling the `CalculateJacobian()` function and uses it as the measurement transition matrix `H` for the nonlinear model. The reason is that the radar generates the measurements in the polar coordinate system, and the conversion from polar to the cartesian coordinate system involves nonlinear equations. 
- `main.cpp`: It establishes a TCP connection between the simulator and the software pipeline. The simulator then transmits the data measurements to the `main.cpp`, which is temporarily stored in an object of the `MeasurementPackage` class. Next, the `main.cpp` calls the `ProcessMeasurement()` function to update the state of the object being tracked. Besides, it also calls the `CalculateRMSE()` function to calculate the RMSE. Thereafter, it transmits the object's updated state and RMSE to the simulator.
- `json.hpp`: It is a single-header C++ library for handling JSON that provides easy, clean ways to read from and write to JSON files.

## Testing and Results
The implemented sensor fusion software pipeline was tested with two datasets provided in Udacity's Self-Driving Car simulator. The RMSE for these datasets must be less than the following threshold values. 
<p></p>
<table>
  <tr>
    <td>Measurement</td>
    <td>RMSE Threshold</td>
  </tr>
  <tr>
    <td>Position in x dimension</td>
    <td>0.11</td>
  </tr>
   <tr>
    <td>Position in y dimension</td>
    <td>0.11</td>
  </tr>
  <tr>
    <td>Velocity in x dimension</td>
    <td>0.52</td>
  </tr>
   <tr>
    <td>Velocity in y dimension</td>
    <td>0.52</td>
  </tr>
 </table>
 <p></p>

The following table lists the results obtained by the implemented sensor fusion software pipeline for both the datasets. 
<p></p>
<table>
  <tr>
    <td>Measurement</td>
    <td>RMSE of Dataset 1</td>
    <td>RMSE of Dataset 2</td>
  </tr>
  <tr>
    <td>Position in x dimension</td>
    <td>0.0973</td>
    <td>0.0726</td>
  </tr>
   <tr>
    <td>Position in y dimension</td>
    <td>0.0855</td>
    <td>0.0965</td>
  </tr>
  <tr>
    <td>Velocity in x dimension</td>
    <td>0.4513</td>
    <td>0.4219</td>
  </tr>
   <tr>
    <td>Velocity in y dimension</td>
    <td>0.4399</td>
    <td>0.4937</td>
  </tr>
 </table>
 <p></p>

It can be observed that the RMSE for all the measurments is less than the given threshold values. The graphical representation of the above mentioned results are shown below. 

<p></p>
<table>
 <center>
  <tr>
    <td>Output of Sensor Fusion for Dataset 1</td>
  </tr>
  <tr>
     <td> <img src="./Output/output_dataset1.png" width="720" height="600"> </td>
 </center>
 </table>
 <p></p>
 
 <p></p>
<table>
 <center>
  <tr>
    <td>Output of Sensor Fusion for Dataset 2</td>
  </tr>
  <tr>
     <td> <img src="./Output/output_dataset2.png" width="720" height="600"> </td>
 </center>
 </table>
 <p></p>

In these pictorials, green triangles represent the estimated state of the object, red circles denote the lidar measurements, and blue circles show the radar measurements with an arrow pointing in the direction of the observed angle. 

## Discussions

Although the implemented software pipeline has achieved the RMSE below the given thresholds, the RMSE can be further minimized using a more sophisticated process model.  It may be noted that the EKF can especially perform poorly when the predict and update functions are highly nonlinear as the covariance is propagated through the linearization of the underlying nonlinear model in EKF.

A better alternative could be to use an Unscented Kalman Filter (UKF) that approximates the probability distribution using sigma points. In particular, the UKF maps some points from the source Gaussian to the target Gaussian by transforming them using a nonlinear function. After that, it estimates the new mean and variance based on the transformed points. In many cases, the sigma points approximate the nonlinear model better than linearization does. In other words, the UKF can better estimate the turn rate of the objects of interest. Besides, the UKF does not require to compute a Jacobian matrix. 
