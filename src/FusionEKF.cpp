#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"
#include <cmath>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;
using std::cerr;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;
  
  // create a 4D state vector and initialize the state ekf_.x_ with 0s.
  ekf_.x_ = VectorXd(4);
  ekf_.x_ << 0, 0, 0, 0;
  
  // state covariance matrix P
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1000, 0,
            0, 0, 0, 1000;

  // the initial transition matrix F_
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_<< 1, 0, 1, 0,
            0, 1, 0, 1,
            0, 0, 1, 0,
            0, 0, 0, 1;
  
  //H matrix for laser
  H_laser_<< 1, 0, 0, 0,
             0, 1, 0, 0;
  
  //H matrix for radar
  Hj_ << 0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0;
     
  //Set the process and measurement noises
  noise_ax = 9.0;
  noise_ay = 9.0;

}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    // Initialize the state ekf_.x_ with the first measurement.
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // Convert radar from polar to cartesian coordinates and initialize state.
      ekf_.x_[0] = measurement_pack.raw_measurements_[0] * cos(measurement_pack.raw_measurements_[1]);
      ekf_.x_[1] = measurement_pack.raw_measurements_[0] * sin(measurement_pack.raw_measurements_[1]);
      ekf_.x_[2] = measurement_pack.raw_measurements_[2] * cos(measurement_pack.raw_measurements_[1]);
      ekf_.x_[3] = measurement_pack.raw_measurements_[2] * sin(measurement_pack.raw_measurements_[1]);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // Set the state with the initial location and zero velocity
      ekf_.x_[0] = measurement_pack.raw_measurements_[0];
      ekf_.x_[1] = measurement_pack.raw_measurements_[1];
      ekf_.x_[2] = 0;
      ekf_.x_[3] = 0;
    }
    
    previous_timestamp_ = measurement_pack.timestamp_;
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */
  // compute the time elapsed between the current and previous measurements
  // dt - expressed in seconds
  double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0; //Time is measured in seconds.
  previous_timestamp_ = measurement_pack.timestamp_;
  //Check if the new data is from a different timestamp than the previous timestamp
  if(dt != 0){
    // 1. Modify the F matrix so that the time is integrated
    ekf_.F_(0,2) = dt;
    ekf_.F_(1,3) = dt;

    // 2. Set the process covariance matrix Q
    ekf_.Q_ = MatrixXd(4, 4);
    ekf_.Q_ <<0, 0, 0, 0,
              0, 0, 0, 0,
              0, 0, 0, 0,
              0, 0, 0, 0;
    ekf_.Q_(0,0) = (pow(dt, 4) / 4) * noise_ax; 
    ekf_.Q_(0,2) = (pow(dt, 3) / 2) * noise_ax; 
  
    ekf_.Q_(1,1) = (pow(dt, 4) / 4) * noise_ay; 
    ekf_.Q_(1,3) = (pow(dt, 3) / 2) * noise_ay;
  
    ekf_.Q_(2,0) = (pow(dt, 3) / 2) * noise_ax; 
    ekf_.Q_(2,2) = pow(dt, 2) * noise_ax; 
  
    ekf_.Q_(3,1) = (pow(dt, 3) / 2) * noise_ay; 
    ekf_.Q_(3,3) = pow(dt, 2) * noise_ay;
  
    // 3. Call the Kalman Filter predict() function
    ekf_.Predict();
  }

  /**
   * Update
   */

  /**
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    // Update the H matrix for radar
    try {
      ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
    } catch (const char* msg) {
      cerr << msg << endl;
    }
    // update the R matrix with radar
    ekf_.R_ = R_radar_;
    try { //perform update for radar data
      ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    } catch (const char* msg) {
      cerr << msg << endl;
    }
    
  } else {
    // Laser updates
    // Update the H matrix with laser
    ekf_.H_ = H_laser_;
    // Update the R matrix with laser
    ekf_.R_ = R_laser_;
    // perform update for the laser data
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
