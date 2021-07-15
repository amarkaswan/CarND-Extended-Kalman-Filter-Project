#include "kalman_filter.h"
#include <math.h>
#include <cmath>
#include <iostream>

using std::cout;
using std::endl;
using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
   * predict the state
   */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   *  update the state by using Kalman Filter equations
   */
  // Compute difference between measurement and predicted state
  VectorXd y = z - H_ * x_; 
  
  // execute common equations of KF and EKF
  UpdateCommon(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * update the state by using Extended Kalman Filter equations
   */
  VectorXd h_ = VectorXd(3);
  h_ << 0, 0, 0;
  if( (x_[0] == 0.0 && x_[1] == 0.0) || (x_[0] == 0.0) ){
    cout << "UpdateEKF() - Error - Divide by Zero" << endl;
    return; // to ignore the measurement
} else{ 
    // Convert state data into radar measurement format using h(x) function
    double sq_px = x_[0] * x_[0];
    double sq_py = x_[1] * x_[1];
    h_[0] = sqrt(sq_px + sq_py); //rho
    h_[1] = atan2(x_[1], x_[0]); //phi
    h_[2] = (x_[0] * x_[2] + x_[1] * x_[3])/ h_[0]; //rho_dot
    
    // Compute difference between measurement and predicted state
    VectorXd y = z - h_;
    // Check if angle is within -pi to pi, if not then make it between -pi to pi
    while(y[1] < -M_PI){
      y[1] += 2.0 * M_PI;
    }
    while(y[1] > M_PI){
      y[1] -= 2.0 * M_PI;
    }
    // execute common equations of KF and EKF
    UpdateCommon(y);
  }
}
     
void KalmanFilter::UpdateCommon(const VectorXd &y){  
  // implement common equations of KF and EKF
  
  // Compute Kalman gain
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  // New estimate
  x_ = x_ + K * y;
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
