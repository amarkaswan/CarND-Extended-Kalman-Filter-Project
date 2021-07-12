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
   * TODO: predict the state
   */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  VectorXd h_ = VectorXd(3);
  if(x_[0] == 0 && x_[1] == 0){
    cout<<"KalmanFilter::UpdateEKF() - Px and Py are zero: Divide by Zero"<<endl;
} else if(x_[0] == 0){
    cout<<"KalmanFilter::UpdateEKF() - Px is zero: Divide by Zero"<<endl;
  }
  else{
    double sq_px = x_[0] * x_[0];
    double sq_py = x_[1] * x_[1];
    h_[0] = sqrt(sq_px + sq_py);
    h_[1] = atan(x_[1]/x_[0]);
    h_[2] = (x_[0] * x_[2] + x_[1] * x_[3])/ sqrt(sq_px + sq_py);
    
    VectorXd y = z - h_;
    // Check if angle is within -pi to pi, if not then make it between -pi to pi
    if(y[1] < -M_PI){
      y[1] += ((double)2 * M_PI);
    }
    if(y[1] > M_PI){
      y[1] -= ((double)2 * M_PI);
    }
    
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd PHt = P_ * Ht;
    MatrixXd K = PHt * Si;
    
    //new estimate
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
}
  }
    
