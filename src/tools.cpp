#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using std::cout;
using std::endl;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  // TODO: YOUR CODE HERE
  // check the validity of the following inputs:
  if(estimations.size() == 0) {
    cout<<"CalculateRMSE () - Error - Estimation Vector Size is Zero"<<endl;
    return rmse;
  } else if(estimations.size() != ground_truth.size()){
    cout<<"CalculateRMSE () - Error - Estimation Vector Size is not Equal to Ground Truth Vector Size"<<endl;
    return rmse;
  } 
  // TODO: accumulate squared residuals
  for (unsigned int i=0; i < estimations.size(); ++i) {
    for (unsigned int j=0; j < estimations[i].size(); ++j) {
        rmse[j] = rmse[j] + (estimations[i][j] - ground_truth[i][j]) * (estimations[i][j] - ground_truth[i][j]);
    }
  }
  for (unsigned int j=0; j < estimations[0].size(); ++j) {
    rmse[j] = rmse[j] / estimations.size();
    rmse[j] = sqrt(rmse[j]);
  }
  // return the result
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
  MatrixXd Hj(3,4);
  // recover state parameters
  double px = x_state(0);
  double py = x_state(1);
  double vx = x_state(2);
  double vy = x_state(3);

  if(px == 0 && py == 0) {
      // check division by zero
      cout<<"CalculateJacobian () - Error - Division by Zero"<<endl;
   }
   else{
      // compute the Jacobian matrix 
      double sq_px = px * px;
      double sq_py = py * py;
      Hj(0,0) = px / sqrt(sq_px + sq_py);
      Hj(0,1) = py / sqrt(sq_px + sq_py);
      
      Hj(1,0) = - (py / (sq_px + sq_py));
      Hj(1,1) = (px / (sq_px + sq_py));
      
      double pow3by2_sq_px_py = (sq_px + sq_py) * sqrt(sq_px + sq_py);
      //cout<<"pow3by2_sq_px_py: "<<pow3by2_sq_px_py<<endl;
      Hj(2,0) = (py * ((vx * py) - (vy * px))) / pow3by2_sq_px_py;
      Hj(2,1) = (px * ((vy * px) - (vx * py))) / pow3by2_sq_px_py;
      Hj(2,2) = px / sqrt(sq_px + sq_py);
      Hj(2,3) = py / sqrt(sq_px + sq_py);
   }
  return Hj;
}
