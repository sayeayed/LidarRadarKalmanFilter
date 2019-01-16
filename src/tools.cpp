#include "tools.h"
#include <iostream>
// #include <math.h>

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
  // assuming state vector size of 4
    VectorXd rmse(4);
    rmse << 0,0,0,0;

  // check the validity of the inputs:
  if (estimations.size() != ground_truth.size()
      || estimations.size() == 0) {
    cout << "Invalid estimation or ground_truth data dimensions" << endl;
    return rmse;
  }

  // accumulate squared residuals
  for (unsigned int i=0; i < estimations.size(); ++i) {

    VectorXd residual = estimations[i] - ground_truth[i];

    // coefficient-wise multiplication
    residual = residual.array()*residual.array();
    rmse += residual;
  }

  // calculate the mean
  rmse = rmse/estimations.size();

  // calculate the squared root
  rmse = rmse.array().sqrt();

  // return the result
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
  
  // assuming a state vector of length 4
  MatrixXd Hj(3,4);
  // recover state parameters
  double px = x_state(0);
  double py = x_state(1);
  double vx = x_state(2);
  double vy = x_state(3);

  // pre-compute a set of terms to avoid repeated calculation
  double c1 = px*px+py*py;
  double c2 = sqrt(c1);
  double c3 = (c1*c2);

  // check division by zero
  if (fabs(c1) < 0.0001) {
    cout << "CalculateJacobian () - Error - Division by Zero" << endl;
    // c1 = 0.0001;
    return Hj;
  }

  // compute the Jacobian matrix
  Hj << (px/c2), (py/c2), 0, 0,
      -(py/c1), (px/c1), 0, 0,
      py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

  return Hj;
}

VectorXd Tools::Cart2Polar(const VectorXd& x_state) {
  VectorXd z_p(3);
  double px = x_state(0);
  double py = x_state(1);
  double vx = x_state(2);
  double vy = x_state(3);
  //pre-compute a set of terms to avoid repeated calculation
  double z1 = sqrt(px*px + py*py);
  double z2 = atan2(py,px);
  z_p << z1,z2,(px*vx + py*vy)/z1;
  return z_p;
  
}

double Tools::Normpi(double y1){
  //define pi to avoid including libraries
  double pi = 3.14159265359;
  //check that phi is outside of range [-pi,pi]
  if(abs(y1)>pi){
    while(y1>pi){
      y1 -= 2*pi; //subtract 2pi if phi too large until it is within range
    }
    while(y1<-pi){
      y1 += 2*pi; //add 2pi if phi too small
    }
  }
  return y1;
}
