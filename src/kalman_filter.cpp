#include "kalman_filter.h"
// #include "tools.h"

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
  //use the h(x) function to map state estimation (Cartesian) to radar space (polar)
//   VectorXd z_pred = tools.Cart2Polar(x_);
  VectorXd z_pred(3);
  double px = x_(0);
  double py = x_(1);
  double vx = x_(2);
  double vy = x_(3);
  //pre-compute a set of terms to avoid repeated calculation
  double z1 = sqrt(px*px + py*py);
  double z2 = atan2(py,px);
  z_pred << z1,z2,(px*vx + py*vy)/z1;
  //calculate y
  VectorXd y = z - z_pred;
  //nomalize phi between -pi and pi, in y
  double pi = 3.14156;
  if(abs(y(1))>pi){
    while(y(1)>pi){
      y(1) -= 2*pi;
    }
    while(y(1)<-pi){
      y(1) += 2*pi;
    }
  }
    
  // H_ should already be initialized to Hj (for this project, in FusionEKF.cpp line 157)
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_; // HJ
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si; // HJ

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_; // HJ
}
