#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

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
  
  //measurement projection matrix - laser
  H_laser_ << 1,0,0,0,
  			  0,1,0,0;
  
//   KalmanFilter ekf_;
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.P_ << 1,0,0,0,
  	   		 0,1,0,0,
  	   		 0,0,1000,0,
  	   		 0,0,0,1000;
  ekf_.F_ << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1, 0,
             0, 0, 0, 1;
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
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      //placeholders to avoid repeat calculations
      double rho, phi, rho_dot, cos_phi, sin_phi, px1, py1, vx1, vy1;
      rho = measurement_pack.raw_measurements_[0];
      phi = measurement_pack.raw_measurements_[1];
      rho_dot = measurement_pack.raw_measurements_[2];
      cos_phi = cos(phi);
      sin_phi = sin(phi);
      //calculate initial state variables
      px1 = rho*cos_phi;
      py1 = rho*sin_phi;
      vx1 = rho_dot*cos_phi; //using rho to calculate vx and vy reduces final RMSE
      vy1 = rho_dot*sin_phi;
      
      ekf_.x_ << px1,py1,vx1,vy1;
	
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
	//Initialize state
      ekf_.x_ << measurement_pack.raw_measurements_[0], 
      		     measurement_pack.raw_measurements_[1], 
                 0, // set vx, vy to 0 as safest way to minimize initial RMSE
                 0;
    }

    // done initializing, no need to predict or update
    previous_timestamp_ = measurement_pack.timestamp_;
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */
  
  //process noise variance
  double noise_ax = 9;
  double noise_ay = 9;
  
  //calculate time difference between current measurement and last measurement
  double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;
  
  double dt_2 = dt * dt;
  double dt_3 = dt_2 * dt;
  double dt_4 = dt_3 * dt;

  // Modify the F matrix so that the time is integrated
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  // set the process covariance matrix Q
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
         	 0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
         	 dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
         	 0, dt_3/2*noise_ay, 0, dt_2*noise_ay;

  ekf_.Predict();

  /**
   * Update
   */

  // Implement extended Kalman filter if the incoming measurement is a radar measurement
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // initialize radar measurement function and covariance
    ekf_.R_ = MatrixXd(3,3);
    ekf_.H_ = MatrixXd(3,4);
    ekf_.R_ = R_radar_;
    // calculate Jacobian of state for measurement function
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.H_ = Hj_;
    // run update for extended Kalman Filter
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    
  // Implement regular Kalman filter if it is a laser measurement
  } else {
    // initialize laser measurement function and covariance
    ekf_.R_ = MatrixXd(2,2);
    ekf_.H_ = MatrixXd(2,4);
    ekf_.R_ = R_laser_;
    ekf_.H_ = H_laser_;
    // run update for normal Kalman filter
    ekf_.Update(measurement_pack.raw_measurements_);

  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
