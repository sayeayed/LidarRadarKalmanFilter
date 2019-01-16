#ifndef TOOLS_H_
#define TOOLS_H_

//#ifndef _USE_MATH_DEFINES
//#define _USE_MATH_DEFINES

#include <vector>
#include "Eigen/Dense"
// #include <math.h>

class Tools {
 public:
  /**
   * Constructor.
   */
  Tools();

  /**
   * Destructor.
   */
  virtual ~Tools();

  /**
   * A helper method to calculate RMSE.
   */
  Eigen::VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations, 
                                const std::vector<Eigen::VectorXd> &ground_truth);

  /**
   * A helper method to calculate Jacobians.
   */
  Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd& x_state);
  
  /**
   * A helper method to convert from cartesian to polar coordinates
   */
  Eigen::VectorXd Cart2Polar(const Eigen::VectorXd& x_state);
};

#endif  // TOOLS_H_
