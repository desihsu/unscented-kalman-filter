#ifndef UKF_H
#define UKF_H

#include <fstream>
#include <string>
#include <vector>
#include "Eigen/Dense"
#include "measurement_package.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF {
public:
  UKF();
  virtual ~UKF();

  bool is_initialized_;
  long long time_us_;

  // State vector
  VectorXd x_;

  // State covariance matrix
  MatrixXd P_;

  // Predicted sigma points matrix
  MatrixXd Xsig_pred_;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  // Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  // Laser measurement x noise standard deviation in m
  double std_laspx_;

  // Laser measurement y noise standard deviation in m
  double std_laspy_;

  // Radar measurement noise standard deviation radius in m
  double std_radr_;

  // Radar measurement noise standard deviation angle in rad
  double std_radphi_;

  // Radar measurement noise standard deviation radius change in m/s
  double std_radrd_;

  // Weights of sigma points
  VectorXd weights_;

  // State dimension
  int n_x_;

  // Augmented state dimension
  int n_aug_;

  // Sigma point spreading parameter
  double lambda_;

  // Sensor fusion 
  // Radar sensor measurements in the form of: (rho, phi, rho_dot)
  // Laser sensor measurements in the form of: (x, y)
  void ProcessMeasurement(MeasurementPackage meas_package);
  
  void Prediction(double dt);
  void UpdateLidar(MeasurementPackage meas_package);
  void UpdateRadar(MeasurementPackage meas_package);
};

#endif  // UKF_H
