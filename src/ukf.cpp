#include "ukf.h"
#include <iostream>
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

UKF::UKF() {
  is_initialized_ = false;
  time_us_ = 0.0;

  n_x_ = 5;
  n_aug_ = 7;
  lambda_ = 3 - n_aug_;

  // Process noise
  std_a_ = 0.5;
  std_yawdd_ = 0.5;

  // Laser measurement noise
  std_laspx_ = 0.15;
  std_laspy_ = 0.15;

  // Radar measurement noise
  std_radr_ = 0.3;
  std_radphi_ = 0.03;
  std_radrd_ = 0.3;

  x_ = VectorXd(n_x_);

  P_ = MatrixXd(n_x_, n_x_);
  P_ << std_laspx_, 0, 0, 0, 0,
        0, std_laspy_, 0, 0, 0,
        0, 0, 1, 0, 0,
        0, 0, 0, 1, 0,
        0, 0, 0, 0, 1;

  weights_ = VectorXd(2*n_aug_+1);
  weights_(0) = lambda_ / (lambda_ + n_aug_);
  for (int i = 1; i <= 2*n_aug_; ++i) {
    weights_(i) = 1 / (2 * (lambda_ + n_aug_));
  }
}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  if (!is_initialized_) {
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      // Polar to cartesian coordinates
      float rho = meas_package.raw_measurements_(0);
      float phi = meas_package.raw_measurements_(1);
      x_ << rho*cos(phi), rho*sin(phi), 1, 1, 0.1;
    }
    else {
      x_ << meas_package.raw_measurements_(0), 
            meas_package.raw_measurements_(1), 1, 1, 0.1;
    }

    time_us_ = meas_package.timestamp_;
    is_initialized_ = true;
    return;
  }

  double dt = (meas_package.timestamp_ - time_us_) / 1000000.0;
  time_us_ = meas_package.timestamp_;

  Prediction(dt);

  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    UpdateRadar(meas_package);
  }
  else {
    UpdateLidar(meas_package);
  }
}

void UKF::Prediction(double dt) {
  // Augmented state vector
  VectorXd x_aug(n_aug_);
  x_aug.head(n_x_) = x_;
  x_aug(n_x_) = 0;
  x_aug(n_x_+1) = 0;

  // Augmented state covariance matrix
  MatrixXd P_aug(n_aug_, n_aug_);
  P_aug.fill(0.0);
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug(n_x_, n_x_) = std_a_ * std_a_;
  P_aug(n_x_+1, n_x_+1) = std_yawdd_ * std_yawdd_;
  MatrixXd P_aug_sqrt = P_aug.llt().matrixL();

  // Sigma point matrix
  MatrixXd Xsig_aug(n_aug_, 2*n_aug_+1);
  Xsig_aug.col(0) = x_aug;

  for (int i = 0; i < n_aug_; i++) {
    Xsig_aug.col(i+1) = x_aug + sqrt(lambda_+n_aug_) * P_aug_sqrt.col(i);
    Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * P_aug_sqrt.col(i);
  }

  // Predicted sigma points
  Xsig_pred_ = MatrixXd(n_x_, 2*n_aug_+1);

  for (int i = 0; i <= 2*n_aug_; i++) {
    double px = Xsig_aug(0, i);
    double py = Xsig_aug(1, i);
    double v = Xsig_aug(2, i);
    double yaw = Xsig_aug(3, i);
    double yawd = Xsig_aug(4, i);
    double nu_a = Xsig_aug(5, i);
    double nu_yawdd = Xsig_aug(6, i);

    Xsig_pred_(0, i) = (px + v / yawd * (sin(yaw + yawd * dt) - sin(yaw)) +
                        dt * dt / 2 * cos(yaw) * nu_a);
    Xsig_pred_(1, i) = (py + v / yawd * (-cos(yaw + yawd * dt) + cos(yaw)) + 
                        dt * dt / 2 * sin(yaw) * nu_a);
    Xsig_pred_(2, i) = v + dt * nu_a;
    Xsig_pred_(3, i) = yaw + dt * yawd + dt * dt / 2 * nu_yawdd;
    Xsig_pred_(4, i) = yawd + dt * nu_yawdd;
  }

  // Predict state mean
  x_.fill(0.0);

  for (int i = 0; i <= 2*n_aug_; i++) {
    x_ += weights_(i) * Xsig_pred_.col(i);
  }

  // Predict state covariance
  P_.fill(0.0);

  for (int i = 0; i <= 2*n_aug_; i++) {
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    while (x_diff(3) > M_PI) { x_diff(3) -= 2 * M_PI; }
    while (x_diff(3) < -M_PI) { x_diff(3) += 2 * M_PI; }
    P_ += weights_(i) * x_diff * x_diff.transpose();
  }
}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  int n_z = 2;

  // Sigma points
  MatrixXd Zsig(n_z, 2*n_aug_+1);

  for (int i = 0; i <= 2*n_aug_; i++) {
    Zsig(0, i) = Xsig_pred_(0, i);
    Zsig(1, i) = Xsig_pred_(1, i);
  }

  // Predict measurement mean
  VectorXd z_pred(n_z);
  z_pred.fill(0.0);

  for (int i = 0; i <= 2*n_aug_; i++) {
    z_pred += weights_(i) * Zsig.col(i);
  }

  // Predict covariance
  MatrixXd S(n_z, n_z);
  S.fill(0.0);

  for (int i = 0; i <= 2*n_aug_; i++) {
    VectorXd z_diff = Zsig.col(i) - z_pred;
    S += weights_(i) * z_diff * z_diff.transpose();
  }

  S(0, 0) += std_laspx_ * std_laspx_;
  S(1, 1) += std_laspy_ * std_laspy_;

  // Cross correlation matrix
  MatrixXd Tc(n_x_, n_z);
  Tc.fill(0.0);

  for (int i = 0; i <= 2*n_aug_; i++) {
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    VectorXd z_diff = Zsig.col(i) - z_pred;
    Tc += weights_(i) * x_diff * z_diff.transpose();
  }

  // Updates state and state covariance
  VectorXd z_diff = meas_package.raw_measurements_ - z_pred;
  MatrixXd K = Tc * S.inverse();
  x_ += K * z_diff;
  P_ -= K * S * K.transpose();
}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  int n_z = 3;

  // Sigma points
  MatrixXd Zsig(n_z, 2*n_aug_+1);

  for (int i = 0; i <= 2*n_aug_; i++) {
    double px = Xsig_pred_(0, i);
    double py = Xsig_pred_(1, i);
    double v = Xsig_pred_(2, i);
    double yaw = Xsig_pred_(3, i);

    Zsig(0, i) = sqrt(px * px + py * py);
    Zsig(1, i) = atan2(py, px);
    Zsig(2, i) = ((px * cos(yaw) * v + py * sin(yaw) * v) / 
                  sqrt(px * px + py * py));
  }

  // Predict measurement mean
  VectorXd z_pred(n_z);
  z_pred.fill(0.0);

  for (int i = 0; i <= 2*n_aug_; i++) {
    z_pred += weights_(i) * Zsig.col(i);
  }

  // Predict covariance
  MatrixXd S(n_z, n_z);
  S.fill(0.0);

  for (int i = 0; i <= 2*n_aug_; i++) {
    VectorXd z_diff = Zsig.col(i) - z_pred;
    while (z_diff(1) > M_PI) { z_diff(1) -= 2 * M_PI; }
    while (z_diff(1) < -M_PI) { z_diff(1) += 2 * M_PI; }
    S += weights_(i) * z_diff * z_diff.transpose();
  }

  S(0, 0) += std_radr_ * std_radr_;
  S(1, 1) += std_radphi_ * std_radphi_;
  S(2, 2) += std_radrd_ * std_radrd_;

  // Cross correlation matrix
  MatrixXd Tc(n_x_, n_z);
  Tc.fill(0.0);

  for (int i = 0; i <= 2*n_aug_; i++) {
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    VectorXd z_diff = Zsig.col(i) - z_pred;

    while (x_diff(3) > M_PI) { x_diff(3) -= 2*M_PI; }
    while (x_diff(3) < -M_PI) { x_diff(3) += 2*M_PI; }
    while (z_diff(1) > M_PI) { z_diff(1) -= 2*M_PI; }
    while (z_diff(1) < -M_PI) { z_diff(1) += 2*M_PI; }
    Tc += weights_(i) * x_diff * z_diff.transpose();
  }

  // Normalize phi to be between -pi and pi
  VectorXd z_diff = meas_package.raw_measurements_ - z_pred;
  while (z_diff(1) > M_PI) { z_diff(1) -= 2*M_PI; }
  while (z_diff(1) < -M_PI) { z_diff(1) += 2*M_PI; }

  // Updates state and state covariance
  MatrixXd K = Tc * S.inverse();
  x_ += K * z_diff;
  P_ -= K * S * K.transpose();
}
