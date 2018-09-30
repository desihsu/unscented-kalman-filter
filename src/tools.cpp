#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const std::vector<VectorXd>& estimations,
                              const std::vector<VectorXd>& ground_truth) {
  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  for (int i = 0; i < estimations.size(); i++) {
    VectorXd residual = estimations[i] - ground_truth[i];
    residual = residual.array() * residual.array();
    rmse += residual;
  }

  rmse /= estimations.size();
  rmse = rmse.array().sqrt();
  return rmse;
}