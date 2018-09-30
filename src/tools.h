#ifndef TOOLS_H
#define TOOLS_H

#include <vector>
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class Tools {
public:
  Tools();
  virtual ~Tools();

  // Root mean square error of estimations
  VectorXd CalculateRMSE(const std::vector<VectorXd>& estimations,
                         const std::vector<VectorXd>& ground_truth);
};

#endif  // TOOLS_H