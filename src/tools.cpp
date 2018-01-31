#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  VectorXd rmse(4);
  rmse.setZero();

  if ((0 < estimations.size()) && (estimations.size() == ground_truth.size()))
  {
    //accumulate squared residuals
    for(int i=0; i < estimations.size(); ++i){
      VectorXd diff(4);
      diff = estimations.at(i) - ground_truth.at(i);
      diff = diff.array() * diff.array();
      rmse += diff;
    }

    //calculate the mean
    rmse = rmse / static_cast<double>(estimations.size());

    //calculate the squared root
    rmse = rmse.array().sqrt();
  }
  else
  {
    std::cout << "DATA ERROR: estimations.size(): " << estimations.size() << "  ground_truth.size(): " << ground_truth.size() << std::endl;
  }

  return rmse;
}
