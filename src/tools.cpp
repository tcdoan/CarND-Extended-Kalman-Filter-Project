#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  if (estimations.size() == 0 || estimations.size() != ground_truth.size()) {
      std::cout << "Invalid input." << std::endl;
      return rmse;
  }

  for (int i=0; i < estimations.size(); ++i) {
    VectorXd c = estimations[i] - ground_truth[i];
    c = c.array() * c.array();
    rmse = rmse + c;    
  }
  // TODO: calculate the mean
  rmse = rmse / estimations.size();

  // TODO: calculate the squared root
  rmse = rmse.array().sqrt();

  // return the result
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
   float px = x_state[0];
   float px2 = px*px;

   float py = x_state[1];
   float py2 = py*py;
   float pxy2 = px2 + py2;
   float pxy2cube = pxy2*pxy2*pxy2;

   MatrixXd Hj(3, 4);
   if (Tools::isEqual(px, 0.0) || Tools::isEqual(py, 0.0) || Tools::isEqual(pxy2, 0.0))
   {
    std::cout << "Invalid input. x_state[0] or x_state[1] is close to zero." << std::endl;
    Hj << 0, 0, 0, 0,
          0, 0, 0, 0,
          0, 0, 0, 0;
    return Hj;
   }

   float vx = x_state[2];
   float vy = x_state[3];

   float h11 = px/sqrt(pxy2);
   float h12 = py/sqrt(pxy2);
   float h21 = -py/pxy2;
   float h22 = px/pxy2;
   float h31 = py*(vx*py - vy*px)/sqrt(pxy2cube);
   float h32 = px*(vy*px - vx*py)/sqrt(pxy2cube);
   float h33 = px/sqrt(pxy2);
   float h34 = py/sqrt(pxy2);
   Hj << h11, h12, 0, 0,
         h21, h22, 0, 0,
         h31, h32, h33, h34;
   return Hj;
}
