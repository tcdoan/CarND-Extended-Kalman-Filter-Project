#include "kalman_filter.h"
#include <iostream>
#include "tools.h"

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

// Radar updates
void KalmanFilter::UpdateEKF(const VectorXd &z) {
  float px = x_[0];
  float px2 = px*px;

  float py = x_[1];
  float py2 = py*py;
  float pxy2 = px2 + py2;

   if (Tools::isEqual(px, 0.0) || Tools::isEqual(py, 0.0) || Tools::isEqual(pxy2, 0.0))
   {
    std::cout << "Invalid input. x_state[0] or x_state[1] is close to zero." << std::endl;
    return;
   }

  float vx = x_[2];
  float vy = x_[3];

  float rho =  sqrt(pxy2);
  float phi =  atan2(py, px);
  float rhodot =  (px*vx + py*vy)/sqrt(pxy2);
  VectorXd hx(3);
  hx << rho, phi, rhodot;

  VectorXd y = z-hx;
  while (y[1] < -M_PI || y[1] > M_PI) {
    if (y[1] < -M_PI) {
      y[1] += 2*M_PI;
    }
    if (y[1] > M_PI) {
      y[1] -= 2*M_PI;
    }
  }

  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ * Ht * Si;
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
