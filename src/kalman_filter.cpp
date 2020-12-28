#include "kalman_filter.h"

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
  
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  float px = x_[0];
  float px2 = px*px;

  float py = x_[1];
  float py2 = py*py;
  float pxy2 = px2 + py2;

  float vx = x_[2];
  float vy = x_[3];

  float rho =  sqrt(pxy2);
  float phi =  atan(py/px);
  float rhodot =  (px*vx + py*vy)/sqrt(pxy2);
  VectorXd hx(3);
  hx << rho, phi, rhodot;

  VectorXd y = z-hx;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  

}
