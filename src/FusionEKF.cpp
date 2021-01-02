#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

FusionEKF::FusionEKF() {
  is_initialized_ = false;
  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  H_laser_  << 1, 0, 0, 0,
               0, 1, 0, 0;

  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  /**
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */  
  this->ekf_.P_ = MatrixXd(4, 4);
  this->ekf_.P_ << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1000, 0,
            0, 0, 0, 1000;
  this->ekf_.F_ = MatrixXd(4, 4);
  this->ekf_.Q_ = MatrixXd(4, 4);
}

FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  if (!is_initialized_) {
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * Need to convert radar from polar to cartesian coordinates.
     */

    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.
      float rho = measurement_pack.raw_measurements_[0];
      float phi = measurement_pack.raw_measurements_[1];
      float py = sin(phi)*rho; // py/rho = sin(phi)
      float px = cos(phi)*rho;

      // TODO: Can we find vx, vy from rho, phi and raw_measurements_[2]?
      float vx = 0;
      float vy = 0;

      ekf_.x_[0] = px;
      ekf_.x_[1] = py;
      ekf_.x_[2] = vx;
      ekf_.x_[3] = vy;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      ekf_.x_[0] = measurement_pack.raw_measurements_[0];
      ekf_.x_[1] = measurement_pack.raw_measurements_[1];
      ekf_.x_[2] = 0;
      ekf_.x_[3] = 0;
    }

    // done initializing, no need to predict or update
    previous_timestamp_ = measurement_pack.timestamp_;
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */
  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  float noise_ax = 9.0;
  float noise_ay = 9.0;
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  this->ekf_.F_ << 1, 0, dt, 0,
            0, 1, 0, dt,
            0, 0, 1, 0,
            0, 0, 0, 1;

  float dt2 = dt*dt;
  float dt3 = dt2*dt;
  float dt4 = dt3*dt;  
  float q11 = (dt4/4.0) * noise_ax;
  float q13 = (dt3/2.0) * noise_ax;
  float q22 = (dt4/4.0) * noise_ay;
  float q24 = (dt3/2.0) * noise_ay;
  float q31 = (dt3/2.0) * noise_ax;
  float q33 = dt2 * noise_ax;
  float q42 = (dt3/2.0) * noise_ay;
  float q44 = dt2 * noise_ay;

  this->ekf_.Q_ << q11, 0, q13, 0,
             0, q22, 0, q24,
             q31, 0, q33, 0,
             0, q42, 0, q44;

  ekf_.Predict();

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // TODO: Radar updates
    this->ekf_.R_ = this->R_radar_;
    this->Hj_ = tools.CalculateJacobian(this->ekf_.x_);
    this->ekf_.H_ = this->Hj_; 
    this->ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // TODO: Laser updates
    this->ekf_.H_ = this->H_laser_;
    this->ekf_.R_ = this->R_laser_;
    this->ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
