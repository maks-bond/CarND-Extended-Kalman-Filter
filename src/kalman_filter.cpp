#include "kalman_filter.h"
#include "FusionEKF.h"
#include <iostream>

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

// Predict step of kalman filter.
void KalmanFilter::Predict() {
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

// Update step of kalman filter.
// Input measurement is in cartesian coordinate system
void KalmanFilter::Update(const VectorXd &z) {
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  UpdateInternal(y);
}

// Update step of extended kalman filter.
// H matrix is expected to be Jacobian
// Input measurement is in polar coordinate system
void KalmanFilter::UpdateEKF(const VectorXd &z) {
  VectorXd z_pred = VectorXd(3);
  FusionEKF::CartesianToPolar(z_pred, x_);
  
  VectorXd y = z - z_pred;
  y(1) = FusionEKF::NormalizeAngle(y(1));
  
  UpdateInternal(y);
}

// Uses previously calculated error of state vs measurement and calculates required update to state vector X and covariance matrix P.
void KalmanFilter::UpdateInternal(const VectorXd &y) {
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
