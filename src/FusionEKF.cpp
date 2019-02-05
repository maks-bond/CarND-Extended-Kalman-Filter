#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  // Init R, H and F matrices with their defualt values
  
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  H_laser_ << 1, 0, 0, 0,
             0, 1, 0, 0;
  
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, 1, 0,
             0, 1, 0, 1,
             0, 0, 1, 0,
             0, 0, 0, 1;
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    // Use first measurement to initialize state vector X
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;
    
    ekf_.P_ = MatrixXd(4, 4);
    ekf_.P_ << 1, 0, 0, 0,
              0, 1, 0, 0,
              0, 0, 1000, 0,
              0, 0, 0, 1000;
    
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // Radar measurement is in polar coordinates, state vector is in cartesian coordinates. Let's onvert.
      FusionEKF::PolarToCartesian(ekf_.x_, measurement_pack);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // Lidar measurements have only pose estimation, no velocity estimation.
      ekf_.x_(0) = measurement_pack.raw_measurements_[0];
      ekf_.x_(1) = measurement_pack.raw_measurements_[1];
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    previous_timestamp_ = measurement_pack.timestamp_;
    
    return;
  }

  /**
   * Prediction
   */
  
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;
  
  // Update F state transition matrix according to time span between current and prev measurements.
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;
  
  float noise_ax = 9, noise_ay = 9;
  
  // Update Q process noise matrix to incorporate timespan
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
             0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
             dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
             0, dt_3/2*noise_ay, 0, dt_2*noise_ay;
  
  ekf_.Predict();
  
  /**
   * Update
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Calculate H measurement function matrix and set R measurement noise matrix for Radar and run Extended Kalman Filter Update step.
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else {
    // Set laser H measurement function matrix and set R measurement noise matrix
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    
    // Run Kalman Filter Update step
    ekf_.Update(measurement_pack.raw_measurements_);

  }
  
  // Update timestamp
  previous_timestamp_ = measurement_pack.timestamp_;
}

void FusionEKF::PolarToCartesian(Eigen::VectorXd& result, const MeasurementPackage &measurement_pack) {
  float ro = measurement_pack.raw_measurements_[0];
  float theta = measurement_pack.raw_measurements_[1];
  float ro_dot = measurement_pack.raw_measurements_[2];
  
  float px = ro*cos(theta);
  float py = ro*sin(theta);
  float vx = ro_dot*cos(theta);
  float vy = ro_dot*sin(theta);
  
  result << px, py, vx, vy;
}

void FusionEKF::CartesianToPolar(Eigen::VectorXd& result, const Eigen::VectorXd& input) {
  float px = input[0];
  float py = input[1];
  float vx = input[2];
  float vy = input[3];

  float ro = sqrt(px*px + py*py);
  float theta = atan2(py,px);
  
  float ro_dot;
  if(fabs(ro) < 0.0001) ro_dot = 0;
  else ro_dot = (px*vx+py*vy)/ro;
  
  
  result << ro,theta,ro_dot;
}

float FusionEKF::NormalizeAngle(float theta) {
  while (theta > M_PI) {
    theta -= 2*M_PI;
  }
  
  while(theta < -M_PI) {
    theta += 2*M_PI;
  }
  
  return theta;
}
