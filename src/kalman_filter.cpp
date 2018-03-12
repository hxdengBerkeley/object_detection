#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

const float PI2 = 2 * M_PI;

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
  /**
  TODO:
    * predict the state
    * use x_k-1|k-1 and state transition equation to compute x_k|k-1
  */
  x_ = F_ * x_;
  MatrixXd F_t = F_.transpose();
  P_ = F_ * P_ * F_t + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
    * z is the true measurement
  */

  MatrixXd z_pred = H_ * x_;
  MatrixXd S_ = H_ * P_ * H_.transpose()+R_;
  MatrixXd K_ = P_ * H_.transpose() * S_.inverse();

  //estimate
  x_ = x_ + K_ * (z - z_pred);
  P_ = P_ - K_ * (H_ * P_ * H_.transpose() + R_) * K_.transpose();
}

VectorXd RadarCartesianToPolar(const VectorXd &x_state){
  /*
   * convert system prediction from cartesian coordinates (x, y, vx, vy) to
   * polar (rho, phi, rho_dot) coordinates
  */
  float px, py, vx, vy;
  px = x_state[0];
  py = x_state[1];
  vx = x_state[2];
  vy = x_state[3];

  float rho, phi, rho_dot;
  rho = sqrt(px*px + py*py);
  phi = atan2(py, px);  // returns values between -pi and pi

  // if rho is very small, set it to 0.0001 to avoid division by 0 in computing rho_dot
  if(rho < 0.000001)
    rho = 0.000001;

  rho_dot = (px * vx + py * vy) / rho;

  VectorXd z_pred = VectorXd(3);
  z_pred << rho, phi, rho_dot;

  return z_pred;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  /**
    Hanxiao: the EKF is just used for radar measurement update, thus z is the radar measurement - rho, phi, rot_dot
  */
  VectorXd z_pred = RadarCartesianToPolar(x_);  // to transform predict x_ from Cartesian coordinates system into Polar coordinate system
  VectorXd y = z - z_pred;                      // the error between radar measurement and system prediction

  // normalize the angle between -pi to pi
  while(y(1) > M_PI){
    y(1) -= PI2;
  }

  while(y(1) < -M_PI){
    y(1) += PI2;
  }

  // following is exact the same as in the function of KalmanFilter::Update()
  MatrixXd S_ = H_ * P_ * H_.transpose()+R_;
  MatrixXd K_ = P_ * H_.transpose() * S_.inverse();

  //estimate
  x_ = x_ + K_ * (z - z_pred);
  P_ = P_ - K_ * (H_ * P_ * H_.transpose() + R_) * K_.transpose();
}


