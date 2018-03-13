#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

const float PI2 = 2 * M_PI;

KalmanFilter::KalmanFilter() {}
KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in, MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) 
{
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict()
{
  /**
   * predict the state x_k|k-1 and state covariance matrix P_k|k-1
   */
  x_ = F_ * x_;
  MatrixXd F_t = F_.transpose();
  P_ = F_ * P_ * F_t + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * measurement update to correct the prediction
   * z is the true measurement
   */
  MatrixXd z_pred = H_ * x_;
  MatrixXd S_ = H_ * P_ * H_.transpose()+R_;
  MatrixXd K_ = P_ * H_.transpose() * S_.inverse();

  //estimate
  x_ = x_ + K_ * (z - z_pred);
  P_ = P_ - K_ * (H_ * P_ * H_.transpose() + R_) * K_.transpose();
}