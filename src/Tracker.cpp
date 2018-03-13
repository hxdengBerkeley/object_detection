#include "Tracker.h"
#include "Eigen/Dense"
#include <iostream>
#include "geometry_msgs/PoseArray.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;


Tracker::Tracker()
{
  is_initialized_ = false;

  //need to be modified
  previous_timestamp_ = 0;

  // initializing matrices
  R_lidar_ = MatrixXd(2, 2);
  H_lidar_ = MatrixXd(2, 4);

  // lidar measurement covariance matrix
  // need to be modified for Velodyne Lidar
  R_lidar_ << 0.0225, 0,
        0, 0.0225;

  // measurement transition matrix
  H_lidar_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  // initialize the kalman filter variables
  // state covariance matrix
  kf_.P_ = MatrixXd(4, 4);
  kf_.P_ << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1000, 0,
             0, 0, 0, 1000;

  // state transition matrix
  kf_.F_ = MatrixXd(4, 4);
  kf_.F_ << 1, 0, 1, 0,
             0, 1, 0, 1,
             0, 0, 1, 0,
             0, 0, 0, 1;
}

Tracker::~Tracker()
{}

VectorXd DataAssociation(const VectorXd &z, const geometry_msgs::PoseArray::ConstPtr& msg)
{
  float distance_sqr = std::numeric_limits<float>::max();
  VectorXd convoy_leader = VectorXd(2);
  
  for (int col = 0; col < Candidates_List.cols(); ++col){
    float distance_x = Candidates_List(0,col) - z(0);
    float distance_y = Candidates_List(1,col) - z(1);
    float distance_sqr_this = distance_x * distance_x + distance_y * distance_y;

    if ( distance_sqr > distance_sqr_this){
      convoy_leader[0] = Candidates_List(0,col);
      convoy_leader[1] = Candidates_List(1,col);
      distance_sqr = distance_sqr_this;
    }
  }
  return convoy_leader;
}

void Tracker::ProcessMeasurement(const geometry_msgs::PoseArray::ConstPtr& msg)
{
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_)
  {
    // first measurement
    kf_.x_ = VectorXd(4);
    kf_.x_ << 1, 1, 1, 1;  //avoid no values in x_

    // we need to select the convoy leader vehicle here
    // we assume that the straight front one is the leader vehicle
    VectorXd z(2);
    z << 0, 0;
    z = DataAssociation(z,msg);
    kf_.x_ << z[0], z[1], 0, 0; // x, y, vx, vy
    previous_timestamp_ = measurement_pack.timestamp_; // set current time stamp
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/
  
  // compute the time elapsed between the current and previous measurements
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;  //  in seconds
  previous_timestamp_ = measurement_pack.timestamp_;

  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;

  // Modify the F matrix so that the time is integrated, from 1 to specific dt (see line 51)
  kf_.F_(0, 2) = dt;
  kf_.F_(1, 3) = dt;

  // set model noises (here is the acceleration)
  float noise_ax = 9;
  float noise_ay = 9;

  //set the process covariance matrix Q (Q = X * a * X_T)
  kf_.Q_ = MatrixXd(4, 4);
  kf_.Q_ << dt_4/4*noise_ax,   0,                dt_3/2*noise_ax,  0,
             0,                 dt_4/4*noise_ay,  0,                dt_3/2*noise_ay,
             dt_3/2*noise_ax,   0,                dt_2*noise_ax,    0,
             0,                 dt_3/2*noise_ay,  0,                dt_2*noise_ay;

  kf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  // Lidar updates
  kf_.H_ = H_lidar_;
  kf_.R_ = R_lidar_;

  VectorXd z(2);
  z << 0, 0;
  // we need to select the convoy leader vehicle here
  // by gated nearest neighbor data association
  z = DataAssociation(kf_.x_,msg);
  kf_.Update(z);
}
