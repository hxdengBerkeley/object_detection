#include "Tracker.h"
#include "kalman_filter.cpp"
#include "Eigen/Dense"
#include <iostream>
#include <stdlib.h>
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Point.h"

using namespace std;
using namespace Eigen;
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

double euclidean_distance(geometry_msgs::Point& p1, geometry_msgs::Point& p2)
{
  return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
}

void Tracker::ProcessMeasurement(geometry_msgs::PoseArray& msg)
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
    geometry_msgs::Point point;
    point.x = numeric_limits<float>::max();
    point.y = numeric_limits<float>::max();
    geometry_msgs::Point origin;
    origin.x = 0.0;
    origin.y = 0.0;
    float distance = euclidean_distance(point,origin);
    int size = msg.poses.size();
    for (int i = 0; i < size; ++i)
    {
      geometry_msgs::Point car_pose = msg.poses[i].position;
      if ( euclidean_distance(origin,car_pose) < distance && car_pose.x > 0.0 && abs(car_pose.y) < 6.0)
      {
        point = car_pose;
        distance = euclidean_distance(origin,car_pose);
      }
    }
    // no leader vehicle detected
    if ( distance > 1000 )
    {
      return;
    }
    kf_.x_ << point.x, point.y, 0, 0; // x, y, vx, vy
    previous_timestamp_ = msg.header.stamp.toSec(); // set current time stamp
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/
  
  // compute the time elapsed between the current and previous measurements
  float dt = (msg.header.stamp.toSec() - previous_timestamp_) / 1000000.0;  //  in seconds
  previous_timestamp_ = msg.header.stamp.toSec();

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

  // we need to select the convoy leader vehicle here
  // by gated nearest neighbor data association but reasonable
  geometry_msgs::Point point;
  point.x = numeric_limits<float>::max();
  point.y = numeric_limits<float>::max();
  
  geometry_msgs::Point car_pred;
  car_pred.x = kf_.x_[0];
  car_pred.y = kf_.x_[1];
  float distance = euclidean_distance(point,car_pred);
  int size = msg.poses.size();
  for (int i = 0; i < size; ++i)
  {
    geometry_msgs::Point car_pose = msg.poses[i].position;
    if ( euclidean_distance(car_pred,car_pose) < distance)
    {
      point = car_pose;
      distance = euclidean_distance(car_pred,car_pose);
    }
  }
  // no leader vehicle associated
  if ( distance > 10 )
  {
    return;
  }
  VectorXd z(2);
  z << point.x, point.y;
  kf_.Update(z);
}
