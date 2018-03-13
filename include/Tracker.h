#ifndef Tracker_H_
#define Tracker_H_

#include <vector>
#include <string>
#include "kalman_filter.h"
#include "Eigen/Dense"
#include "geometry_msgs/PoseArray.h"

using namespace Eigen;

class Tracker
{
public:
  /**
   * Constructor.
   */
  Tracker();

  /**
   * Destructor.
   */
  virtual ~Tracker();

  /**
   * To Initialize the first frame and then make prediction correction
   */
  void ProcessMeasurement(const geometry_msgs::PoseArray::ConstPtr& msg);

  /**
   * To select the the convoy leader vehicle by gated nearest neighbor data association
   */
  VectorXd DataAssociation(VectorXd &z, const geometry_msgs::PoseArray::ConstPtr& msg);

  /**
   * Kalman Filter update and prediction math lives in here.
   */
  KalmanFilter kf_;

private:
  // check whether the tracking toolbox was initialized or not (first measurement)
  bool is_initialized_;

  // previous timestamp
  float previous_timestamp_;
  Eigen::MatrixXd R_lidar_;
  Eigen::MatrixXd H_lidar_;
};

#endif /* Tracker_H_ */