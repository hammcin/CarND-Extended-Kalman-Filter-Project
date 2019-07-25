#include "kalman_filter.h"
#include <cmath>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::sqrt;
using std::atan2;
using std::fabs;

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
  /**
   * TODO: predict the state
   */

  // predict the state
  x_ = F_ * x_;

  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;

}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */

  // update the state by using Kalman Filter equations
  VectorXd y = z - H_ * x_;

  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;

  MatrixXd Si = S.inverse();
  MatrixXd K = P_ * Ht * Si;

  // new estimate
  x_ = x_ + (K * y);

  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */

  // Retrieve predicted position and speed from predicted state
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);

  // check division by zero
  float epsilon = 0.001;
  if ((fabs(px)<epsilon) && (fabs(py)<epsilon))
  {
    px = epsilon;
    py = epsilon;
  }

  // map predicted position and speed to polar coordinates
  VectorXd z_pred(3);
  z_pred(0) = sqrt(px*px + py*py);
  z_pred(1) = atan2(py, px);
  z_pred(2) = (px*vx + py*vy)/sqrt(px*px + py*py);
  VectorXd y = z - z_pred;

  // normalize angle
  const double PI = 3.14159265358979323846;
  while (y(1)>PI)
  {
    y(1) = y(1) - (2*PI);
  }
  while (y(1)<(-PI))
  {
    y(1) = y(1) + (2*PI);
  }

  // update the state by using Extended Kalman Filter equations
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;

  MatrixXd Si = S.inverse();
  MatrixXd K = P_ * Ht * Si;

  // new estimate
  x_ = x_ + (K * y);

  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
  
}
