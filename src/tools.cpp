#include "tools.h"
#include <iostream>
#include <cmath>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::cout;
using std::endl;
using std::vector;
using std::fabs;
using std::sqrt;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */

  VectorXd rmse(4);
  rmse << 0,0,0,0;

  // check the validity of the following inputs:
  unsigned int m = estimations.size();
  unsigned int n = ground_truth.size();

  //  * the estimation vector size should not be zero
  if (m==0)
  {
    cout << "Error - Zero Vector Size" << endl;
    return rmse;
  }

  //  * the estimation vector size should equal ground truth vector size
  if (m!=n)
  {
    cout << "Error - Vector Size Mismatch" << endl;
    return rmse;
  }

  // accumulate squared residuals
  for (unsigned int i=0; i < estimations.size(); ++i)
  {
    VectorXd res = estimations[i] - ground_truth[i];
    VectorXd res_2 = res.array()*res.array();
    rmse += res_2;
  }

  // calculate the mean
  rmse /= m;

  // calculate the squared root
  rmse = rmse.array().sqrt();

  // return the result
  return rmse;

}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */

   MatrixXd Hj(3,4);

  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // check division by zero
  float epsilon = 0.001;
  if ((fabs(px)<epsilon) && (fabs(py)<epsilon))
  {
    cout << "CalculateJacobian () - Error - Division by Zero" << endl;
    px = epsilon;
    py = epsilon;
  }

  // compute the Jacobian matrix
  float ro = sqrt(px*px + py*py);
  float ro_2 = ro*ro;
  float ro_3 = ro_2*ro;

  float dro_dpx = px/ro;
  float dro_dpy = py/ro;
  float dphi_dpx = -py/ro_2;
  float dphi_dpy = px/ro_2;
  float drodot_dpx = py*(vx*py - vy*px)/ro_3;
  float drodot_dpy = px*(vy*px - vx*py)/ro_3;
  float drodot_dvx = dro_dpx;
  float drodot_dvy = dro_dpy;

  Hj << dro_dpx, dro_dpy, 0, 0,
        dphi_dpx, dphi_dpy, 0, 0,
        drodot_dpx, drodot_dpy, drodot_dvx, drodot_dvy;

  return Hj;

}
