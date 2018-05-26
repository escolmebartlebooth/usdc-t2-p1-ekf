#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize
// VectorXd or MatrixXd objects with zeros upon creation.

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
  */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;

}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  std::cout << "1" << std::endl;
  VectorXd z_pred = H_ * x_;
  std::cout << "2" << std::endl;
  VectorXd y = z - z_pred;
  std::cout << "3" << std::endl;
  MatrixXd Ht = H_.transpose();
  std::cout << "4" << std::endl;
  MatrixXd S = H_ * P_ * Ht + R_;
  std::cout << "5" << std::endl;
  MatrixXd SI = S.inverse();
  std::cout << "6" << std::endl;
  MatrixXd K = P_ * Ht * SI;
  std::cout << "7" << std::endl;
  long x_size = x_.size();
  std::cout << "8" << std::endl;
  MatrixXd I = MatrixXd::Identity(x_size, x_size);

  std::cout << "9" << std::endl;
  x_ = x_ + (K * y);
  std::cout << "10" << std::endl;
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);

  float rho = sqrt(px*px + py*py);
  float theta = atan2(py,px);
  float rho_dot = (px*vx+py*vy)/rho;

  VectorXd z_pred = VectorXd(3);
  z_pred << rho, theta, rho_dot;

  VectorXd y = z - z_pred;

  //should normalise the angle here y(1)
  double pi = 3.1415926535897;
  float new_theta = y(1);
  if (new_theta < -pi) {
    while (new_theta < -pi){
      new_theta += 2 * pi;
    }
  } else if (new_theta > pi) {
    while (new_theta > pi){
      new_theta -= 2 * pi;
    }
  }
  //update phi in y - even though i called it theta...
  y(1) = new_theta;

  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd SI = S.inverse();
  MatrixXd K = P_ * Ht * SI;
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);

  x_ = x_ + (K * y);
  P_ = (I - K * H_) * P_;
}
