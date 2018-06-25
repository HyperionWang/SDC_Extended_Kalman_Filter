#include <iostream>
#include "kalman_filter.h"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in; // estimation
  P_ = P_in; // uncertainty covariance
  F_ = F_in; // State transition machine
  H_ = H_in; // Measurement Function
  R_ = R_in; // Measurement Noise
  Q_ = Q_in; // Covariance matrix
}

void KalmanFilter::Predict() {
  /**
  TODO -- Finished
    * predict the state
  */

  x_ = F_ * x_;
  // Here to assume the mean of accelerate is 0
  // and all the acceleration is included in the speed error and it follows Guassion Distribution)
  P_ = F_ * P_ * F_.transpose() + Q_;
//  std::cout << "Prediction:" << endl;
//  std::cout << "x_ = " << x_ << endl;
//  std::cout << "P_ = " << P_ << endl;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO -- Finished
    * update the state by using Kalman Filter equations
  */

  VectorXd y = z - H_ * x_;
  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();

  x_ = x_ + K * y;
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
//  std::cout << "KF Updates:" << endl;
//  std::cout << "x_ = " << x_ << endl;
//  std::cout << "P_ = " << P_ << endl;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO -- Finished
    * update the state by using Extended Kalman Filter equations
  */

  Tools tools;

  VectorXd h = car2pol(x_);

  //  std::cout << "The angle is:" << h(1) << endl;
  VectorXd y = z - h;

  while (y(1) > PI || y(1) < -PI) {
    if (y(1) > PI) {
      y(1) = y(1) - 2 * PI;
      std::cout << y(1) << endl;
    }

    if (y(1) < -PI) {
      y(1) = y(1) + 2 * PI;
      std::cout << y(1) << endl;
    }
  }
//  std::cout << "The angles are (h,z,y) :" << h(1) << ',' << z(1) << ',' << y(1) << endl;
//  std::cout << "Start to get the Jacobian Array" << endl;
  MatrixXd JH = tools.CalculateJacobian(x_);
//  std::cout << "Size of Jacobian Array is:" << JH.size() << endl;
  MatrixXd JHT = JH.transpose();
  MatrixXd S = JH * P_ * JHT + R_;
//  std::cout << "Finished to get the Jacobian Array" << endl;
  MatrixXd K = P_ * JH.transpose() * S.inverse();

  x_ = x_ + K * y;

  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);

  P_ = (I - K * JH) * P_;
//  std::cout << "EKF Updates:" << endl;
//  std::cout << "x_ = " << x_ << endl;
//  std::cout << "P_ = " << P_ << endl;
}


VectorXd KalmanFilter::car2pol(VectorXd &x_state) {


  VectorXd h(3);
  h << 0, 0, 0;
  auto px = x_state(0);
  auto py = x_state(1);
  auto vx = x_state(2);
  auto vy = x_state(3);

  //First check if the dividen would be zero

  if (fabs(px * px + py * py) < 0.0001 || fabs(px) < 0.0001) {

    std::cout << "The dividen (px*px + py*py) is zero, please check the input" << endl;
    return h;
  }

  h(0) = std::sqrt(px * px + py * py);
  h(1) = std::atan2(py, px);
  h(2) = (px * vx + py * vy) / std::sqrt(px * px + py * py);

  return h;
}