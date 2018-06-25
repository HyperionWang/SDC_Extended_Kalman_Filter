#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) { // Input is the vector array of VectorXd
  /**
  TODO -- Finished
    * Calculate the RMSE here.
  */
  VectorXd rmse(4); // The RMSE is a four row vector for [Px, Py, Vx, Vy]
  rmse << 0, 0, 0, 0;

  // First, to check validity of input:
  //  the sizes of two input vector should be the same

  if (estimations.size() != ground_truth.size() || estimations.size() == 0) {
    std::cout << "The input for estimation is not valid, please check the size of the estimation input vector" << endl;
    return rmse;
  }

  for (int i = 0; i < estimations.size(); ++i) {

    VectorXd errorVec = estimations[i] - ground_truth[i];
    errorVec = errorVec.array() * errorVec.array();
    rmse += errorVec;
  }
  VectorXd lastVec = estimations[estimations.size()-1] - ground_truth[estimations.size()-1];
  rmse = rmse / estimations.size();
  rmse = rmse.array().sqrt();

//  std::cout<<"RMSE is:"<<rmse(0)<<','<<rmse(1)<<','<<rmse(2)<<','<<rmse(3)<<endl;
//
//  std::cout<<"The error in last run is:" << lastVec<< endl;
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd &x_state) {
  /**
  TODO -- Finished
    * Calculate a Jacobian here.
  */

  MatrixXd JacobbianMatrix(3, 4);
  JacobbianMatrix << 0, 0, 0, 0,
          0, 0, 0, 0,
          0, 0, 0, 0;

  // Start to check the validity of the input x_state

  if (x_state.size() != 4) {
    std::cout <<
              "The size of the x_state is not the same as expected 4, please check the input"<<endl;
    return JacobbianMatrix;
  }
  double px = x_state(0);
  double py = x_state(1);
  double vx = x_state(2);
  double vy = x_state(3);

  //First check if the dividen would be zero

  if (fabs(px*px + py*py) < 0.0001) {

    std::cout<< "The dividen (px*px + py*py) is zero, please check the input"<<endl;
    return JacobbianMatrix;
  }

  JacobbianMatrix(0,0) = px/std::sqrt(px*px + py*py);
  JacobbianMatrix(0,1) = py/std::sqrt(px*px + py*py);
  JacobbianMatrix(1,0) = -py/(px*px + py*py);
  JacobbianMatrix(1,1) = px/(px*px + py*py);
  JacobbianMatrix(2,0) = py*(vx*py - vy*px)/(px*px + py*py)/std::sqrt(px*px + py*py);
  JacobbianMatrix(2,1) = -px*(vx*py - vy*px)/(px*px + py*py)/std::sqrt(px*px + py*py);
  JacobbianMatrix(2,2) = px/std::sqrt(px*px + py*py);
  JacobbianMatrix(2,3) = py/std::sqrt(px*px + py*py);

  return JacobbianMatrix;
}
