#include "kalman_filter.h"
#include <iostream>
#include <math.h>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in)
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
  TODO:
    * predict the state
  */

  //cout << "PREDICT: entering predict function" << endl;
  // prediction should be the same for both radar and laser data.  The following code is taken directly from the lecture slides
  //cout << "PREDICT: F matrix" << endl;
  //cout << F_ << endl;
  //cout <<"PREDICT: x vector" << endl;
  //cout << x_ << endl;
  x_ = F_ * x_;
  //cout<<"PREDICT: F*x result"<<endl;
  //cout<<x_<<endl;
  MatrixXd Ft = F_.transpose();
  //cout<<"PREDICT: transposed F matrix"<<endl;
  //cout<<"PREDICT: P matrix"<<endl;
  //cout<<P_<<endl;
  //cout<<"PREDICT: Q matrix"<<endl;
  //cout<<Q_<<endl;
  P_ = F_ * P_ * Ft + Q_;
  //cout<<"PREDICT: Finished predict function"<<endl;
}

void KalmanFilter::Update(const VectorXd &z)
{
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */

  // This update is only for the laser data.  The following code is taken directly from the lecture slides
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z)
{
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */

  //must map state from cartesian to polar
  float precomp1 = x_[0] * x_[2] + x_[1] * x_[3];
  float precomp2 = sqrt(pow(x_[0], 2) + pow(x_[1], 2));
  VectorXd h_x = VectorXd(3);

  h_x << precomp2, atan2(x_[1], x_[0]), precomp1 / precomp2;

  //use regular update equations for the rest.  The H_ matrix should have been set to the Jacobian immediately before this function is called
  VectorXd y = z - h_x;

  //make sure phi is between -PI and PI
  while (y(1) > M_PI)
  {
    y(1) -= 2 * M_PI;
  }
  while (y(1) < -M_PI)
  {
    y(1) += 2 * M_PI;
  }

  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
