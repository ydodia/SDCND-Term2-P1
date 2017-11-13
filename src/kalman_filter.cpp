#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

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
	x_ = F_ * x_;
	P_ = F_ * P_ * F_.transpose() + Q_;
}

// LASER
void KalmanFilter::Update(const VectorXd &z)
{
	VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred;
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd K = P_ * Ht * Si;

	x_ = x_ + K * y;
	const int x_sz = x_.size();
	MatrixXd I = MatrixXd::Identity(x_sz, x_sz);
	P_ = (I - K * H_) * P_;
}

// RADAR
void KalmanFilter::UpdateEKF(const VectorXd &z)
{
	VectorXd z_pred(3);
	float x = x_(0);
	if (fabs(x) < 0.00001)
	{
		if(x < 0) x = -0.00001;
		else x = 0.00001;
	}
	float rho = sqrt( x*x + x_(1)*x_(1) );
	if (rho < 0.00001)
		rho = 0.00001;
	float phi = atan2(x_(1), x);
	float rho_dot = ( x*x_(2) + x_(1)*x_(3) ) / rho;
	z_pred << rho, phi, rho_dot;
	VectorXd y = z - z_pred;
	y(1) = atan2( sin(y(1)), cos(y(1)) );

	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd K = P_ * Ht * Si;

	x_ = x_ + K * y;
	const int x_sz = x_.size();
	MatrixXd I = MatrixXd::Identity(x_sz, x_sz);
	P_ = (I - K * H_) * P_;
}
