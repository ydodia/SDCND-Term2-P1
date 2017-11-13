#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth)
{
	VectorXd rmse = VectorXd::Zero(4);
	VectorXd diff = VectorXd::Zero(4);
	unsigned int n = estimations.size();
	for (auto k = 0; k < n; ++k)
	{
		diff = estimations[k] - ground_truth[k];
		diff = diff.array().pow(2.0);
		rmse += VectorXd(diff);
	}
	rmse /= n;
	rmse = rmse.array().sqrt();
	//std::cout << rmse << std::endl;
	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state)
{
	MatrixXd jacobian = MatrixXd::Zero(3,4);
	float px = x_state(0), py = x_state(1);
	float vx = x_state(2), vy = x_state(3);
	float px2_py2 = px*px + py*py;
	if (px2_py2 < 0.00001)
		px2_py2 = 0.00001;
	float sqrt_px2_py2 = std::sqrt(px2_py2);
	jacobian(0,0) = px / sqrt_px2_py2;
	jacobian(0,1) = py / sqrt_px2_py2;
	jacobian(1,0) = (-1.0 * py) / px2_py2;
	jacobian(1,1) = px / px2_py2;
	jacobian(2,0) = py * (vx*py - vy*px) / (px2_py2 * sqrt_px2_py2);
	jacobian(2,1) = px * (vy*px - vx*py) / (px2_py2 * sqrt_px2_py2);
	jacobian(2,2) = jacobian(0,0);
	jacobian(2,3) = jacobian(0,1);

	return jacobian;
}
