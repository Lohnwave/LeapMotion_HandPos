#pragma once
#ifndef _WAMIKINE_H_
#define _WAMIKINE_H_
#include <cmath>
#include <vector>
#include <corecrt_math_defines.h>
#include <eigen3/Eigen/Dense>

namespace Ikine {
	/* Barrett WAM D-H parmeters */
	//extern double DH_alpha[] = { -M_PI / 2, M_PI / 2, -M_PI / 2, M_PI / 2, -M_PI / 2, M_PI / 2, 0 };
	//extern double DH_a[] = { 0, 0, 0.045, -0.045, 0, 0, 0 };
	//extern double DH_d[] = { 0, 0, 0.55, 0, 0.3, 0, 0.06 };
	//extern double theta_L[] = { -2.6, -2, -2.8, -0.9, -4.76, -1.6, -3 };
	//extern double theta_U[] = { 2.6, 2, 2.8, 3.1, 1.24, 1.6, 3 };
	//MatrixXd T01, T12, T23, T34, T45, T56, T67, T07;
	//VectorXd qoptim(7);
	/* Get Homogeneous transformation matrix */
	void DHmatrix(double alpha, double a, double d, double &theta, Eigen::MatrixXd &T);
	Eigen::MatrixXd inline DHmatrix(double alpha, double a, double d, double theta);
	/* Initialize Barrett WAM arm using D-H parmeters */
	//void InitiRobot();
	bool isValid(double angles[]);
	bool solve(Eigen::Vector3d position, Eigen::Quaterniond orientation, double phi, double theta[]);
	bool solve_IK(Eigen::Vector3d position, Eigen::Quaterniond orientation,
		std::vector<double> currentjoints, std::vector<double>& joints);
	
}
#endif // !_WAMIKINE_H_
