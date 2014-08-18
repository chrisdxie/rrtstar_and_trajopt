/*
 * test.cpp
 *
 *  Created on: Aug 15, 2014
 *      Author: ChrisXie
 */

#define X_DIM 4
#define U_DIM 2

#include <iostream>

#include "double_integrator_dynamics.hpp"
using namespace double_integrator_dynamics;

#include <eigen3/Eigen/Eigen>
using namespace Eigen;

int main() {

	Vector4d x;
	x << 4.5, 15, 0, 0;
	Vector4d x_next;
	x_next << 4, 16, 30, -3;
	Vector2d u;
	u << -1, 1;
	double d = .1;

	// Must qualify this with the correct namespace, apparently Eigen has this method too, which was a contribution from some user
	MatrixXd jac = -1*double_integrator_dynamics::numerical_jacobian(continuous_double_integrator_dynamics, x, u, d);
	//VectorXd ans = dynamics_difference(continuous_double_integrator_dynamics, x, x_next, u, d);

	Matrix<double, X_DIM, X_DIM> DH_X = jac.leftCols(X_DIM);
	Matrix<double, X_DIM, U_DIM> DH_U = jac.middleCols(X_DIM, U_DIM);
	Matrix<double, X_DIM, 1> DH_delta = jac.rightCols(1);
	Matrix<double, X_DIM, 2*X_DIM+U_DIM+1> DH;
	DH << DH_X, MatrixXd::Identity(X_DIM, X_DIM), DH_U, DH_delta;

	std::cout << "Answer:\n" << DH << "\n";

	DH.middleRows(0, 1) << 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1;

	std::cout << DH << "\n";

	std::cout << x + 3*MatrixXd::Ones(X_DIM,1) << "\n";

	VectorXd v;
	v.setLinSpaced(10, -4, 4);

	std::cout << (v.cwiseAbs()).sum() << "\n";

	printf("%.3f, %.3f\n", 123456.123456, 5678.79888);

}

