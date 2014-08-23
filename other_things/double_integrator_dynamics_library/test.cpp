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
	x << 4, 3, 0, 0;
	Vector4d x_next;
	x_next << 4, 8, 30, -3;
	Vector2d u;
	u << -1, 1;
	double d = .1;
	double d_safe = 0.05;

	// Must qualify this with the correct namespace, apparently Eigen has this method too, which was a contribution from some user
	MatrixXd jac = -1*double_integrator_dynamics::numerical_jacobian(continuous_double_integrator_dynamics, x, u, d);
	//VectorXd ans = dynamics_difference(continuous_double_integrator_dynamics, x, x_next, u, d);

	Matrix<double, X_DIM, X_DIM> DH_X = jac.leftCols(X_DIM);
	Matrix<double, X_DIM, U_DIM> DH_U = jac.middleCols(X_DIM, U_DIM);
	Matrix<double, X_DIM, 1> DH_delta = jac.rightCols(1);
	Matrix<double, X_DIM, 2*X_DIM+U_DIM+1> DH;
	DH << DH_X, MatrixXd::Identity(X_DIM, X_DIM), DH_U, DH_delta;

	//std::cout << "Answer:\n" << DH << "\n";

	Matrix<double, 4, 3> obstacles; obstacles.setZero();
	obstacles.col(0) << 3, 2, 3, 1;
	obstacles.col(1) << 2, .5, 8, 1.5;
	obstacles.col(2) << 7, 1, 7, 1;

	//MatrixXd v = swept_out_volume_collision_and_jacobian(x, x_next, d_safe, obstacles);
	MatrixXd v = collision_and_jacobian(x, d_safe, obstacles);
//	std::cout << "Answer:\n" << v << "\n";

	x << 6, 7, 8, 9; // Can reassign using << operator
	std::cout << x << "\n";

}

