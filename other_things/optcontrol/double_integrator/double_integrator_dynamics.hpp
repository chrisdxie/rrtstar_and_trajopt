/*
 * double_integrator_dynamics.hpp
 *
 *  Created on: Aug 15, 2014
 *      Author: ChrisXie
 */

#ifndef DOUBLE_INTEGRATOR_DYNAMICS_HPP_
#define DOUBLE_INTEGRATOR_DYNAMICS_HPP_

#include <eigen3/Eigen/Eigen>
using namespace Eigen;

#include "double_integrator_dynamics.h"

VectorXd double_integrator_dynamics::rk4(VectorXd (*f)(VectorXd, VectorXd),
		VectorXd x, VectorXd u, double delta) {

	VectorXd k1 = delta*f(x, u);
	VectorXd k2 = delta*f(x + .5*k1, u);
	VectorXd k3 = delta*f(x + .5*k2, u);
	VectorXd k4 = delta*f(x + k3, u);

	VectorXd x_new = x + (k1 + 2*k2 + 2*k3 + k4)/6;
	return x_new;

}

VectorXd double_integrator_dynamics::continuous_double_integrator_dynamics(VectorXd x, VectorXd u) {

	int nX = x.size();
	int nU = u.size();

	MatrixXd A(nX, nX);
	A.setZero();
	A.topRightCorner(nX/2, nX/2) = MatrixXd::Identity(nX/2, nX/2);
	MatrixXd B(nX, nU);
	B.setZero();
	B.bottomRows(nX/2) = MatrixXd::Identity(nX/2, nX/2);

	VectorXd x_dot = A*x + B*u;
	return x_dot;

}

MatrixXd double_integrator_dynamics::numerical_jacobian(VectorXd (*f)(VectorXd, VectorXd), VectorXd x, VectorXd u, double delta) {

	double eps = 1e-5;

	int nX = x.size();
	int nU = u.size();

	// Create matrix, set it to all zeros
	MatrixXd jac(nX, nX+nU+1);
	jac.setZero();

	int index = 0;

	MatrixXd I;
	I.setIdentity(nX, nX);
	for(int i = 0; i < nX; ++i) {
		jac.col(index) = rk4(f, x + .5*eps*I.col(i), u, delta) -
						 rk4(f, x - .5*eps*I.col(i), u, delta);
		index++;
	}

	I.setIdentity(nU, nU);
	for(int i = 0; i < nU; ++i) {
		jac.col(index) = rk4(f, x, u + .5*eps*I.col(i), delta) -
						 rk4(f, x, u - .5*eps*I.col(i), delta);
		index++;
	}

	jac.col(index) = rk4(f, x, u, delta + .5*eps) - rk4(f, x, u, delta - .5*eps);

	// Must divide by eps for finite differences formula
	jac /= eps;

	return jac;

}

VectorXd double_integrator_dynamics::dynamics_difference(VectorXd (*f)(VectorXd, VectorXd), VectorXd x, VectorXd x_next, VectorXd u, double delta) {

	int nX = x.size();
	assert(nX == x_next.size()); // Just checking

	VectorXd simulated_x_next = rk4(f, x, u, delta);
	return x_next - simulated_x_next;

}

/* UTILS */

int double_integrator_dynamics::sign(double n) {
	if (n < 0) {
		return -1;
	} else if (n == 0) {
		return 0;
	} else {
		return 1;
	}
}

MatrixXd double_integrator_dynamics::calcJacobians(VectorXd point, VectorXd x, VectorXd x_next) {

	// Preprocess: Get dimension of position, and position vectors only.
	int d = x.size()/2;
	x = x.head(d);
	x_next = x_next.head(d);

	// Jacobians
	MatrixXd jacs(d, 4*d); // Remember, left half is dP/dx, right half is dP/dx_next
	jacs.setZero();

	double alpha = (point - x).norm()/(x_next - x).norm();
	jacs.leftCols(d) = (1-alpha)*MatrixXd::Identity(d, d);
	jacs.middleCols(2*d, d) =  alpha * MatrixXd::Identity(d, d);

	return jacs;

}

#endif /* DOUBLE_INTEGRATOR_DYNAMICS_HPP_ */
