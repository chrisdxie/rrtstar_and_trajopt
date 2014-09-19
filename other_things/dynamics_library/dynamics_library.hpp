/*
 * double_integrator_dynamics.hpp
 *
 *  Created on: Aug 15, 2014
 *      Author: ChrisXie
 */

#ifndef DYNAMICS_LIBRARY_HPP_
#define DYNAMICS_LIBRARY_HPP_

#include <math.h>
#include <eigen3/Eigen/Eigen>
using namespace Eigen;

#include "dynamics_library.h"
#include "../2d_signed_distance_library_cpp/signedDistancePolygons.hpp"

VectorXd dynamics_library::rk4(VectorXd (*f)(VectorXd, VectorXd),
		VectorXd x, VectorXd u, double delta) {

	VectorXd k1 = delta * f(x, u);
	VectorXd k2 = delta * f(x + .5*k1, u);
	VectorXd k3 = delta * f(x + .5*k2, u);
	VectorXd k4 = delta * f(x + k3, u);

	VectorXd x_new = x + (k1 + 2*k2 + 2*k3 + k4)/6;
	return x_new;

}

VectorXd dynamics_library::continuous_double_integrator_dynamics(
		VectorXd x, VectorXd u) {

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

void dynamics_library::set_cartpole_parameters(double mc, double mp, double l) {
	cp_params.g = 9.81;
	cp_params.mc = mc;
	cp_params.mp = mp;
	cp_params.l = l;
}

VectorXd dynamics_library::continuous_cartpole_dynamics(VectorXd z, VectorXd u) {

	double control = u(0); // u should be a vector of length 1

	Vector4d zdot; zdot.setZero();

	// Set x_dot and theta_dot first
	zdot(0) = z(1);
	zdot(2) = z(3);

	double mp = cp_params.mp;
	double mc = cp_params.mc;
	double l =  cp_params.l;
	double g =  cp_params.g;

	zdot(1) = (2*mp*l*pow(z(3),2)*sin(z(2)) + 3*mp*g*sin(z(2))*cos(z(2)) + 4*control)
					/ (4*(mc + mp) - 3*mp*pow(cos(z(2)), 2));

	zdot(3) = (-3*mp*l*pow(z(3),2)*sin(z(2))*cos(z(2)) - 6*(mc + mp)*g*sin(z(2)) - 6*control*cos(z(2)))
					/ (4*l*(mc + mp) - 3*mp*l*pow(cos(z(2)),2));

	return zdot;

}

MatrixXd dynamics_library::numerical_jacobian(VectorXd (*f)(VectorXd, VectorXd), VectorXd x,
								VectorXd u, double delta) {

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

VectorXd dynamics_library::dynamics_difference(VectorXd (*f)(VectorXd, VectorXd), VectorXd x,
								 VectorXd x_next, VectorXd u, double delta) {

	int nX = x.size();
	assert(nX == x_next.size()); // Just checking

	VectorXd simulated_x_next = rk4(f, x, u, delta);
	return x_next - simulated_x_next;

}

MatrixXd dynamics_library::collision_and_jacobian(VectorXd x, double d_safe, MatrixXd obstacles) {

	int nO = obstacles.cols();
	int nX = x.size();
	Matrix<double, 3, 5> ret; ret.setZero();

	for(int i = 0; i < nO; i++) {

		// Convert obstacle into it's corners
		Matrix<double, 4, 2> obs; obs.setZero();
		double x_min = obstacles(0,i) - .5*obstacles(1,i);
		double x_max = obstacles(0,i) + .5*obstacles(1,i);
		double y_min = obstacles(2,i) - .5*obstacles(3,i);
		double y_max = obstacles(2,i) + .5*obstacles(3,i);
		obs.row(0) << x_min, y_min; obs.row(1) << x_min, y_max;
		obs.row(2) << x_max, y_max; obs.row(3) << x_max, y_min;

		// Create 2D polygon spanned by x
		Matrix<double, 2, 2> traj_line; traj_line.setZero();
		traj_line.row(0) << x(0), x(1);
		traj_line.row(1) << x(0), x(1);

		// Call signed distance function, retrieve value.
		Matrix<double, 3, 2> temp = signedDistancePolygons(traj_line, obs);
		ret(i, 0) = d_safe - temp(0,0);

		// Compute jacobian (translated from Matlab code in CS287 HW3)
		Vector2d point_on_robot = temp.row(1);
		Vector2d point_on_obs = temp.row(2);
		Vector2d normal_obs_to_robot = point_on_robot - point_on_obs;
		if ((normal_obs_to_robot.cwiseAbs()).sum() != 0) {
			normal_obs_to_robot.normalize(); // Dangerous operation, if vector of zeros, then will return NaNs
		} else {
			normal_obs_to_robot << 0, 0; // Set to 0
		}
		normal_obs_to_robot *= -1 * sign(temp(0,0));

		Matrix<double, 2, 4> pt_jac(nX/2, nX); pt_jac.setZero();
		pt_jac.leftCols(nX/2) = MatrixXd::Identity(nX/2, nX/2);

		ret.block(i, 1, 1, nX) = normal_obs_to_robot.transpose() * pt_jac;

	}

	return ret;

}

MatrixXd dynamics_library::swept_out_volume_collision_and_jacobian(VectorXd x,
		VectorXd x_next, double d_safe, MatrixXd obstacles) {

	int nO = obstacles.cols();
	int nX = x.size();
	//MatrixXd ret(nO, 2*nX+1); ret.setZero();
	// For debugging:
	Matrix<double, 3, 9> ret; ret.setZero();

	for(int i = 0; i < nO; i++) {

		// Convert obstacle into it's corners
		Matrix<double, 4, 2> obs; obs.setZero();
		double x_min = obstacles(0,i) - .5*obstacles(1,i);
		double x_max = obstacles(0,i) + .5*obstacles(1,i);
		double y_min = obstacles(2,i) - .5*obstacles(3,i);
		double y_max = obstacles(2,i) + .5*obstacles(3,i);
		obs.row(0) << x_min, y_min; obs.row(1) << x_min, y_max;
		obs.row(2) << x_max, y_max; obs.row(3) << x_max, y_min;

		// Compute line (2D polygon) spanned by x and x_next
		Matrix<double, 2, 2> traj_line; traj_line.setZero();
		traj_line.row(0) << x(0), x(1);
		traj_line.row(1) << x_next(0), x_next(1);

		// Call signed distance function, retrieve value.
		Matrix<double, 3, 2> temp = signedDistancePolygons(traj_line, obs);
		ret(i, 0) = d_safe - temp(0,0);

		// Compute jacobian (translated from Matlab code in CS287 HW3)
		Vector2d point_on_SWVolume = temp.row(1);
		Vector2d point_on_obs = temp.row(2);
		Vector2d normal_obs_to_robot = point_on_SWVolume - point_on_obs;
		if ((normal_obs_to_robot.cwiseAbs()).sum() != 0) {
			normal_obs_to_robot.normalize(); // Dangerous operation, if vector of zeros, then will return NaNs
		} else {
			normal_obs_to_robot << 0, 0; // Set to 0
		}
		normal_obs_to_robot *= -1 * sign(temp(0,0));

		Matrix<double, 2, 8> jacs = calcJacobians(point_on_SWVolume, x, x_next);
		ret.block(i, 1, 1, nX) = normal_obs_to_robot.transpose() * jacs.leftCols(nX);
		ret.block(i, nX+1, 1, nX) = normal_obs_to_robot.transpose() * jacs.middleCols(nX, nX);

	}

	return ret;

}


/* UTILS */

int dynamics_library::sign(double n) {
	if (n < 0) {
		return -1;
	} else if (n == 0) {
		return 0;
	} else {
		return 1;
	}
}

MatrixXd dynamics_library::calcJacobians(VectorXd point, VectorXd x, VectorXd x_next) {

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

#endif /* DYNAMICS_LIBRARY_HPP_ */
