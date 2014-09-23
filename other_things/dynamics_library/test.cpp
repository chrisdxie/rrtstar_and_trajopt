/*
 * test.cpp
 *
 *  Created on: Aug 15, 2014
 *      Author: ChrisXie
 */

#define X_DIM 4
#define U_DIM 2

#include <iostream>
#include <math.h>

#include "dynamics_library.hpp"
using namespace dynamics_library;

#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/SVD>
using namespace Eigen;

void mod_cartpole_state(Vector4d& z) {

	// Format: z = [w, v, theta, x] = [theta_dot, x_dot, theta, x]	
	double theta = z(2);

	if (theta > 2*M_PI) {
		theta -= 2*M_PI;
	} else if (theta < 0) {
		theta += 2*M_PI;
	}

	z(2) = theta;

}

int main() {

	Vector4d z;
	z << 4.9368, -2.1114, 4.0367, -5.7592;
	Vector4d x_next;
	x_next << 4, 8, 30, -3;
	VectorXd u(1);
	u << -0.7847;
	double delta = .33;

	double mc = .5;
	double mp = .5;
	double l = .5;
	double b = .1;
	double cw = 0;
	double ch = 0;
	set_cartpole_parameters(mc, mp, l, b, cw, ch);

	VectorXd zdot = continuous_cartpole_dynamics(z, u);
	std::cout << "zdot:\n" << zdot << "\n";

	mod_cartpole_state(z);
	zdot = continuous_cartpole_dynamics(z, u);
	std::cout << "zdot after modding cartpole state:\n" << zdot << "\n";	

	VectorXd z_next = rk4(continuous_cartpole_dynamics, z, u, delta);
	std::cout << "propogated by " << delta << " seconds:\n" << z_next << "\n";

}

