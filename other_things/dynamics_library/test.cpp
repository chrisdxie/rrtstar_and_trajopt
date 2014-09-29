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

	VectorXd z(8);
	z << 18.0002, -29.3624, 0.00550246, 15.8682, 1.57, -8.15236e-05, 1.77549, 40;
	VectorXd u(3);
	u << -0.178751, -652.562, 1186.94;
	double delta = .33;

	double mc = .5;
	double mp = .5;
	double l = .5;
	double b = .1;
	double cw = 0;
	double ch = 0;
	set_cartpole_parameters(mc, mp, l, b, cw, ch);

	VectorXd zdot = continuous_rally_car_dynamics(z, u);
	std::cout << "zdot:\n" << zdot << "\n";

	VectorXd z_next = rk4(continuous_rally_car_dynamics, z, u, delta);
	std::cout << "Propagated by " << delta << " seconds:\n" << z_next << "\n";



}

