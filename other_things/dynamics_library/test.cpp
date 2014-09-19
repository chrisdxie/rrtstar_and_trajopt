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

int main() {

	Vector4d z;
	z << 0, 2, 0, 0;
	Vector4d x_next;
	x_next << 4, 8, 30, -3;
	VectorXd u(1);
	u << 1;
	double delta = .33;

	double mc = 1;
	double mp = .5;
	double l = 2;
	set_cartpole_parameters(mc, mp, l);

	VectorXd zdot = continuous_cartpole_dynamics(z, u);
	std::cout << "Answer:\n" << zdot << "\n";

	VectorXd z_next = rk4(continuous_cartpole_dynamics, z, u, delta);

}

