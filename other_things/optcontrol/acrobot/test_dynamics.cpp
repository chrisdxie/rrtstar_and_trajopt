#include <iostream>
#include "cartpole_dynamics.h"

#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/LU>
using namespace Eigen;

//double initial_state[NX] = { 0.0, 0.0, -0.00116139991132, 0.312921810594 };

//double initial_state[NX] = { 0.0, 0.0, 0.010977943328, 0.120180320014 };
//double control_min[NU] = { -1.0 };
//double control_max[NU] = { 1.0 };
//
//// Target state may only be specified partially.
//// Indices of target state components that are constrained
//int target_ind[NT] = { 0, 1, 2, 3 };
//// Constraint values
//double target_val[NT] = { 0.0, 0.0, 3.14159265359, 0.0 };

int main() {

	VectorXd x(NX);
	VectorXd u(NU);

	x << 4.9368, -2.1114, 4.0367, -5.7592;
	u << -0.7847;

	VectorXd xdot(NX);
	xdot = cartpole::continuous_dynamics(x,u);

	std::cout << xdot << std::endl;

	// ground truth weights corresponding to known system
	return 0;
}
