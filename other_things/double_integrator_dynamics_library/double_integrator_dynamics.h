/*
 * double_integrator_dynamics.h
 *
 *  Created on: Aug 15, 2014
 *      Author: ChrisXie
 */

#ifndef DOUBLE_INTEGRATOR_DYNAMICS_H_
#define DOUBLE_INTEGRATOR_DYNAMICS_H_

#include <eigen3/Eigen/Eigen>
using namespace Eigen;

namespace double_integrator_dynamics {

	/*
	 *  This function performs RK4 integration to approximate the solution of
	 *  an ODE given an initial state and input. The integration is performed
	 *  for a duration of delta. f is assumed to be the function of a continuous
	 *  dynamical system.
	 *
	 *  The return value is the approximated state delta time later.
	 */
	VectorXd rk4(VectorXd (*f)(VectorXd, VectorXd), VectorXd x,
			VectorXd u, double delta);

	/*
	 *  This function is a simple function to calculate the velocity given a state
	 *  and input. This is simply x_dot = Ax + Bu.
	 *
	 *  The return value is x_dot.
	 */
	VectorXd continuous_double_integrator_dynamics(VectorXd x, VectorXd u);

	/*
	 *  This function returns the jacobian of the function f w.r.t. the state,
	 *  the input, and delta.
	 *
	 *  The return value is a matrix in the form of
	 *  [J_state, J_input, J_delta]. The user will have to extract these
	 *  individual jacobians his/herself.
	 */
	MatrixXd numerical_jacobian(VectorXd (*f)(VectorXd, VectorXd), VectorXd x,
								VectorXd u, double delta);

	/*
	 *  This function takes as parameters a (possibly) nonlinear dynamics function f,
	 *  a state and input at time t, and the state at time t+1, and time variable delta.
	 *  It calculates the difference in the next_state and the state calculated by
	 *  performing RK4 integration on the dynamics f given the state, input, and delta.
	 *
	 *  The return value is a vector of x_{t+1} - f(x_t, u_t).
	 */
	VectorXd dynamics_difference(VectorXd (*f)(VectorXd, VectorXd), VectorXd x,
								 VectorXd x_next, VectorXd u, double delta);


}



#endif /* DOUBLE_INTEGRATOR_DYNAMICS_H_ */
