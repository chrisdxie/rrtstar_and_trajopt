/*
 * double_integrator_dynamics.h
 *
 *  Created on: Aug 15, 2014
 *      Author: ChrisXie
 */

#ifndef DYNAMICS_LIBRARY_H_
#define DYNAMICS_LIBRARY_H_

#include <eigen3/Eigen/Eigen>
using namespace Eigen;

struct cartpole_parameters {
	double g;  // Alway 9.81
	double mc; // Cart mass
	double mp; // Pole mass
	double l;  // Pole length
	double b;  // Friction coefficient

	// Useless things (but useful for plotting)
	double cw; // cart width
	double ch; // cart height
};

namespace dynamics_library {

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
	 *  These fields are for the cartpole dynamics.
	 */
 	cartpole_parameters cp_params;

 	/*
 	 *  Set fields of cartpole parameters
 	 */
 	void set_cartpole_parameters(double mc, double mp, double l, double b, double cw, double ch);

	/*
	 *  This function is a function to calculate the derivative of given a state
	 *  and input. The input is the current state z = [x, x_dot, theta, theta_dot]
	 *  and the parameters cart_mass, pole_mass, pole_length.
	 *
	 *  The return value is z_dot.
	 */
	VectorXd continuous_cartpole_dynamics(VectorXd z, VectorXd u);

	/*
	 *  This function is a function to calculate the derivative of given a state
	 *  and input. The input is the current state z = [x, x_dot, theta, theta_dot]
	 *  and the parameters cart_mass, pole_mass, pole_length.
	 *
	 *  The return value is z_dot.
	 */
	 
	VectorXd continuous_rally_car_dynamics(VectorXd z, VectorXd u);

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

	/*
	 *  This function returns the value of the collision function (makes use of
	 *  signed distance). Specifically, it is -1 * SD(x, obstacles) + d_safe. It also
	 *  returns the jacobian that results in taking the first order Taylor series
	 *  approximation of the swept out volume collision function.
	 *
	 *  The matrix "obstacles" should be supplied as a 4 x nO matrix, where nO is the
	 *  number of obstacles. Each column vector represents an obstacle. The column
	 *  has this format: [x1 center, x1 size, x2 center, x2 size]'. For example, the
	 *  obstacle represented by [3, 2, 7, 4] has corners: (2, 5), (2, 9), (4, 9), (4, 5).
	 *
	 *  The first column of the returned matrix is the signed distance between
	 *  the line create by x and x_next evaluated between each obstacle. The rest
	 *  of it is the jacobian. It should be a matrix of dimension nO x nX, where nO
	 *  is the number of obstacles and nX is the dimension of the state vector x.
	 */
	MatrixXd collision_and_jacobian(VectorXd x, double d_safe, MatrixXd obstacles);

	/*
	 *  This function returns the value of the swept out volume collision function.
	 *  Specifically, it is -1 * SD(x, x_next, obstacles) + d_safe. It also
	 *  returns the two jacobians that result in taking the first order
	 *  Taylor series approximation of the swept out volume collision function.
	 *
	 *  The matrix "obstacles" should be supplied as a 4 x nO matrix, where nO is the
	 *  number of obstacles. Each column vector represents an obstacle. The column
	 *  has this format: [x1 center, x1 size, x2 center, x2 size]'. For example, the
	 *  obstacle represented by [3, 2, 7, 4] has corners: (2, 5), (2, 9), (4, 9), (4, 5).
	 *
	 *  The first column of the returned matrix is the signed distance between
	 *  the line create by x and x_next evaluated between each obstacle. Of the next
	 *  part of the matrix, the left half of the matrix is dg/dx, and
	 *  the right half of the matrix is dg/dx_next.
	 */
	MatrixXd swept_out_volume_collision_and_jacobian(VectorXd x, VectorXd x_next, double d_safe, MatrixXd obstacles);

	/* UTILS */

	/*
	 *  Returns the sign of n
	 */
	int sign(double n);

	/*
	 *  Returns the jacobians of the point w.r.t x and x_next. This function is
	 *  specific to the double integrator point robot. It works when all three
	 *  parameters are 2 dimensional.
	 *
	 *  The left half of the return value is dP/dx, the right half is dP/dx_next
	 */
	MatrixXd calcJacobians(VectorXd point, VectorXd x, VectorXd x_next);

}



#endif /* DYNAMICS_LIBRARY_H_ */
