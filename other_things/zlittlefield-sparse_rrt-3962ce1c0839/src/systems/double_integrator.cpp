/**
 * @file point.cpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Zakary Littlefield, Kostas Bekris 
 * 
 */

#include "systems/double_integrator.hpp"
#include "utilities/random.hpp"
#include <cmath>

#include <eigen3/Eigen/Eigen>
using namespace Eigen;

// State: z = [x, y, v_x, v_y] = [x, y, x_dot, y_dot]
#define MIN_X -10
#define MAX_X 10
#define MIN_Y -10
#define MAX_Y 10
#define MIN_V_X -1
#define MAX_V_X 1
#define MIN_V_Y -1
#define MAX_V_Y 1

#define MIN_U -1
#define MAX_U 1

VectorXd rk4(VectorXd (*f)(VectorXd, VectorXd),
		VectorXd x, VectorXd u, double delta) {

	VectorXd k1 = delta * f(x, u);
	VectorXd k2 = delta * f(x + .5*k1, u);
	VectorXd k3 = delta * f(x + .5*k2, u);
	VectorXd k4 = delta * f(x + k3, u);

	VectorXd x_new = x + (k1 + 2*k2 + 2*k3 + k4)/6;
	return x_new;

}

VectorXd continuous_double_integrator_dynamics(
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

double double_integrator_t::distance(double* point1,double* point2)
{
	return std::sqrt( (point1[0]-point2[0]) * (point1[0]-point2[0]) + (point1[1]-point2[1]) * (point1[1]-point2[1]) 
					+ (point1[2]-point2[2]) * (point1[2]-point2[2]) + (point1[3]-point2[3]) * (point1[3]-point2[3]));
}

void double_integrator_t::random_state(double* state)
{
	state[0] = uniform_random(MIN_X,MAX_X);
	state[1] = uniform_random(MIN_Y,MAX_Y);
	state[2] = uniform_random(MIN_V_X,MAX_V_X);
	state[3] = uniform_random(MIN_V_Y,MAX_V_Y);

}

void double_integrator_t::random_control(double* control)
{
	control[0] = uniform_random(MIN_U,MAX_U);
	control[1] = uniform_random(MIN_U,MAX_U);
}

bool double_integrator_t::propagate( double* start_state, double* control, int min_step, int max_step, double* result_state, double& duration )
{

	temp_state[0] = start_state[0];
	temp_state[1] = start_state[1];
	temp_state[2] = start_state[2];
	temp_state[3] = start_state[3];

    VectorXd z(4);
    z << start_state[0], start_state[1], start_state[2], start_state[3];
    VectorXd u(2);
    u << control[0], control[1];
    double delta = uniform_int_random(min_step,max_step) * params::integration_step;

    VectorXd next_state(4); next_state.setZero();
    next_state = rk4(continuous_double_integrator_dynamics, z, u, delta);

    result_state[0] = next_state(0);
    result_state[1] = next_state(1);
    result_state[2] = next_state(2);
    result_state[3] = next_state(3);

	bool validity = valid_state(result_state);

    duration = delta;	

	return validity;
}


bool double_integrator_t::valid_state(double* state)
{

	VectorXd curr_pos(2), end_pos(2);
	curr_pos << temp_state[0], temp_state[1];
	end_pos << state[0], state[1];

	bool obstacle_collision = false;
	int num_discretization = 5;
	for (int j = 1; j <= num_discretization; j++) {
		VectorXd direction = (end_pos - curr_pos)/(end_pos - curr_pos).norm();
		double length = (end_pos - curr_pos).norm();
		Vector2d point = curr_pos + length * j/num_discretization * direction;				
		for(unsigned i=0;i<obstacles.size() && !obstacle_collision;i++)
		{
			if(	point(0) > obstacles[i].low_x && 
				point(0) < obstacles[i].high_x && 
				point(1) > obstacles[i].low_y && 
				point(1) < obstacles[i].high_y)
			{
				obstacle_collision = true;
			}
		}
	}

	bool inBounds = false;
	// Check bounds
	if(	state[0] > MIN_X && 
		state[0] < MAX_X && 
		state[1] > MIN_Y && 
		state[1] < MAX_Y &&
		state[2] > MIN_V_X && 
		state[2] < MAX_V_X &&
		state[3] > MIN_V_Y && 
		state[3] < MAX_V_Y)
	{
		inBounds = true;
	}

	return !obstacle_collision && inBounds;
}

bool double_integrator_t::valid_state() {
	return true;
}

void double_integrator_t::enforce_bounds() {
	return;
}

svg::Point double_integrator_t::visualize_point(double* state, svg::Dimensions dims)
{
	double x = (state[0]-MIN_X)/(MAX_X-MIN_X) * dims.width; 
	double y = (state[1]-MIN_Y)/(MAX_Y-MIN_Y) * dims.height; 
	return svg::Point(x,y);
}

void double_integrator_t::visualize_obstacles(svg::Document& doc ,svg::Dimensions dims)
{// doc << Rectangle(Point(-4.5*50+500-175,5*50+500+37.5), 7*50, 1.5*50, Color::Red);
	double temp[2];
	for(unsigned i=0;i<obstacles.size();i++)
	{
		temp[0] = obstacles[i].low_x;
		temp[1] = obstacles[i].high_y;
		doc<<svg::Rectangle(visualize_point(temp,dims), 
							(obstacles[i].high_x-obstacles[i].low_x)/(MAX_X-MIN_X) * dims.width,
							(obstacles[i].high_y-obstacles[i].low_y)/(MAX_Y-MIN_Y) * dims.height,
							svg::Color::Red);
	}
}
