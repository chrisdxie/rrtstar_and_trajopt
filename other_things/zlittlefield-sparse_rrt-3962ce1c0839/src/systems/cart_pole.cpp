/**
 * @file cart_pole.cpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Zakary Littlefield, Kostas Bekris 
 * 
 */


#include "systems/cart_pole.hpp"
#include "utilities/random.hpp"

#define _USE_MATH_DEFINES

#include <eigen3/Eigen/Eigen>
using namespace Eigen;

//#include "../../../2d_signed_distance_library_cpp/signedDistancePolygons.hpp"
//#include "../../../dynamics_library/dynamics_library.hpp"
//using namespace dynamics_library;

#include <cmath>

#define I 10 // Doesn't matter since I use my dynamics
#define L .25
#define M .5
#define m .5
#define g 9.8

#define STATE_X 0
#define STATE_V 1
#define STATE_THETA 2
#define STATE_W 3
#define CONTROL_A 0

#define MIN_X -6
#define MAX_X 6
#define MIN_V -10
#define MAX_V 10
#define MIN_W -10
#define MAX_W 10
#define MIN_U -10
#define MAX_U 10

double cart_pole_t::distance(double* point1,double* point2)
{
        double val = fabs(point1[STATE_THETA]-point2[STATE_THETA]);
        if(val > M_PI)
                val = 2*M_PI-val;
        return std::sqrt( val * val + pow(point1[0]-point2[0],2.0) + pow(point1[1]-point2[1],2.0)+ pow(point1[3]-point2[3],2.0) );
}

void cart_pole_t::random_state(double* state)
{
        state[0] = uniform_random(MIN_X,MAX_X);
        state[1] = uniform_random(MIN_V,MAX_V);
        state[2] = uniform_random(-M_PI,M_PI);
        state[3] = uniform_random(MIN_W,MAX_W);
}

void cart_pole_t::random_control(double* control)
{
        control[0] = uniform_random(MIN_U,MAX_U);
}

bool cart_pole_t::propagate( double* start_state, double* control, int min_step, int max_step, double* result_state, double& duration )
{

        // The state for this is [x, xdot, theta, thetadot]
        // But I use             [thetadot, xdot, theta, x]

        /*
        temp_state[0] = start_state[0];
        temp_state[1] = start_state[1];
        temp_state[2] = start_state[2];
        temp_state[3] = start_state[3];

        double mc = .5;
        double mp = .5;
        double l = .5;
        double b = .1;
        double cw = .5;
        double ch = .2;
        set_cartpole_parameters(mc, mp, l, b, cw, ch);

        VectorXd z(4);
        z << start_state[3], start_state[1], start_state[2], start_state[0];
        VectorXd u(1);
        u << control[CONTROL_A];
        double delta = uniform_int_random(min_step,max_step) * params::integration_step;

        VectorXd next_state(4); next_state.setZero();
        next_state = rk4(continuous_cartpole_dynamics, z, u, delta);

        result_state[3] = next_state(0);
        result_state[1] = next_state(1);
        result_state[2] = next_state(2);
        result_state[0] = next_state(3);

        duration = delta;

        bool validity = valid_state(result_state);

        return validity;
        */

        // STUPID STUPID STUPID
        return true;

        /*
        temp_state[0] = start_state[0]; 
        temp_state[1] = start_state[1];
        temp_state[2] = start_state[2];
        temp_state[3] = start_state[3];
        int num_steps = uniform_int_random(min_step,max_step);
        bool validity = true;
        for(int i=0;i<num_steps;i++)
        {
                update_derivative(control);
                temp_state[0] += params::integration_step*deriv[0];
                temp_state[1] += params::integration_step*deriv[1];
                temp_state[2] += params::integration_step*deriv[2];
                temp_state[3] += params::integration_step*deriv[3];
                enforce_bounds();
                validity = validity && valid_state();
        }
        result_state[0] = temp_state[0];
        result_state[1] = temp_state[1];
        result_state[2] = temp_state[2];
        result_state[3] = temp_state[3];
        duration = num_steps*params::integration_step;
        return validity;
        */
}

void cart_pole_t::enforce_bounds()
{
        if(temp_state[0]<MIN_X)
                temp_state[0]=MIN_X;
        else if(temp_state[0]>MAX_X)
                temp_state[0]=MAX_X;

        if(temp_state[1]<MIN_V)
                temp_state[1]=MIN_V;
        else if(temp_state[1]>MAX_V)
                temp_state[1]=MAX_V;

        if(temp_state[2]<-M_PI)
                temp_state[2]+=2*M_PI;
        else if(temp_state[2]>M_PI)
                temp_state[2]-=2*M_PI;

        if(temp_state[3]<MIN_W)
                temp_state[3]=MIN_W;
        else if(temp_state[3]>MAX_W)
                temp_state[3]=MAX_W;
}


bool cart_pole_t::valid_state(double* state)
{
    return true; // gotta use this stuff in other class... stupid object linking stuff

    // The state for this is [x, xdot, theta, thetadot] = [x, v, theta, w]

    /* Compute line (2D polygon) spanned by x and x_next.
     * The polygon created is represent by 4 points: the cart positions (beginning of pole)
     * and the end of the pole. traj_line will look like:
     *
     * [x0, 0;
     *  x1, 0;
     *  p1x, p1y;
     *  p2x, p2y]
     *
     * x0 < x1, and p1x > p2x
     */

     /*
    Matrix<double, 2, 2> traj_line;
    traj_line.setZero();
    traj_line.row(0) << state[0], 0;

    double length = .5;
    double p_new_x = length * sin(state[2]) + state[0];
    double p_new_y = -1*length * cos(state[2]);

    traj_line.row(1) << p_new_x, p_new_y;

    // Actual obstacle detection is here
    bool obstacle_collision_individ = false;
    for(unsigned i=0; i<obstacles.size() && !obstacle_collision_individ;i++) {

        Matrix<double, 4, 2> obs;
        obs.setZero(4,2);

        obs.row(0) << obstacles[i].low_x, obstacles[i].low_y; 
        obs.row(1) << obstacles[i].low_x, obstacles[i].high_y;
        obs.row(2) << obstacles[i].high_x, obstacles[i].high_y; 
        obs.row(3) << obstacles[i].high_x, obstacles[i].low_y;

        // Call signed distance function, retrieve value.
        Matrix<double, 3, 2> temp = signed_distance_2d::signedDistancePolygons(traj_line, obs);

        if (temp(0,0) < 0) {
            obstacle_collision_individ = true;
        }
    }

    bool obstacle_collision_swv = false;
    Matrix<double, 4, 2> swv_poly;
    swv_poly.setZero();
    if (temp_state[0] < state[0]) {
        swv_poly.row(0) << temp_state[0], 0;
        swv_poly.row(1) << state[0], 0;
    } else {
        swv_poly.row(0) << state[0], 0;
        swv_poly.row(1) << temp_state[0], 0;
    }

    double p_near_x = length * sin(temp_state[2]) + temp_state[0];
    double p_near_y = -1*length * cos(temp_state[2]);
    p_new_x = length * sin(state[2]) + state[0];
    p_new_y = -1*length * cos(state[2]);    

    if (p_near_x > p_new_x) {
        swv_poly.row(2) << p_near_x, p_near_y;
        swv_poly.row(3) << p_new_x, p_new_y;
    } else {
        swv_poly.row(2) << p_new_x, p_new_y;
        swv_poly.row(3) << p_near_x, p_near_y;
    }

    for(unsigned i=0; i<obstacles.size() && !obstacle_collision_swv;i++) {

        Matrix<double, 4, 2> obs;
        obs.setZero(4,2);

        obs.row(0) << obstacles[i].low_x, obstacles[i].low_y; 
        obs.row(1) << obstacles[i].low_x, obstacles[i].high_y;
        obs.row(2) << obstacles[i].high_x, obstacles[i].high_y; 
        obs.row(3) << obstacles[i].high_x, obstacles[i].low_y;

        // Call signed distance function, retrieve value.
        Matrix<double, 3, 2> temp = signed_distance_2d::signedDistancePolygons(swv_poly, obs);

        if (std::isfinite(temp(0,0)) && temp(0,0) < 0) {
            obstacle_collision_swv = true;
        }

    }

    bool inBounds = false;
    // Check bounds
    if( state[0] > MIN_X && 
        state[0] < MAX_X && 
        state[1] > MIN_V && 
        state[1] < MAX_V &&
        state[3] > MIN_W && 
        state[3] < MAX_W)
    {
        inBounds = true;
    }

    // Put state[2] into [-pi, pi] range
    if(state[2] < -M_PI) {
        state[2] += 2*M_PI;
    }
    else if (state[2] > M_PI) {
        state[2] -= 2*M_PI;
    }

    return !obstacle_collision_swv && !obstacle_collision_individ && inBounds;
    */    

}

bool cart_pole_t::valid_state() {
    return true;
}

svg::Point cart_pole_t::visualize_point(double* state, svg::Dimensions dims)
{
        double x = state[STATE_X] + (L / 2.0) * sin(state[STATE_THETA]);
        double y = -(L / 2.0) * cos(state[STATE_THETA]);

        x = (x-MIN_X)/(MAX_X-MIN_X) * dims.width; 
        // y = (y+L)/(2*L) * dims.height; 
        y = (y-MIN_X)/(MAX_X-MIN_X) * dims.height; 
        return svg::Point(x,y);
}

void cart_pole_t::update_derivative(double* control)
{
    double _v = temp_state[STATE_V];
    double _w = temp_state[STATE_W];
    double _theta = temp_state[STATE_THETA];
    double _a = control[CONTROL_A];
    double mass_term = (M + m)*(I + m * L * L) - m * m * L * L * cos(_theta) * cos(_theta);

    deriv[STATE_X] = _v;
    deriv[STATE_THETA] = _w;
    mass_term = (1.0 / mass_term);
    deriv[STATE_V] = ((I + m * L * L)*(_a + m * L * _w * _w * sin(_theta)) + m * m * L * L * cos(_theta) * sin(_theta) * g) * mass_term;
    deriv[STATE_W] = ((-m * L * cos(_theta))*(_a + m * L * _w * _w * sin(_theta))+(M + m)*(-m * g * L * sin(_theta))) * mass_term;
}


