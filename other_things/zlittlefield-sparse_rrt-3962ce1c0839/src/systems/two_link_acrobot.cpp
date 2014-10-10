/**
 * @file two_link_acrobot.cpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Zakary Littlefield, Kostas Bekris 
 * 
 */


#include "systems/two_link_acrobot.hpp"
#include "utilities/random.hpp"
#include <eigen3/Eigen/Eigen>
using namespace Eigen;

#include "../../../2d_signed_distance_library_cpp/signedDistancePolygons.hpp"

#define _USE_MATH_DEFINES


#include <cmath>

#define LENGTH 1.0
#define m 1.0

#define lc  .5
#define lc2  .25
#define l2  1
#define I1  0.2
#define I2  1.0
#define l  1.0
#define g  9.8

// z = [theta_1, theta_2, theta_dot_1, theta_dot_2]

#define STATE_THETA_1 0
#define STATE_THETA_2 1
#define STATE_V_1 2
#define STATE_V_2 3
#define CONTROL_T 0

#define MIN_V_1 -6
#define MAX_V_1 6
#define MIN_V_2 -6
#define MAX_V_2 6
#define MIN_T -8
#define MAX_T 8

VectorXd _rk4(VectorXd (*f)(VectorXd, VectorXd),
        VectorXd x, VectorXd u, double delta) {

    VectorXd k1 = delta * f(x, u);
    VectorXd k2 = delta * f(x + .5*k1, u);
    VectorXd k3 = delta * f(x + .5*k2, u);
    VectorXd k4 = delta * f(x + k3, u);

    VectorXd x_new = x + (k1 + 2*k2 + 2*k3 + k4)/6;
    return x_new;

}

VectorXd _continuous_acrobot_dynamics(VectorXd z, VectorXd u) {


    // z = [theta_1, theta_2, theta_dot_1, theta_dot_2]

    Vector4d zdot; zdot.setZero();

    double theta2 = z(1);
    double theta1 = z(0) - M_PI / 2;
    double theta1dot = z(2);
    double theta2dot = z(3);
    double _tau = u(0);

    //extra term m*lc2
    double d11 = m * lc2 + m * (l2 + lc2 + 2 * l * lc * cos(theta2)) + I1 + I2;

    double d22 = m * lc2 + I2;
    double d12 = m * (lc2 + l * lc * cos(theta2)) + I2;
    double d21 = d12;

    //extra theta1dot
    double c1 = -m * l * lc * theta2dot * theta2dot * sin(theta2) - (2 * m * l * lc * theta1dot * theta2dot * sin(theta2));
    double c2 = m * l * lc * theta1dot * theta1dot * sin(theta2);
    double g1 = (m * lc + m * l) * g * cos(theta1) + (m * lc * g * cos(theta1 + theta2));
    double g2 = m * lc * g * cos(theta1 + theta2);

    zdot(0) = theta1dot;
    zdot(1) = theta2dot;

    double u2 = _tau - 1 * .1 * theta2dot;
    double u1 = -1 * .1 * theta1dot;
    double theta1dot_dot = (d22 * (u1 - c1 - g1) - d12 * (u2 - c2 - g2)) / (d11 * d22 - d12 * d21);
    double theta2dot_dot = (d11 * (u2 - c2 - g2) - d21 * (u1 - c1 - g1)) / (d11 * d22 - d12 * d21);

    //std::cout << "SST: " << theta1dot_dot << " " << theta2dot_dot << "\n"; 

    double _theta1dot_dot = (-d12 * (_tau - c2 - g2) - d22 * (c1 + g1)) / (d11 * d22 - d12 * d12); 
    double _theta2dot_dot = (d11 * (_tau - c2 - g2) + d12 * (c1 + g1)) / (d11 * d22 - d12 * d12);

    //std::cout << "GAT: " << _theta1dot_dot << " " << _theta2dot_dot << "\n";

    zdot(2) = theta1dot_dot;
    zdot(3) = theta2dot_dot;

    return zdot;

}

double two_link_acrobot_t::distance(double* point1,double* point2)
{
        double x = (LENGTH) * cos(point1[STATE_THETA_1] - M_PI / 2)+(LENGTH) * cos(point1[STATE_THETA_1] + point1[STATE_THETA_2] - M_PI / 2);
        double y = (LENGTH) * sin(point1[STATE_THETA_1] - M_PI / 2)+(LENGTH) * sin(point1[STATE_THETA_1] + point1[STATE_THETA_2] - M_PI / 2);
        double x2 = (LENGTH) * cos(point2[STATE_THETA_1] - M_PI / 2)+(LENGTH) * cos(point2[STATE_THETA_1] + point2[STATE_THETA_2] - M_PI / 2);
        double y2 = (LENGTH) * sin(point2[STATE_THETA_1] - M_PI / 2)+(LENGTH) * sin(point2[STATE_THETA_1] + point2[STATE_THETA_2] - M_PI / 2);
        return std::sqrt(pow(x-x2,2.0)+pow(y-y2,2.0));
}

void two_link_acrobot_t::random_state(double* state)
{
        state[0] = uniform_random(-M_PI,M_PI);
        state[1] = uniform_random(-M_PI,M_PI);
        state[2] = uniform_random(MIN_V_1,MAX_V_1);
        state[3] = uniform_random(MIN_V_2,MAX_V_2);
}

void two_link_acrobot_t::random_control(double* control)
{
        control[0] = uniform_random(MIN_T,MAX_T);
}

bool two_link_acrobot_t::propagate( double* start_state, double* control, int min_step, int max_step, double* result_state, double& duration )
{

        temp_state[0] = start_state[0];
        temp_state[1] = start_state[1];
        temp_state[2] = start_state[2];
        temp_state[3] = start_state[3];

        VectorXd z(4);
        z << start_state[0], start_state[1], start_state[2], start_state[3];
        VectorXd u(1);
        u << control[0];

        double delta = uniform_int_random(min_step,max_step) * params::integration_step;

        VectorXd next_state(4); next_state.setZero();
        next_state = _rk4(_continuous_acrobot_dynamics, z, u, delta);

        result_state[0] = next_state(0);
        result_state[1] = next_state(1);
        result_state[2] = next_state(2);
        result_state[3] = next_state(3);

        duration = delta;

        // std::cout << "Propagated state: " << "\n";
        // std::cout << next_state << "\n";

        return valid_state(result_state); // Always true

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

bool two_link_acrobot_t::valid_state(double* state) {

    // Check if in bounds
    bool inBounds = false;
    if (MIN_V_1 <= state[2] && state[2] <= MAX_V_1 && 
        MIN_V_2 <= state[3] && state[3] <= MAX_V_2) {
        inBounds = true;
    }

    // Check collision detection
    double l1x = (LENGTH) * cos(state[STATE_THETA_1] - M_PI / 2);
    double l1y = (LENGTH) * sin(state[STATE_THETA_1] - M_PI / 2);
    double l2x = (LENGTH) * cos(state[STATE_THETA_1] - M_PI / 2) + (LENGTH) * cos(state[STATE_THETA_1] + state[STATE_THETA_2] - M_PI / 2);
    double l2y = (LENGTH) * sin(state[STATE_THETA_1] - M_PI / 2) + (LENGTH) * sin(state[STATE_THETA_1] + state[STATE_THETA_2] - M_PI / 2); 

    /*
    Matrix<double, 2, 2> end_state_link1;
    end_state_link1.setZero();
    end_state_link1.row(0) << 0, 0;
    end_state_link1.row(1) << l1x, l1y;
    */

    Matrix<double, 2, 2> end_state_link2;
    end_state_link2.setZero();
    end_state_link2.row(0) << l1x, l1y;
    end_state_link2.row(1) << l2x, l2y;

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
        //Matrix<double, 3, 2> temp1 = signed_distance_2d::signedDistancePolygons(end_state_link1, obs);
        Matrix<double, 3, 2> temp2 = signed_distance_2d::signedDistancePolygons(end_state_link2, obs);

        if (/*temp1(0,0) < 0 ||*/ temp2(0,0) < 0) {
            obstacle_collision_individ = true;
        }
    }

    // Swept out volume of two states
    double t_l2x = (LENGTH) * cos(temp_state[STATE_THETA_1] - M_PI / 2) + (LENGTH) * cos(temp_state[STATE_THETA_1] + temp_state[STATE_THETA_2] - M_PI / 2);
    double t_l2y = (LENGTH) * sin(temp_state[STATE_THETA_1] - M_PI / 2) + (LENGTH) * sin(temp_state[STATE_THETA_1] + temp_state[STATE_THETA_2] - M_PI / 2); 

    bool obstacle_collision_swv = false;
    Matrix<double, 2, 2> swv_poly; swv_poly.setZero();
    swv_poly.row(0) << l2x, l2y;
    swv_poly.row(1) << t_l2x, t_l2y;

    for(unsigned i=0; i<obstacles.size() && !obstacle_collision_swv;i++) {

        Matrix<double, 4, 2> obs;
        obs.setZero(4,2);

        obs.row(0) << obstacles[i].low_x, obstacles[i].low_y; 
        obs.row(1) << obstacles[i].low_x, obstacles[i].high_y;
        obs.row(2) << obstacles[i].high_x, obstacles[i].high_y; 
        obs.row(3) << obstacles[i].high_x, obstacles[i].low_y;

        // Call signed distance function, retrieve value.
        Matrix<double, 3, 2> temp = signed_distance_2d::signedDistancePolygons(swv_poly, obs);

        if (temp(0,0) < 0) {
            obstacle_collision_swv = true;
        }

    }

    return !obstacle_collision_swv && !obstacle_collision_individ && inBounds;

}

void two_link_acrobot_t::enforce_bounds()
{

    if(temp_state[0]<-M_PI)
            temp_state[0]+=2*M_PI;
    else if(temp_state[0]>M_PI)
            temp_state[0]-=2*M_PI;
    if(temp_state[1]<-M_PI)
            temp_state[1]+=2*M_PI;
    else if(temp_state[1]>M_PI)
            temp_state[1]-=2*M_PI;
    if(temp_state[2]<MIN_V_1)
            temp_state[2]=MIN_V_1;
    else if(temp_state[2]>MAX_V_1)
            temp_state[2]=MAX_V_1;
    if(temp_state[3]<MIN_V_2)
            temp_state[3]=MIN_V_2;
    else if(temp_state[3]>MAX_V_2)
            temp_state[3]=MAX_V_2;
}


bool two_link_acrobot_t::valid_state()
{
    return true;
}

svg::Point two_link_acrobot_t::visualize_point(double* state, svg::Dimensions dims)
{
        double x = (LENGTH) * cos(state[STATE_THETA_1] - M_PI / 2)+(LENGTH) * cos(state[STATE_THETA_1] + state[STATE_THETA_2] - M_PI / 2);
        double y = (LENGTH) * sin(state[STATE_THETA_1] - M_PI / 2)+(LENGTH) * sin(state[STATE_THETA_1] + state[STATE_THETA_2] - M_PI / 2);
        x = (x+2*LENGTH)/(4*LENGTH) * dims.width; 
        y = (y+2*LENGTH)/(4*LENGTH) * dims.height; 
        return svg::Point(x,y);
}

void two_link_acrobot_t::update_derivative(double* control)
{

    // z = [theta_1, theta_2, theta_dot_1, theta_dot_2]

    double theta2 = temp_state[STATE_THETA_2];
    double theta1 = temp_state[STATE_THETA_1] - M_PI / 2;
    double theta1dot = temp_state[STATE_V_1];
    double theta2dot = temp_state[STATE_V_2];
    double _tau = control[CONTROL_T];

    //extra term m*lc2
    double d11 = m * lc2 + m * (l2 + lc2 + 2 * l * lc * cos(theta2)) + I1 + I2;

    double d22 = m * lc2 + I2;
    double d12 = m * (lc2 + l * lc * cos(theta2)) + I2;
    double d21 = d12;

    //extra theta1dot
    double c1 = -m * l * lc * theta2dot * theta2dot * sin(theta2) - (2 * m * l * lc * theta1dot * theta2dot * sin(theta2));
    double c2 = m * l * lc * theta1dot * theta1dot * sin(theta2);
    double g1 = (m * lc + m * l) * g * cos(theta1) + (m * lc * g * cos(theta1 + theta2));
    double g2 = m * lc * g * cos(theta1 + theta2);

    deriv[STATE_THETA_1] = theta1dot;
    deriv[STATE_THETA_2] = theta2dot;

    double u2 = _tau - 1 * .1 * theta2dot;
    double u1 = -1 * .1 * theta1dot;
    double theta1dot_dot = (d22 * (u1 - c1 - g1) - d12 * (u2 - c2 - g2)) / (d11 * d22 - d12 * d21);
    double theta2dot_dot = (d11 * (u2 - c2 - g2) - d21 * (u1 - c1 - g1)) / (d11 * d22 - d12 * d21);

    deriv[STATE_V_1] = theta1dot_dot;
    deriv[STATE_V_2] = theta2dot_dot;
}


