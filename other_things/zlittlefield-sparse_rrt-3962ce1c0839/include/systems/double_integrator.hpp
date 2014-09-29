/**
 * @file double_integrator.hpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Chris Xie
 * 
 */

#ifndef SPARSE_DOUBLE_INTEGRATOR_HPP
#define SPARSE_DOUBLE_INTEGRATOR_HPP

#include "systems/system.hpp"

#include <iostream>
#include <string>
#include <iterator>
#include <fstream>
#include <eigen3/Eigen/Eigen>
using namespace Eigen;


MatrixXd read_in_obstacle_file(std::string file_name) {

	std::ifstream fin(file_name.c_str());
	std::vector<double> data;

	std::copy(std::istream_iterator<double>(fin), // this denotes "start of stream"
        	  std::istream_iterator<double>(),   // this denodes "end of stream"
        	  std::back_inserter<std::vector<double> >(data));

	MatrixXd obstacles(4, data.size()/4); obstacles.setZero();

	int index = 0;
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < data.size()/4; j++) {
			obstacles(i, j) = data[index++];
		}
	}

	return obstacles;

}

class double_integrator_Rectangle_t
{
public:
	/**
	 * @brief Create a rectangle using two corners
	 * 
	 * @param lx Bottom Left X coordinate
	 * @param ly Bottom Left Y coordinate
	 * @param hx Top Right X coordinate
	 * @param hy Top Right Y coordinate
	 */
	double_integrator_Rectangle_t(double lx,double ly,double hx,double hy)
	{
		low_x = lx;
		low_y = ly;
		high_x = hx;
		high_y = hy;
	}
	/**
	 * @brief Create a rectangle with center position and dimensions.
	 * 
	 * @param pos_x Center X coordinate.
	 * @param pos_y Center Y coordinate.
	 * @param dim_x X Dimension
	 * @param dim_y Y Dimension
	 * @param value Just a flag to denote which constructor is used.
	 */
	double_integrator_Rectangle_t(double pos_x,double dim_x,double pos_y,double dim_y,bool value)
	{
		low_x = pos_x-dim_x/2;
		low_y = pos_y-dim_y/2;
		high_x = pos_x+dim_x/2;
		high_y = pos_y+dim_y/2;
	}
	double low_x;
	double low_y;
	double high_x;
	double high_y;
};

/**
 * @brief A simple system implementing a 2d double integrator. 
 * @details A simple system implementing a 2d double integrator. Controls are accelerations in each of the dimensions
 */
class double_integrator_t : public system_t
{
public:
	double_integrator_t()
	{
		state_dimension = 4;
		control_dimension = 2;
		temp_state = new double[state_dimension];

		std::string obs_file = "../../random_scene_generation/double_integrator/scene_" + std::to_string(params::scene_number);
		MatrixXd scene_obstacles = read_in_obstacle_file(obs_file);

		for (int i = 0; i < scene_obstacles.cols(); i++) {
			obstacles.push_back(double_integrator_Rectangle_t(scene_obstacles(0,i), 
										    				  scene_obstacles(1,i),
										    				  scene_obstacles(2,i),
										    				  scene_obstacles(3,i), true));
		}


		// obstacles.push_back(double_integrator_Rectangle_t(   1,  -1.5,    5,  9.5));
		// obstacles.push_back(Rectangle_t(  -8,  4.25,   -1, 5.75));
		// obstacles.push_back(Rectangle_t(   5,   3.5,    9,  4.5));
		// obstacles.push_back(Rectangle_t(-8.5,  -7.5, -3.5, -2.5));
		// obstacles.push_back(Rectangle_t(   5,  -8.5,    9, -4.5));
	}
	virtual ~double_integrator_t(){}

	/**
	 * @copydoc system_t::distance(double*, double*)
	 */
	virtual double distance(double* point1, double* point2);

	/**
	 * @copydoc system_t::random_state(double*)
	 */
	virtual void random_state(double* state);

	/**
	 * @copydoc system_t::random_control(double*)
	 */
	virtual void random_control(double* control);

	/**
	 * @copydoc system_t::propagate(double*, double*, int, int, double*, double& )
	 */
	virtual bool propagate( double* start_state, double* control, int min_step, int max_step, double* result_state, double& duration );

	/**
	 * @copydoc system_t::enforce_bounds()
	 */
	virtual void enforce_bounds();
	
	/**
	 * @copydoc system_t::valid_state()
	 */
	virtual bool valid_state();
	virtual bool valid_state(double* state);

	/**
	 * @copydoc system_t::visualize_point(double*, svg::Dimensions)
	 */
	svg::Point visualize_point(double* state, svg::Dimensions dims);

	/**
	 * @copydoc system_t::visualize_obstacles(svg::Document&, svg::Dimensions)
	 */
    virtual void visualize_obstacles(svg::Document& doc ,svg::Dimensions dims);

protected:

	std::vector<double_integrator_Rectangle_t> obstacles;

};


#endif
