/**
 * @file cart_pole.hpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Zakary Littlefield, Kostas Bekris 
 * 
 */

#ifndef SPARSE_CART_POLE_HPP
#define SPARSE_CART_POLE_HPP

#include "systems/system.hpp"

class cartpole_Rectangle_t
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
	cartpole_Rectangle_t(double lx,double ly,double hx,double hy)
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
	cartpole_Rectangle_t(double pos_x,double dim_x,double pos_y,double dim_y,bool value)
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

class cart_pole_t : public system_t
{
public:
	cart_pole_t()
	{
		state_dimension = 4;
		control_dimension = 1;
		temp_state = new double[state_dimension];
		deriv = new double[state_dimension];

		// Hard code obstacle scenario here
		obstacles.push_back(cartpole_Rectangle_t( 0, .03, .5, .22, true));
		obstacles.push_back(cartpole_Rectangle_t( 0, .03,-.5, .22, true));

		//obstacles.push_back(cartpole_Rectangle_t(-2, 2, -.5, .8, true));
		//obstacles.push_back(cartpole_Rectangle_t( 2, 2, -.5, .8, true));
		//obstacles.push_back(cartpole_Rectangle_t( 0, .6, .6, .6, true));

		//obstacles.push_back(cartpole_Rectangle_t(-2.5, 2, -.5, .8, true));
                //obstacles.push_back(cartpole_Rectangle_t( 2.5, 2, -.5, .8, true));
                //obstacles.push_back(cartpole_Rectangle_t( 0, 2, .6, .6, true));
	}
	virtual ~cart_pole_t(){}

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
	virtual bool valid_state(double* state);
	virtual bool valid_state();

	/**
	 * @copydoc system_t::visualize_point(double*, svg::Dimensions)
	 */
	svg::Point visualize_point(double* state, svg::Dimensions dims);
	
protected:
	double* deriv;
	void update_derivative(double* control);
	std::vector<cartpole_Rectangle_t> obstacles;	
};


#endif
