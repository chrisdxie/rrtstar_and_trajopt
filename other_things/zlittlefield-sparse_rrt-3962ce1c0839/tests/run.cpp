/**
 * @file run.cpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Zakary Littlefield, Kostas Bekris 
 * 
 */

#include "utilities/parameter_reader.hpp"
#include "utilities/condition_check.hpp"
#include "utilities/random.hpp"
#include "utilities/timer.hpp"

#include "systems/pendulum.hpp"
#include "systems/point.hpp"
#include "systems/double_integrator.hpp"
#include "systems/car.hpp"
#include "systems/rally_car.hpp"
#include "systems/cart_pole.hpp"
#include "systems/two_link_acrobot.hpp"
#include "motion_planners/sst.hpp"
#include "motion_planners/rrt.hpp"

#include <iostream>

#include "../../My_BITStar_Impl/cartpole/plot_bitstar.h"
#include "../../My_BITStar_Impl/double_integrator/plot_bitstar.h"
#include "../../My_BITStar_Impl/acrobot/plot_bitstar.h"

int main(int ac, char* av[])
{
	read_parameters(ac,av);
	//****************After reading in from input, we need to instantiate classes
	init_random(params::random_seed);
	system_t* system;
	if(params::system=="point")
	{
		system = new point_t();
	}
	else if(params::system=="double_integrator")
	{
		system = new double_integrator_t();
	}
	else if(params::system=="pendulum")
	{
		system = new pendulum_t();
	}
	else if(params::system=="car")
	{
		system = new car_t();
	}
	else if(params::system=="rally_car")
	{
		system = new rally_car_t();
	}
	else if(params::system=="cart_pole")
	{
		system = new cart_pole_t();
	}
	else if(params::system=="two_link_acrobot")
	{
		system = new two_link_acrobot_t();
	}

	planner_t* planner;
	if(params::planner=="rrt")
	{
		planner = new rrt_t(system);
	}
	else if(params::planner=="sst")
	{
		planner = new sst_t(system);
	}
	planner->set_start_state(params::start_state);
	planner->set_goal_state(params::goal_state,params::goal_radius);
	planner->setup_planning();

	condition_check_t checker(params::stopping_type,params::stopping_check);
	condition_check_t* stats_check=NULL;
	if(params::stats_check!=0)
	{
		stats_check = new condition_check_t(params::stats_type,params::stats_check);
	}

	checker.reset();
	std::cout<<"Starting the planner: "<<params::planner<<" for the system: "<<params::system<<std::endl;
	if(stats_check==NULL)
	{
		do
		{
			planner->step();
		}
		while(!checker.check());
		std::vector<std::pair<double*,double> > controls;
		planner->get_solution(controls);
		double solution_cost = 0;
		for(unsigned i=0;i<controls.size();i++)
		{
			solution_cost+=controls[i].second;
		}
		std::cout<<"Time: "<<checker.time()<<" Iterations: "<<checker.iterations()<<" Nodes: "<<planner->number_of_nodes<<" Solution Quality: " <<solution_cost<<std::endl ;
		planner->visualize_tree(0);
		planner->visualize_nodes(0);
	}
	else
	{
		int count = 0;
		bool execution_done = false;
		bool stats_print = false;
		while(true)
		{
			do
			{
				planner->step();
				execution_done = checker.check();
				stats_print = stats_check->check();
			}
			while(!execution_done && !stats_print);
			if(stats_print)
			{
				std::vector<std::pair<double*,double> > controls;
				planner->get_solution(controls);
				double solution_cost = 0;
				for(unsigned i=0;i<controls.size();i++)
				{
					solution_cost+=controls[i].second;
				}
				std::cout<<"Time: "<<checker.time()<<" Iterations: "<<checker.iterations()<<" Nodes: "<<planner->number_of_nodes<<" Solution Quality: " <<solution_cost<<std::endl ;

				stats_print = false;
				if(params::intermediate_visualization)
				{
					planner->visualize_tree(count);
					planner->visualize_nodes(count);
					count++;
				}				
				stats_check->reset();
			}
			if (execution_done)
			{
				std::vector<std::pair<double*,double> > controls;
				planner->get_solution(controls);
				double solution_cost = 0;
				for(unsigned i=0;i<controls.size();i++)
				{
					solution_cost+=controls[i].second;
				}
				std::cout<<"Time: "<<checker.time()<<" Iterations: "<<checker.iterations()<<" Nodes: "<<planner->number_of_nodes<<" Solution Quality: " <<solution_cost<<std::endl ;
				planner->visualize_tree(count);
				planner->visualize_nodes(count);

				/*
				 * My visualization of tree
				 */

				if (params::system=="double_integrator") {

					py::object plotter = di_init_display(); // Initialize python interpreter and pyplot plot 

					// Convert Eigen matrices and vectors to Numpy ND arrays
					np::ndarray states_np = di_eigen_to_ndarray(planner->tree_to_matrix_states());
					np::ndarray parents_np = di_eigen_to_ndarray(planner->tree_to_matrix_parents());
					np::ndarray goal_path_np = di_eigen_to_ndarray(planner->get_solution_path());
					np::ndarray obstacles_np = di_eigen_to_ndarray(planner->get_obstacles());

					di_plot(plotter, states_np, parents_np, goal_path_np, obstacles_np, params::stopping_check, solution_cost);
				} else if (params::system=="cart_pole") {

					py::object plotter = cp_init_display(); // Initialize python interpreter and pyplot plot 					

					// Have to flip 0th and 3rd rows of goal_path to fit the format of my plotting
					MatrixXd goal_path = planner->get_solution_path();
					MatrixXd row0 = goal_path.row(0);
					goal_path.row(0) = goal_path.row(3);
					goal_path.row(3) = row0;
					np::ndarray goal_path_np = cp_eigen_to_ndarray(goal_path);

					// Hard code obstacles
					//MatrixXd obstacles(4,3);
					//obstacles.col(0) << -2, 2, -.5, .8;
					//obstacles.col(1) <<  2, 2, -.5, .8;
					//obstacles.col(2) <<  0, .6, .6, .6;

					MatrixXd obstacles(4,2);
					obstacles.col(0) << 0, .03, .5, .22;
                			obstacles.col(1) << 0, .03,-.5, .22;
	
					//MatrixXd obstacles(4,3);
			                //obstacles.col(0) << -2.5, 3, -.5, .8;
			                //obstacles.col(1) << 2.5, 3, -.5, .8;
			                //obstacles.col(2) << 0, 2, .6, .6;


					np::ndarray obstacles_np = cp_eigen_to_ndarray(obstacles);

					// Hard code some of the parameters, whatevs
					cp_plot(plotter, goal_path_np, obstacles_np, checker.iterations(), solution_cost, .4, .15, .5);
				} else if (params::system=="two_link_acrobot") {

					py::object plotter = ac_init_display(); // Initialize python interpreter and pyplot plot 					

					// Convert Eigen matrices and vectors to Numpy ND arrays
					np::ndarray goal_path_np = ac_eigen_to_ndarray(planner->get_solution_path());
					//std::cout << "Solution:\n" << (planner->get_solution_path()).transpose() << "\n";

					// Hard code obstacles
					MatrixXd obstacles(4,2);
					obstacles.col(0) <<  1.2, .8,  1.2, .8;
					obstacles.col(1) << -1.2, .8,  1.2, .8;
					np::ndarray obstacles_np = ac_eigen_to_ndarray(obstacles);
					ac_plot(plotter, goal_path_np, obstacles_np, params::stopping_check, solution_cost, 1, params::scene_number);
				}

				/*
				 * Done with my visualization of tree
				 */

				break;
			}
		}
	}
	std::cout<<"Done planning."<<std::endl;

}
