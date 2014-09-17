#include <iostream>

#include <boost/python.hpp>
#include <boost/python/numeric.hpp>
#include <boost/python/tuple.hpp>
#include <boost/numpy.hpp>
#include <boost/filesystem.hpp>

namespace py = boost::python;
namespace np = boost::numpy;

#include <eigen3/Eigen/Eigen>
using namespace Eigen;

#include <SetupObject.h>
#include <Node.h>
#include <set>
#include <ctime>
#include <iomanip>
#include <fstream>

#include "../../dynamics_library/dynamics_library.hpp"
using namespace dynamics_library;

// Put setup in global storage
SetupObject setup_values;

inline double uniform(double low, double high) {
	return (high - low)*(rand() / double(RAND_MAX)) + low;
}

np::ndarray eigen_to_ndarray(const MatrixXd& m) {
	if (m.cols() == 1) {
		py::tuple shape = py::make_tuple(m.rows());
		np::dtype dtype = np::dtype::get_builtin<float>();
		np::ndarray n = np::zeros(shape, dtype);
		for(int i=0; i < m.rows(); ++i) {
			n[py::make_tuple(i)] = m(i);
		}
		return n;
	} else {
		py::tuple shape = py::make_tuple(m.rows(), m.cols());
		np::dtype dtype = np::dtype::get_builtin<float>();
		np::ndarray n = np::zeros(shape, dtype);
		for(int i=0; i < m.rows(); ++i) {
			for(int j=0; j < m.cols(); ++j) {
				n[py::make_tuple(i,j)] = m(i,j);
			}
		}

		return n;
	}
}

py::object init_display() {
    // necessary initializations
	Py_Initialize();
	np::initialize();
	py::numeric::array::set_module_and_type("numpy", "ndarray");

    // use boost to find directory of python_plot.py
	std::string working_dir = boost::filesystem::current_path().normalize().string();
	std::string plot_cpp_dir = working_dir + "/";

    // necessary imports
	py::object main_module = py::import("__main__");
	py::object main_namespace = main_module.attr("__dict__");
	py::exec("import sys, os", main_namespace);
	// add plot_cpp_dir to sys.path
	py::exec(py::str("sys.path.append('"+plot_cpp_dir+"')"), main_namespace);
	// get python file module
	py::object plot_mod = py::import("python_plot");

    // get function from module
	py::object plotter = plot_mod.attr("plot");
	return plotter;
}

void plot(py::object plotter, np::ndarray states, np::ndarray parents,
		np::ndarray goal_path, np::ndarray obstacles, np::ndarray goal_region,
		int iters, double cost, double dv, double ds) {

    bool pause = true;

	try {
	    // pass control to python now
		plotter(states, parents, goal_path, obstacles, goal_region, iters, cost, dv, ds);
	}
	catch(py::error_already_set const &) {
	    // will pass python errors to cpp for printing
		PyErr_Print();
	}
}

void setup(int max_iters, std::string randomize, double delta_v, double delta_s) {

	// This function populates a matrix of values for setting up the problem.
	// Setup variables:
	// 		max_iters
	// 		dimension of problem
	// 		initial state
	//		goal state
	//		delta used for steering
	//		Rectangular obstacle x range
	// 		Rectangular obstacle y range

	int d = 4;
	VectorXd initial_state(d);
	initial_state << 0, 0, 0, 0;
	Vector4d goal_region;
	goal_region << 9, 1, 9, 1;

	int num_obstacles = 3;
	MatrixXd obstacles(4, num_obstacles);
	obstacles.col(0) << 3, 4, 3, 2;
	obstacles.col(1) << 2, 1, 8, 3;
	obstacles.col(2) << 7, 2, 7, 2;

	// Max Iterations
	setup_values.max_iters = max_iters;

	// Dimension of problem
	setup_values.dimension = d;

	// Initial State
	setup_values.initial_state = initial_state;

	// Goal Region
	setup_values.goal_region = goal_region;

	// Delta stuffs
	setup_values.delta_v = delta_v;
	setup_values.delta_s = delta_s;

	// Randomize argument
	if (randomize == "true") {
		setup_values.randomize = true;
	} else {
		setup_values.randomize = false;
	}

	// Obstacles
	setup_values.obstacles = obstacles;

	// Limits
	setup_values.x_min = -1; // Center of 5, side length of 12
	setup_values.x_max = 11;
	setup_values.v_min = -1;
	setup_values.v_max = 1;
	setup_values.u_min = -1;
	setup_values.u_max = 1;

	// T_prop
	setup_values.T_prop = 1;

}

// Returns true if point is inside rectangular polygon
bool inside_rectangular_obs(Vector2d point, double x_min, double x_max,
		double y_min, double y_max) {

	return (x_min <= point(0)) && (point(0) <= x_max)
			&& (y_min <= point(1)) && (point(1) <= y_max);

}

// 2 dimensional collision check. Return true if point is contained inside rectangle
bool exists_collision(VectorXd x_near, VectorXd x_new) {

	// ONLY WORKS WITH EIGEN3. IF FOR SOME REASON YOU ARE RUNNING THIS SOMEWHERE ELSE,
	// HARD CODE THIS
	x_near = x_near.head(2);
	x_new = x_new.head(2);

	MatrixXd obstacles = setup_values.obstacles;
	int num_obstacles = obstacles.cols();

	for (int n = 0; n < num_obstacles; n++) {

		double x_min = obstacles(0,n) - .5*obstacles(1,n);
		double x_max = obstacles(0,n) + .5*obstacles(1,n);
		double y_min = obstacles(2,n) - .5*obstacles(3,n);
		double y_max = obstacles(2,n) + .5*obstacles(3,n);

		// Determine if point is inside obstacle by discretization
		float dis_num = 20;
		VectorXd direction = (x_new - x_near)/(x_new - x_near).norm();
		double length = (x_new - x_near).norm();
		for (int i = 1;i <= dis_num; i++) {
			Vector2d point = x_near + length * i/dis_num * direction;
			if (inside_rectangular_obs(point, x_min, x_max, y_min, y_max)) {
				return true;
			}
		}

	}
	return false;
}


bool inBounds(VectorXd state) {
	Vector2d pos, vel;
	pos = state.head(2);
	vel = state.tail(2);
	if (inside_rectangular_obs(pos, setup_values.x_min, setup_values.x_max,
			setup_values.x_min, setup_values.x_max) &&
		inside_rectangular_obs(vel, setup_values.v_min, setup_values.v_max,
			setup_values.v_min, setup_values.v_max)) {
		return true;
	}
	return false;
}

// Returns a matrix of states for Python plotting
MatrixXd tree_to_matrix_states(Node* node) {

	// Populate this matrix by an iterative DFS. MUST follow the same protocol
	// as sister function for creating matrix of parents.
	MatrixXd states(setup_values.dimension, setup_values.max_iters);
	int i = 0;

	std::stack<Node*> fringe;
	fringe.push(node);
	while (fringe.size() > 0) {
		Node* candidate = fringe.top();
		fringe.pop();
		states.col(i++) = candidate->state;
		for (std::set<Node*>::iterator child = candidate->children.begin();
				child != candidate->children.end(); ++child) {
			fringe.push(*child);
		}
	}

	return states.leftCols(i);

}

// Returns a matrix of parents for Python plotting
MatrixXd tree_to_matrix_parents(Node* node) {

	// Populate this matrix by an iterative DFS. MUST follow the same protocol
	// as sister function for creating matrix of states.
	MatrixXd parents(setup_values.dimension, setup_values.max_iters);
	int i = 0;

	std::stack<Node*> fringe;
	fringe.push(node);
	while (fringe.size() > 0) {
		Node* candidate = fringe.top();
		fringe.pop();
		parents.col(i++) = candidate->parent->state;
		for (std::set<Node*>::iterator child = candidate->children.begin();
				child != candidate->children.end(); ++child) {
			fringe.push(*child);
		}
	}
	return parents.leftCols(i);
}

// int num_nodes(Node* tree) {

// 	int total = 0;
// 	if (tree->children.size() == 0) {
// 		return 1;
// 	}
// 	for (std::set<Node*>::iterator child = tree->children.begin();
// 			child != tree->children.end(); child++) {
// 		Node* kid = *child;
// 		total += num_nodes(kid);
// 	}
// 	return total+1;

// }

/* Returns path from root */
MatrixXd get_path(Node* x) {
	MatrixXd P(4, setup_values.max_iters);
	int index = 0;
	while (x->state != setup_values.initial_state) {
		P.col(index) = x->state;
		x = x->parent;
		index++;
	}

	MatrixXd R = P.leftCols(index).rowwise().reverse();
	return R;

}

/* Return distance between two vectors */
double dist(VectorXd p1, VectorXd p2) {
	return (p1 - p2).norm();
}

// Finds best cost vertex in a ball of radius gamma. If no vertices are within the gamma ball,
// return the nearest vertex in V_active (in terms of distance, NOT cost)
Node* bestNear(std::set<Node*> V_active, VectorXd sample, double gamma) {

	Node* best_in_gamma = NULL;
	Node* best_overall = NULL;

	std::set<Node*>::iterator it;
	for( it = V_active.begin(); it != V_active.end(); ++it) {
		Node* candidate = *it;
		VectorXd state = candidate->state;

		// Do ball stuff first
		if (dist(state, sample) <= gamma) { // If it's in the gamma ball, compute cost
			if (best_in_gamma == NULL || candidate->cost < best_in_gamma->cost) {
				best_in_gamma = candidate;
			}
		}

		// Do overall stuff next
		if (best_overall == NULL) { // If this is the first one
			best_overall = candidate;
		} else {
			VectorXd best_state = best_overall->state;
			if (dist(state, sample) < dist(best_state, sample)) {
				best_overall = candidate;
			}
		}

	}

	if (best_in_gamma != NULL) {
		return best_in_gamma;
	} else {
		return best_overall;
	}
}

/* Finds nearest witness vertex in set S in terms of distance */
Witness* nearest_witness(std::set<Witness*> S, Node* x) {

	VectorXd state = x->state;
	Witness* best = NULL;

	std::set<Witness*>::iterator it;
	for( it = S.begin(); it != S.end(); ++it) {
		Witness* candidate = *it; // Grab candidate
		if (best == NULL) { // If it is the first one
			best = candidate;
		} else { // Otherwise, see who is closer
			VectorXd candidate_state = candidate->state;
			VectorXd best_state = best->state;
			if (dist(candidate_state, state) < dist(best_state, state)) {
				best = candidate;
			}
		}
	}

	return best;

}

/* Propogates a state forward according to RK4 integration of the double integrator */
void double_integrator_MonteCarlo_prop(Node* prop_node, Node* new_node) {

	VectorXd x_prop = prop_node->state;

	double prop_time = uniform(0, setup_values.T_prop);
	double u1 = uniform(setup_values.u_min, setup_values.u_max);
	double u2 = uniform(setup_values.u_min, setup_values.u_max);

	Vector2d controls;
	controls << u1, u2;
	VectorXd x_new = rk4(continuous_double_integrator_dynamics, x_prop, controls, prop_time);

	new_node->state = x_new;
	new_node->parent = prop_node;
	new_node->cost = prop_node->cost + prop_time;

}

bool isLeaf(Node* x) {
	return x->children.size() == 0;
}

bool inSet(Node* x, std::set<Node*> Set) {
	return Set.find(x) != Set.end();
}

bool inGoal(Node* x) {
	VectorXd goal_region = setup_values.goal_region;
	Vector2d state = x->state.head(2);

	double x_min = goal_region(0) - .5*goal_region(1);
	double x_max = goal_region(0) + .5*goal_region(1);
	double y_min = goal_region(2) - .5*goal_region(3);
	double y_max = goal_region(2) + .5*goal_region(3);

	if (inside_rectangular_obs(state, x_min, x_max, y_min, y_max)) {
		return true;
	}
	return false;

}

// Sample a state uniformly in sample space (hard coded for 4d)
Vector4d sample_state() {
	Vector4d sample;
	double x1 = uniform(setup_values.x_min, setup_values.x_max);
	double x2 = uniform(setup_values.x_min, setup_values.x_max);
	double v1 = uniform(setup_values.v_min, setup_values.v_max);
	double v2 = uniform(setup_values.v_min, setup_values.v_max);
	sample << x1, x2, v1, v2;
	return sample;
}

double SST() {

	// Time it
	std::clock_t start;
	double duration;
	start = std::clock();

	// Initialize random seed using current time.
	// Uncomment this line if you want feed the random number generator a seed based on time.
	if (setup_values.randomize) {
		srand(time(NULL));
	}

	double best_cost = 1e10;

	// Initialize sets
	std::set<Node*> V_active, V_inactive;
	std::set<Witness*> S;

	// Setup intial state and add it to V_active
	Node* x0 = new Node;
	x0->state = setup_values.initial_state;
	x0->cost = 0;
	x0->parent = x0; // Convention for function tree_to_matrix_parents()
	V_active.insert(x0);

	// Setup initial witness and add it to
	Witness* s0 = new Witness;
	s0->copy_fields(x0);
	s0->rep = x0;
	S.insert(s0);

	Node* best_solution = NULL;

	// Begin iterations here
	int k = 1;
	while (k <= setup_values.max_iters) {

		Vector4d sample = sample_state();
		Node* x_nearest = bestNear(V_active, sample, setup_values.delta_v);

		// Monte Carlo propagation
		Node* x_new = new Node;
		double_integrator_MonteCarlo_prop(x_nearest, x_new);

		if (!exists_collision(x_nearest->state, x_new->state) &&
				inBounds(x_nearest->state) && inBounds(x_new->state)) { // Redundant check for x_nearest, but whatevs

			Witness* s_new = nearest_witness(S, x_new);
			if (dist(x_new->state, s_new->state) > setup_values.delta_s) {
				Witness* new_witness = new Witness; // Create a new witness object
				new_witness->copy_fields(x_new); // Copy x_new stuff
				S.insert(new_witness); // Insert
				s_new = new_witness; // Set pointer of s_new to the new one... will this work?
				s_new->rep = NULL;
			}

			Node* x_peer = s_new->rep;
			if (x_peer == NULL || x_new->cost < x_peer->cost) {

				s_new->rep = x_new;
				V_active.insert(x_new);
				x_nearest->children.insert(x_new);

				if (inGoal(x_new)) {
					if (x_peer == NULL) {
						std::cout << "Found NEW goal w/ cost: " << x_new->cost << "\n";
					} else {
						std::cout << "Found UPDATED goal w/ cost: " << x_new->cost << "\n";
					}
					best_cost = std::min(best_cost, x_new->cost);

				}

				// Write time, # of nodes, cost to file
				ofstream outfile("statistics_" + std::to_string(setup_values.max_iters) + "_iters.txt", ios::app);
				if (outfile.is_open()) {
					outfile << std::setprecision(10) << ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
					outfile << ", " << V_active.size() + V_inactive.size();
					outfile << ", " << best_cost << std::endl;
					outfile.close();
				}

				if (x_peer != NULL) {
//					std::cout << "Here" << "\n";
					V_active.erase(x_peer); // This works if x_peer is in V_active, which it should be..
					V_inactive.insert(x_peer);
					while (isLeaf(x_peer) && inSet(x_peer, V_inactive)) {

						Node* x_parent = x_peer->parent;
						x_parent->children.erase(x_peer);
						V_inactive.erase(x_peer);
						delete x_peer; // Free the memory
						x_peer = x_parent;
					}
				}
			}


		} else { // x_new was not put into graph
			delete x_new;
		}

		if (k % 100 == 0) {
			std::cout << "Iteration: " << k << "\n";
		}
		k++;

	}

	// Find best goal node. Search V_active and V_inactive
	Node* goal_node = NULL;
	std::set<Node*>::iterator it;
	for(it = V_active.begin(); it != V_active.end(); ++it) {
		Node* candidate = *it;
		if (inGoal(candidate)) {
			if (goal_node == NULL || candidate->cost < goal_node->cost) {
				goal_node = candidate;
			}
		}
	}
	for(it = V_inactive.begin(); it != V_inactive.end(); ++it) {
		Node* candidate = *it;
		if (inGoal(candidate)) {
			if (goal_node == NULL || candidate->cost < goal_node->cost) {
				goal_node = candidate;
			}
		}
	}

	// Now, find best path.
	if (goal_node != NULL) {

		// For returning
		best_cost = goal_node->cost;

		MatrixXd goal_path = get_path(goal_node);

		// Plot in Python
		std::cout << "Solution found!\n";
		std::cout << "Initializing display...\n";
		py::object plotter = init_display(); // Initialize python interpreter and pyplot plot

		// Convert Eigen matrices and vectors to Numpy ND arrays
		np::ndarray states_np = eigen_to_ndarray(tree_to_matrix_states(x0));
		np::ndarray parents_np = eigen_to_ndarray(tree_to_matrix_parents(x0));
		np::ndarray goal_path_np = eigen_to_ndarray(goal_path);
		np::ndarray obstacles_np = eigen_to_ndarray(setup_values.obstacles);
		np::ndarray goal_region_np = eigen_to_ndarray(setup_values.goal_region);

		std::cout << "Plotting...\n";
		plot(plotter, states_np, parents_np, goal_path_np, obstacles_np, goal_region_np,
			 setup_values.max_iters, goal_node->cost, setup_values.delta_v, setup_values.delta_s); // Finally, plot

	} else {
		std::cout << "No path found.\n";
	}

	return best_cost;

}

/*
 * Just a note: This function MUST be called from directory that
 * plot_sst.cpp lives in.
 *
 * USAGE: build/bin/plot_sst <MAX_ITERS> <RANDOMIZE> <DELTA_V> <DELTA_S>
 */

int main(int argc, char* argv[]) {

    // Assumes a command line argument of MAX_ITERS RANDOMIZE {true, false}
    int max_iters = atoi(argv[1]);
    std::string randomize;
    if (argc >= 3) {
    	randomize = argv[2];
    } else {
    	randomize = "false";
    }

    double delta_v, delta_s;
    if (argc >= 5) {
    	delta_v = atof(argv[3]); delta_s = atof(argv[4]);
    } else {
    	delta_v = 1; delta_s = .5;
    }

    // Setup function
    setup(max_iters, randomize, delta_v, delta_s);

    // Run the SST
    // double bcost = 1e11;
    // int bi, bj;
    // const int num_vals = 13;
    // double delta_values[num_vals] = {5.0, 4.0, 3.0, 2.0, 1.8, 1.6, 1.4, 1.2, 1.0, .8, .6, .4, .2};
    // for(int i = 0; i < num_vals; ++i) {
    // 	for(int j = 0; j < num_vals; ++j) {
    // 		setup(max_iters, "false", delta_values[i], delta_values[j]);
    // 		std::cout << "Running SST with values: delta_v = " << delta_values[i] << ", delta_s = " << delta_values[j] << "\n";
    // 		double cand = SST();
    // 		if (cand < bcost) {
    // 			bcost = cand; bi = i; bj = j;
    // 		}
    // 	}
    // }

    // std::cout << "Best values: delta_v = " << delta_values[bi] << ", j = " << delta_values[bj] << ", with cost of " << bcost << "\n";
   std::cout << "Running SST...\n";
   SST();

    std::cout << "exiting\n";
}

