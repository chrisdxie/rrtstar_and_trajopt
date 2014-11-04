#include <iostream>

#include <boost/random.hpp>

#include <eigen3/Eigen/Eigen>
using namespace Eigen;

#include <fstream>
#include <iomanip>
#include <math.h>
#include <set>
#include <stack>
#include <queue>
#include <ctime>

#include "anytimeRRT.h"
#include "plot_anytimeRRT.h"

#include "../../2d_signed_distance_library_cpp/signedDistancePolygons.hpp"

#include "../../dynamics_library/dynamics_library.hpp"

// Global variables
SetupObject setup_values;
std::set<Node*> V;

double distance_weight;
double cost_weight;
double best_cost;
double T_Cn;

// Time it
std::clock_t start = std::clock();
double duration;

double goal_sampling_probability = 0.999;
int max_sample_attempts = 50;

Node* root_node;
Node* goal_node;
Node* q_new;

// For sampling in ball
typedef boost::mt19937 RANDOM_ENGINE;
typedef boost::uniform_real<> U_DIST;
typedef boost::variate_generator<RANDOM_ENGINE, U_DIST> U_GENERATOR;

RANDOM_ENGINE *r_eng;
U_DIST *u_dist;
U_GENERATOR *u_sampler;

int num_collision_check_calls = 0;

double max_speed = sqrt(2); // Hard coded for this example

inline double uniform(double low, double high) {
	return (high - low)*(*u_sampler)() + low;
}

// Cost of node from root of tree
inline double tree_cost(Node* x) {

	if (x->state == setup_values.initial_state) {
		return x->cost; // Should be 0 for root node!
	} else {
		return x->cost + tree_cost(x->parent);
	}

}

void setup(int max_time, std::string& randomize, MatrixXd obstacles, int stats_id) {

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
	VectorXd goal_state(d);
	goal_state << 9, 9, 0, 0;

	// Max Iterations
	setup_values.max_time = max_time;

	// Dimension of problem
	setup_values.dimension = d;

	// Initial State
	setup_values.initial_state = initial_state;

	// Goal Region
	setup_values.goal_state = goal_state;

	// Goal Radius
	setup_values.goal_radius = 1.5;

	// Setup intial state and add it to V; note that T and E are implicity represented
	// by root node. Can perform DFS to find T, E are children pointers of every node in T
	root_node = new Node();
	root_node->state = setup_values.initial_state;
	root_node->cost = 0;
	root_node->parent = root_node; // Convention for function tree_to_matrix_parents()
	V.insert(root_node);

	goal_node = new Node;
	goal_node->state = goal_state;

	// Randomize argument
	if (randomize == "true") {
		setup_values.randomize = true;
	} else {
		setup_values.randomize = false;
	}

	// Obstacles
//	MatrixXd obstacles_(4, 3);
//	obstacles_.col(0) << 3, 4, 3, 2;
//	obstacles_.col(1) << 2, 1, 8, 3;
//	obstacles_.col(2) << 7, 2, 7, 2;
//	setup_values.obstacles = obstacles_;
	setup_values.obstacles = obstacles;

	// Parameters for AnytimeRRTs
	setup_values.eps_f = 0.05;
	setup_values.delta_d = 0.15;
	setup_values.delta_c = 0.15;
	setup_values.max_time_per_rrt = 600;

	// Limits
	setup_values.x_min = -10;
	setup_values.x_max = 10;
	setup_values.v_min = -1;
	setup_values.v_max = 1;
	setup_values.u_min = -1;
	setup_values.u_max = 1;

	// Sampling stuff
	if (setup_values.randomize) {
		r_eng = new RANDOM_ENGINE(time(NULL));
	} else {
		r_eng = new RANDOM_ENGINE();
	}

	// To specifically seed it
	if (randomize != "true" && randomize != "false") {
		delete r_eng;
		//std::cout << randomize << "\n";
		//std::cout << randomize.c_str() << "\n";
		r_eng = new RANDOM_ENGINE(atoi(randomize.c_str())); // seed
	};

	// uniform dist
	u_dist = new U_DIST(0, 1);
	u_sampler = new U_GENERATOR(*r_eng, *u_dist);

	setup_values.stats_id = stats_id;

}

// Returns true if point is inside rectangular polygon
inline bool inside_rectangular_obs(VectorXd& point, double x_min, double x_max,	double y_min, double y_max) {
	return (x_min <= point(0)) && (point(0) <= x_max) && (y_min <= point(1)) && (point(1) <= y_max);
}

inline bool inside_sphere_obs(VectorXd& point, VectorXd& center, double radius) {
	return (point - center).norm() < radius;
}

inline bool inBounds(Vector4d& state) {
	VectorXd pos(2), vel(2);
	pos << state(0), state(1);
	vel << state(2), state(3);
	if (inside_rectangular_obs(pos, setup_values.x_min, setup_values.x_max,
							   setup_values.x_min, setup_values.x_max) &&
		inside_rectangular_obs(vel, setup_values.v_min, setup_values.v_max,
				   setup_values.v_min, setup_values.v_max)) {
		return true;
	}
	return false;
}

bool isLeaf(Node* x) {
	return x->children.size() == 0;
}

// 2 dimensional collision check. Return true if point is contained inside rectangle
bool exists_collision(Vector4d& x_near, Vector4d& x_new) {

	// Hard coded for point robot since states are in 2D

	MatrixXd obstacles = setup_values.obstacles;
	int num_obstacles = obstacles.cols();

	for (int n = 0; n < num_obstacles; n++) {

		double x_min = obstacles(0,n) - .5*obstacles(1,n);
		double x_max = obstacles(0,n) + .5*obstacles(1,n);
		double y_min = obstacles(2,n) - .5*obstacles(3,n);
		double y_max = obstacles(2,n) + .5*obstacles(3,n);

		Matrix<double, 4, 2> obs;
		obs.setZero(4,2);
		obs.row(0) << x_min, y_min; obs.row(1) << x_min, y_max;
		obs.row(2) << x_max, y_max; obs.row(3) << x_max, y_min;

		// Compute line (2D polygon) spanned by x and x_next
		Matrix<double, 2, 2> traj_line;
		traj_line.setZero(2,2);
		traj_line.row(0) << x_near(0), x_near(1);
		traj_line.row(1) << x_new(0), x_new(1);

		// Call signed distance function, retrieve value.
		Matrix<double, 3, 2> temp = signed_distance_2d::signedDistancePolygons(traj_line, obs);
		num_collision_check_calls++;

		if (temp(0,0) < 0) {
			return true;
		}

	}
	return false;
}
// Returns a matrix of states for Python plotting
MatrixXd tree_to_matrix_states(Node* node) {

	// Populate this matrix by an iterative DFS. MUST follow the same protocol
	// as sister function for creating matrix of parents.
	MatrixXd states(setup_values.dimension, V.size());
	int index = 0;

	std::stack<Node*> fringe;
	fringe.push(node);
	while (fringe.size() > 0) {
		Node* candidate = fringe.top();
		fringe.pop();
		states.col(index++) = candidate->state;
		for (std::set<Node*>::iterator child = candidate->children.begin(); child != candidate->children.end(); ++child) {
			fringe.push(*child);
		}
	}

	return states.leftCols(index);

}

// Returns a matrix of parents for Python plotting
MatrixXd tree_to_matrix_parents(Node* node) {

	// Populate this matrix by an iterative DFS. MUST follow the same protocol
	// as sister function for creating matrix of states.
	MatrixXd parents(setup_values.dimension, V.size());
	int index = 0;

	std::stack<Node*> fringe;
	fringe.push(node);
	while (fringe.size() > 0) {
		Node* candidate = fringe.top();
		fringe.pop();
		parents.col(index++) = candidate->parent->state;
		for (std::set<Node*>::iterator child = candidate->children.begin(); child != candidate->children.end(); ++child) {
			fringe.push(*child);
		}
	}
	return parents.leftCols(index);
}

/* Returns path from root */
MatrixXd get_path(Node* x) {
	MatrixXd P(setup_values.dimension, V.size());
	int index = 0;
	while (x->state != setup_values.initial_state) {
		P.col(index++) = x->state;
		x = x->parent;
	}
	P.col(index++) = x->state;
	MatrixXd R = P.leftCols(index).rowwise().reverse();
	//std::cout << "R:\n" << R << "\n";
	return R;

}

/* Return distance between two vectors */
inline double sq_dist(VectorXd& p1, VectorXd& p2) {
	return (p1 - p2).transpose()*(p1 - p2);
}

inline double dist(Vector4d& p1, Vector4d& p2) {
	return (p1 - p2).norm();
}

inline double dist(Node* x, Node* y) {
	return dist(x->state, y->state);
}

inline bool inSet(Node* x, std::set<Node*>& Set) {
	return Set.find(x) != Set.end();
}

// Sample uniformly from sample space. Specific to double integrator with square environment
Vector4d sample_uniform() {

	// Sample the new state
	VectorXd x_sample(setup_values.dimension);

	// This is specific to the square environment (and 4d state)
	x_sample(0) = uniform(setup_values.x_min, setup_values.x_max);
	x_sample(1) = uniform(setup_values.x_min, setup_values.x_max);
	x_sample(2) = uniform(setup_values.v_min, setup_values.v_max);
	x_sample(3) = uniform(setup_values.v_min, setup_values.v_max);

	return x_sample;

}

/* HEURISTICS GO HERE */

// Cost of connecting two nodes heuristic
inline double h(Vector4d x, Vector4d y) { // This is specific to double integrator
	Vector2d x_state;
	Vector2d y_state;
	x_state << x(0), x(1);
	y_state << y(0), y(1);
	return (x_state - y_state).norm()/max_speed;
}
/* DONE WITH HEURISTICS */

class distanceComparison {
	Vector4d target;
public:
	distanceComparison(Vector4d target_param) {
		target = target_param;
	}

	// Returns true if x is closer to q than y
	bool operator() (Node* x, Node* y) {
		bool closer;
		if (dist(x->state, target) < dist(y->state, target)) {
			return true;
		} else {
			return false;
		}
	}
};

/* Propogates a state forward according to RK4 integration of the double integrator */
void double_integrator_MonteCarlo_prop(Node* prop_node, Node* new_node) {

	Vector4d x_prop = prop_node->state;

	double prop_time = uniform(0, 3); // Choose a random time to propagate
	double u1 = uniform(setup_values.u_min, setup_values.u_max);
	double u2 = uniform(setup_values.u_min, setup_values.u_max);

	Vector2d controls;
	controls << u1, u2;

	Vector4d x_new = dynamics_library::rk4(dynamics_library::continuous_double_integrator_dynamics, x_prop, controls, prop_time);
	while (!inBounds(x_new)) {
		// Rechoose everything
		prop_time = uniform(0, 3);
		u1 = uniform(setup_values.u_min, setup_values.u_max);
		u2 = uniform(setup_values.u_min, setup_values.u_max);
		controls << u1, u2;

		x_new = dynamics_library::rk4(dynamics_library::continuous_double_integrator_dynamics, x_prop, controls, prop_time);
	}

	new_node->state = x_new;
	new_node->parent = prop_node;
	new_node->cost = prop_time;

	// Collision checking happens here
	if (exists_collision(x_prop, x_new)) {
		new_node->cost = INFTY;
	}

}

void reInitializeRRT() {
	// Delete all nodes, then clear V
	std::set<Node*>::iterator n_it;
	for(n_it = V.begin(); n_it != V.end(); ) {
		Node* n = *n_it;
		V.erase(n_it++);
	}
	V.clear();

	// Create and Insert root back in to V
	root_node = new Node();
	root_node->state = setup_values.initial_state;
	root_node->cost = 0;
	root_node->parent = root_node; // Convention for function tree_to_matrix_parents()
	V.insert(root_node);
}

double selCost(Node* q, Vector4d q_target) {
	return distance_weight * dist(q->state, q_target) + cost_weight * tree_cost(q);
}

void extendToTarget(Vector4d target_state) {

	// Find k nearest neighbors.
	int k = 5;
	std::priority_queue<Node*, std::vector<Node*>, distanceComparison> Q_near (target_state);
	std::set<Node*>::iterator n_it;
	for (n_it = V.begin(); n_it != V.end(); n_it++) {
		Node* n = *n_it;
		if (Q_near.size() < k) {
			Q_near.push(n);
		} else {
			Node* top = Q_near.top();
			if (dist(n->state, target_state) < dist(top->state, target_state)) {
				Q_near.push(n);
				Q_near.pop();
			}
		}
	}

	// Put everything in a set now, to ease coding
	std::set<Node*> Q_near_list;
	while (Q_near.size() > 0) {
		Node* temp = Q_near.top();
		Q_near.pop();
		Q_near_list.insert(temp);
	}

	// Move on to while loop as given in pseudocode
	while (Q_near_list.size() > 0) {

		Node* q_tree = NULL;

		// Find q_tree
		for (n_it = Q_near_list.begin(); n_it != Q_near_list.end(); n_it++) {
			Node* n = *n_it;
			if (q_tree == NULL || selCost(n, target_state) < selCost(q_tree, target_state)) {
				q_tree = n;
			}
		}

		// Remove q_tree from Q_near_list
		Q_near_list.erase(q_tree);

		// Generate extensions (collision checking happens here)
		int num_extensions = 5;
		Node* candidate = NULL;
		for (int i = 0; i < num_extensions; i++) {
			Node* t = new Node;
			double_integrator_MonteCarlo_prop(q_tree, t);
			if (candidate == NULL || t->cost < candidate->cost) {
				if (candidate != NULL) { // Only delete memory when changing candidate
					delete candidate;
				}
				candidate = t;
			}
		}

		// If best cost bound satisfied, update q_new and tree, return success
		if (tree_cost(q_tree) + candidate->cost + h(candidate->state, goal_node->state) <= best_cost) {
			q_new = candidate; // Update q_new
			q_tree->children.insert(q_new); // Update child pointer, parent pointer handled in MonteCarlo prop
			V.insert(q_new);
		} else { // Free up memory from candidate
			delete candidate;
		}

	}

	// Extension is not successful
	//std::cout << "Extension NOT successful... Size of V: " << V.size() << "\n";

}

Vector4d chooseTarget() {

	double p = uniform(0,1);
	if (p >= goal_sampling_probability) {
		return goal_node->state;
	} else {

		Vector4d candidate = sample_uniform();
		int attempts = 0;

		while (h(root_node->state, candidate) + h(candidate, goal_node->state) > best_cost) {
			candidate = sample_uniform();
			attempts += 1;

			if (attempts > max_sample_attempts) {
				for (int i = 0; i < 4; i++) {
					candidate(i) = INFTY;
				}
				std::cout << "Target choice failure...\n";
				break;
			}

		}
		return candidate;
	}

}

double growRRT() {

	// Time it
	std::clock_t this_start;
	double time_elapsed_for_this_run;
	double time_elapsed;
	this_start = std::clock();
	//double last_time = start / (double) CLOCKS_PER_SEC;

	q_new = root_node;

	while (dist(q_new->state, goal_node->state) > setup_values.goal_radius) {
		Vector4d target_state = chooseTarget();

		if (target_state(0) != INFTY) {
			// extendToTarget will add to V, and do the parent/child pointer stuff
			extendToTarget(target_state); // Leaves q_new as is if not successful, otherwise updates the tree and q_new
			//std::cout << "Size of V: " << V.size() << "\n";

			std::clock_t now = std::clock();
			time_elapsed_for_this_run = (now - this_start) / (double) CLOCKS_PER_SEC;
			time_elapsed = (now - start) / (double) CLOCKS_PER_SEC;
			//std::cout << "Time elapsed: " << time_elapsed << "\n";
			if (time_elapsed_for_this_run > setup_values.max_time_per_rrt || time_elapsed > setup_values.max_time) {
	        	std::cout << "Out of time...\n";
				return -1; // Ran out of time, return -1
			}
		}
	}

	return tree_cost(q_new);
}

double anytimeRRT() {

	// Time it
	double last_time = start / (double) CLOCKS_PER_SEC;

	// Begin iterations here
	int k = 1;

	// Initialize values
	distance_weight = 1;
	cost_weight = 0;
	T_Cn = INFTY;
	best_cost = INFTY;
	MatrixXd states;
	MatrixXd parents;
	MatrixXd goal_path;

	double cost;
	while (true) {

		reInitializeRRT();
		cost = growRRT();

		if (cost == -1) {
			break;
		} else {

			// Update stuff best stuffs here
			T_Cn = cost;
			//states = tree_to_matrix_states(root_node);
			//parents = tree_to_matrix_parents(root_node);
			//states = MatrixXd::Zero(setup_values.dimension, 5);
			//parents = MatrixXd::Zero(setup_values.dimension, 5);
			//goal_path = MatrixXd::Zero(setup_values.dimension, 5);
			//goal_path = get_path(q_new);

			// Update new parameters here
			best_cost = (1 - setup_values.eps_f) * T_Cn;
			distance_weight -= setup_values.delta_d;
			if (distance_weight < 0) {
				distance_weight = 0;
			}
			cost_weight += setup_values.delta_c;
			if (cost_weight > 1) {
				cost_weight = 1;
			}

	        //double report_interval = 1; // Report stats every x secs
	        double curr_time = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;

	        //if (curr_time - last_time > report_interval) {
	            // Write time, # of nodes, cost to file
	            ofstream outfile("AnytimeRRT_double_integrator_statistics_" + std::to_string(setup_values.max_time) + "_seconds_run_" + std::to_string(setup_values.stats_id) + ".txt", ios::app);
	            if (outfile.is_open()) {
	                outfile << std::setprecision(10) << ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
	                outfile << ", " << V.size();
	                outfile << ", " << T_Cn << std::endl;
	                outfile.close();
	            }
	            //last_time = curr_time;
	        //}

			// Print stuff
			std::cout << "Iteration number: " << k << "\n";
			std::cout << "Cost found: " << T_Cn << "\n";
			std::cout << "Number of nodes: " << V.size() << "\n";

	        if (curr_time > setup_values.max_time) {
	        	//break;
	        }

			k++;

		}

	}

//	if (T_Cn < INFTY) {
//
//		// Plot in Python
//		std::cout << "Solution found!\n";
//		std::cout << "Initializing display...\n";
//		py::object plotter = di_init_display(); // Initialize python interpreter and pyplot plot
//
//		// Convert Eigen matrices and vectors to Numpy ND arrays
//		np::ndarray states_np = di_eigen_to_ndarray(states);
//		np::ndarray parents_np = di_eigen_to_ndarray(parents);
//		np::ndarray goal_path_np = di_eigen_to_ndarray(goal_path);
//		np::ndarray obstacles_np = di_eigen_to_ndarray(setup_values.obstacles);
//
//		std::cout << "Plotting...\n";
//		di_plot(plotter, states_np, parents_np, goal_path_np, obstacles_np, setup_values.max_time, T_Cn);
//
//	}
	return T_Cn;

}

MatrixXd read_in_obstacle_file(std::string file_name) {

	ifstream fin(file_name);
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

/*
 * Just a note: This function MUST be called from directory that
 * plot_sst.cpp lives in.
 *
 * USAGE: ./anytimeRRT <TIME IN SECONDS> <RANDOMIZE> <OBSTACLE_FILE> <ID_NUMBER_FOR_STATS>
 */

int main(int argc, char* argv[]) {

	// Assumes an optional command line argument of TIME IN SECONDS RANDOMIZE {true, false} BATCH_SIZE
	int max_time = 60; // 1 minute
	std::string randomize = "false";
	int batch_size = 100;
	MatrixXd obstacles(0,0); obstacles.setZero();
	int stats_id = 1;

	if (argc >= 2) {
		max_time = atoi(argv[1]);
	}
	if (argc >= 3) {
		randomize = argv[2];
	}
	if (argc >= 4) {
		obstacles = read_in_obstacle_file(argv[3]);
	}
	if(argc >= 5) {
		stats_id = atoi(argv[4]);
	}

	// Setup function
	setup(max_time, randomize, obstacles, stats_id);

	// Running of BIT*
	std::cout << "Running AnytimeRRT...\n";
	double path_length = anytimeRRT();

	std::cout << "Number of calls to signed distance checker: " << num_collision_check_calls << "\n";
	std::cout << "Best path cost: " << path_length << "\n";
	std::cout << "exiting\n";
}
