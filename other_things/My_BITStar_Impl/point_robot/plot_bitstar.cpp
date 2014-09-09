#include <iostream>

#include <boost/python.hpp>
#include <boost/python/numeric.hpp>
#include <boost/python/tuple.hpp>
#include <boost/numpy.hpp>
#include <boost/filesystem.hpp>
#include <boost/random.hpp>

namespace py = boost::python;
namespace np = boost::numpy;

#include <eigen3/Eigen/Eigen>
using namespace Eigen;

#include <SetupObject.h>
#include <Node.h>
#include <Edge.h>
#include <math.h>
#include <set>
#include <queue>

#define INFTY 1e10

#include "../../double_integrator_dynamics_library/double_integrator_dynamics.hpp"
using namespace double_integrator_dynamics;

// Global variables
SetupObject setup_values;
std::set<Node*> V;
std::set<Node*> X_sample;
std::set<Edge*> Q_edge;
std::set<Edge*> Q_rewire;
double r; // Radius from RRT*, updated every iteration
int n; // Number of vertices in V, updated every iteration
double f_max, f_prev;
Node* root_node;
Node* goal_node;

// For sampling in ball
typedef boost::mt19937 RANDOM_ENGINE;
typedef boost::normal_distribution<> G_DIST;
typedef boost::uniform_real<> U_DIST;
typedef boost::variate_generator<RANDOM_ENGINE, G_DIST> G_GENERATOR;
typedef boost::variate_generator<RANDOM_ENGINE, U_DIST> U_GENERATOR;

RANDOM_ENGINE *r_eng;
G_DIST *g_dist;
U_DIST *u_dist;
G_GENERATOR *g_sampler;
U_GENERATOR *u_sampler;

MatrixXd C;

int num_true_cost = 0;

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
		np::ndarray goal_path, np::ndarray obstacles, int iters, double cost) {

	try {
	    // pass control to python now
		plotter(states, parents, goal_path, obstacles, iters, cost);
	}
	catch(py::error_already_set const &) {
	    // will pass python errors to cpp for printing
		PyErr_Print();
	}
}

void setup(int max_iters, std::string randomize) {

	// This function populates a matrix of values for setting up the problem.
	// Setup variables:
	// 		max_iters
	// 		dimension of problem
	// 		initial state
	//		goal state
	//		delta used for steering
	//		Rectangular obstacle x range
	// 		Rectangular obstacle y range

	int d = 2;
	VectorXd initial_state(d);
	initial_state << 0, 0;
	VectorXd goal_state(d);
	goal_state << 9, 9;

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
	setup_values.goal_state = goal_state;

	// Gamma
	setup_values.gamma = 10; // Looked up thing in paper

	// Randomize argument
	if (randomize == "true") {
		setup_values.randomize = true;
	} else {
		setup_values.randomize = false;
	}

	// Obstacles
	setup_values.obstacles = obstacles;

	// Limits
	setup_values.x_min = -10; // Center of 5, side length of 12
	setup_values.x_max = 10;

	// Sampling stuff
	if (setup_values.randomize) {
		r_eng = new RANDOM_ENGINE(time(NULL));
	} else {
		r_eng = new RANDOM_ENGINE();
	}

	// To specifically seed it
	if (randomize != "true" && randomize != "false") {
		delete r_eng;
		std::cout << randomize << "\n";
		std::cout << randomize.c_str() << "\n";
		r_eng = new RANDOM_ENGINE(atoi(randomize.c_str())); // seed
	};

	// normal dist
	g_dist = new G_DIST(0, 1);
	g_sampler = new G_GENERATOR(*r_eng, *g_dist);

	// uniform dist
	u_dist = new U_DIST(0, 1);
	u_sampler = new U_GENERATOR(*r_eng, *u_dist);

	// SVD stuff for ball
	VectorXd a1 = goal_state - initial_state;
  	MatrixXd M(d, d); M.setZero();
  	M.col(0) = a1;

  	// Perform SVD on M
  	MatrixXd U = M.jacobiSvd(ComputeFullU | ComputeFullV).matrixU();
  	MatrixXd V = M.jacobiSvd(ComputeFullU | ComputeFullV).matrixV();

  	// Create W matrix
  	MatrixXd W(d, d); W.setZero();
  	for(int i = 0; i < d; ++i) {
  		if (i == d - 1) {
  			W(i, i) = U.determinant() * V.determinant();
  		} else {
  			W(i, i) = 1;
  		}
  	}

  	// Multiply to update C matrix
  	C = U * W * V.transpose();

}

// Returns true if point is inside rectangular polygon
bool inside_rectangular_obs(Vector2d point, double x_min, double x_max,
		double y_min, double y_max) {

	return (x_min <= point(0)) && (point(0) <= x_max)
			&& (y_min <= point(1)) && (point(1) <= y_max);

}

bool inBounds(VectorXd state) {
	if (inside_rectangular_obs(state, setup_values.x_min, setup_values.x_max,
			setup_values.x_min, setup_values.x_max)) {
		return true;
	}
	return false;
}

// 2 dimensional collision check. Return true if point is contained inside rectangle
bool exists_collision(VectorXd x_near, VectorXd x_new) {

	// Hard coded for point robot since states are in 2D

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
// Returns a matrix of states for Python plotting
MatrixXd tree_to_matrix_states(Node* node) {

	// Populate this matrix by an iterative DFS. MUST follow the same protocol
	// as sister function for creating matrix of parents.
	MatrixXd states(setup_values.dimension, V.size());
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
	MatrixXd parents(setup_values.dimension, V.size());
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

//int num_nodes(Node* tree) {
//
//	int total = 0;
//	if (tree->children.size() == 0) {
//		int i = 0;
//		return 1;
//	}
//	for (std::vector<Node*>::iterator child = tree->children.begin();
//			child != tree->children.end(); child++) {
//		Node* kid = *child;
//		total += num_nodes(kid);
//	}
//	return total+1;
//
//}

/* Returns path from root */
MatrixXd get_path(Node* x) {
	MatrixXd P(2, setup_values.max_iters); // HARD CODE FOR 2D
	int index = 0;
	while (x->state != setup_values.initial_state) {
		P.col(index) = x->state;
		x = x->parent;
		index++;
	}
	P.col(index) = x->state;
	index++;
	MatrixXd R = P.leftCols(index).rowwise().reverse();
	std::cout << "R:\n" << R << "\n";
	return R;

}

/* Return distance between two vectors */
double dist(VectorXd p1, VectorXd p2) {
	return (p1 - p2).norm();
}

int factorial(int n) {
	return n == 0 ? 1 : n * factorial(n-1);
}

// Used in calculating high dimensional ellipsoid volume
double unitBallVolume() {
	// Returns the volume of the unit ball in n dimensions
	int n = setup_values.dimension;
	if (n % 2 == 0) { // Even
		return pow(M_PI, n/2) / factorial(n/2 + 1);
	} else { // Odd
		return pow(M_PI, n/2.0) / ( (factorial(2*n)*sqrt(M_PI)) / (pow(4,n) * factorial(n)) );
	}
}

// Sample from unit ball: http://math.stackexchange.com/questions/87230/picking-random-points-in-the-volume-of-sphere-with-uniform-probability/87238#87238
VectorXd sample_from_unit_ball() {

    double R = 1; // Unit ball has radius 1

    // Sample length
    double length = R * pow((*u_sampler)(), 1.0/setup_values.dimension);

    // Sample direction
    VectorXd dir(setup_values.dimension);
    for (int i = 0; i < setup_values.dimension; i++) {
        dir(i) = (*g_sampler)();
    }
    // Normalize direction to length one, the multiply by sampled length
    dir.normalize();
    dir *= length;

    return dir;

}

bool inSet(Node* x, std::set<Node*> Set) {
	return Set.find(x) != Set.end();
}

/* HEURISTICS GO HERE */

// Cost to come heuristic
double g_hat(Node* x) {
	return (x->state - setup_values.initial_state).norm();
}
// Cost to go heuristic
double h_hat(Node* x) {
	return (x->state - setup_values.goal_state).norm();
}
// Cost of x_start to x_goal where path is constrained to go through x heuristic
double f_hat(Node* x) {
	return g_hat(x) + h_hat(x);
}
// Exact cost of connecting two nodes
double c(Node* x, Node* y) {
	num_true_cost++;
	return (x->state - y->state).norm();
}
// Cost of connecting two nodes heuristic
double c_hat(Node* x, Node* y) { // This is specific to point robot example in paper
	if (exists_collision(x->state, y->state)) {
		return INFTY;
	}
	return c(x, y);
}
// Cost of node from root of tree
double g_T(Node* x) {
	if (!inSet(x, V)) {
		return INFTY;
	}
	return x->cost;
}
/* DONE WITH HEURISTICS */


Edge* bestPotentialEdge() {
	std::set<Edge*>::iterator it;
	Edge* best_edge = NULL;
	double best_cost;
	for(it = Q_edge.begin(); it != Q_edge.end(); ++it) {
		Edge* e = *it;
		if (best_edge == NULL || g_T(e->v) + c_hat(e->v, e->x) + h_hat(e->x) < best_cost) {
			best_edge = e;
			best_cost = g_T(e->v) + c_hat(e->v, e->x) + h_hat(e->x);
		}
	}
	return best_edge;
}

Edge* bestPotentialRewiring() {
	std::set<Edge*>::iterator it;
	Edge* best_edge = NULL;
	double best_cost;
	for(it = Q_rewire.begin(); it != Q_rewire.end(); ++it) {
		Edge* e = *it;
		if (best_edge == NULL || g_T(e->v) + c_hat(e->v, e->x) + h_hat(e->x) < best_cost) {
			best_edge = e;
			best_cost = g_T(e->v) + c_hat(e->v, e->x) + h_hat(e->x);
		}
	}
	return best_edge;
}

// Lol, fancy name. These calculations are found using the radii of the ellipse. The
// radii of the ellipse are found by techniques in the Informed RRT* paper.
double prolateHyperSpheroidShellVolume(double smaller_diameter, double bigger_diameter) {

	double ubv = unitBallVolume();
	double c_min = g_hat(goal_node);

	double small_vol = ubv * pow(sqrt(pow(smaller_diameter,2) - pow(c_min,2))/2.0, setup_values.dimension-1)
						   * smaller_diameter / 2.0;
	if (smaller_diameter < c_min) {
		small_vol = 0;
	}
	double big_vol = ubv * pow(sqrt(pow(bigger_diameter,2) - pow(c_min,2))/2.0, setup_values.dimension-1)
						 * bigger_diameter / 2.0;

	return big_vol - small_vol;

}

void sample_batch(double smaller_diameter, double bigger_diameter, double rho) {

	int num_samples = 0;
	double lambda_shell_vol = prolateHyperSpheroidShellVolume(smaller_diameter, bigger_diameter);
	VectorXd x_center = (setup_values.initial_state + setup_values.goal_state)/2.0;

	// Create L matrix from bigger radius
	MatrixXd L_big(setup_values.dimension, setup_values.dimension); L_big.setZero();
	MatrixXd L_small(setup_values.dimension, setup_values.dimension); L_small.setZero();
	double c_min = g_hat(goal_node);
  	for(int i = 0; i < setup_values.dimension; ++i) {
  		if (i == 0) {
  			L_big(i, i) = bigger_diameter/2; // Float division
  			L_small(i,i) = smaller_diameter/2;
  		} else {
  			L_big(i, i) = sqrt(pow(bigger_diameter,2) - pow(c_min,2))/2.0;
  			L_small(i, i) = sqrt(pow(smaller_diameter,2) - pow(c_min,2))/2.0;
  		}
  	}

	// Perform rejection sampling on the big ellipse until ratio is satisfied
	while (num_samples < rho * lambda_shell_vol) {
  		VectorXd x_ball = sample_from_unit_ball();
  		VectorXd x_sample = C*L_big*x_ball + x_center;

  		// Check whether it's in the smaller ellipse by using the formula for quadratic form
  		// This can ONLY be done when smaller radius is greater than c_min. Otherwise L_small will have stuff
  		// Also, it's correct to just sample from total ellipse otherwise
  		//if (inBounds(x_sample)) {
  		if (smaller_diameter > c_min && (x_sample - x_center).transpose() * (C*L_small*L_small.transpose()*C.transpose()).inverse() * (x_sample - x_center) < 1) {
  			continue;
  		} else {
  			Node* n = new Node;
  			n->state = x_sample;
  			X_sample.insert(n);
  			num_samples++;
  		}
  		//}

	}

}

void clearEdgeQueue() {
	// Delete all edges, then clear edge queue
	std::set<Edge*>::iterator e_it;
	for(e_it = Q_edge.begin(); e_it != Q_edge.end(); ) {
		Edge* e = *e_it;
		delete e; // Delete memory
		Q_edge.erase(e_it++);
	}
	Q_edge.clear();
}

void clearRewiringQueue() {
	// Delete all edges, then clear rewire queue
	std::set<Edge*>::iterator e_it;
	for(e_it = Q_rewire.begin(); e_it != Q_rewire.end(); ) {
		Edge* e = *e_it;
		delete e;
		Q_rewire.erase(e_it++);
	}
	Q_rewire.clear();
}

void updateFreeQueue(Node* v) {

	double f_sample = std::min(f_hat(v) + 2*r, f_max);
	if (f_sample > f_prev) {
		double lambda_sample = prolateHyperSpheroidShellVolume(f_prev, f_sample);
		double rho = n / lambda_sample;
		sample_batch(f_prev, f_sample, rho);
		f_prev = f_sample;
	}

}

void updateEdgeQueue(Node* v) {
	updateFreeQueue(v);

	// Remove all edges with v in it
	std::set<Edge*>::iterator e_it;
	for(e_it = Q_edge.begin(); e_it != Q_edge.end(); ) {
		Edge* e = *e_it;
		if (e->v == v || e->x == v) {
			delete e; // Delete memory
			Q_edge.erase(e_it++);
		} else {
			++e_it;
		}
	}

	// Find all samples in X_sample in ball of radius r w/ center v
	std::set<Node*> X_n;
	std::set<Node*>::iterator n_it;
	for(n_it = X_sample.begin(); n_it != X_sample.end(); ++n_it) {
		Node* x = *n_it;
		if (dist(x->state, v->state) <= r) {
			X_n.insert(x);
		}
	}

	// Add all potential edges to Q_edge
	for(n_it = X_n.begin(); n_it != X_n.end(); ++n_it) {
		Node* x = *n_it;
		if (g_hat(v) + c_hat(v, x) + h_hat(x) < g_T(goal_node)) {
			Q_edge.insert(new Edge(v, x));
		}
	}

}

void updateRewireQueue(Node* v) {

	// Find all nodes in V in a ball of radius r w/ center v
	std::set<Node*> V_n;
	std::set<Node*>::iterator n_it;
	for(n_it = V.begin(); n_it != V.end(); ++n_it) {
		Node* w = *n_it;
		if (dist(w->state, v->state) <= r) {
			V_n.insert(w);
		}
	}

	// Add all potential rewirings to Q_rewire
	for(n_it = V_n.begin(); n_it != V_n.end(); ++n_it) {
		Node* w = *n_it;
		if (g_T(v) + c_hat(v, w) < g_T(w)) {
			Q_rewire.insert(new Edge(v, w));
		}
	}

}

void Rewire() {

	while (Q_rewire.size() > 0) {

		Edge* e = bestPotentialRewiring();
		Q_rewire.erase(e);
		Node* u = e->v; Node* w = e->x;
		delete e; // After grabbing pointers to u and w, we have no need for the Edge e

		if (g_T(u) + c_hat(u, w) + h_hat(w) < g_T(goal_node)) {

			if (g_T(u) + c(u, w) < g_T(w)) {
				// Erase edge (w_parent, w) from tree, set new edge (u, w)
				Node* w_parent = w->parent; // First erase parent->child pointer
				w_parent->children.erase(w);
				w->parent = u; // This erases child->parent pointer and sets new one
				w->cost = g_T(u) + c(u, w);
			}

		} else {
			clearRewiringQueue();
		}
	}

}

double BITStar() {

	// Initialize random seed using current time.
	// Uncomment this line if you want feed the random number generator a seed based on time.

	// Setup intial state and add it to V; note that T and E are implicity represented
	// by root node. Can perform DFS to find T, E are children pointers of every node in T
	root_node = new Node;
	root_node->state = setup_values.initial_state;
	root_node->cost = 0;
	root_node->parent = root_node; // Convention for function tree_to_matrix_parents()
	V.insert(root_node);

	// Add goal state to X_sample
	goal_node = new Node;
	goal_node->state = setup_values.goal_state;
	X_sample.insert(goal_node);

	// f_max from paper
	f_max = INFTY;

	// Begin iterations here
	int k = 1;

	while (k <= setup_values.max_iters) {

		std::cout << "Iteration: " << k << "\n";

		// Set values
		n = V.size();
		r = setup_values.gamma * pow(log(n)/n, 1.0/setup_values.dimension); // Copied from RRT* code given by Sertac Karaman
		if (r == 0) { // Radius hack to fight degeneracy of first iteration
			r = setup_values.gamma * pow(log(2)/2, 1.0/setup_values.dimension);
		}

		// Set f_prev to 0 for this iteration
		f_prev = 0;

		// Update Q_edge for all nodes in V
		std::set<Node*>::iterator it;
		for( it = V.begin(); it != V.end(); ++it) {
			Node* v = *it;
			updateEdgeQueue(v);
		}

		// Process edges in order of potential
		while (Q_edge.size() > 0) {

			// Grab best potential edge
			Edge* e = bestPotentialEdge();
			Q_edge.erase(e);
			Node* v = e->v; Node* x = e->x;
			delete e; // After grabbing pointers to v and x, we have no need for the Edge e

			// Collision checking happens implicitly here, in c_hat function
			if (g_T(v) + c_hat(v, x) + h_hat(x) < g_T(goal_node)) {
				if (g_hat(v) + c(v, x) + h_hat(x) < g_T(goal_node)) {

					// Update costs, insert node, create edge (parent and child pointer)
					x->cost = g_T(v) + c(v, x);
					V.insert(x);
					x->parent = v;
					v->children.insert(x);

					// Remove x from sample
					X_sample.erase(x);

					// Stuff in paper
					updateEdgeQueue(x);
					updateRewireQueue(x);
					Rewire();
					f_max = g_T(goal_node);

				}
			} else {
				clearEdgeQueue();
			}
		}
		k++;
	}

	if (g_T(goal_node) < INFTY) {

		std::cout << "Size of V: " << V.size() << "\n";

		MatrixXd goal_path = get_path(goal_node);

		// Plot in Python
		std::cout << "Solution found!\n";
		std::cout << "Initializing display...\n";
		py::object plotter = init_display(); // Initialize python interpreter and pyplot plot

		// Convert Eigen matrices and vectors to Numpy ND arrays
		np::ndarray states_np = eigen_to_ndarray(tree_to_matrix_states(root_node));
		np::ndarray parents_np = eigen_to_ndarray(tree_to_matrix_parents(root_node));
		np::ndarray goal_path_np = eigen_to_ndarray(goal_path);
		np::ndarray obstacles_np = eigen_to_ndarray(setup_values.obstacles);

		std::cout << "Plotting...\n";
		plot(plotter, states_np, parents_np, goal_path_np, obstacles_np, setup_values.max_iters, g_T(goal_node));

	}
	return g_T(goal_node);

}

/*
 * Just a note: This function MUST be called from directory that
 * plot_sst.cpp lives in.
 *
 * USAGE: build/bin/plot_sst <MAX_ITERS> <RANDOMIZE>
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

    // Setup function
    setup(max_iters, randomize);

    std::cout << "Running BIT*...\n";
    double path_length = BITStar();

    std::cout << "Number of calls to true cost calculator: " << num_true_cost << "\n";
    std::cout << "Best path cost: " << path_length << "\n";
    std::cout << "exiting\n";
}

