#include <iostream>

#include <boost/random.hpp>

#include <eigen3/Eigen/Eigen>
using namespace Eigen;

#include <fstream>
#include <iomanip>
#include <math.h>
#include <set>
#include <stack>
#include <map>
#include <queue>
#include <ctime>
#include <unordered_set>

#include "bitstar.h"
#include "plot_bitstar.h"

//#include "../../double_integrator_dynamics_library/double_integrator_dynamics.hpp"
//using namespace double_integrator_dynamics;

#include "../../2d_signed_distance_library_cpp/signedDistancePolygons.hpp"

#include "../../optcontrol/rally_car/rally_car_sqp.hpp"

// Global variables
SetupObject setup_values;
std::set<Node*> V;
std::set<Node*> Q_V;
std::set<Node*> G; // Set of goal nodes in V. It is a strict subset of V
std::set<Node*> X_sample;
std::set<Edge*> Q_edge;

double r; // Radius from RRT*, updated every iteration
int n; // Number of vertices in V, updated every iteration
double f_max;
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

MatrixXd C_ellipse;

// Cache useless SQP calls
std::set< std::vector<double> > failedSQPCalls;

int num_true_cost_calls = 0;
int num_collision_check_calls = 0;
int num_samples_pruned = 0;
int num_vertices_pruned = 0;
int num_sample_batches = 0;

double max_speed = sqrt(pow(18,2) + pow(18,2)); // Hard coded for this example

inline double uniform(double low, double high) {
	return (high - low)*(rand() / double(RAND_MAX)) + low;
}

// Cost of node from root of tree
inline double g_T(Node* x) {
	if (x->inV) {

		if (x->state == setup_values.initial_state) {
			return x->cost; // Should be 0 for root node!
		} else {
			return x->cost + g_T(x->parent);
		}

	} else {
		return INFTY;
	}
}

// This needs to update every time we find a new goal
void update_C_ellipse_Matrix() {
	int d = setup_values.dimension;
	VectorXd goal_state = goal_node->state;

	// SVD stuff for ball
	VectorXd a1 = goal_state - setup_values.initial_state;
	MatrixXd M(d, d);
	M.setZero(d,d);
	M.col(0) = a1;

	// Perform SVD on M
	MatrixXd U = M.jacobiSvd(ComputeFullU | ComputeFullV).matrixU();
	MatrixXd V = M.jacobiSvd(ComputeFullU | ComputeFullV).matrixV();

	// Create W matrix
	MatrixXd W(d, d);
	W.setZero(d,d);
	for(int i = 0; i < d; ++i) {
		if (i == d - 1) {
			W(i, i) = U.determinant() * V.determinant();
		} else {
			W(i, i) = 1;
		}
	}

	// Multiply to update C matrix
	C_ellipse = U * W * V.transpose();

}

void setup(int max_time, std::string& randomize, int batch_size) {

	// This function populates a matrix of values for setting up the problem.
	// Setup variables:
	// 		max_iters
	// 		dimension of problem
	// 		initial state
	//		goal state
	//		delta used for steering
	//		Rectangular obstacle x range
	// 		Rectangular obstacle y range

	int d = 8;
	VectorXd initial_state(d);
	initial_state << 18, -30, 0, 16, 1.57, 0, 0, 0;
	VectorXd goal_state(d);
	goal_state << -18, -30, 0, -16, -1.57, 0, 0, 0;

	// Max Iterations
	setup_values.max_time = max_time;

	// Dimension of problem
	setup_values.dimension = d;

	// Initial State
	setup_values.initial_state = initial_state;

	// Goal Region
	setup_values.goal_state = goal_state;

	// Setup intial state and add it to V; note that T and E are implicity represented
	// by root node. Can perform DFS to find T, E are children pointers of every node in T
	root_node = new Node();
	root_node->state = setup_values.initial_state;
	root_node->cost = 0;
	root_node->parent = root_node; // Convention for function tree_to_matrix_parents()
	root_node->inV = true;
	root_node->old = false;
	V.insert(root_node);

	goal_node = new Node;
	goal_node->state = goal_state;
	goal_node->inV = false;
	goal_node->old = false;
	G.insert(goal_node);
	X_sample.insert(goal_node);

	// Update for the first time
	update_C_ellipse_Matrix();

	// Gamma
	setup_values.gamma = 65; // Looked up thing in paper

	// Randomize argument
	if (randomize == "true") {
		setup_values.randomize = true;
	} else {
		setup_values.randomize = false;
	}

	// Batch size
	setup_values.batch_size = batch_size;

	// Obstacles
	// Hard code obstacles
	MatrixXd obstacles(4,6);
	obstacles.col(0) << 0, 42, 20.5, 1;
	obstacles.col(1) << 20.5, 1, -5.5, 53;
	obstacles.col(2) << 9.5, 1, -16, 32;
	obstacles.col(3) << 15, 12, -32.5, 1;
	obstacles.col(4) << -9.5, 1, -16, 32;
	obstacles.col(5) << -20.5, 1, -5.5, 53;
	setup_values.obstacles = obstacles;

	// #x y xdot ydot theta thetadot wf wr
	// state_space: 
	//   min: [-25, -35, -18, -18, -3.14, -17, -40, -40]
	//   max: [25, 25, 18, 18, 3.14, 17, 40, 40]

	// Limits for state
	setup_values.x_min = -25;
	setup_values.x_max = 25;
	setup_values.y_min = -35;
	setup_values.y_max = 25;
	setup_values.xdot_min = -18;
	setup_values.xdot_max = 18;
	setup_values.ydot_min = -18;
	setup_values.ydot_max = 18;
	setup_values.theta_min = -M_PI;
	setup_values.theta_max = M_PI;
	setup_values.thetadot_min = -17;
	setup_values.thetadot_max = 17;
	setup_values.wf_min = -40;
	setup_values.wf_max = 40;
	setup_values.wr_min = -40;
	setup_values.wr_max = 40;

	// Limits for controls
	// input_control_space:
	//   min: [-1.0472, -700, -700]
	//   max: [1.0472, 0, 1200]
	setup_values.sta_min = -1.0472;
	setup_values.sta_max = 1.0472;
	setup_values.tf_min = -700;
	setup_values.tf_max = 0;
	setup_values.tr_min = -700;
	setup_values.tr_max = 1200;


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

	// normal dist
	g_dist = new G_DIST(0, 1);
	g_sampler = new G_GENERATOR(*r_eng, *g_dist);

	// uniform dist
	u_dist = new U_DIST(0, 1);
	u_sampler = new U_GENERATOR(*r_eng, *u_dist);

}

// Returns true if point is inside rectangular polygon
inline bool inside_rectangular_obs(VectorXd& point, double x_min, double x_max,	double y_min, double y_max) {
	return (x_min <= point(0)) && (point(0) <= x_max) && (y_min <= point(1)) && (point(1) <= y_max);
}

inline bool inside_sphere_obs(VectorXd& point, VectorXd& center, double radius) {
	return (point - center).norm() < radius;
}

inline bool inBounds(VectorXd& state) {

	VectorXd pos(2), vel(2), theta_stuff(2), w_stuff(2);
	pos << state(0), state(1);
	vel << state(2), state(3);
	theta_stuff << state(4), state(5);
	w_stuff << state(6), state(7);
	if (inside_rectangular_obs(pos, setup_values.x_min, setup_values.x_max,
							   setup_values.y_min, setup_values.y_max) &&
		inside_rectangular_obs(vel, setup_values.xdot_min, setup_values.xdot_max,
				   setup_values.ydot_min, setup_values.ydot_max) && 
		inside_rectangular_obs(theta_stuff, setup_values.theta_min, setup_values.theta_max,
				   setup_values.thetadot_min, setup_values.thetadot_max) && 
		inside_rectangular_obs(w_stuff, setup_values.wf_min, setup_values.wf_max,
				   setup_values.wr_min, setup_values.wr_max)) {
		return true;
	}
	return false;
}

// bool inGoal(Node* x) {
// 	VectorXd pos(2), vel(2);
// 	pos << x->state(0), x->state(1);
// 	vel << x->state(2), x->state(3);

// 	double pos_x_min = setup_values.goal_region(0) - .5*setup_values.goal_region(1);
// 	double pos_x_max = setup_values.goal_region(0) + .5*setup_values.goal_region(1);
// 	double pos_y_min = setup_values.goal_region(2) - .5*setup_values.goal_region(3);
// 	double pos_y_max = setup_values.goal_region(2) + .5*setup_values.goal_region(3);

// 	double vel_x_min = setup_values.goal_region(4) - .5*setup_values.goal_region(5);
// 	double vel_x_max = setup_values.goal_region(4) + .5*setup_values.goal_region(5);
// 	double vel_y_min = setup_values.goal_region(6) - .5*setup_values.goal_region(7);
// 	double vel_y_max = setup_values.goal_region(6) + .5*setup_values.goal_region(7);

// 	return inside_rectangular_obs(pos, pos_x_min, pos_x_max, pos_y_min, pos_y_max) &&
// 		   inside_rectangular_obs(vel, vel_x_min, vel_x_max, vel_y_min, vel_y_max);
// }

bool isLeaf(Node* x) {
	return x->children.size() == 0;
}

// 2 dimensional collision check. Return true if point is contained inside rectangle
bool exists_collision(VectorXd& x_near, VectorXd& x_new) {

	// Hard coded for point robot since states are in 2D

	MatrixXd obstacles = setup_values.obstacles;
	int num_obstacles = obstacles.cols();

	for (int n = 0; n < num_obstacles; n++) {

		double x_min = obstacles(0,n) - .5*obstacles(1,n); x_min -= 1;
		double x_max = obstacles(0,n) + .5*obstacles(1,n); x_max += 1;
		double y_min = obstacles(2,n) - .5*obstacles(3,n); y_min -= 1;
		double y_max = obstacles(2,n) + .5*obstacles(3,n); y_max += 1;

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
	MatrixXd states(setup_values.dimension, V.size()*T);
	int index = 0;

	std::stack<Node*> fringe;
	fringe.push(node);
	while (fringe.size() > 0) {
		Node* candidate = fringe.top();
		fringe.pop();

		if (candidate != root_node) {
			for (int i = 1; i <= T-2; ++i) { // Hacked, hard coded
				states.col(index++) = candidate->states[i];
			}
		}
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
	MatrixXd parents(setup_values.dimension, V.size()*T);
	int index = 0;

	std::stack<Node*> fringe;
	fringe.push(node);
	while (fringe.size() > 0) {
		Node* candidate = fringe.top();
		fringe.pop();

		//parents.col(index++) = candidate->parent->state;
		if (candidate != root_node) {
			for (int i = 0; i <= T-2; ++i) {
				parents.col(index++) = candidate->states[i];
			}
		} else {
			parents.col(index++) = candidate->state; // Root node state
		}

		for (std::set<Node*>::iterator child = candidate->children.begin(); child != candidate->children.end(); ++child) {
			fringe.push(*child);
		}
	}
	return parents.leftCols(index);
}

/* Returns path from root */
// Hacks everywhere in this method... not clean at all. Oh well
MatrixXd get_path(Node* x) {
	MatrixXd P(setup_values.dimension, V.size()*T);
	int index = 0;

	while (x->state != setup_values.initial_state) {

		P.col(index) = x->state; index++;
		for (int i = T-2; i >= 1; --i) { // Hacked, hard coded
			P.col(index) = x->states[i];
			index ++;
		}

		x = x->parent;
	}
	P.col(index) = x->state;
	index++;
	MatrixXd R = P.leftCols(index).rowwise().reverse();
	//std::cout << "R:\n" << R << "\n";
	return R;

}

/* Return distance between two vectors */
inline double sq_dist(VectorXd& p1, VectorXd& p2) {
	return (p1 - p2).transpose()*(p1 - p2);
}

inline double dist(VectorXd& p1, VectorXd& p2) {
	VectorXd _p1(3), _p2(3), residual(3);
	_p1 << p1(0), p1(1), p1(4);
	_p2 << p2(0), p2(1), p2(4);
	residual = _p1 - _p2; residual(2) = fabs(residual(2));
	if (residual(2) > M_PI) {
		residual(2) = 2*M_PI - residual(2);
	}
	return residual.norm();
}

inline int factorial(int n) {
	return n == 0 ? 1 : n * factorial(n-1);
}

// Sample from unit ball: http://math.stackexchange.com/questions/87230/picking-random-points-in-the-volume-of-sphere-with-uniform-probability/87238#87238
inline VectorXd sample_from_unit_ball() {

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

inline bool inSet(Node* x, std::set<Node*>& Set) {
	return Set.find(x) != Set.end();
}

/* HEURISTICS GO HERE */

// Cost to come heuristic
inline double g_hat(Node* x) {
	VectorXd pos(2), init_pos(2);
	pos << x->state(0), x->state(1);
	init_pos << setup_values.initial_state(0), setup_values.initial_state(1);
	return (pos - init_pos).norm()/max_speed;
	//return x->g_hat;
}
// Cost to go heuristic
inline double h_hat(Node* x) {
	VectorXd pos(2), goal_pos(2);
	pos << x->state(0), x->state(1);
	goal_pos << goal_node->state(0), goal_node->state(1);
	return (pos - goal_pos).norm()/max_speed;
	//return x->h_hat;
}
// Cost of x_start to x_goal where path is constrained to go through x heuristic
inline double f_hat(Node* x) {
	return g_hat(x) + h_hat(x);
	//return x->f_hat;
}
// Exact cost of connecting two nodes
double c(Edge* e) {

	Node* v = e->v;
	Node* x = e->x;

	// Reject if this was called before and it fails
	std::vector<double> start_and_end_states;
	for (int i = 0; i < setup_values.dimension; i++) { // Order doesn't matter, as long as it's consistent
		start_and_end_states.push_back(v->state(i));
	}
	for (int i = 0; i < setup_values.dimension; i++) { // Order doesn't matter, as long as it's consistent
		start_and_end_states.push_back(x->state(i));
	}
	if (failedSQPCalls.find(start_and_end_states) != failedSQPCalls.end()) {
		return INFTY;
	}

	// Reject if orientation differs by more than pi/2. HACK to reduce number of useless TrajOpt calls
	Vector2d v_orien, x_orien;
	v_orien << v->state(2), v->state(3);
	x_orien << x->state(2), x->state(3);
	if (v_orien.any() && x_orien.any()) { // If either of the velocities is 0, 0: it could be an initial/goal state. Always connect
		v_orien.normalize();
		x_orien.normalize();
		if (v_orien.transpose()*x_orien < 0 || v_orien.transpose()*x_orien > 1) {
			return INFTY;
		}
		// If the start state is in front of the goal state, chuck it as well
		Vector2d x_pos, v_pos;
		x_pos << x->state(0), x->state(1);
		v_pos << v->state(0), v->state(1);
		if (v_orien.transpose() * (x_pos-v_pos) < 0) {
			return INFTY;
		}
	}

	num_true_cost_calls++;

	// Instantiate StdVectorsX and StdVectorU
	StdVectorX X(T);
	StdVectorU U(T-1);

	// Init bounds
	bounds_t bounds;

	bounds.u_min << setup_values.u_min, setup_values.u_min;
	bounds.u_max << setup_values.u_max, setup_values.u_max;
	bounds.x_min << setup_values.x_min, setup_values.x_min, setup_values.v_min, setup_values.v_min;
	bounds.x_max << setup_values.x_max, setup_values.x_max, setup_values.v_max, setup_values.v_max;
	bounds.delta_min = 0;
	bounds.delta_max = 10; // I guess we are putting a max on delta
	bounds.x_start = v->state;
	bounds.x_goal = x->state;

	// Initialize pointer to time variable, delta
	double delta = 1;

	// Call SQP (Redundant call, whatever. fix later)
	int success = solve_double_integrator_BVP(X, U, delta, bounds);

	// If not success, say the cost is infinity
	if (success == 0) {
		//std::cout << "Unsuccessful...\nStart state:\n" << bounds.x_start << "\nGoal state:\n" << bounds.x_goal << "\n";
		failedSQPCalls.insert(start_and_end_states);
		return INFTY;
	}
	// Collision check of intermediate states
	for (int i = 0; i < T-1; i++) {
		VectorXd x1(setup_values.dimension), x2(setup_values.dimension);
		x1 << X[i];
		x2 << X[i+1];
		if (exists_collision(x1, x2)) {
			failedSQPCalls.insert(start_and_end_states);
			return INFTY;
		}
	}

	StdVectorX allButLastState(X.begin(), X.begin()+T-1);
	e->states = allButLastState;
	e->controls = U;
	return delta * (T-1);

}
// Cost of connecting two nodes heuristic
inline double c_hat(Node* x, Node* y) { // This is specific to double integrator
	VectorXd x_pos(2);
	VectorXd y_pos(2);
	x_pos << x->state(0), x->state(1);
	y_pos << y->state(0), y->state(1);
	if (exists_collision(x_pos, y_pos)) {
		return INFTY;
	}
	return (x_pos - y_pos).norm()/max_speed;
}
/* DONE WITH HEURISTICS */

Edge* bestPotentialEdge() {
	std::set<Edge*>::iterator it;
	Edge* best_edge = NULL;
	double best_cost = INFTY;
	double curr_cost = 0;
	for(it = Q_edge.begin(); it != Q_edge.end(); ++it) {
		Edge* e = *it;
		curr_cost = g_T(e->v) + e->heuristic_cost + h_hat(e->x);
		if (best_edge == NULL || curr_cost < best_cost) {
			best_edge = e;
			best_cost = curr_cost;
		}
	}

	//std::cout << "best_cost: " << best_cost << " map best cost: " << Q_edge.begin()->first << std::endl;

	return best_edge;
}

Node* bestPotentialNode() {
	std::set<Node*>::iterator it;
	Node* best_node = NULL;
	double best_cost = INFTY;
	double curr_cost = 0;
	for(it = Q_V.begin(); it != Q_V.end(); ++it) {
		Node* v = *it;
		curr_cost = g_T(v) + h_hat(v);
		if (best_node == NULL || curr_cost < best_cost) {
			best_node = v;
			best_cost = curr_cost;
		}
	}
	return best_node;
}

// Sample uniformly from goal region
// void sample_from_goal_region() {

// 	int num_samples = 0;
// 	int goal_region_batch_size = setup_values.batch_size/10;

// 	double pos_x_min = setup_values.goal_region(0) - .5*setup_values.goal_region(1);
// 	double pos_x_max = setup_values.goal_region(0) + .5*setup_values.goal_region(1);
// 	double pos_y_min = setup_values.goal_region(2) - .5*setup_values.goal_region(3);
// 	double pos_y_max = setup_values.goal_region(2) + .5*setup_values.goal_region(3);

// 	double vel_x_min = setup_values.goal_region(4) - .5*setup_values.goal_region(5);
// 	double vel_x_max = setup_values.goal_region(4) + .5*setup_values.goal_region(5);
// 	double vel_y_min = setup_values.goal_region(6) - .5*setup_values.goal_region(7);
// 	double vel_y_max = setup_values.goal_region(6) + .5*setup_values.goal_region(7);

// 	while (num_samples < goal_region_batch_size) {

// 		// Sample the new state
// 		VectorXd x_sample(setup_values.dimension);

// 		// This is specific to the square environment (and 4d state)
// 		x_sample(0) = uniform(pos_x_min, pos_x_max);
// 		x_sample(1) = uniform(pos_y_min, pos_y_max);
// 		x_sample(2) = uniform(vel_x_min, vel_x_max);
// 		x_sample(3) = uniform(vel_y_min, vel_y_max);

// 		// Create the node and insert it into X_sample
// 		Node* n = new Node();
// 		n->state = x_sample;
// 		n->inV = false;
// 		n->old = false;
// 		X_sample.insert(n);
// 		num_samples++;

// 	}	

// }

// Sample uniformly from sample space. 
void sample_uniform_batch() {

	int num_samples = 0;

	while (num_samples < setup_values.batch_size) {

		// Sample the new state
		VectorXd x_sample(setup_values.dimension);

		// This is specific to the environment (and 8d state)
		x_sample(0) = uniform(setup_values.x_min, setup_values.x_max);
		x_sample(1) = uniform(setup_values.y_min, setup_values.y_max);
		x_sample(2) = uniform(setup_values.xdot_min, setup_values.xdot_max);
		x_sample(3) = uniform(setup_values.ydot_min, setup_values.ydot_max);
		x_sample(4) = uniform(setup_values.theta_min, setup_values.theta_max);
		x_sample(5) = uniform(setup_values.thetadot_min, setup_values.thetadot_max);
		x_sample(6) = uniform(setup_values.wf_min, setup_values.wf_max);
		x_sample(7) = uniform(setup_values.wr_min, setup_values.wr_max);


		// Create the node and insert it into X_sample
		Node* n = new Node();
		n->state = x_sample;
		n->inV = false;
		n->old = false;
		X_sample.insert(n);
		num_samples++;

	}	

}

// Sample from bounding ellipse
void sample_batch() {

	num_sample_batches++;

	double best_cost = g_T(goal_node);
	double c_min = g_hat(goal_node);

	//sample_from_goal_region();

	if (best_cost == INFTY) { // Just sample uniformly from state space
		sample_uniform_batch();
		return;
	}

	int num_samples = 0;
	VectorXd x_center = (setup_values.initial_state + goal_node->state)/2.0;

	// Create L matrix from bigger radius
	MatrixXd L(setup_values.dimension, setup_values.dimension);
	L.setZero(setup_values.dimension, setup_values.dimension);
	for(int i = 0; i < setup_values.dimension; ++i) {
		if (i == 0) {
			L(i, i) = best_cost*0.5;
		} else {
			L(i, i) = sqrt((best_cost*best_cost) - (c_min*c_min))*0.5;
		}
	}

	//std::cout << "c_min: " << c_min << " best_cost: " << best_cost << std::endl;

	MatrixXd CL = C_ellipse*L;
	while (num_samples < setup_values.batch_size) {
		VectorXd x_ball = sample_from_unit_ball();
		VectorXd x_sample = CL*x_ball + x_center;

		if (inBounds(x_sample)) {
			Node* n = new Node();
			n->state = x_sample;
			n->inV = false;
			n->old = false;
			X_sample.insert(n);
			num_samples++;
		}

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

void clearVertexQueue() {
	// Delete pointers to nodes. These nodes live in the tree, so DON'T ACTUALLY DELETE
	Q_V.clear(); // Make sure this doesn't dynamically delete objects.
}

void pruneSampleSet() {
	// Prune samples that are useless
	std::set<Node*>::iterator n_it;
	for(n_it = X_sample.begin(); n_it != X_sample.end(); ) {
		Node* n = *n_it;
		if (f_hat(n) > g_T(goal_node)) {
			num_samples_pruned++;
			if (inSet(n,G)) {
				G.erase(n);
			}
			delete n;
			X_sample.erase(n_it++);
		} else {
			++n_it;
		}
	}
}

// Throw this node and all of its chidren into X_sample
void pruneVertex(Node* v) {

	// Prune it's children first. Iterate over a copy of the children set
	std::set<Node*> children(v->children.begin(), v->children.end());
	std::set<Node*>::iterator n_it;
	for(n_it = children.begin(); n_it != children.end(); n_it++) {
		Node* child = *n_it;
		pruneVertex(child);
	}

	// Disconnect it's edge
	Node* p = v->parent;
	p->children.erase(v);
	v->parent = NULL;

	// Mark as new, throw into X_sample, erase from other sets.
	v->old = false;
	v->inV = false;
	X_sample.insert(v);
	V.erase(v);
	if (inSet(v, G)) {
		G.erase(v);
	}
	if (inSet(v, Q_V)) {
		Q_V.erase(v);
	}

}

void pruneVertexSet() {
	// Prune vertices that are useless, along with their subtrees. Throw them all into X_sample
	std::set<Node*>::iterator n_it;
	n_it = V.begin();
	while (n_it != V.end()) {
		Node* n = *n_it;
		if (f_hat(n) > g_T(goal_node)) {
			num_vertices_pruned++;
			pruneVertex(n);

			// Reset iterator since it will be invalidated
			n_it = V.begin();

		} else {
			n_it++;
		}
	}
}

void pruneEdgeQueue(Node* w) {
	std::set<Edge*>::iterator e_it;
	for(e_it = Q_edge.begin(); e_it != Q_edge.end(); ) {
		Edge* e = *e_it;
		if (e->x == w) {
			if (g_T(e->v) + e->heuristic_cost >= g_T(w)) {
				delete e; // Delete memory
				Q_edge.erase(e_it++);
			} else {
				++e_it;
			}
		} else {
			++e_it;
		}
	}
}

void updateEdgeQueue(Node* v) {

	Q_V.erase(v);

	// Find all samples in X_sample in ball of radius r w/ center v
	std::set<Node*> X_near;
	std::set<Node*>::iterator n_it;
	for(n_it = X_sample.begin(); n_it != X_sample.end(); ++n_it) {
		Node* x = *n_it;
		if (dist(x->state, v->state) <= r) {
			X_near.insert(x);
		}
	}

	// Add all potential edges to Q_edge
	for(n_it = X_near.begin(); n_it != X_near.end(); ++n_it) {
		Node* x = *n_it;
		double heuristic_edge_cost = c_hat(v, x);
		if (g_hat(v) + heuristic_edge_cost + h_hat(x) < g_T(goal_node)) {
			Edge* e = new Edge(v, x);
			e->heuristic_cost = heuristic_edge_cost;
			Q_edge.insert(e);
		}
	}

	// New vertices, adding to rewire queue happens here
	if (!v->old) {

		// Find all nodes in V in ball of radius r w/ center v
		std::set<Node*> V_near;
		for(n_it = V.begin(); n_it != V.end(); ++n_it) {
			Node* w = *n_it;
			if (dist(w->state, v->state) <= r) {
				V_near.insert(w);
			}
		}

		// Add some of these potential rewirings
		for(n_it = V_near.begin(); n_it != V_near.end(); ++n_it) {
			Node* w = *n_it;
			double heuristic_edge_cost = c_hat(v, w);
			if (g_hat(v) + heuristic_edge_cost + h_hat(w) < g_T(goal_node)) { // Same condition as above
				if (g_T(v) + heuristic_edge_cost < g_T(w)) { // could improve cost-to-come to the target child
					if (v != w->parent && w != v->parent) { // Make sure this edge isn't already in the tree
						Edge* e = new Edge(v, w);
						e->heuristic_cost = heuristic_edge_cost;
						Q_edge.insert(e);
					}
				}
			}
		}

		v->old = true;
	}

}

// Lines 9-10 in the pseudocode
void updateQueue() {

	bool expand = true;
	while (expand) {
		if (Q_V.size() > 0) {

			Edge* best_edge = bestPotentialEdge();
			double best_edge_cost;
			if (best_edge == NULL) {
				best_edge_cost = INFTY;
			} else {
				best_edge_cost = g_T(best_edge->v) + best_edge->heuristic_cost + h_hat(best_edge->x);
			}

			Node* best_node = bestPotentialNode();
			double best_node_cost = g_T(best_node) + h_hat(best_node);

			if (best_node_cost <= best_edge_cost) {
				updateEdgeQueue(best_node);
			} else {
				expand = false;
			}

			/* Use this if that problem shows up again. */

			/*
			// Edge* best_edge = bestPotentialEdge();
			// double best_edge_cost;
			// if (best_edge == NULL) {
			// 	best_edge_cost = INFTY;
			// } else {
			// 	best_edge_cost = g_T(best_edge->v) + best_edge->heuristic_cost + h_hat(best_edge->x);
			// }

			Node* best_node = bestPotentialNode();
			double best_node_cost = g_T(best_node) + h_hat(best_node);

			updateEdgeQueue(best_node);

			// if (best_node_cost <= best_edge_cost) {
			// 	updateEdgeQueue(best_node);
			// } else {
			// 	expand = false;
			// }
			*/

		} else {
			expand = false;
		}
	}

}

// Iterate through all goal nodes and find best one
double costOfBestGoalNode() {

	bool changed = false;
	double best_cost = g_T(goal_node);
	std::set<Node*>::iterator n_it;

	for(n_it = G.begin(); n_it != G.end(); n_it++) {
		Node* candidate = *n_it;
		if (g_T(candidate) < best_cost) {
			goal_node = candidate;
			best_cost = g_T(candidate);
			changed = true;
		}
	}
	// Now update C matrix if need be
	if (changed) {
		update_C_ellipse_Matrix();
	}

	return best_cost;
}

double BITStar() {

	// Time it
	std::clock_t start;
	double duration;
	start = std::clock();

	// f_max from paper
	f_max = INFTY;

	// Begin iterations here
	int k = 1;

	while (true) {

		if (k % 100 == 0) {
			std::cout << "Iteration: " << k << "\n";
			std::cout << "Size of sample set: " << X_sample.size() << "\n";
			std::cout << "Size of vertex set: " << V.size() << "\n";
		}

		// New Batch of samples!!
		if (Q_edge.size() == 0) {

			std::cout << "New batch! Batch number: " << num_sample_batches+1 << "\n";

			pruneVertexSet();			
			pruneSampleSet();
			clearVertexQueue();

			sample_batch();

			for (std::set<Node*>::iterator n_it = V.begin(); n_it != V.end(); n_it++) {
				Node* n = *n_it;
				Q_V.insert(n);
				n->old = true; // Mark them as old
			}

			// Set radius value
			n = V.size() + X_sample.size();
			r = setup_values.gamma * pow(log(n)/n, 1.0/setup_values.dimension); // Copied from RRT* code given by Sertac Karaman

		}

		// Update Q_edge for vertices in the expansion queue Q_V
		updateQueue();

		// Grab best potential edge
		Edge* e = bestPotentialEdge();
		Q_edge.erase(e);
		Node* v = e->v; Node* x = e->x;

		if (g_T(v) + e->heuristic_cost + h_hat(x) < g_T(goal_node)) {

			double cvx = c(e); // Calculate true cost of edge
			
			if (g_hat(v) + cvx + h_hat(x) < g_T(goal_node)) {

				if (g_T(v) + cvx < g_T(x)) {

					// Update the new controls
					x->states = e->states;
					x->controls = e->controls;
					delete e; // After grabbing pointers to v, x, heuristic cost, and using states/controls, we have no need for the Edge e

					if (x->inV) {
						Node* p = x->parent;
						p->children.erase(x); // Erase child pointer
					} else {
						// Remove x from sample
						X_sample.erase(x);
						// Insert x into V and Q_V
						V.insert(x);
						Q_V.insert(x);
					}

					// Update costs, create edge (parent and child pointer)
					x->cost = cvx;
					x->parent = v;
					v->children.insert(x);

					// Check if node is in goal
					// if (inGoal(x) && !x->inV) {
					// 	G.insert(x);
					// 	std::cout << "New goal state found:\n" << x->state << "\n";
					// }
					x->inV = true;

					// Update best cost stuff
					double old_cost = f_max;
					f_max = costOfBestGoalNode(); // Best goal node is update here too
					if (f_max < old_cost) {
						std::cout << "New goal cost: " << f_max << "\n";
						//std::cout << "State of goal:\n" << goal_node->state << "\n";
					}

					pruneEdgeQueue(x);

					// Write time, # of nodes, cost to file
					ofstream outfile("BITSTAR_rally_car_statistics_" + std::to_string(setup_values.max_time) + "_seconds.txt", ios::app);
					if (outfile.is_open()) {
						outfile << std::setprecision(10) << ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
						outfile << ", " << V.size();
						outfile << ", " << f_max << std::endl;
						outfile.close();
					}

					double curr_time = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
					if (curr_time > setup_values.max_time) {
						std::cout << "Done\n";
						break;
					}

				} else {
					delete e; // Don't leak memory
				}

			} else {
				delete e; // Don't leak memory
			}
		} else {
			delete e; // Don't leak memory
			clearEdgeQueue();
		}
		k++;
	}

	if (g_T(goal_node) < INFTY) {

		std::cout << "Size of V: " << V.size() << "\n";
		std::cout << "Number of samples pruned: " << num_samples_pruned << "\n";
		std::cout << "Number of vertices pruned: " << num_vertices_pruned << "\n";
		std::cout << "Number of batches: " << num_sample_batches << "\n";

		MatrixXd goal_path = get_path(goal_node);

		// Plot in Python
		std::cout << "Solution found!\n";
		std::cout << "Initializing display...\n";
		py::object plotter = di_init_display(); // Initialize python interpreter and pyplot plot

		// Convert Eigen matrices and vectors to Numpy ND arrays
		np::ndarray states_np = di_eigen_to_ndarray(tree_to_matrix_states(root_node));
		np::ndarray parents_np = di_eigen_to_ndarray(tree_to_matrix_parents(root_node));
		np::ndarray goal_path_np = di_eigen_to_ndarray(goal_path);
		np::ndarray obstacles_np = di_eigen_to_ndarray(setup_values.obstacles);

		std::cout << "Plotting...\n";
		di_plot(plotter, states_np, parents_np, goal_path_np, obstacles_np, setup_values.max_time, g_T(goal_node));

	}
	return g_T(goal_node);

}

/*
 * Just a note: This function MUST be called from directory that
 * plot_sst.cpp lives in.
 *
 * USAGE: build/bin/bitstar <TIME IN SECONDS> <RANDOMIZE> <BATCH_SIZE>
 */

int main(int argc, char* argv[]) {

	// Assumes an optional command line argument of TIME IN SECONDS RANDOMIZE {true, false} BATCH_SIZE
	int max_time = 60; // 1 minute
	std::string randomize = "false";
	int batch_size = 100;

	if (argc >= 2) {
		max_time = atoi(argv[1]);
	}
	if (argc >= 3) {
		randomize = argv[2];
	} 
	if (argc >= 4) {
		batch_size = atoi(argv[3]);
	}

	// Setup function
	setup(max_time, randomize, batch_size);

	// Running of BIT*
	std::cout << "Running BIT*...\n";
	double path_length = BITStar();

	std::cout << "Number of calls to true cost calculator: " << num_true_cost_calls << "\n";
	std::cout << "Number of calls to signed distance checker: " << num_collision_check_calls << "\n";
	std::cout << "Best path cost: " << path_length << "\n";
	std::cout << "exiting\n";
}
