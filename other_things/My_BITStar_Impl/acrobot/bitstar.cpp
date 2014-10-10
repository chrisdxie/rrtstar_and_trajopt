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

#include "../../optcontrol/acrobot/acrobot_sqp.hpp"

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

std::vector<std::pair<double,double> > stats;

// Cache useless SQP calls
std::set< std::vector<double> > failedSQPCalls;

int num_true_cost_calls = 0;
int num_collision_check_calls = 0;
int num_samples_pruned = 0;
int num_vertices_pruned = 0;
int num_sample_batches = 0;

int num_failed_adds = 0;
int num_failed_rewires = 0;

std::vector<double> SQP_times;

// TODO
double max_speed = 1;//sqrt(125); // Hard coded for this example

inline double uniform(double low, double high) {
	return (high - low)*(*u_sampler)() + low;
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

void setup(int max_time, std::string& randomize, int batch_size, int stats_id) {

	// This function populates a matrix of values for setting up the problem.
	// Setup variables:
	// 		max_iters
	// 		dimension of problem
	// 		initial state
	//		goal state
	//		delta used for steering
	//		Rectangular obstacle x range
	// 		Rectangular obstacle y range

	// Remember: z = [w, v, theta, x] = [thetadot, xdot, theta, x]

	int d = 4;
	VectorXd initial_state(d);
	initial_state << 0, 0, 0, 0;
	VectorXd goal_state(d);
	goal_state << M_PI, 0, 0, 0;

	int num_obstacles = 2;
	MatrixXd obstacles(4, num_obstacles);
	obstacles.col(0) <<  1.2, .8, 1.2, .8;
	obstacles.col(1) << -1.2, .8, 1.2, .8;

	// Max time in seconds
	setup_values.max_time = max_time;

	// Dimension of problem
	setup_values.dimension = d;

	// Initial State
	setup_values.initial_state = initial_state;

	// Goal Region
	setup_values.goal_state = goal_state;

	// Goal radius
	setup_values.goal_radius = 1;

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
	goal_node->state = setup_values.goal_state;
	goal_node->inV = false;
	goal_node->old = false;
	G.insert(goal_node); // Add goal state
	X_sample.insert(goal_node);

	// Update for the first time
	update_C_ellipse_Matrix();

	// Gamma TODO
	setup_values.gamma = 30; // Looked up thing in paper

	// Randomize argument
	if (randomize == "true") {
		setup_values.randomize = true;
	} else {
		setup_values.randomize = false;
	}

	// Batch size
	setup_values.batch_size = batch_size;

	// Obstacles
	setup_values.obstacles = obstacles;

	// Limits
	setup_values.theta1_min = 0;
	setup_values.theta1_max = 2*M_PI;
	setup_values.theta2_min = 0;
	setup_values.theta2_max = 2*M_PI;
	setup_values.theta1_dot_min = -6;
	setup_values.theta1_dot_max = 6;
	setup_values.theta2_dot_min = -6;
	setup_values.theta2_dot_max = 6;
	setup_values.u_min = -8;
	setup_values.u_max = 8;

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

	setup_values.stats_id = stats_id;

	// Parameters of the system. 
        double m = 1.0;
        double l = 1.0;
        double l2 = 1.0;
        double lc = .5;
        double lc2 = .25;
        double I1 = 0.2;
        double I2 = 1.0;
        set_acrobot_parameters(m, l, l2, lc, lc2, I1, I2);
}

// Returns true if point is inside rectangular polygon
inline bool inside_rectangular_obs(VectorXd& point, double x_min, double x_max,	double y_min, double y_max) {
	return (x_min <= point(0)) && (point(0) <= x_max) && (y_min <= point(1)) && (point(1) <= y_max);
}

inline bool inside_sphere_obs(VectorXd& point, VectorXd& center, double radius) {
	return (point - center).norm() < radius;
}

inline bool inBounds(VectorXd& state) {
	VectorXd pos(2), vel(2);
	pos << state(0), state(1);
	vel << state(2), state(3);
	if (inside_rectangular_obs(pos, setup_values.theta1_min, setup_values.theta1_max,
							   setup_values.theta2_min, setup_values.theta2_max) &&
		inside_rectangular_obs(vel, setup_values.theta1_dot_min, setup_values.theta1_dot_max,
				   setup_values.theta2_dot_min, setup_values.theta2_dot_max)) {
		return true;
	}
	return false;
}

// bool inGoal(Node* x) {
// 	return inside_sphere_obs(x->state, setup_values.goal_state, setup_values.goal_radius);
// }

bool isLeaf(Node* x) {
	return x->children.size() == 0;
}

// 2 dimensional collision check
bool exists_collision(VectorXd& x_near, VectorXd& x_new) {

	// Hard coded for point robot since states are in 2D

	MatrixXd obstacles = setup_values.obstacles;
	int num_obstacles = obstacles.cols();

    	double l1x = acrobot_params.l * cos(x_new(0) - M_PI / 2);
    	double l1y = acrobot_params.l * sin(x_new(0) - M_PI / 2);
    	double l2x = acrobot_params.l * cos(x_new(0) - M_PI / 2) + acrobot_params.l * cos(x_new(0) + x_new(1) - M_PI / 2);
    	double l2y = acrobot_params.l * sin(x_new(0) - M_PI / 2) + acrobot_params.l * sin(x_new(0) + x_new(1) - M_PI / 2); 

    	Matrix<double, 2, 2> end_state_link2;
    	end_state_link2.setZero();
    	end_state_link2.row(0) << l1x, l1y;
    	end_state_link2.row(1) << l2x, l2y;

    	// Actual obstacle detection is here
    	bool obstacle_collision_individ = false;
    	for(unsigned n=0; n < num_obstacles && !obstacle_collision_individ; n++) {

		double x_min = obstacles(0,n) - .5*obstacles(1,n);
		double x_max = obstacles(0,n) + .5*obstacles(1,n);
		double y_min = obstacles(2,n) - .5*obstacles(3,n);
		double y_max = obstacles(2,n) + .5*obstacles(3,n);

		Matrix<double, 4, 2> obs;
		obs.setZero(4,2);
		obs.row(0) << x_min, y_min; obs.row(1) << x_min, y_max;
		obs.row(2) << x_max, y_max; obs.row(3) << x_max, y_min;

        	// Call signed distance function, retrieve value.
        	//Matrix<double, 3, 2> temp1 = signed_distance_2d::signedDistancePolygons(end_state_link1, obs);
        	Matrix<double, 3, 2> temp2 = signed_distance_2d::signedDistancePolygons(end_state_link2, obs);

        	if (/*temp1(0,0) < 0 ||*/ temp2(0,0) < 0) {
        	    obstacle_collision_individ = true;
        	}
    	}

    	// Swept out volume of two states
    	double t_l2x = acrobot_params.l * cos(x_near(0) - M_PI / 2) + acrobot_params.l * cos(x_near(0) + x_near(1) - M_PI / 2);
    	double t_l2y = acrobot_params.l * sin(x_near(0) - M_PI / 2) + acrobot_params.l * sin(x_near(0) + x_near(1) - M_PI / 2); 

    	bool obstacle_collision_swv = false;
    	Matrix<double, 2, 2> swv_poly; swv_poly.setZero();
    	swv_poly.row(0) << l2x, l2y;
    	swv_poly.row(1) << t_l2x, t_l2y;

    	for(unsigned n=0; n < num_obstacles && !obstacle_collision_swv; n++) {
 
                double x_min = obstacles(0,n) - .5*obstacles(1,n);
                double x_max = obstacles(0,n) + .5*obstacles(1,n);
                double y_min = obstacles(2,n) - .5*obstacles(3,n);
                double y_max = obstacles(2,n) + .5*obstacles(3,n);

                Matrix<double, 4, 2> obs;
                obs.setZero(4,2);
                obs.row(0) << x_min, y_min; obs.row(1) << x_min, y_max;
                obs.row(2) << x_max, y_max; obs.row(3) << x_max, y_min;

       		// Call signed distance function, retrieve value.
        	Matrix<double, 3, 2> temp = signed_distance_2d::signedDistancePolygons(swv_poly, obs);

        	if (temp(0,0) < 0) {
        	    obstacle_collision_swv = true;
        	}

    	}

    	return obstacle_collision_swv || obstacle_collision_individ;    
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
		for (std::set<Node*>::iterator child = candidate->children.begin(); child != candidate->children.end(); ++child) {
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
		for (std::set<Node*>::iterator child = candidate->children.begin(); child != candidate->children.end(); ++child) {
			fringe.push(*child);
		}
	}
	return parents.leftCols(i);
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

        double p1_l2x = acrobot_params.l * cos(p1(0) - M_PI / 2) + acrobot_params.l * cos(p1(0) + p1(1) - M_PI / 2);
        double p1_l2y = acrobot_params.l * sin(p1(0) - M_PI / 2) + acrobot_params.l * sin(p1(0) + p1(1) - M_PI / 2);

        double p2_l2x = acrobot_params.l * cos(p2(0) - M_PI / 2) + acrobot_params.l * cos(p2(0) + p2(1) - M_PI / 2);
        double p2_l2y = acrobot_params.l * sin(p2(0) - M_PI / 2) + acrobot_params.l * sin(p2(0) + p2(1) - M_PI / 2);

	return sqrt(pow(p1_l2x - p2_l2x, 2) + pow(p1_l2y - p2_l2y, 2));
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
	return max(fabs(pos(0) - init_pos(0))/setup_values.theta1_dot_max, 
		   fabs(pos(1) - init_pos(1))/setup_values.theta2_dot_max);
	//return x->g_hat;
}
// Cost to go heuristic
inline double h_hat(Node* x) {
	VectorXd pos(2), goal_pos(2);
	pos << x->state(0), x->state(1);
	goal_pos << goal_node->state(0), goal_node->state(1);
        return max(fabs(pos(0) - goal_pos(0))/setup_values.theta1_dot_max,
                   fabs(pos(1) - goal_pos(1))/setup_values.theta2_dot_max);
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

	// Instantiate StdVectorsX and StdVectorU
	StdVectorX X(T);
	StdVectorU U(T-1);

	// Init bounds
	bounds_t bounds;

	bounds.u_min << setup_values.u_min;
	bounds.u_max << setup_values.u_max;
	bounds.x_min << -8*M_PI, -8*M_PI, setup_values.theta1_dot_min, setup_values.theta2_dot_min;
	bounds.x_max <<  8*M_PI,  8*M_PI, setup_values.theta1_dot_max, setup_values.theta2_dot_max;
	bounds.delta_min = 0;
	bounds.delta_max = .5; // I guess we are putting a max on delta
	bounds.x_start = v->state;
	bounds.x_goal = x->state;

        double thetas[3] = {x->state(0), x->state(0)-2*M_PI, x->state(0)+2*M_PI};
	double closest = 50;
	for (int i = 0; i < 3; i++) {
		if (fabs(v->state(0)-thetas[i]) < fabs(v->state(0) - closest)) {
			bounds.x_goal(0) = thetas[i];
			closest = thetas[i];
		}
	}

	thetas[0] = x->state(1);
	thetas[1] = x->state(1) - 2*M_PI;
	thetas[2] = x->state(2) + 2*M_PI;
        closest = 50;
        for (int i = 0; i < 3; i++) {
                if (fabs(v->state(1)-thetas[i]) < fabs(v->state(1) - closest)) {
                        bounds.x_goal(1) = thetas[i];
                        closest = thetas[i];
                }
        }


	/*
	if (x == goal_node) {
		std::cout << "Attempting to connect to GOAL_NODE!!\n";
	}
	*/

	// Initialize pointer to time variable, delta
	double delta = 1;

	num_true_cost_calls++;

	std::clock_t start = std::clock();
	// Call SQP
	int success = solve_acrobot_BVP(X, U, delta, bounds, false /*!x->inV && !inSet(x,G)*/);
        SQP_times.push_back((std::clock() - start) / (double) CLOCKS_PER_SEC);

	// If not success, say the cost is infinity
	if (success == 0) {
		if (!x->inV) {
			//std::cout << "ADDING STATE Unsuccessful...\n";
			//std::cout << "Unsuccessful...\nStart state:\n" << bounds.x_start(0) << " " << bounds.x_start(1) << " " << bounds.x_start(2) << " " << bounds.x_start(3) << "\nGoal state:\n" << bounds.x_goal(0) << " " << bounds.x_goal(1) << " " << bounds.x_goal(2) << " " << bounds.x_goal(3) << "\n";
			num_failed_adds++;
		} else {
			//std::cout << "REWIRING Unsuccessful...\n";
			num_failed_rewires++;
		}		
		//std::cout << "Unsuccessful...\nStart state:\n" << bounds.x_start(0) << " " << bounds.x_start(1) << " " << bounds.x_start(2) << " " << bounds.x_start(3) << "\nGoal state:\n" << bounds.x_goal(0) << " " << bounds.x_goal(1) << " " << bounds.x_goal(2) << " " << bounds.x_goal(3) << "\n";
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

	// If x is considered to be added to the tree, fix new values of velocies
	if (!x->inV && !inSet(x,G)) {
		x->state(2) = X[T-1](2,0); 
		x->state(3) = X[T-1](3,0); 
	}
	if (!x->inV) {
		//std::cout << "ADDING STATE Successful!!!\n";
		//std::cout << "Successful...\nStart state:\n" << bounds.x_start(0) << " " << bounds.x_start(1) << " " << bounds.x_start(2) << " " << bounds.x_start(3) << "\nGoal state:\n" << bounds.x_goal(0) << " " << bounds.x_goal(1) << " " << bounds.x_goal(2) << " " << bounds.x_goal(3) << "\n";
	} else {
		//std::cout << "REWIRING Successful!!!\n";
	}	
	//std::cout << "Start state:\n" << bounds.x_start << "\nGoal state:\n" << bounds.x_goal << "\n";

	StdVectorX allButLastState(X.begin(), X.begin()+T-1);
	e->states = allButLastState;
	e->controls = U;
	return delta * (T-1);

}
// Cost of connecting two nodes heuristic
inline double c_hat(Node* x, Node* y) { // This is specific to cart pole
	if (exists_collision(x->state, y->state)) {
		return INFTY;
	}

        VectorXd x_pos(2), y_pos(2);
        x_pos << x->state(0), x->state(1);
        y_pos << y->state(0), y->state(1);
        return max(fabs(x_pos(0) - y_pos(0))/setup_values.theta1_dot_max,
                   fabs(x_pos(1) - y_pos(1))/setup_values.theta2_dot_max);

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

// Sample uniformly from goal region. Specific to cartpole example. It is sampling from a ball with the goal state as the center
// void sample_from_goal_region() {

// 	int num_samples = 0;
// 	int goal_region_batch_size = setup_values.batch_size/10;

// 	while (num_samples < goal_region_batch_size) {

// 		// Sample the new state
// 		VectorXd x_sample = sample_from_unit_ball();

// 		x_sample *= setup_values.goal_radius;
// 		x_sample += setup_values.goal_state;

// 		// Create the node and insert it into X_sample
// 		if (inBounds(x_sample)) {
// 			Node* n = new Node();
// 			n->state = x_sample;
// 			n->inV = false;
// 			n->old = false;
// 			X_sample.insert(n);
// 			num_samples++;
// 		}
// 	}	
// }

// Sample uniformly from sample space. Specific to cartpole environment
void sample_uniform_batch() {

	int num_samples = 0;

	while (num_samples < setup_values.batch_size) {

		// Sample the new state
		VectorXd x_sample(setup_values.dimension);

		// This is specific to the acrobot environment (and 4d state)
		x_sample(0) = uniform(setup_values.theta1_min, setup_values.theta1_max);
		x_sample(1) = uniform(setup_values.theta2_min, setup_values.theta2_max);
		x_sample(2) = uniform(setup_values.theta1_dot_min, setup_values.theta1_dot_max);
		x_sample(3) = uniform(setup_values.theta2_dot_min, setup_values.theta2_dot_max);


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

	//sample_uniform_batch();
	//return;

	//sample_from_goal_region();

	if (best_cost == INFTY) { // Just sample uniformly from state space
		sample_uniform_batch();
		return;
	}

	int num_samples = 0;
	VectorXd x_center = (setup_values.initial_state + goal_node->state)/2.0;

	best_cost *= max_speed;

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

		// Specific to acrobot, mod by 2PI
		while (x_sample(0) > 2*M_PI) {
			x_sample(0) -= 2*M_PI;	
		}
		while (x_sample(0) < 0) {
			x_sample(0) += 2*M_PI;		
		}
                while (x_sample(1) > 2*M_PI) {
                        x_sample(1) -= 2*M_PI;
                }
                while (x_sample(1) < 0) {
                        x_sample(1) += 2*M_PI;
                }


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
	double last_time = start / (double) CLOCKS_PER_SEC;

	// f_max from paper
	f_max = INFTY;

	// Begin iterations here
	int k = 1;

	while (true) {

		/*
		if (k % 10 == 0) {
			std::cout << "Iteration: " << k << "\n";
			std::cout << "Size of sample set: " << X_sample.size() << "\n";
			std::cout << "Size of vertex set: " << V.size() << "\n";
		}
		*/

		// New Batch of samples!!
		if (Q_edge.size() == 0) {

			//std::cout << "New batch! Batch number: " << num_sample_batches+1 << "\n";

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
						stats.push_back(std::make_pair(f_max, ( std::clock() - start ) / (double) CLOCKS_PER_SEC ));
						//std::cout << "New goal cost: " << f_max << "\n";
						//std::cout << "State of goal:\n" << goal_node->state << "\n";
					}

					pruneEdgeQueue(x);

					double report_interval = 1; // Report stats every x secs
					double curr_time = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;

					if (curr_time - last_time > report_interval) {
						// Write time, # of nodes, cost to file
						ofstream outfile("BITSTAR_acrobot_statistics_" + std::to_string(setup_values.max_time) + "_seconds_run_" + std::to_string(setup_values.stats_id) + ".txt", ios::app);
						if (outfile.is_open()) {
							outfile << std::setprecision(10) << ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
							outfile << ", " << V.size();
							outfile << ", " << f_max << std::endl;
							outfile.close();
						}
						last_time = curr_time;
					}

					if (curr_time > setup_values.max_time) {
						//std::cout << "Done\n";
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
		py::object plotter = ac_init_display(); // Initialize python interpreter and pyplot plot

		// Convert Eigen matrices and vectors to Numpy ND arrays
		np::ndarray goal_path_np = ac_eigen_to_ndarray(goal_path);
		np::ndarray obstacles_np = ac_eigen_to_ndarray(setup_values.obstacles);

		std::cout << "Plotting...\n";
		ac_plot(plotter, goal_path_np, obstacles_np, setup_values.max_time, g_T(goal_node), acrobot_params.l, setup_values.stats_id);

	}
	return g_T(goal_node);

}

/*
 * USAGE: build/bin/bitstar <TIME IN SECONDS> <RANDOMIZE> <BATCH_SIZE> <ID_NUMBER_FOR_STATS>
 */

int main(int argc, char* argv[]) {

	// Assumes an optional command line argument of TIME IN SECONDS RANDOMIZE {true, false} BATCH_SIZE
	int max_time = 60; // 1 minute
	std::string randomize = "false";
	int batch_size = 100;
	int stats_id = 1;

	if (argc >= 2) {
		max_time = atoi(argv[1]);
	}
	if (argc >= 3) {
		randomize = argv[2];
	} 
	if (argc >= 4) {
		batch_size = atoi(argv[3]);
	}
	if (argc >= 5) {
		stats_id = atoi(argv[4]);
	}

	// Setup function
	setup(max_time, randomize, batch_size, stats_id);

	// Running of BIT*
	std::cout << "Running BIT*...\n";
	double path_length = BITStar();

	std::cout << "Number of calls to true cost calculator: " << num_true_cost_calls << "\n";
	std::cout << "Number of unique failed SQP calls: " << failedSQPCalls.size() << "\n";
	std::cout << "As a percentage: " << ((double) failedSQPCalls.size())/num_true_cost_calls << "\n";
	std::cout << "Number of failed adds: " << num_failed_adds << ", as a percentage: " << (double) num_failed_adds/failedSQPCalls.size() << "\n";
	std::cout << "Number of failed rewires: " << num_failed_rewires << ", as a percentage: " << (double) num_failed_rewires/failedSQPCalls.size() << "\n";
	std::cout << "Number of calls to signed distance checker: " << num_collision_check_calls << "\n";
	std::cout << "Best path cost: " << path_length << "\n";

	double avg = 0;
	double std_dev = 0;
	for (std::vector<double>::iterator it = SQP_times.begin(); it != SQP_times.end(); ++it) {
		double n = *it;
		avg += n;
		std_dev += pow(n,2);
	}
	avg /= SQP_times.size();
	std_dev = sqrt(std_dev/SQP_times.size());
	std::cout << "Average call time for SQP: " << avg << "\n";
	std::cout << "Standard deviation for call time for SQP: " << std_dev << "\n";

	std::cout << "Stats:\n";
	for(std::vector<std::pair<double,double> >::iterator it = stats.begin(); it != stats.end(); ++it) {
		std::cout << it->first << " " << it->second << std::endl;
	}

	std::cout << "exiting\n";
}
