#include <iostream>

#include <boost/random.hpp>

#include <eigen3/Eigen/Eigen>
using namespace Eigen;

#include <math.h>
#include <set>
#include <stack>
#include <map>
#include <queue>
#include <ctime>

#include "bitstar.h"
#include "plot_bitstar.h"

//#include "../../double_integrator_dynamics_library/double_integrator_dynamics.hpp"
//using namespace double_integrator_dynamics;

#include "../../2d_signed_distance_library_cpp/signedDistancePolygons.hpp"

#include "../../double_integrator_FORCES/SQP/without_CD/double_integrator_noCD_sqp.hpp"

// Global variables
SetupObject setup_values;
std::vector<Node*> V;
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

int num_true_cost_calls = 0;
int num_collision_check_calls = 0;
int num_samples_pruned = 0;

double max_speed = sqrt(2); // Hard coded for this example

inline double uniform(double low, double high) {
	return (high - low)*(rand() / double(RAND_MAX)) + low;
}

void setup(int max_iters, std::string& randomize) {

	// This function populates a matrix of values for setting up the problem.
	// Setup variables:
	// 		max_iters
	// 		dimension of problem
	// 		initial state
	//		goal state
	//		delta used for steering
	//		Rectangular obstacle x range
	// 		Rectangular obstacle y range

	VectorXd initial_state(2);
	initial_state << 0, 0;
	VectorXd goal_state(2);
	goal_state << 9, 9;

	int num_obstacles = 3;
	MatrixXd obstacles(4, num_obstacles);
	obstacles.col(0) << 3, 4, 3, 2;
	obstacles.col(1) << 2, 1, 8, 3;
	obstacles.col(2) << 7, 2, 7, 2;

	// Max Iterations
	setup_values.max_iters = max_iters;

	// Dimension of problem
	setup_values.dimension = 4;

	// Initial State
	setup_values.initial_state = initial_state;

	// Goal Region
	setup_values.goal_state = goal_state;

	// Gamma
	setup_values.gamma = 6; // Looked up thing in paper

	// Randomize argument
	if (randomize == "true") {
		setup_values.randomize = true;
	} else {
		setup_values.randomize = false;
	}

	// Obstacles
	setup_values.obstacles = obstacles;

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

	// normal dist
	g_dist = new G_DIST(0, 1);
	g_sampler = new G_GENERATOR(*r_eng, *g_dist);

	// uniform dist
	u_dist = new U_DIST(0, 1);
	u_sampler = new U_GENERATOR(*r_eng, *u_dist);

	// SVD stuff for ball
	int d = 4;
	VectorXd a1(d);
	a1 << goal_state - initial_state, 0, 0;
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
	C = U * W * V.transpose();

}

// Returns true if point is inside rectangular polygon
inline bool inside_rectangular_obs(VectorXd& point, double x_min, double x_max,	double y_min, double y_max) {
	return (x_min <= point(0)) && (point(0) <= x_max) && (y_min <= point(1)) && (point(1) <= y_max);
}

inline bool inBounds(VectorXd& state) {
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

// 2 dimensional collision check. Return true if point is contained inside rectangle
bool exists_collision(VectorXd& x_near, VectorXd& x_new) {

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
// Hacks everywhere in this method... not clean at all. Oh well
MatrixXd get_path(Node* x) {
	MatrixXd P(setup_values.dimension, V.size()*T);
	int index = 0;
	VectorXd extended_initial_state(setup_values.dimension);
	extended_initial_state << setup_values.initial_state, 0, 0;

	while (x->state != extended_initial_state) {

		P.col(index) = x->state; index++;
		for (int i = T-2; i >= 0; --i) { // Hacked, hard coded
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
	return (p1 - p2).norm();
}

inline int factorial(int n) {
	return n == 0 ? 1 : n * factorial(n-1);
}

// Used in calculating high dimensional ellipsoid volume
inline double unitBallVolume() {
	// Returns the volume of the unit ball in n dimensions
	//int n = setup_values.dimension;
	//	if (n % 2 == 0) { // Even
	//		return pow(M_PI, n/2) / factorial(n/2 + 1);
	//	} else { // Odd
	//		return pow(M_PI, n/2.0) / ( (factorial(2*n)*sqrt(M_PI)) / (pow(4,n) * factorial(n)) );
	//	}
	return M_PI;
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
//inline double g_hat(Node* x) {
	//return (x->state - setup_values.initial_state).norm();
	//return x->g_hat;
//}
// Cost to go heuristic
//inline double h_hat(Node* x) {
	//return (x->state - setup_values.goal_state).norm();
	//return x->h_hat;
//}
// Cost of x_start to x_goal where path is constrained to go through x heuristic
//inline double f_hat(Node* x) {
	//return g_hat(x) + h_hat(x);
	//return x->f_hat;
//}
// Exact cost of connecting two nodes
double c(Edge* e) {
	num_true_cost_calls++;

	Node* v = e->v;
	Node* x = e->x;

	// Instantiate StdVectorsX and StdVectorU
	StdVectorX X(T);
	StdVectorU U(T-1);

	// Init bounds
	bounds_t bounds;

	bounds.u_max = setup_values.u_max;
	bounds.u_min = setup_values.u_min;
	bounds.x_max = setup_values.x_max;
	bounds.x_min = setup_values.x_min;
	bounds.v_max = setup_values.v_max;
	bounds.v_min = setup_values.v_min;
	bounds.x_start = v->state;
	bounds.x_goal = x->state;

	// Initialize pointer to time variable, delta
	double* delta;
	double delta_init = 1;
	delta = &delta_init;

	// Call SQP (Redundant call, whatever. fix later)
	int success = solve_double_integrator_noCD_BVP(bounds.x_start, bounds.x_goal, X, U, delta, bounds);

	// If not success, say the cost is infinity
	if (success == 0) {
		//std::cout << "Unsuccessful...\nStart state:\n" << bounds.x_start << "\nGoal state:\n" << bounds.x_goal << "\n";
		return INFTY;
	}

	StdVectorX allButLastState(X.begin(), X.begin()+T-1);
	e->states = allButLastState;
	e->controls = U;
	return *delta * (T-1);

}
// Cost of connecting two nodes heuristic
inline double c_hat(Node* x, Node* y) { // This is specific to double integrator
	VectorXd x_state(2);
	VectorXd y_state(2);
	x_state << x->state(0), x->state(1);
	y_state << y->state(0), y->state(1);
	if (exists_collision(x_state, y_state)) {
		return INFTY;
	}
	return (x_state - y_state).norm()/max_speed;
}
/* DONE WITH HEURISTICS */


Edge* bestPotentialEdge() {
	std::set<Edge*>::iterator it;
	Edge* best_edge = NULL;
	double best_cost = INFTY;
	double curr_cost = 0;
	for(it = Q_edge.begin(); it != Q_edge.end(); ++it) {
		Edge* e = *it;
		curr_cost = g_T(e->v) + e->heuristic_cost + (e->x->h_hat);
		if (best_edge == NULL || curr_cost < best_cost) {
			best_edge = e;
			best_cost = curr_cost;
		}
	}

	//std::cout << "best_cost: " << best_cost << " map best cost: " << Q_edge.begin()->first << std::endl;

	return best_edge;
}

Edge* bestPotentialRewiring() {
	std::set<Edge*>::iterator it;
	Edge* best_edge = NULL;
	double best_cost = INFTY;
	double curr_cost = 0;
	for(it = Q_rewire.begin(); it != Q_rewire.end(); ++it) {
		Edge* e = *it;
		curr_cost = g_T(e->v) + e->heuristic_cost + (e->x->h_hat);
		if (best_edge == NULL || curr_cost < best_cost) {
			best_edge = e;
			best_cost = curr_cost;
		}
	}
	return best_edge;
}

// Lol, fancy name. These calculations are found using the radii of the ellipse. The
// radii of the ellipse are found by techniques in the Informed RRT* paper.
double prolateHyperSpheroidShellVolume(double smaller_diameter, double bigger_diameter) {

	double ubv = unitBallVolume();
	double c_min = (goal_node->g_hat);

	double small_vol = ubv * pow(sqrt((smaller_diameter*smaller_diameter) - (c_min*c_min))*0.5, setup_values.dimension-1) * smaller_diameter * 0.5;
	if (smaller_diameter < c_min) {
		small_vol = 0;
	}
	double big_vol = ubv * pow(sqrt((bigger_diameter*bigger_diameter) - (c_min*c_min))*0.5, setup_values.dimension-1) * bigger_diameter * 0.5;

	return big_vol - small_vol;

}

void sample_batch(double smaller_diameter, double bigger_diameter, double rho)
{
	double c_min = (goal_node->g_hat);
	if (smaller_diameter > c_min) {
		return;
	}

	int num_samples = 0;
	double lambda_shell_vol = prolateHyperSpheroidShellVolume(smaller_diameter, bigger_diameter);
	VectorXd x_center(4); // Hard coded
	x_center << (setup_values.initial_state + setup_values.goal_state)/2.0, 0, 0;

	// Create L matrix from bigger radius
	MatrixXd L_big(setup_values.dimension, setup_values.dimension);
	L_big.setZero(setup_values.dimension, setup_values.dimension);
	MatrixXd L_small(setup_values.dimension, setup_values.dimension);
	L_small.setZero(setup_values.dimension, setup_values.dimension);
	for(int i = 0; i < setup_values.dimension; ++i) {
		if (i == 0) {
			L_big(i, i) = bigger_diameter*0.5;
			L_small(i,i) = smaller_diameter*0.5;
		} else {
			L_big(i, i) = sqrt((bigger_diameter*bigger_diameter) - (c_min*c_min))*0.5;
			L_small(i, i) = sqrt((smaller_diameter*smaller_diameter) - (c_min*c_min))*0.5;
		}
	}

	std::cout << "c_min: " << c_min << " smaller_diameter: " << smaller_diameter << " bigger_diameter: " << bigger_diameter << " lambda shell: " << lambda_shell_vol << " rho: " << rho << " num samples: " << rho * lambda_shell_vol << std::endl;

	MatrixXd CLLTCTInv = (C*L_small*L_small.transpose()*C.transpose()).inverse();
	MatrixXd CLBig = C*L_big;
	// Perform rejection sampling on the big ellipse until ratio is satisfied
	while (num_samples < rho * lambda_shell_vol) {
		VectorXd x_ball = sample_from_unit_ball();
		VectorXd x_sample = CLBig*x_ball + x_center;

		// Check whether it's in the smaller ellipse by using the formula for quadratic form
		// This can ONLY be done when smaller radius is greater than c_min. Otherwise L_small will have stuff
		// Also, it's correct to just sample from total ellipse otherwise
		if (inBounds(x_sample)) {
			if (/*(smaller_diameter > c_min) && */ ((x_sample - x_center).transpose() * CLLTCTInv * (x_sample - x_center) < 1)) {
				continue;
			} else {
				Node* n = new Node;
				n->state = x_sample;
				VectorXd temp_state(2);
				temp_state << n->state(0), n->state(1);
				n->g_hat = (temp_state - setup_values.initial_state).norm()/max_speed;
				n->h_hat = (temp_state - setup_values.goal_state).norm()/max_speed;
				n->f_hat = n->g_hat + n->h_hat;
				X_sample.insert(n);
				num_samples++;
			}
		}

	}

	//std::cout << "Num samples in batch: " << num_samples << std::endl;

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

void pruneSampleSet() {
	// Prune samples that are useless
	std::set<Node*>::iterator n_it;
	for(n_it = X_sample.begin(); n_it != X_sample.end(); ) {
		Node* n = *n_it;
		if (n->f_hat > g_T(goal_node)) {
			num_samples_pruned++;
			delete n;
			X_sample.erase(n_it++);
		} else {
			++n_it;
		}
	}
}

void updateFreeQueue(Node* v) {

	double f_sample = std::min((v->f_hat) + 2*r, f_max);
	// f_reqd or f_sample?
	if (f_sample > f_prev) {
		double lambda_sample = prolateHyperSpheroidShellVolume(f_prev, f_sample);
		double rho = (double)n / lambda_sample;
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
		double heuristic_edge_cost = c_hat(v, x);
		if ((v->g_hat) + heuristic_edge_cost + (x->h_hat) < g_T(goal_node)) {
			Edge* e = new Edge(v, x);
			e->heuristic_cost = heuristic_edge_cost;
			//double cost = g_T(e->v) + e->heuristic_cost + (e->x->h_hat);
			Q_edge.insert(e);
		}
	}

}

void updateRewireQueue(Node* v) {

	// Find all nodes in V in a ball of radius r w/ center v
	std::vector<Node*> V_n;
	std::vector<Node*>::iterator n_it;
	for(n_it = V.begin(); n_it != V.end(); ++n_it) {
		Node* w = *n_it;
		if (dist(w->state, v->state) <= r) {
			V_n.push_back(w);
		}
	}

	// Add all potential rewirings to Q_rewire
	for(n_it = V_n.begin(); n_it != V_n.end(); ++n_it) {
		Node* w = *n_it;
		double heuristic_edge_cost = c_hat(v, w);
		if (g_T(v) + heuristic_edge_cost < g_T(w)) {
			Edge* e = new Edge(v, w);
			e->heuristic_cost = heuristic_edge_cost;
			//double cost = g_T(e->v) + e->heuristic_cost + (e->x->h_hat);
			Q_rewire.insert(e);
		}
	}

}

void Rewire()
{
	while (Q_rewire.size() > 0) {

		Edge* e = bestPotentialRewiring();
		Q_rewire.erase(e);
		Node* u = e->v; Node* w = e->x;

		if (g_T(u) + e->heuristic_cost + (w->h_hat) < g_T(goal_node))
		{
			double wcost = g_T(u) + c(e);
			if (wcost < g_T(w)) {
				w->states = e->states;
				w->controls = e->controls;
				delete e; // After grabbing pointers to v, x and using heuristic cost, we have no need for the Edge e

				//if (exists_collision(u->state, w->state)) {
				//	continue;
				//}

				// Erase edge (w_parent, w) from tree, set new edge (u, w)
				Node* w_parent = w->parent; // First erase parent->child pointer
				w_parent->children.erase(w);
				w->parent = u; // This erases child->parent pointer and sets new one
				w->cost = wcost;
			}

		} else {
			clearRewiringQueue();
		}
	}

}

double BITStar() {

	// Time it
	std::clock_t start;
	double duration;
	start = std::clock();

	// Initialize random seed using current time.
	// Uncomment this line if you want feed the random number generator a seed based on time.

	// Setup intial state and add it to V; note that T and E are implicity represented
	// by root node. Can perform DFS to find T, E are children pointers of every node in T
	root_node = new Node();
	VectorXd extended_initial_state(setup_values.dimension);
	extended_initial_state << setup_values.initial_state, 0, 0; // Tack on 0 velocities
	root_node->state = extended_initial_state;
	root_node->g_hat = 0;
	root_node->h_hat = (setup_values.initial_state - setup_values.goal_state).norm()/max_speed;
	root_node->f_hat = root_node->h_hat;
	root_node->cost = 0;
	root_node->parent = root_node; // Convention for function tree_to_matrix_parents()
	root_node->inV = true;
	V.push_back(root_node);

	// Add goal state to X_sample
	goal_node = new Node;
	VectorXd extended_goal_state(setup_values.dimension);
	extended_goal_state << setup_values.goal_state, 0, 0; // Tack on 0 velocities
	goal_node->state = extended_goal_state;
	goal_node->g_hat = (setup_values.goal_state - setup_values.initial_state).norm()/max_speed;
	goal_node->h_hat = 0;
	goal_node->f_hat = root_node->g_hat;
	X_sample.insert(goal_node);

	// f_max from paper
	f_max = INFTY;

	// Begin iterations here
	int k = 1;

	while (k <= setup_values.max_iters) {

		std::cout << "Iteration: " << k << "\n";

		pruneSampleSet();
		std::cout << "Size of sample set: " << X_sample.size() << "\n";

		// Set values
		n = V.size();
		r = setup_values.gamma * pow(log(n)/n, 1.0/setup_values.dimension); // Copied from RRT* code given by Sertac Karaman
		if (r == 0) { // Radius hack to fight degeneracy of first iteration
			r = setup_values.gamma * pow(log(2)/2, 1.0/setup_values.dimension);
		}

		// Set f_prev to 0 for this iteration
		f_prev = 0;

		// Update Q_edge for all nodes in V
		std::vector<Node*>::iterator it;
		for( it = V.begin(); it != V.end(); ++it) {
			updateEdgeQueue(*it);
		}

		// Process edges in order of potential
		while (Q_edge.size() > 0) {

			// Grab best potential edge
			Edge* e = bestPotentialEdge();
			Q_edge.erase(e);
			Node* v = e->v; Node* x = e->x;

			// Collision checking happens implicitly here, in c_hat function
			if (g_T(v) + e->heuristic_cost + (x->h_hat) < g_T(goal_node)) {
				double cvx = c(e);
				if ((v->g_hat) + cvx + (x->h_hat) < g_T(goal_node)) {
					x->states = e->states;
					x->controls = e->controls;
					delete e; // After grabbing pointers to v, x and using heuristic cost, we have no need for the Edge e

					//if (exists_collision(v->state, x->state)) {
					//	continue;
					//}

					// Update costs, insert node, create edge (parent and child pointer)
					x->cost = g_T(v) + cvx;
					V.push_back(x);
					x->inV = true;
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
		std::cout << "cost of goal node: " << g_T(goal_node) << std::endl;
	}

	// More timing stuff
	duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
	std::cout << "Duration of algorithm for " << setup_values.max_iters << " iterations: " << duration << "\n";

	if (g_T(goal_node) < INFTY) {

		std::cout << "Size of V: " << V.size() << "\n";
		std::cout << "Number of samples pruned: " << num_samples_pruned << "\n";

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

	// Running of BIT*
	std::cout << "Running BIT*...\n";
	double path_length = BITStar();

	std::cout << "Number of calls to true cost calculator: " << num_true_cost_calls << "\n";
	std::cout << "Number of calls to signed distance checker: " << num_collision_check_calls << "\n";
	std::cout << "Best path cost: " << path_length << "\n";
	std::cout << "exiting\n";
}

