#include <iostream>

#include <boost/random.hpp>

#include <eigen3/Eigen/Eigen>
using namespace Eigen;

#include <math.h>
#include <set>
#include <stack>
#include <map>
#include <queue>

#include "bitstar.h"
#include "plot_bitstar.h"

//#include "../../double_integrator_dynamics_library/double_integrator_dynamics.hpp"
//using namespace double_integrator_dynamics;

#include "../../2d_signed_distance_library_cpp/signedDistancePolygons.hpp"

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
Node* best_goal_node;

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
int num_vertices_pruned = 0;
int num_sample_batches = 0;

inline double uniform(double low, double high) {
	return (high - low) * ( (*u_sampler)() ) + low;
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
void updateCMatrix() {
	int d = setup_values.dimension;
	VectorXd goal_state = best_goal_node->state;

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
	C = U * W * V.transpose();

}


void setup(int max_iters, std::string& randomize, int batch_size) {

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
	VectorXd goal_region(2*d); // x_center, x_size, y_center, y_size
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

	VectorXd temp_goal_state(d);
	// Top right corner is furthest point from start state. This is hard coded for this example
	temp_goal_state << goal_region(0) + .5*goal_region(1), goal_region(2) + .5*goal_region(3);

	// Setup intial state and add it to V; note that T and E are implicity represented
	// by root node. Can perform DFS to find T, E are children pointers of every node in T
	root_node = new Node();
	root_node->state = setup_values.initial_state;
	root_node->cost = 0;
	root_node->parent = root_node; // Convention for function tree_to_matrix_parents()
	root_node->inV = true;
	root_node->old = false;
	V.insert(root_node);

	best_goal_node = new Node();
	best_goal_node->state = temp_goal_state;
	best_goal_node->inV = false;
	best_goal_node->old = false;
	G.insert(best_goal_node);
	X_sample.insert(best_goal_node);

	// Update for the first time
	updateCMatrix();

	// Gamma
	setup_values.gamma = 10; // Looked up thing in paper

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
	setup_values.x_min = -10;
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

inline bool inBounds(VectorXd& state) {
	if (inside_rectangular_obs(state, setup_values.x_min, setup_values.x_max, setup_values.x_min, setup_values.x_max)) {
		return true;
	}
	return false;
}

bool inGoal(Node* x) {
	double x_min = setup_values.goal_region(0) - .5*setup_values.goal_region(1);
	double x_max = setup_values.goal_region(0) + .5*setup_values.goal_region(1);
	double y_min = setup_values.goal_region(2) - .5*setup_values.goal_region(3);
	double y_max = setup_values.goal_region(2) + .5*setup_values.goal_region(3);
	return inside_rectangular_obs(x->state, x_min, x_max, y_min, y_max);
}

bool isLeaf(Node* x) {
	return x->children.size() == 0;
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
// HARD CODE FOR 2D
MatrixXd get_path(Node* x) {
	MatrixXd P(2, V.size());
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
inline double g_hat(Node* x) {
	return (x->state - setup_values.initial_state).norm();
	//return x->g_hat;
}
// Cost to go heuristic
inline double h_hat(Node* x) {
	return (x->state - best_goal_node->state).norm();
	//return x->h_hat;
}
// Cost of x_start to x_goal where path is constrained to go through x heuristic
inline double f_hat(Node* x) {
	return g_hat(x) + h_hat(x);
	//return x->f_hat;
}
// Exact cost of connecting two nodes
inline double c(Node* x, Node* y) {
	num_true_cost_calls++;
	return (x->state - y->state).norm();
}
// Cost of connecting two nodes heuristic
inline double c_hat(Node* x, Node* y) { // This is specific to point robot example in paper
	if (exists_collision(x->state, y->state)) {
		return INFTY;
	}
	return (x->state - y->state).norm();
}
/* DONE WITH HEURISTICS */


Edge* bestPotentialEdge() {
	std::set<Edge*>::iterator it;
	Edge* best_edge = NULL;
	double best_cost = INFTY;
	double curr_cost = 0;
	for(it = Q_edge.begin(); it != Q_edge.end(); ++it) {
		Edge* e = *it;
		curr_cost = g_T(e->v) + e->heuristic_cost + (h_hat(e->x));
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

// Sample uniformly from sample space. Specific to point robot with square environment
void sample_uniform_batch() {

	int num_samples = 0;

	while (num_samples < setup_values.batch_size) {

		// Sample the new state
		VectorXd x_sample(setup_values.dimension);
		for (int i = 0; i < setup_values.dimension; i++) {
			x_sample(i) = uniform(setup_values.x_min, setup_values.x_max);
		}

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

	double best_cost = g_T(best_goal_node);
	double c_min = g_hat(best_goal_node);

	if (best_cost == INFTY) { // Just sample uniformly from state space
		sample_uniform_batch();
		return;
	}

	int num_samples = 0;
	VectorXd x_center = (setup_values.initial_state + best_goal_node->state)/2.0;

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

	MatrixXd CL = C*L;
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
		if (f_hat(n) > g_T(best_goal_node)) {
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

void pruneVertexSet() {
	// Prune leaf vertices that are useless. Pruning other
	std::set<Node*>::iterator n_it;
	for(n_it = V.begin(); n_it != V.end(); ) {
		Node* n = *n_it;
		if (f_hat(n) > g_T(best_goal_node) && isLeaf(n)) {
			num_vertices_pruned++;
			Node* p = n->parent;
			p->children.erase(n);
			V.erase(n_it++);
			if (inSet(n, G)) {
				G.erase(n);
			}
			if (inSet(n, Q_V)) {
				Q_V.erase(n);
			}
			delete n;
		} else {
			++n_it;
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
		if (g_hat(v) + heuristic_edge_cost + h_hat(x) < g_T(best_goal_node)) {
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
			if (g_hat(v) + heuristic_edge_cost + h_hat(w) < g_T(best_goal_node)) { // Same condition as above
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

		} else {
			expand = false;
		}
	}

}

// Iterate through all goal nodes and find best one
double costOfBestGoalNode() {

	bool changed = false;
	double best_cost = g_T(best_goal_node);
	std::set<Node*>::iterator n_it;

	for(n_it = G.begin(); n_it != G.end(); n_it++) {
		Node* candidate = *n_it;
		if (g_T(candidate) < best_cost) {
			best_goal_node = candidate;
			best_cost = g_T(candidate);
			changed = true;
		}
	}
	// Now update C matrix if need be
	if (changed) {
		updateCMatrix();
	}

	return best_cost;
}

double BITStar() {

	// f_max from paper
	f_max = INFTY;

	// Begin iterations here
	int k = 1;

	while (k <= setup_values.max_iters) { // Max_iters will be much more now. Maybe put some other termination condition here

		if (k % 100 == 0) {
			std::cout << "Iteration: " << k << "\n";
			std::cout << "Size of sample set: " << X_sample.size() << "\n";
			std::cout << "Size of vertex set: " << V.size() << "\n";
		}

		// New Batch of samples!!
		if (Q_edge.size() == 0) {

			std::cout << "New batch! Batch number: " << num_sample_batches+1 << "\n";
			
			pruneSampleSet();
			pruneVertexSet();
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

			updateQueue();

		}

		// Update Q_edge for vertices in the expansion queue Q_V
		updateQueue();

			// Grab best potential edge
		Edge* e = bestPotentialEdge();
		Q_edge.erase(e);
		Node* v = e->v; Node* x = e->x;

		// Collision checking happens implicitly here, in c_hat function
		if (g_T(v) + e->heuristic_cost + h_hat(x) < g_T(best_goal_node)) {

			double cvx = c(v,x); // Calculate true cost of edge
			
			if (g_hat(v) + cvx + h_hat(x) < g_T(best_goal_node)) {

				delete e; // After grabbing pointers to v, x and using heuristic cost, we have no need for the Edge e

				if (g_T(v) + cvx < g_T(x)) {

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
					if (inGoal(x) && !x->inV) {
						G.insert(x);
						std::cout << "New goal state found:\n" << x->state << "\n";
					}
					x->inV = true;

					// Update best cost stuff
					double old_cost = f_max;
					f_max = costOfBestGoalNode(); // Best goal node is update here too
					if (f_max < old_cost) {
						std::cout << "New goal cost: " << f_max << "\n";
						//std::cout << "State of goal:\n" << best_goal_node->state << "\n";
					}

					pruneEdgeQueue(x);

				}

			} else {
				delete e; // Don't leak memory
			}
		} else {
			delete e;
			clearEdgeQueue();
		}
		k++;
	}

	if (g_T(best_goal_node) < INFTY) {

		std::cout << "Size of V: " << V.size() << "\n";
		std::cout << "Number of samples pruned: " << num_samples_pruned << "\n";
		std::cout << "Number of vertices pruned: " << num_vertices_pruned << "\n";
		std::cout << "Number of batches: " << num_sample_batches << "\n";

		MatrixXd goal_path = get_path(best_goal_node);

		// Plot in Python
		std::cout << "Solution found!\n";
		std::cout << "Initializing display...\n";
		py::object plotter = init_display(); // Initialize python interpreter and pyplot plot

		// Convert Eigen matrices and vectors to Numpy ND arrays
		np::ndarray states_np = eigen_to_ndarray(tree_to_matrix_states(root_node));
		np::ndarray parents_np = eigen_to_ndarray(tree_to_matrix_parents(root_node));
		np::ndarray goal_path_np = eigen_to_ndarray(goal_path);
		np::ndarray goal_region_np = eigen_to_ndarray(setup_values.goal_region);
		np::ndarray obstacles_np = eigen_to_ndarray(setup_values.obstacles);

		std::cout << "Plotting...\n";
		plot(plotter, states_np, parents_np, goal_path_np, goal_region_np, obstacles_np, setup_values.max_iters, g_T(best_goal_node));

	}
	return g_T(best_goal_node);

}

/*
 * Just a note: This function MUST be called from directory that
 * plot_sst.cpp lives in.
 *
 * USAGE: build/bin/plot_sst <MAX_ITERS> <RANDOMIZE> <BATCH_SIZE>
 */

int main(int argc, char* argv[]) {

	// Assumes a command line argument of MAX_ITERS RANDOMIZE {true, false}
	int max_iters = 5000;
	std::string randomize = "false";
	int batch_size = 100;

	if (argc >= 2) {
		max_iters = atoi(argv[1]);
	}
	if (argc >= 3) {
		randomize = argv[2];
	} 
	if (argc >= 4) {
		batch_size = atoi(argv[3]);
	}

	// Setup function
	setup(max_iters, randomize, batch_size);

	std::cout << "Running BIT*...\n";
	double path_length = BITStar();

	std::cout << "Number of calls to true cost calculator: " << num_true_cost_calls << "\n";
	std::cout << "Number of calls to signed distance checker: " << num_collision_check_calls << "\n";
	std::cout << "Best path cost: " << path_length << "\n";
	std::cout << "Optimal path cost: " << 12.9347 << "\n";
	std::cout << "exiting\n";
}

