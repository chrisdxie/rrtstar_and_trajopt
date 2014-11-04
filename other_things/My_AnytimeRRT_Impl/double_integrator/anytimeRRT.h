#ifndef BITSTAR_H_
#define BITSTAR_H_

#include <eigen3/Eigen/Eigen>
using namespace Eigen;
#include <set>

#define INFTY 1e10

class Node {
public:
	Vector4d state;
	double cost;
	Node* parent; // Parent node

	std::set<Node* > children; // A vector of pointers to children nodes
};

class SetupObject {
public:
	int max_time;
	int dimension;
	VectorXd initial_state;
	VectorXd goal_state;
	double goal_radius;
	bool randomize;
	// API: Each column is a rectangular obstacle represented by a 4d vector.
	// The vector is comprised of: [x_center; x_size; y_center; y_size]
	MatrixXd obstacles;

	double eps_f;
	double delta_d;
	double delta_c;
	double max_time_per_rrt;

	double x_min;
	double x_max;
	double v_min;
	double v_max;
	double u_min;
	double u_max;

	int stats_id;
};

#endif /* BITSTAR_H_ */
