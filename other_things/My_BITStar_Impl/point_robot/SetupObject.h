/*
 * SetupObject.h
 *
 *  Created on: Jul 21, 2014
 *      Author: ChrisXie
 */

#ifndef SETUPOBJECT_H_
#define SETUPOBJECT_H_

#include <eigen3/Eigen/Eigen>
using namespace Eigen;

class SetupObject {
public:
	int max_iters;
	int dimension;
	VectorXd initial_state;
	VectorXd goal_state;
	double gamma; // Set to be at least side length of state space
	bool randomize;
	// API: Each column is a rectangular obstacle represented by a 4d vector.
	// The vector is comprised of: [x_center; x_size; y_center; y_size]
	MatrixXd obstacles;

	double x_min;
	double x_max;

};


#endif /* SETUPOBJECT_H_ */
