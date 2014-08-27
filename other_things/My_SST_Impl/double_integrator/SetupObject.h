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
	VectorXd goal_region;
	double delta_v, delta_s;
	bool randomize;
	// API: Each column is a rectangular obstacle represented by a 4d vector.
	// The vector is comprised of: [x_center; x_size; y_center; y_size]
	MatrixXd obstacles;

	double x_min;
	double x_max;
	double v_min;
	double v_max;
	double u_min;
	double u_max;

	double T_prop;
};


#endif /* SETUPOBJECT_H_ */
