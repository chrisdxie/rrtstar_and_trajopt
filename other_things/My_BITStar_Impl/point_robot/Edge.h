/*
 * Edge.h
 *
 *  Created on: Sep 5, 2014
 *      Author: ChrisXie
 */

#ifndef EDGE_H_
#define EDGE_H_

#include <eigen3/Eigen/Eigen>
using namespace Eigen;

#ifndef NODE_H_
#define NODE_H_
#include "Node.h"
#endif

class Edge {
public:

	Node* v;
	Node* x;
	// Edge is represented by (v, x)
	// Invariant: v is in the vertex set V, x in in the set X_sample

	Edge(Node* v_, Node* x_) { v = v_; x = x_; }

};



#endif /* EDGE_H_ */
