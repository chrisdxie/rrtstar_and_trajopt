/*
 * Node.h
 *
 *  Created on: Jul 21, 2014
 *      Author: ChrisXie
 */

#ifndef NODE_H_
#define NODE_H_

#include <eigen3/Eigen/Eigen>
using namespace Eigen;

#include <set>

class Node {
public:
	VectorXd state;
	double cost;
	Node* parent; // Parent node
	std::set<Node* > children; // A vector of pointers to children nodes
};

/*
 *  I believe witnesses don't need children, costs, or parents,
 *  just a state and representative..
 */
class Witness: public Node {
public:
	Node* rep;

	void copy_fields(Node* x) {
		this->state = x->state;
		this->cost = x->cost;
		this->parent = x->parent;
	}

};



#endif /* NODE_H_ */
