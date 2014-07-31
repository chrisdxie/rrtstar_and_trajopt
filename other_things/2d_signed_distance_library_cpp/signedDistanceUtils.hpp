/*
 * signedDistanceUtils.hpp
 *
 *  Created on: Jul 30, 2014
 *      Author: ChrisXie
 */

#ifndef SIGNEDDISTANCEUTILS_HPP_
#define SIGNEDDISTANCEUTILS_HPP_
#include "signedDistanceUtils.h"

#include <eigen3/Eigen/Eigen>
using namespace Eigen;
using namespace signed_distance_2d;

using namespace std;

Vector2d signed_distance_2d:: perpendicularAxis(Vector2d line) {

	// Initialize to 0
	Vector2d axis(0,0);

	// Flip coordiantes, negate first one.
	axis(0) = line(1); axis(1) = line(0);
	axis(0) *= -1;

	if (axis.norm() != 0) {
		axis.normalize();
	} else {
		axis << 0, 0;
	}
	return axis;

}

MatrixXd signed_distance_2d::polygonProjectionOntoUnitVector(MatrixXd polygon, Vector2d line) {

	// Initialize by projecting first vertex
	double max = polygon.row(0) * line;
	double min = max;
	int max_index = 0;
	int min_index = 0;

	// Loop through all vertices of polygon
	for (int i = 1; i < polygon.rows(); i++) {
		double p = polygon.row(i) * line;
		if (p < min) {
			min = p;
			min_index = i;
		} else if (p > max) {
			max = p;
			max_index = i;
		}
	}

	Matrix<double, 2, 2> proj;
	proj << min, max,
			min_index, max_index;
	return proj;

}

bool signed_distance_2d::onEdgeOfPoly(MatrixXd poly, Vector2d point) {

	int i,j;
	for (i = 0, j = poly.rows() - 1;i < poly.rows(); j=i++) {
		Vector2d p1 = poly.row(i);
		Vector2d p2 = poly.row(j);
		Vector2d a = perpendicularAxis(p1 - p2); // Unit normal of the line
		double d = a.dot(p1); // Constant, a'*x + d = 0
		double eps = 1e-5;
		if (abs(a.dot(point) - d) <= eps && (point(0) >= min(p1(0),p2(0))) && (point(0) <= max(p1(0),p2(0))) &&
				(point(1) >= min(p1(1),p2(1))) && (point(1) <= max(p1(1),p2(1)))) {
			return true;
		}
	}
	return false;
}


Vector2d signed_distance_2d::closestPointOnLineSegment(Vector2d point, Vector2d p1, Vector2d p2) {

	// We are working with line representation L = {u*z + d | z is a real number, u is normalized}
	// u and d are in R^2 (2D vectors)

	// Get Unit vector in direction of edge
	Vector2d u = p1 - p2;
	u.normalize(); // Assuming u has nonzero length
	Vector2d d = p1; // d can be any point on the line. p1 or p2 will do

	double t = (point - d).dot(u);

	// Find if point is on line segment or if it needs to be projected to an end of segment
	// Do this in projected 1D space. Find t for both p1 and p2, and compare to t from the point.
	double p1t = (p1 - d).dot(u);
	double p2t = (p2 - d).dot(u);
	if (t <= min(p1t, p2t)) {
		t = min(p1t, p2t);
	} else if (t >= max(p1t, p2t)) {
		t = max(p1t, p2t);
	}
	Vector2d proj = u.array() * t;
	proj += d;

	// Sanity check
	Vector2d a = perpendicularAxis(u);
	double c = a.dot(p1); // Constant, a'*x + d = 0
	double eps = 1e-5;
	if (abs(a.dot(proj) - c) > eps) {
		std::cout << "Warning: Projection on line is not on line with tolerance " << eps << std::endl;
	}

	return proj;

}


#endif /* SIGNEDDISTANCEUTILS_HPP_ */
