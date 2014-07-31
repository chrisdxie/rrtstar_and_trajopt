/*
 * signedDistanceUtils.h
 *
 *  Created on: Jul 30, 2014
 *      Author: ChrisXie
 */

#ifndef SIGNEDDISTANCEUTILS_H_
#define SIGNEDDISTANCEUTILS_H_


#include <eigen3/Eigen/Eigen>
using namespace Eigen;

namespace signed_distance_2d {

	/**
	 *  Returns a 2d orthogonal unit vector to the line.
	 *
	 *  The parameter "line" is a 2d vector represented by 2 coordinates.
	 *
	 *  This is easy in 2D, just flip the coordinates and negate one of them.
	 */
	Vector2d perpendicularAxis(Vector2d line);

	/**
	 *  Returns 2 numbers which represent projection of polygon onto a unit vector,
	 *  and the 2 indices that give those numbers.
	 *
	 *  The format of the first row vector is: [min, max]
	 *  The second row vector is the indices of the polygon that give min and max.
	 *
	 *  Example: The box represented by [1 2;1 1;2 1;2 2] projected onto the x axis
	 *  (which is represented by line [0; 1]) is [1 2;0 2].
	 */
	MatrixXd polygonProjectionOntoUnitVector(MatrixXd polygon, Vector2d line);

	/**
	 *  Return true if point is on the edge of the polygon within a small tolerance.
	 */
	bool onEdgeOfPoly(MatrixXd polygon, Vector2d point);

	/**
	 *  Find the closest point on the line segment described by points p1 and p2.
	 */
	Vector2d closestPointOnLineSegment(Vector2d point, Vector2d p1, Vector2d p2);

}





#endif /* SIGNEDDISTANCEUTILS_H_ */
