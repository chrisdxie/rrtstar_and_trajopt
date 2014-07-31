/*
 * penetrationDepth.h
 *
 *  Created on: Jul 29, 2014
 *      Author: ChrisXie
 */

#ifndef SIGNEDDISTANCEPOLYGONS_H_
#define SIGNEDDISTANCEPOLYGONS_H_

#include <eigen3/Eigen/Eigen>
using namespace Eigen;

namespace signed_distance_2d {

	/**
	 *  Compute the signed distance between 2 convex polygons.
	 *
	 *  Returns the signed distance between 2 convex polygons. The polygons
	 *  are assumed to be intersecting each other at first. If a gap is found,
	 *  then the polygons are not intersecting, and distancePolygons
	 *  is called, which finds the minimum distance between two polygons.
	 *
	 *  This algorithm is the separation of axis algorithm. This link contains
	 *  an intuitive explanation of it:
	 *  	http://www.codezealot.org/archives/55
	 *
	 *	If the return value is called DIST_POINTS, then
	 *	DIST_POINTS(1,1) is the value of the signed distance function.
	 *	DIST_POINTS(2,:) is the first point belonging to POLYGON1.
	 *	DIST_POINTS(3,:) is the second point belonging to POLYGON2.
	 */
	MatrixXd signedDistancePolygons(MatrixXd polygon1, MatrixXd polygon2);

}



#endif /* SIGNEDDISTANCEPOLYGONS_H_ */
