/*
 * distancePolygons.h
 *
 *  Created on: Jul 30, 2014
 *      Author: ChrisXie
 */

#ifndef DISTANCEPOLYGONS_H_
#define DISTANCEPOLYGONS_H_


#include <eigen3/Eigen/Eigen>
using namespace Eigen;

namespace signed_distance_2d {

	/**
	 *  This function finds the closest distance between two polygons.
	 *
	 *  The algorithm is:
	 *   - For each vertex in each polygon,
	 *   	- Find the corresponding point on the opposing polygon such that
	 *   	  it minimizes the length between the point and this vertex.
	 *   	  - to do this, project the vertex on each line segment of the polygon
	 *   	    and find the best one. I proved this in my head, I hope it's right..
	 *   	    this is also how it seems to be defined in the matGeom library.
	 *   	- Keep track of the shortest distance found.
	 *
	 *	If the return value is called DIST_POINTS, then
	 *	DIST_POINTS(1,1) is the value of the signed distance function.
	 *	DIST_POINTS(2,:) is the first point belonging to POLYGON1.
	 *	DIST_POINTS(3,:) is the second point belonging to POLYGON2.
	 */
	MatrixXd distancePolygons(MatrixXd polygon1, MatrixXd polygon2);

}



#endif /* DISTANCEPOLYGONS_H_ */
