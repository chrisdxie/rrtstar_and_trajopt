/*
 * distancePolygons.hpp
 *
 *  Created on: Jul 30, 2014
 *      Author: ChrisXie
 */

#ifndef DISTANCEPOLYGONS_HPP_
#define DISTANCEPOLYGONS_HPP_
#include <float.h>
#include "distancePolygons.h"
#include "signedDistanceUtils.hpp"

#include <eigen3/Eigen/Eigen>
using namespace Eigen;

using namespace std;
using namespace signed_distance_2d;

MatrixXd signed_distance_2d::distancePolygons(MatrixXd polygon1, MatrixXd polygon2) {

	// Keep track of best distance and points
	double closest_dist = DBL_MAX;
	Vector2d point_from_polygon1;
	Vector2d point_from_polygon2;

	for (int i = 0; i < 2; i++) {

		// We are always evaluating vertices of poly1 against edges poly2
		MatrixXd poly1;
		MatrixXd poly2;
		if (i == 0) {
			poly1 = polygon1;
			poly2 = polygon2;
		} else { // i == 1
			poly1 = polygon2;
			poly2 = polygon1;
		}

		int num_of_vertices = poly1.rows();

		for (int j = 0;j < num_of_vertices;j++) {

			// Get vertex
			Vector2d vertex = poly1.row(j);

			// Iterate through all edges of other polygon
			int num_of_edges = poly2.rows();
			int l,k;
			for (l = 0, k = num_of_edges - 1; l < num_of_edges; k = l++) {

				// Project vertex onto edge of poly2
				Vector2d p1 = poly2.row(l);
				Vector2d p2 = poly2.row(k);
				Vector2d proj = closestPointOnLineSegment(vertex, p1, p2);

				// Update best values
				double length = (vertex - proj).norm();
				if (length < closest_dist) {
					closest_dist = length;
					if (i == 0) {
						point_from_polygon1 = vertex;
						point_from_polygon2 = proj;
					} else {
						point_from_polygon1 = proj;
						point_from_polygon2 = vertex;
					}
				}
			}
		}
	}

	MatrixXd return_vals(3,2);
	return_vals.row(0) << closest_dist, 0;
	return_vals.row(1) = point_from_polygon1;
	return_vals.row(2) = point_from_polygon2;

	return return_vals;

}



#endif /* DISTANCEPOLYGONS_HPP_ */
