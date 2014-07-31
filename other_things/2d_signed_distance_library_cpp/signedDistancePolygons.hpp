/*
 * penetrationDepth.hpp
 *
 *  Created on: Jul 29, 2014
 *      Author: ChrisXie
 */

#ifndef SIGNEDDISTANCEPOLYGONS_HPP_
#define SIGNEDDISTANCEPOLYGONS_HPP_
#include <float.h>
#include "signedDistancePolygons.h"
#include "distancePolygons.hpp"
#include "signedDistanceUtils.hpp"

#include <eigen3/Eigen/Eigen>
using namespace Eigen;
using namespace signed_distance_2d;

using namespace std;

MatrixXd signed_distance_2d::signedDistancePolygons(MatrixXd polygon1, MatrixXd polygon2) {

	double overlap = DBL_MAX;
	Vector2d point_from_polygon1;
	Vector2d point_from_polygon2;

	for (int i = 0; i < 2; i++) {

		MatrixXd poly1;
		MatrixXd poly2;
		if (i == 0) {
			poly1 = polygon1;
			poly2 = polygon2;
		} else { // i == 1
			poly1 = polygon2;
			poly2 = polygon1;
		}

		int num_of_faces = poly1.rows();

		/**
		 *  In this for loop, get the axes corresponding to the normal of
		 *  each face of poly1. Project the points and find the overlaps
		 */
		int j, k;
		for (k = 0, j = num_of_faces - 1; k < num_of_faces; j=k++) {

			// Get the orthogonal unit vector to the face.
			Vector2d edge_vec = poly1.row(k) - poly1.row(j);
			Vector2d axis = perpendicularAxis(edge_vec);

			// Project both poly1 and poly2 onto the orthogonal unit vector
			Matrix<double, 2, 2> projpoly1 = polygonProjectionOntoUnitVector(poly1, axis);
			Matrix<double, 2, 2> projpoly2 = polygonProjectionOntoUnitVector(poly2, axis);

			// If either of these conditions are true, then the objects are NOT in collision!
			if (projpoly1(0,1) < projpoly2(0,0) || projpoly2(0,1) < projpoly1(0,0)) {
				return distancePolygons(polygon1, polygon2);
			}

			// Get overlap of the projections
			double o;
			Vector2d point_poly1;
			Vector2d point_poly2;

			if (projpoly1(0,1) - projpoly2(0,0) < projpoly2(0,1) - projpoly1(0,0)) { // poly1 is on the "left" in the projection
				o = projpoly1(0,1) - projpoly2(0,0);
				point_poly1 = poly1.row((int) projpoly1(1,1));
				point_poly2 = poly2.row((int) projpoly2(1,0));
			} else { // poly1 is on the "right" in the projection
				o = projpoly2(0,1) - projpoly1(0,0);
				point_poly1 = poly1.row((int) projpoly1(1,0));
				point_poly2 = poly2.row((int) projpoly2(1,1));
			}
			o *= -1; // Negate the overlap

			if (abs(o) < abs(overlap)) {

				// Update best points
				Vector2d contact_point = ((point_poly1 - point_poly2).dot(axis)) * axis.array();
				contact_point += point_poly2;

				if (!onEdgeOfPoly(poly1, contact_point)) {
					continue; // There must be another vertex that will give same distance
				}

				overlap = o;

				if (i == 0) { // poly1 = polygon1
					point_from_polygon1 = contact_point;
					point_from_polygon2 = point_poly2;
				} else {
					point_from_polygon1 = point_poly2;
					point_from_polygon2 = contact_point;
				}
			}
		}
	}

	MatrixXd return_vals(3,2);
	return_vals.row(0) << overlap, 0;
	return_vals.row(1) = point_from_polygon1;
	return_vals.row(2) = point_from_polygon2;

	return return_vals;

}



#endif /* SIGNEDDISTANCEPOLYGONS_HPP_ */
