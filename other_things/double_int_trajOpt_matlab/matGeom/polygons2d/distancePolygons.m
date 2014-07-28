function varargout = distancePolygons(poly1, poly2)
%DISTANCEPOLYGONS Compute the shortest distance between 2 polygons
%   DIST = distancePolygons(POLY1, POLY2)
%   Returns the shortest distance between 2 polygons
%
%   [DIST, POINTS] = distancePolygons(POLY1, POLY2)
%   Also returns the 2 points involved with the smallest distance. The
%   first point belongs to POLY1 and the second point belongs to POLY2.
%
%   Example
%   distancePolygons
%
%   See also
%
%
% ------
% Author: David Legland
% e-mail: david.legland@grignon.inra.fr
% Created: 2009-06-17,    using Matlab 7.7.0.471 (R2008b)
% Copyright 2009 INRA - Cepia Software Platform.

% compute distance of each vertex of a polygon to the other polygon
[dists1, points2] = distancePointPolygon(poly1, poly2);
[dists2, points1] = distancePointPolygon(poly2, poly1);
[dist1, poly1_point_i] = min(dists1);
[dist2, poly2_point_i] = min(dists2);

% keep the minimum of the two distances
[dist, poly_i] = min([dist1 dist2]);

if 1 <= nargout <= 2
    varargout{1} = dist;
end
if nargout == 2
    if poly_i==1
        points(1,:) = poly1(poly1_point_i,:);
        points(2,:) = points2(poly1_point_i,:);
    else
        points(2,:) = poly2(poly2_point_i,:);
        points(1,:) = points1(poly2_point_i,:);
    end
    varargout{2} = points;
end
