function [dist pts] = signedDistancePolygons(poly1, poly2)
%SIGNEDDISTANCEPOLYGONS Compute the signed distance between 2 polygons
%   DIST = signedDistancePolygons(POLY1, POLY2)
%   Returns the signed distance between 2 polygons
%
%   [DIST, POINTS] = signedDistancePolygons(POLY1, POLY2)
%   Also returns the 2 points involved with the distance. The
%   first point belongs to POLY1 and the second point belongs to POLY2.
%
%   Example
%   signedDistancePolygons
%
%   See also
%   distancePolygons, penetrationDepth
%
%
% ------
% Author: Alex Lee

% check if the polygons are intersecting each other
if any(isPointInPolygon(poly1, poly2)) || any(isPointInPolygon(poly2, poly1))
    [dist pts] = penetrationDepth(poly1, poly2);
else
    [dist pts] = distancePolygons(poly1, poly2);
end
