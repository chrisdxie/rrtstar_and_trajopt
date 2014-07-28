function [minDist closestPt] = distancePointPolyline(point, poly, varargin)
%DISTANCEPOINTPOLYLINE  Compute shortest distance between a point and a polyline
%   minDist = distancePointPolyline(POINT, POLYLINE)
%   Return the shortest distance between point POINT and polyline POLYLINE.
%
%   [minDist closestPt] = distancePointPolyline(POINT, POLYLINE)
%   Also returns the closest point on the polyline.
%
%   Example:
%       pt1 = [30 20];
%       pt2 = [30 5];
%       poly = [10 10;50 10;50 50;10 50];
%       distancePointPolyline([pt1;pt2], poly)
%       ans =
%           10
%            5
%
%   See also
%   polygons2d, points2d
%   distancePointEdge, projPointOnPolyline
%
% ------
% Author: David Legland
% e-mail: david.legland@grignon.inra.fr
% Created: 2009-04-30,    using Matlab 7.7.0.471 (R2008b)
% Copyright 2009 INRA - Cepia Software Platform.

%   HISTORY
%   2009-06-23 compute all distances in one call

% number of points
Np = size(point, 1);

% allocate memory for result
minDist = inf * ones(Np, 1);
closestPt = inf * ones(Np, 2);

% process each point
for p = 1:Np
    % construct the set of edges
    edges = [poly(1:end-1, :) poly(2:end, :)];
    
    % compute distance between current each point and all edges
    [dist pos] = distancePointEdge(point(p, :), edges);
    
    % direction vector of the edges (row vectors)
    vx = (edges(:, 3) - edges(:,1))';
    vy = (edges(:, 4) - edges(:,2))';
    % point corresponding to the closest distance
    pt = edges(:,1:2) + [bsxfun(@times, pos, vx)' bsxfun(@times, pos, vy)'];
    
    % update distance if necessary
    [minDist(p), i] = min(dist);
    closestPt(p,:) = pt(i,:);
end
