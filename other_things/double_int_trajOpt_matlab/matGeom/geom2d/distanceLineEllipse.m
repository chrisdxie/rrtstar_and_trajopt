function dist = distanceLineEllipse(line, ellipse)
%DISTANCELINEELLIPSE Minimum distance between a line and an ellipse
%
%   DIST = distanceLineEllipse(LINE, ELLIPSE);
%   Return the euclidean distance between line LINE and ellipse ELLIPSE.
%   LINE has the form: [x0 y0 dx dy], and ELLIPSE is [XC YC A B THETA].
%
% ------
% Author: Alex Lee

vector = line(3:4);
points = intersectVectorEllipse(vector, ellipse);
dist = min(distancePointLine(points, line));
