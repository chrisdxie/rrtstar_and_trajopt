function points = intersectVectorEllipse(vector, ellipse)
%INTERSECTVECTORELLIPSE Return intersections between a vector and a ellipse
%
%   points = intersectVectorEllipse(VECTOR, ELLIPSE);
%   returns the two intersection points of vector VECTOR and ellipse ELLIPSE.
%
% ------
% Author: Alex Lee

theta = deg2rad(ellipse(5));
vector = rotateVector(vector, -theta);
slope = vector(2)/vector(1);
a = ellipse(3);
b = ellipse(4);
x = 1/sqrt( 1/a^2 + (b^2)/(slope^2*a^4) );
y = -1/sqrt( (slope^2*a^2)/(b^4) + 1/b^2 );

if slope>0
    points = [x y; -x -y];
else
    points = [x -y; -x y];
end
points = transformPoint(points, createRotation(theta));
points = transformPoint(points, createTranslation([ellipse(1), ellipse(2)]));
