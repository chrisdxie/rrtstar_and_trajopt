function [s] = sampleUnitBall(d)

    % Taken from here: 
    % http://math.stackexchange.com/questions/87230/picking-random-points-in-the-volume-of-sphere-with-uniform-probability/87238#87238
    
    % Sample a point uniformly from the unit ball of dimension d
    R = 1; % Unit ball ball has radius 1
    
    len = R*rand()^(1/d);

    dir = randn(d,1);
    dir = dir/norm(dir);
    
    s = len*dir;

end