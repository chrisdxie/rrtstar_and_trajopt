function [val,jac,jacnext] = swept_out_volume_collisions(x, x_next, dsafe, obstacles)
assert(size(x,2)==1);

% This is specific to the double integrator example
d = 1/2*size(x,1);
nX = 2*d;
nO = length(obstacles);

val = zeros(nO,1);
jac = zeros(nO, nX);
jacnext = zeros(nO, nX);

icontact = 1;

swept_out_polygon = [x(1:d)'; x_next(1:d)']; % Grab x, y positions manually

for iobs=1:length(obstacles)
    [d,pts] = signedDistancePolygons(...
        swept_out_polygon, ...
        obstacles{iobs});
    ptOnSWVolume = pts(1,:);
    ptOnObs = pts(2,:);
    normalObsToRobot = -sign(d)*normr(ptOnSWVolume - ptOnObs);
    
    [pt_jac pt_jac_next] = calcJacobians(ptOnSWVolume, x, x_next);
    gradd = normalObsToRobot * pt_jac;
    jac(icontact,1:nX) = gradd;
    
    gradd = normalObsToRobot * pt_jac_next;
    jacnext(icontact,1:nX) = gradd;
    
    val(icontact) = dsafe - d;
    icontact = icontact+1;
end


end

% This is specific to double integrator for now.
function [jac, jacnext] = calcJacobians(pt, x, x_next)
    pt = pt'; % pt is passed in as a row vector
    d = 1/2*size(x,1);
    x = x(1:d);
    x_next = x_next(1:d);
    
    jac = zeros(d, 2*d);
    jacnext = zeros(d, 2*d);
    
    alpha = norm(pt - x)/norm(x_next - x);
    jac(1:d,1:d) = (1-alpha)*eye(d);
    jacnext(1:d,1:d) = alpha*eye(d);
    
end