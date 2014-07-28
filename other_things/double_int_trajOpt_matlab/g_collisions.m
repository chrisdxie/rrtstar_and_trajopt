function [val,jac] = g_collisions(x, dsafe, traj_shape, make_robot_poly, obstacles)
assert(size(x,2)==1);

% This is specific to the double integrator example
K = traj_shape(1);
T = traj_shape(2);
traj = reshape(x(1:K*T), traj_shape);

val = zeros(length(obstacles)*size(traj,2),1);
jac = zeros(size(val,1), size(x,1));

icontact = 1;


for t=1:T
    xt = traj(:,t);
    for iobs=1:length(obstacles)
        [d,pts] = signedDistancePolygons(...
                make_robot_poly(xt), ...
                obstacles{iobs});
        ptOnRobot = pts(1,:);
        ptOnObs = pts(2,:);
        normalObsToRobot = -sign(d)*normr(ptOnRobot - ptOnObs);

        gradd = normalObsToRobot * calcJacobian(ptOnRobot, xt);
        
        val(icontact) = dsafe - d;
        jac(icontact,K*(t-1)+1:K*t) = gradd;
        icontact = icontact+1; 
   end 
end


end

% This is specific to double integrator for now.
function jac = calcJacobian(pt, x0)
    n = 1/2*size(x0,1);
    jac = [eye(n) zeros(n,n)];
end