function [val, jac] = swv_collisions(x, dsafe, traj_shape, obstacles)
assert(size(x,2)==1);

% This is specific to the double integrator example of a point robot
K = traj_shape(1); dim = K/2;
T = traj_shape(2);
traj = reshape(x(1:K*T), traj_shape); % Shave off controls and delta

val = zeros(length(obstacles)*(T-1),1);
jac = zeros(size(val,1), size(x,1));

icontact = 1;

for t=1:T-1
    x = traj(:,t);
    x_next = traj(:,t+1);
    for iobs=1:length(obstacles)
        
        swept_out_polygon = [x(1:dim)'; x_next(1:dim)']; % Grab x, y positions manually
        
        [d,pts] = signedDistancePolygons(...
            swept_out_polygon, ...
            obstacles{iobs});
        ptOnSWVolume = pts(1,:);
        ptOnObs = pts(2,:);
        normalObsToRobot = -sign(d)*normr(ptOnSWVolume - ptOnObs);
        
        [pt_jac pt_jac_next] = calcJacobians(ptOnSWVolume, x, x_next);
        grad = normalObsToRobot * pt_jac;
        
        grad_next = normalObsToRobot * pt_jac_next;
        
        jac(icontact, K*(t-1)+1:K*t) = grad;
        jac(icontact, K*t+1:K*(t+1)) = grad_next;
        
        val(icontact) = dsafe - d;
        icontact = icontact+1;
    end
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