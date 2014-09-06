function [ opt_cost, best_t ] = exactConnectDoubleIntegrator(x0, x1)
% Returns the exact cost of a connecting two states optimally under the
% differential constraints of a double integrator. The cost function is:
% c(traj) = \int_{0}^{tau} 1 + u(t)'*u(t) dt

% This was calculated exactly by me. I calculated the cost function
% according to Eqn. 11 as describe here: http://arl.cs.utah.edu/pubs/ICRA2013-1.pdf
   
    % Dimension
    d = size(x0, 1)/2;

    % Keep track of variables
    opt_cost = Inf;
    best_t = 0;
    
    % How much to propagate forward
    eps = 1e-3;

    t = 0;
    while (t < opt_cost)
       
        % Calculate optimal cost for fixed arrival time t
        c = t + (x1 - drift(t,x0))' * Ginv(t,d) * (x1 - drift(t,x0));
        
        if c < opt_cost
            best_t = t;
            opt_cost = c;
        end
        t = t + eps;
    end
    
end

function [ ret ] = Ginv(t,d)
    ret = [12/t^3*eye(d) -6/t^2*eye(d);-6/t^2*eye(d) 4/t*eye(d)];
end

function [ ret ] = drift(t, x0)
    d = size(x0, 1)/2;
    ret = [eye(d) t*eye(d); zeros(d) eye(d)] * x0;
end