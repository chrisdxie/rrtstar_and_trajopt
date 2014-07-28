function [ A, B, C ] = sim_double_integrator_jacobian(f, x, u, delta, eps)

    % This function numerically calculates an approximation to the jacobian
    % vector of the function f by using the finite difference formula. f is
    % an equation that describes continuous time dynamics, so we use RK4
    % integration  here. The time interval of propogation is delta.
    
    % Let f_tilde be the discrete time function that corresponds to f.
    % A is the jacobian of f_tilde w.r.t x, B is the jacobian of f_tilde w.r.t u.
    % This function requires one state x, and one control u (at one
    % timestep). It is not expecting a long vector of all states and
    % controls over a horizon T.
    
    nX = length(x);
    nU = length(u);
    
    % df/dx
    A = zeros(nX,nX);
    I = eye(nX);
    for i=1:nX
        A(:,i) = rk4(f, x + .5*eps*I(:,i), u, delta) - rk4(f, x - .5*eps*I(:,i), u, delta);
    end
    A = A/eps;
    
    % df/du
    B = zeros(nX, nU);
    I = eye(nU);
    for i=1:nU
        B(:,i) = rk4(f, x, u + .5*eps*I(:,i), delta) - rk4(f, x, u - .5*eps*I(:,i), delta);
    end
    B = B/eps;
    
    % df/d\delta
    C = zeros(nX, 1);
    C = rk4(f, x, u, delta + .5*eps) - rk4(f, x, u, delta - .5*eps);
    C = C/eps;
    
end