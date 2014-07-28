function [ x_new ] = double_integrator_dynamics(x, u)
% Continuous time dynamics of double integrator.
% Assumes x is a column vector of size n. Top half of x is configuration,
% bottom half is derivative. u is a column vector of size m.

    n = size(x,1);
    m = size(u,1);
    A = [zeros(n/2,n/2) eye(n/2); zeros(n/2, n)];
    B = [zeros(n/2, m); eye(m)];
    
    x_new = A*x + B*u;

end