function [ x_new ] = exact_double_integrator_integration(x, u, T)
% Exactly calculates double integrator solution for constant input u

    n = size(x,1);
    m = size(u,1);
    
    x1 = x(1:n/2);
    x2 = x(n/2 + 1:n);
    
    x_new = zeros(n,1);
    x_new(1:n/2) = x1 + T*x2 + 1/2*u*T^2;
    x_new(n/2+1:n) = x2 + u*T;

end