function [ x_new ] = rk4(f, x, u, delta)
% Uses 4th order Runge Kutta integration 
% Implemented as given in appendix of draft of paper

    k1 = delta * f(x, u);
    k2 = delta * f(x + k1/2, u);
    k3 = delta * f(x + k2/2, u);
    k4 = delta * f(x + k3, u);

    x_new = x + (k1 + 2*k2 + 2*k3 + k4)/6;


end