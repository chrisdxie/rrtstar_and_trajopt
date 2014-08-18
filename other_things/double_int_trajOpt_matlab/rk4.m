function [ x_new ] = rk4(f, x, u, delta)
% Uses 4th order Runge Kutta integration 
% Implemented as given in appendix of draft of paper

    k1 = f(x, u);
    k2 = f(x + k1/2, u);
    k3 = f(x + k2/2, u);
    k4 = f(x + k3, u);

    x_new = x + delta * (k1 + 2*k2 + 2*k3 + k4)/6;


end