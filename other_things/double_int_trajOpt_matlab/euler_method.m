function [ x_new ] = euler_method(f, x, u, delta)
% Uses Euler's method to solve differential equation

    x_new = x + delta * f(x, u);


end