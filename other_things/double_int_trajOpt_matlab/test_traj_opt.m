close all
clear
setup

addpath ../../../Courses/CS287/Homeworks/HW3/starter_PS3/q2_starter/  %TODO: your path might be different depending on where you solved q2

T = 40;
K = 3;
traj_init = [linspace(-2,2,T);...
             zeros(1,T);...
             zeros(1,T)];
         
load obstacles4

% The trajectory is a 3xT matrix
% the rows are the degrees of freedom of the robot: x,y,angle.
% the columns are timesteps 1 to T.

% We will flatten this matrix for optimization in the column-major
% way: x = traj_init(:) = (x_1, y_1, theta_1, x_2, y_2, theta_2, ...)
% Keep that in mind when defining the cost and constraints below.

x0 = traj_init(:);

dsafe = 0.05; % Safety margin
KT = K*T;

car_length = .4;
car_width = .2;

% Function that maps state vector to polygon (which is used for collision
% checking)
make_robot_poly = @(x) orientedBoxToPolygon([x(1), x(2), car_length, car_width, rad2deg(x(3))]);


q = zeros(1,KT);

% TODO: create Q matrix for the sum-of-squared-displacements cost
% \sum_t || \theta_{t+1} - \theta_t ||^2
% where \theta is the state vector (x, y, angle)

%YOUR_CODE_HERE;
Q = 2*eye(KT) ... % diagonal
    - [zeros(K, KT); eye(K*(T-1)) zeros(K*(T-1), K)] ... % Lower diagonal
    - [zeros(K*(T-1),K) eye(K*(T-1)); zeros(K, KT)]; % Upper diagonal
for i=1:K
    Q(i,i) = 1;
    Q(end+1-i,end+1-i) = 1;
end


f = @(x) 0;
% The constraint function g does all the work of computing signed distances
% and their gradients
g = @(x) g_collisions(x, dsafe, [K,T], make_robot_poly, obstacles);
h = @(x) 0;

% TODO: create linear inequality constraints to enforce that the
% displacement is smaller than .2 (componentwise) over each timestep
A = [-1*eye(K*(T-1)) zeros(K*(T-1), K)] + [zeros(K*(T-1), K) eye(K*(T-1))];
A_ineq = [A; -1*A];

b_ineq = .2 * ones(2*K*(T-1),1);

% TODO: create linear equality constraints to fix the beginning and end of the
% trajectory to the values in traj_init(:,1) and traj_init(:,end)

A_eq = [eye(K) zeros(K, K*(T-1)); zeros(K, K*(T-1)) eye(K)];

b_eq = [traj_init(:,1); traj_init(:,end)];

cfg = struct();
cfg.callback = @(x,~) plot_traj(make_robot_poly, obstacles, reshape(x,size(traj_init))); % This does the plotting
cfg.initial_trust_box_size=.1;
cfg.g_use_numerical = false;
cfg.min_approx_improve = 1e-2;

x = penalty_sqp(x0, Q, q, f, A_ineq, b_ineq, A_eq, b_eq, g, h, cfg);