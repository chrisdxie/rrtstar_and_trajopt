function [ x ] = find_path_with_trajopt(x_start, x_goal, obstacles, u_min, u_max, v_min, v_max)

    setup

    addpath ../../../Courses/CS287/Homeworks/HW3/starter_PS3/q2_starter/  %TODO: your path might be different depending on where you solved q2

    n = size(x_start,1); % Dimension of state
    m = n/2; % Dimension of input
    T = 3*n; % As mentioned in the paper draft (number of steps = 3n)
    nT = n*T;
    for i = 1:n
        traj_init(i,:) = linspace(x_start(i), x_goal(i), T);
    end
    input_init = zeros(m,T-1); % Set inputs to 0

    % Time is the final input variable, last variable in state. It is initialized to 1
    delta_init = 1;
    x0 = [traj_init(:); input_init(:); delta_init];

    dsafe = 0.05; % Safety margin

    % Point robot
    robot_length = 0.1;
    robot_width = 0.1;

    % Function that maps state vector to polygon (which is used for collision
    % checking)
    make_robot_poly = @(x) orientedBoxToPolygon([x(1), x(2), robot_length, robot_width, 0]);

    % Minimize the last variable, which is time
    q = [zeros(1,size(x0,1)-1) 1];

    % Q matrix is 0.
    Q = zeros(size(x0,1),size(x0,1));

    % No nonconvex cost
    f = @(x) 0;
    
    % The constraint function g does all the work of computing signed distances
    % and their gradients. This is a nonconvex inequality constraint
    g = @(x) g_collisions(x, dsafe, [n,T], make_robot_poly, obstacles);
    
    % The nonconvex system dynamics
    traj_dynamics_cfg = struct();
    traj_dynamics_cfg.nX = n;
    traj_dynamics_cfg.nU = m;
    traj_dynamics_cfg.T = T;
    traj_dynamics_cfg.f = @double_integrator_dynamics;
    
    h = @(x) double_integrator_trajectory_dynamics(x, traj_dynamics_cfg);

    % Linear inequalities: u_min, u_max, delta, velocities
    A_ineq = zeros(n*T + 2*m*(T-1) + 1, size(x0,1));
    b_ineq = zeros(n*T + 2*m*(T-1) + 1, 1);
    % Velocities first:
    row = 1;
    for i=1:n*T
        if mod(i,n) > n/2 || mod(i,n) == 0
            A_ineq(row, i) = 1; % Max velocity
            b_ineq(row) = v_max;
            A_ineq(row+1, i) = -1; % Min velocity
            b_ineq(row+1) = -1*v_min;
            row = row + 2;
        end
    end
    % u_min and u_max next
    for i=n*T+1:n*T+m*(T-1)
        A_ineq(row,i) = 1; % Max input
        b_ineq(row) = u_max;
        A_ineq(row+1,i) = -1; % Min input
        b_ineq(row+1) = -1*u_min;
        row = row+2;
    end
    % Delta last
    A_ineq(end,end) = -1;
    

    % Linear equality constraints: start and end goal

    A_eq = [eye(n) zeros(n, size(x0,1)-n); zeros(n, n*T-n) eye(n) zeros(n, size(x0,1)-n*T)];

    b_eq = [traj_init(:,1); traj_init(:,T)];

    % Configuration stuff
    cfg = struct();
    cfg.callback = @(x,~) plot_traj(make_robot_poly, obstacles, reshape(x(1:n*T),size(traj_init))); % This does the plotting
    cfg.initial_trust_box_size=.1;
    cfg.g_use_numerical = false;
    cfg.min_approx_improve = 1e-2;

    x = penalty_sqp(x0, Q, q, f, A_ineq, b_ineq, A_eq, b_eq, g, h, cfg);
    
    %plot_traj(make_robot_poly, obstacles, reshape(x(1:n*T),size(traj_init)));
    
end