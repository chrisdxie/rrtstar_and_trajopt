function [output, exitflag, info] = run_QP_solver_noCD(args)

    % Args should be a struct with these fields:
    %   N, the discretization number
    %   z0 = z_bar, linearize around this point
    %   nX, dimension of state (position and velocity, so 2*d)
    %   nU, dimension of control
    %   nO, number of obstacles and dimension of collision function
    %   mu, penalty coeff
    %   x_min/max, bounding box of states
    %   v_min/max, velocity bounds
    %   u_min/max, control bounds
    %   x_start/goal, obvious enough
    %   trust_box_size, obvious enough
    %   obstacles, struct of obstacles (2D polygons)
    %   d_safe, safety boundary. Should be a double.
    %
    % Things to be created:
    %   f_t, f_N
    %   z_{low,t}, z_{high,t}
    %   c1, c_N
    %   A_t, A_N
    %   b_t, b_N
    
    N = args.N;
    z0 = args.z0;
    nX = args.nX;
    nU = args.nU;
    nO = args.nO;
    mu = args.mu;
    x_min = args.x_min;
    x_max = args.x_max;
    v_min = args.v_min;
    v_max = args.v_max;
    u_min = args.u_min;
    u_max = args.u_max;
    x_start = args.x_start;
    x_goal = args.x_goal;
    trust_box_size = args.trust_box_size;
    obstacles = args.obstacles;
    d_safe = args.d_safe;
    
    params = struct();
    
    %---------------- Fill out f's --------------------%
    
    % First create the string vector of each f dependent on mu
    f_string = '[';
    for j=1:2*nX+2*nU+1
        f_string = strcat(f_string, ' 0;');
    end
    for j=1:nX
        f_string = strcat(f_string, ' mu;');
    end
    f_string = strcat(f_string, ']');
    
    % Now add it to params struct for f_2, ..., f_{N-2}
    for i=2:N-2
        i_str = sprintf('%d', i);
        eval(['params.f' i_str ' = ' f_string ';']);
    end
    
    % f_1
    f1_string = '[';
    for j=1:2*nX+2*nU
        f1_string = strcat(f1_string, ' 0;');
    end
    f1_string = strcat(f1_string, ' 1;');
    for j=1:nX
        f1_string = strcat(f1_string, ' mu;');
    end
    f1_string = strcat(f1_string, ']');
    eval(['params.f' num2str(1) ' = ' f1_string ';']);
    
    % f_{N-1}
    f_almost_string = '[';
    for j=1:2*nX+nU+1
        f_almost_string = strcat(f_almost_string, ' 0;');
    end
    for j=1:nX
        f_almost_string = strcat(f_almost_string, ' mu;');
    end
    f_almost_string = strcat(f_almost_string, ']');
    eval(['params.f' num2str(N-1) ' = ' f_almost_string ';']);
    
    % Lastly, create the last fN string and add it to params struct
    fN_string = '[';
    for j=1:nX
        fN_string = strcat(fN_string, ' 0;');
    end
    fN_string = strcat(fN_string, ']');
    eval(['params.f' sprintf('%d', N) ' = ' fN_string ';']);
    
    %---------------- Fill out z's --------------------%
    
    % Create the string vector for lower and upper bound
    lb_string = '['; ub_string = '[';
%     for j=1:nX/2 % d = nX/2
%         lb_string = strcat(lb_string, ' x_min;');
%         ub_string = strcat(ub_string, ' x_max;');
%     end
    for j=1:nX/2
        lb_string = strcat(lb_string, ' v_min;');
        ub_string = strcat(ub_string, ' v_max;');
    end
    for j=1:nU
        lb_string = strcat(lb_string, ' u_min;');
        ub_string = strcat(ub_string, ' u_max;');
    end
    lb_string = strcat(lb_string, ']');
    ub_string = strcat(ub_string, ']');
                        
    % Now add it to params struct
    for i=2:N-1
        i_str = sprintf('%d', i);
        eval(['params.lb' i_str ' = ' lb_string ';']);
        eval(['params.ub' i_str ' = ' ub_string ';']);
    end
    
    % Enforce start state and goal state using lower bounds and eps
    % Also enforce that delta >= 0
    lb1_string = '['; ub1_string = '[';
    for j=1:nX
        lb1_string = strcat(lb1_string, [' ' num2str(x_start(j) - eps) ';']);
        ub1_string = strcat(ub1_string, [' ' num2str(x_start(j) + eps) ';']);        
    end
    for j=1:nU
        lb1_string = strcat(lb1_string, ' u_min;');
        ub1_string = strcat(ub1_string, ' u_max;');
    end
    lb1_string = strcat(lb1_string, ' 0]');
    ub1_string = strcat(ub1_string, ']');
    eval(['params.lb' sprintf('%d', 1) ' = ' lb1_string ';']);
    eval(['params.ub' sprintf('%d', 1) ' = ' ub1_string ';']);    
    
    % Lastly, create lbN and ubN strings
    lbN_string = '['; ubN_string = '[';
    for j=1:nX
        lbN_string = strcat(lbN_string, [' ' num2str(x_goal(j) - eps) ';']);
        ubN_string = strcat(ubN_string, [' ' num2str(x_goal(j) + eps) ';']);
    end
    lbN_string = strcat(lbN_string, ']');
    ubN_string = strcat(ubN_string, ']');
    eval(['params.lb' sprintf('%d', N) ' = ' lbN_string ';']);
    eval(['params.ub' sprintf('%d', N) ' = ' ubN_string ';']);
    
    %---------------- Fill out A's and b's --------------------%
    
    % Must linearize around z0
    addpath('../../other_things/double_int_trajOpt_matlab/');
    
    % Get function handle on double integrator
    f = @double_integrator_dynamics;
    finite_diff_eps = 1e-5; % For numerical differentiation
    
    % z0 is in format [x1 ... xN u1 ... u_{N-1} delta]
    x0 = z0(1:N*nX);
    u0 = z0(N*nX+1:end-1);
    delta0 = z0(end);
    
    for i=1:N-1
        % Setup loop iteration
        i_str = sprintf('%d',i);
        A = zeros(4*nX+2*nU+2, 2*nX+2*nU+1+nX);
        b = zeros(4*nX+2*nU+2, 1);
        curr_row = 1;

        % Grab corresponding state, control, and time
        x = x0((i-1)*nX+1 : (i-1)*nX+nX);
        x_next = x0(i*nX+1 : i*nX+nX);
        u = u0((i-1)*nU+1 : (i-1)*nU+nU);
        delta = delta0;
                        
        % Grab dynamics jacobian
        x_prime = [x;x_next;u;delta];
        [DH_X DH_U DH_delta] = sim_double_integrator_jacobian(f, x, u, delta, finite_diff_eps);
        DH_X = -1*DH_X; DH_U = -1*DH_U; DH_delta = -1*DH_delta;
        DH = [DH_X eye(nX) DH_U DH_delta];
        cfg.T=2;cfg.nX=nX;cfg.nU=nU;cfg.f=f;
        hval = double_integrator_trajectory_dynamics([x;x_next;u;delta], cfg);
        A(curr_row:curr_row+nX-1, :) = [DH_X DH_U eye(nX) zeros(nX, nU) DH_delta -1*eye(nX)];
        b(curr_row:curr_row+nX-1, :) = DH*x_prime - hval;
        curr_row = curr_row + nX;
        A(curr_row:curr_row+nX-1, :) = [-1*DH_X -1*DH_U -1*eye(nX) zeros(nX, nU) -1*DH_delta -1*eye(nX)];
        b(curr_row:curr_row+nX-1, :) = hval - DH*x_prime;
        curr_row = curr_row + nX;
                
        % Trust box stuff
        A(curr_row:curr_row+nX+nU+1-1, :) = [eye(nX+nU) zeros(nX+nU, nX+nU+1+nX); zeros(1, 2*nX+2*nU+1+nX)];
        A(curr_row+nX+nU+1-1, 2*nX+2*nU+1) = 1;
        b(curr_row:curr_row+nX-1) = x + trust_box_size;
        b(curr_row+nX:curr_row+nX+nU-1) = u + trust_box_size;
        b(curr_row+nX+nU) = delta + trust_box_size;
        curr_row = curr_row + nX+nU+1;
        
        A(curr_row:curr_row+nX+nU+1-1, :) = [-1*eye(nX+nU) zeros(nX+nU, nX+nU+1+nX); zeros(1, 2*nX+2*nU+1+nX)];
        A(curr_row+nX+nU+1-1, 2*nX+2*nU+1) = -1;        
        b(curr_row:curr_row+nX-1) = trust_box_size - x;
        b(curr_row+nX:curr_row+nX+nU-1) = trust_box_size - u;
        b(curr_row+nX+nU) = trust_box_size - delta;
        curr_row = curr_row + nX+nU+1;
        
        if (i == N-1)
            A(:, 2*nX+nU+1:2*nX+2*nU) = [];
        end
        
        % Now put it in params struct
        eval(['params.A' i_str ' = ' mat2str(A) ';']);
        eval(['params.b' i_str ' = ' mat2str(b) ';']);
        
    end
    
    % A_N, b_N
    i = N;
    i_str = sprintf('%d',i);
    A = zeros(2*nX, nX);
    b = zeros(2*nX, 1);
    curr_row = 1;
    
    % Grab corresponding state
    x = x0((i-1)*nX+1 : (i-1)*nX+nX);
    
    A(curr_row:curr_row+nX-1, :) = eye(nX);
    b(curr_row:curr_row+nX-1) = x + trust_box_size;
    curr_row = curr_row + nX;
    A(curr_row:curr_row+nX-1, :) = -1*eye(nX);
    b(curr_row:curr_row+nX-1) = trust_box_size - x;
    curr_row = curr_row + nX;
    
    eval(['params.A' i_str ' = ' mat2str(A) ';']);
    eval(['params.b' i_str ' = ' mat2str(b) ';']);
    
    %--------------------- DONE ---------------------%
    
    % Solve
    s = cputime;
    [output exitflag info] = double_integrator_QP_solver_noCD(params);
    e = cputime - s;
    disp(['Time elapsed for QP solver (in seconds): ' num2str(e)]);
    
end