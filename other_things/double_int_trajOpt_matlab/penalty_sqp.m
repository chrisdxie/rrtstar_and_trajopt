function [x,success] = penalty_sqp(x0, Q, q, f, A_ineq, b_ineq, A_eq, b_eq, g, h, user_cfg)
%PENALTY_SQP    solves constrained optimization problem
%
%   minimize (1/2) x'*Q*x + x'*q + f(x)
%   subject to
%       A_ineq*x <= b_ineq
%       A_eq*x == b_eq
%       g(x) <= 0
%       h(x) == 0
%
%   x0: initialization
%
%   f, g, h are function handles.
%   These functions will be numerically differentiated unless you specify otherwise.
%   To use an analytic gradient and hessian for f, set
%   cfg.f_use_numerical = false
%   Then f should have the form
%   [y, grad, hess] = f(x)
%   Similarly, g and h, set cfg.g_use_numerical and cfg.h_use_numerical and provide
%   [y, grad] = g(x)
%   


assert(size(x0,2) == 1);

cfg = {};
cfg.improve_ratio_threshold = .25;
cfg.min_trust_box_size = 1e-4;
cfg.min_approx_improve = 1e-4;
cfg.max_iter = 50;
cfg.trust_shrink_ratio = .1;
cfg.trust_expand_ratio = 1.5;
cfg.cnt_tolerance = 1e-4;
cfg.max_merit_coeff_increases = 5;
cfg.merit_coeff_increase_ratio = 10;
cfg.initial_trust_box_size = 1;
cfg.initial_penalty_coeff = 1.;
cfg.max_penalty_iter = 4;
cfg.f_use_numerical = true;
cfg.g_use_numerical = false;
cfg.h_use_numerical = false;
cfg.full_hessian = true;
cfg.callback = [];

cfg = load_user_cfg(cfg, user_cfg);

disp('Optimizer parameters:');
disp(cfg)

% First we find a point that satisfies linear constraints.
% We'll enforce these constraints exactly in the optimization that follows.
% (Whereas the nonlinear constraints will be initially violated and treated
% with penalties.)
[x,success] = find_closest_feasible_point(x0, A_ineq, b_ineq, A_eq, b_eq); 
if (~success)
    return;
end

trust_box_size = cfg.initial_trust_box_size; % The trust region will be a box around the current iterate x.
penalty_coeff = cfg.initial_penalty_coeff; % Coefficient of l1 penalties 

if ~isempty(cfg.callback), cfg.callback(x,{}); end;

% TODO: Write the outer loop of the sqp algorithm, which repeatedly minimizes
% the merit function f(x) + penalty_coeff*( pospart(g(x)) + abs(h(x)) )
% Call minimize_merit_function defined below

% After this call, check to see if the
% constraints are satisfied.
% - If some constraint is violated, increase penalty_coeff by a factor of cfg.merit_coeff_increase_ratio
% You should also reset the trust region size to be larger than cfg.min_trust_box_size,
% which is used in the termination condition for the inner loop.
% - If all constraints are satisfied (which in code means if they are satisfied up to tolerance cfg.cnt_tolerance), we're done.
%

merit_coeff_increases = 0;

while true    
        
    % YOUR CODE HERE
    [x, trust_box_size, success] = minimize_merit_function(x, Q, q, ...
    f, A_ineq, b_ineq, A_eq, b_eq, g, h, cfg, penalty_coeff, trust_box_size);

    if ~success
        display('Merit function not minimized successfully');
        break;
    end

    % Check constraints
    constraints_satisfied = true;
    nonconvex_ineq_constraints = g(x);
    nonconvex_ineq_constraints(nonconvex_ineq_constraints < 0) = 0;
    nonconvex_eq_constraints = h(x);
    if any(A_ineq*x0 - b_ineq > cfg.cnt_tolerance) || ...
       any(A_eq*x0 - b_eq > cfg.cnt_tolerance) || ...
       sum(nonconvex_ineq_constraints) +  ...
            sum(abs(nonconvex_eq_constraints)) > cfg.cnt_tolerance
        constraints_satisfied = false;
    end
    
    if constraints_satisfied
        break; % We're done!
    else
        penalty_coeff = penalty_coeff * cfg.merit_coeff_increase_ratio; % Increase mu (penalty coefficient)
        merit_coeff_increases = merit_coeff_increases + 1;
        if merit_coeff_increases > cfg.max_merit_coeff_increases
            display('Number of penalty coefficient increases exceeded maximum allowed.');
            break;
        end
        trust_box_size = cfg.initial_trust_box_size; % Reset trust box size to initial value
    end
	
end

end


function full_cfg = load_user_cfg(full_cfg, user_cfg)
userkeys = fieldnames(user_cfg);
for iuserkey = 1:numel(userkeys)
    userkey = userkeys{iuserkey};
    if isfield(full_cfg, userkey)
        full_cfg.(userkey) = user_cfg.(userkey);
    else
        error(['Unknown parameter: ' userkey])
    end
end
end

function [x,success] = find_closest_feasible_point(x0, A_ineq, b_ineq, A_eq, b_eq)
% Find a point that satisfies linear constraints, if x0 doesn't
% satisfy them

success = true;
if any(A_ineq*x0 > b_ineq) || any(A_eq*x0 ~= b_eq)
    fprintf('initialization doesn''t satisfy linear constraints. finding the closest feasible point\n');

    cvx_begin
    cvx_quiet('true')
    variables('x(length(x0))')
    minimize('sum((x-x0).^2)')
    subject to
        A_ineq*x <= b_ineq;
        A_eq*x == b_eq;
    cvx_end
    
    if strcmp(cvx_status,'Failed')
        success = false;
        fprintf('Couldn''t find a point satisfying linear constraints\n')
        return;
    end
else
    x = x0;
end
end

function [x, trust_box_size, success] = minimize_merit_function(x, Q, q, ...
    f, A_ineq, b_ineq, A_eq, b_eq, g, h, cfg, penalty_coeff, trust_box_size)
    
    dim_x = length(x);


    success = true;
    sqp_iter = 1;

    fquadlin = @(x) q*x + .5*x'*(Q*x);
    hinge = @(x) sum(max(x,0));
    abssum = @(x) sum(abs(x));
    
    while  true
        % In this loop, we repeatedly construct a quadratic approximation
        % to the nonlinear part of the objective f and a linear approximation to the nonlinear
        % constraints f and g.
        fprintf('  sqp iter: %i\n', sqp_iter);
                    
        if cfg.f_use_numerical
            fval = f(x);
            [fgrad, fhess] = numerical_grad_hess(f,x,cfg.full_hessian);
            % diagonal adjustment
            mineig = min(eigs(fhess));
            if mineig < 0
                fprintf('    negative hessian detected. adjusting by %.3g\n',-mineig);
                fhess = fhess + eye(dim_x) * ( - mineig);
            end
        else
            [fval, fgrad, fhess] = f(x);
        end
        if cfg.g_use_numerical
            gval = g(x);
            gjac = numerical_jac(g,x);
        else           
            [gval, gjac] = g(x);
        end
        if cfg.h_use_numerical
            hval = h(x);
            hjac = numerical_jac(h,x);
        else
            [hval, hjac] = h(x);
        end
        
        merit = fval + fquadlin(x) + penalty_coeff * ( hinge(gval) + abssum(hval) );
        
        while true 
            % This is the trust region loop
            % Using the approximations computed above, this loop shrinks
            % the trust region until the progress on the approximate merit
            % function is a sufficiently large fraction of the progress on
            % the exact merit function.
            
            fprintf('    trust region size: %.3g\n', trust_box_size);

            
            % YOUR CODE INSIDE CVX_BEGIN and CVX_END BELOW
			% Write CVX code to minimize the convex approximation to
            % the merit function, using the jacobians computed above.
            % It should create variable xp, which is the candidate for
            % updating x -> xp.
            % You should enforce the linear constraints exactly.
			% Make sure to include the constant term f(x) in the merit function
			% objective as the resulting cvx_optval is used further below.
                        
            %{
            cvx_begin quiet
                variables xp(dim_x, 1);

				%YOUR CODE HERE
                minimize( .5*xp'*Q*xp + q*xp + ... % convex part
                          fval + fgrad*(xp-x) + .5*(xp-x)'*fhess*(xp-x) + ... % Quadratic Approximation to nonconvex part
                          penalty_coeff * (hinge(gval + gjac*(xp-x)) + abssum(hval + hjac*(xp-x)))); % Linear approximation to nonconvex constraints
				subject to
                    norm(xp - x, Inf) <= trust_box_size; % Infinity norm gives a trust "box"
                    A_ineq*xp <= b_ineq;
                    A_eq*xp == b_eq;
                      
            cvx_end
            
            
            if strcmp(cvx_status,'Failed')
                fprintf('Failed to solve QP subproblem.\n');
                success = false;
                return;
            end
            %}
            
            arguments; % This script will need to coincide exactly with some certain things. Look at it to be sure it's right
            %[output, exitflag, info] = run_QP_solver_CD(args); % w/ collision detection
            [output, exitflag, info] = run_QP_solver_noCD(args); % w/out collision detection
            if (exitflag ~= 1)
                info
                disp('Problem in QP Solver'); % Fix this to return bad success
                success = false;
                return;
            end
            xp_ = [];
            % States
            for i=1:args.N-1
                eval(['xp_ = [xp_; output.z' num2str(i) '(1:args.nX)];']);
            end
            eval(['xp_ = [xp_; output.z' num2str(args.N) '];']);
            % Inputs
            for i=1:args.N-1
                eval(['xp_ = [xp_; output.z' num2str(i) '(args.nX+1:args.nX+args.nU)];']);
            end
            % Time
            eval(['xp_ = [xp_; output.z' num2str(1) '(end)];']);
            
            xp = xp_;
            cvx_optval = info.pobj;
            %}
            
            model_merit = cvx_optval;
            new_merit = f(xp) + fquadlin(xp) + penalty_coeff * ( hinge(g(xp)) + abssum(h(xp)) ) ;
            approx_merit_improve = merit - model_merit;
            exact_merit_improve = merit - new_merit;
            merit_improve_ratio = exact_merit_improve / approx_merit_improve;
                        
            info = struct('trust_box_size',trust_box_size);
            
                      
            fprintf('      approx improve: %.3g. exact improve: %.3g. ratio: %.3g\n', approx_merit_improve, exact_merit_improve, merit_improve_ratio);
            if approx_merit_improve < -1e-5
                fprintf('Approximate merit function got worse (%.3e).\n',approx_merit_improve);
                fprintf('Either convexification is wrong to zeroth order, or you''re in numerical trouble\n');
                success = false;
                return;
            elseif approx_merit_improve < cfg.min_approx_improve
                fprintf('Converged: y tolerance\n');
                x = xp;
                if ~isempty(cfg.callback), cfg.callback(x,info); end
                return;
            elseif (exact_merit_improve < 0) || (merit_improve_ratio < cfg.improve_ratio_threshold)
                trust_box_size = trust_box_size * cfg.trust_shrink_ratio;
            else
                trust_box_size = trust_box_size * cfg.trust_expand_ratio;
                x = xp;
                if ~isempty(cfg.callback), cfg.callback(x,info); end
                break; % from trust region loop
            end
            
            if trust_box_size < cfg.min_trust_box_size
                fprintf('Converged: x tolerance\n');
                return;
            end
        end % tr
        sqp_iter = sqp_iter + 1;
    end % sqp

end



