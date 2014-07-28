function [val, jac] = double_integrator_trajectory_dynamics(xu_trajectory, cfg)
% Calculate value of x_t+1 - x_t for all t, and jacobian of h = x_t+1 - f(x_t, u_t)

T = cfg.T; n = cfg.nX; m = cfg.nU;

x_traj = reshape(xu_trajectory(1:T*n),n, T);
u_traj = reshape(xu_trajectory(T*n+1:end-1), m, T-1);
delta = xu_trajectory(end);

f = cfg.f; % continuous dynamics model dx(t)/dt = f(x_t, u_t, dt);

% Remember: H(x) = 0. So we want h_trajectory_dynamics = x_{t+1} - f(x_t, u_t)
% x_{t+1} is predicted next state, f(x_t, u_t) is simulated next state
predicted_x_next = [x_traj(:,2:end)];
simulated_x_next = zeros(n, T-1);
for t=1:T-1
    % Approximate the integration with Runge-Kutta-4
    simulated_x_next(:,t) = rk4(f, x_traj(:,t), u_traj(:,t), delta);
end
val = predicted_x_next - simulated_x_next;
val = val(:);

% For jacobian, note that only x_t and u_t contribute to x_{t+1}.
finite_diff_eps = 1e-5;
jac_A = [];
jac_B = [];
jac_C = [];
for t=1:T-1
    [A B C] = sim_double_integrator_jacobian(f, x_traj(:,t), u_traj(:,t), delta, finite_diff_eps);
    % Note that this is the jacobian of f. We need to negate this since we
    % want the jacobian of h, where h = x_{t+1} - f(x_t, u_t). We must also
    % take into account the jacobian of x_{t+1}.
    jac_A = blkdiag(jac_A, -A);
    jac_B = blkdiag(jac_B, -B);
    jac_C = [jac_C; -C];
end

% This says that the last state has no effect on the predicted state.
% This is because I defined u_1 to u_{T-1}, where I should have left u_T in
% there. So annoying to take care of these extra cases.
jac_A = [jac_A zeros(n*(T-1),n)];

% This portion takes care of the jacobian of x_{t+1}(x_t)
jac_A = jac_A + [zeros(n*(T-1), n) eye(n*(T-1))];

% Finally create the jacobian
jac = [jac_A jac_B jac_C];

end