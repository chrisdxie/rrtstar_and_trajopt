function record_trajectory(filename)
    x_init = [-15 -15 0 10]';
    c = car(x_init);
    xest = c.x;
    Vest = eye(car.x_size);

    x_traj = x_init;
    u_traj = zeros(car.u_size,0);
    Theta_traj = compose_belief(xest, Vest);

    while(~isempty(c.last_u))
        c.x = car.simulate(c.x, c.last_u);
        z = car.observe(c.x);

        [xest, Vest] = ekf_update(car, z, c.last_u, xest, Vest);
        elli = beliefToEllipse(xest(1:2), Vest(1:2,1:2)*Vest(1:2,1:2)');
    
        c.visualizeState();
        c.h_state(end+1) = drawEllipse(elli, 'b');
        
        x_traj = [x_traj, c.x];
        u_traj = [u_traj, c.last_u];
        Theta_traj = [Theta_traj, compose_belief(xest, Vest)];

        c.last_u = zeros(size(c.last_u));
        pause(car.DT);
    end

    save(filename, 'x_traj', 'u_traj', 'Theta_traj');
end
