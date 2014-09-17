function [ inputs ] = recover_inputs_double_integrator(states)

    n = size(states, 1);
    d = size(states, 2); %Hard coding for d = 4..
    
    inputs = zeros(n-1, d/2+1); % Time, u1, u2

    for i=1:n-1
       
        % Solve for delta first
        u0_times_delta = states(i+1, 3) - states(i,3);
        delta = (states(i+1, 1) - states(i, 1))/(states(i,3) + .5*u0_times_delta);
        u0 = u0_times_delta/delta;
        u1 = (states(i+1, 4) - states(i, 4))/delta;
        
        inputs(i,:) = [delta, u0, u1];
        
    end

end