function [ states, controls ] = constructStatesControls(tau, x0, x1)
% Given a optimal time tau, start and end state, construct the states and
% controls

    % Dimension
    d = size(x0, 1)/2;
    A = [zeros(d) eye(d);zeros(d,2*d)];
    B = [zeros(d); eye(d)];
    
    % Discretization
    T = 12; % Just to show the halfway point. switch back to 12 later
    
    states = zeros(2*d, T); states(:,end) = x1; % Fill in last state with goal
    controls = zeros(d, T-1);
    
    d_tau = Ginv(tau, d) * (x1 - drift(tau, x0));
    
    for i=1:T-1 % No control at the last state
        t = (i-1)/(T-1) * tau;
        temp = expm([A B*B';zeros(2*d) -A'] * (t - tau)) * [x1;d_tau];
        states(:,i) = temp(1:2*d);
        controls(:,i) = B'*temp(2*d+1:end);
    end

end

function [ ret ] = Ginv(t,d)
    ret = [12/t^3*eye(d) -6/t^2*eye(d);-6/t^2*eye(d) 4/t*eye(d)];
end

function [ ret ] = drift(t, x0)
    d = size(x0, 1)/2;
    ret = [eye(d) t*eye(d); zeros(d) eye(d)] * x0;
end