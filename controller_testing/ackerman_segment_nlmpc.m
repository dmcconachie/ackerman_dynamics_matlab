function [x_history, u_history, y_ref] = ...
    ackerman_segment_nlmpc(nlobj, nlopt, x0, y_target, l, M1, M2, dscale)

Ts = nlobj.Ts;
prediction_horizon = nlobj.PredictionHorizon;
control_horizon = nlobj.ControlHorizon;

dist = se3_dist(x0(1:3), y_target);
duration = dist / 5.0;
Tsteps = ceil(duration / Ts);

t = 0:Ts:Ts*Tsteps;
x_history = zeros(5, Tsteps + 1);
x_history(:, 1) = x0;
u_history = zeros(2, Tsteps);
u_prev = [0; 0];

% Create reference targets along the line segment to avoid overshooting
% http://www.cs.cornell.edu/courses/cs4620/2013fa/lectures/16spline-curves.pdf
y_ref = se3_spline(x0(1:3), y_target, Tsteps, dscale)';
y_ref = y_ref(2:end, :);

% dist = se3_dist(x0(1:3), y_target);
% delta = se3_delta(x0(1:3), y_target);
% dx = linspace(0, delta(1), Tsteps + 1);
% dy = linspace(0, delta(2), Tsteps + 1);
% dtheta = linspace(0, delta(3), Tsteps + 1);
% y_ref = ones(Tsteps + 1, 1) * x0(1:3)' + [dx', dy', dtheta'];
% y_ref = y_ref(2:end, :);

hbar = waitbar(0,'Simulation Progress');
for k = 1:Tsteps
    
    xk = x_history(:, k);
    % Update the nlobj to not predict past the total number of steps
    % i.e, allow 'residual' velocity
    steps_left = Tsteps - k + 1;
    ph = min(prediction_horizon, steps_left);
    ch = min(control_horizon, steps_left);    
    nlobj.PredictionHorizon = ph;
    nlobj.ControlHorizon = ch;
    if size(nlopt.MV0, 1) > nlobj.ControlHorizon
        nlopt.MV0 = nlopt.MV0(1:nlobj.ControlHorizon, :);
    end
        
    [uk, nlopt, info] = nlmpcmove(nlobj, xk, u_prev, y_ref(k:k+ph-1, :), [], nlopt);
    u_history(:, k) = uk;
    u_prev = uk;
    
    ODEFUN = @(t, xk) ackerman_dynamics(xk, uk, l, M1, M2);
    [TOUT, YOUT] = ode45(ODEFUN,[0 Ts], xk');
    x_history(:, k+1) = YOUT(end, :);
    waitbar(k/Tsteps, hbar);
end
close(hbar)

end