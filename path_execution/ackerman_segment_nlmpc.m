function [x_history, u_history, y_ref] = ackerman_segment_nlmpc(car, x0, y_target)

Ts = car.nlobj.Ts;
dist = se3_dist(x0(1:3), y_target);
duration = dist / car.se3_speed;
Tsteps = ceil(duration / Ts);

x_history = zeros(5, Tsteps + 1);
x_history(:, 1) = x0;
u_history = zeros(2, Tsteps);
u_prev = [0; 0];

% Create reference targets along the line segment to avoid overshooting
% http://www.cs.cornell.edu/courses/cs4620/2013fa/lectures/16spline-curves.pdf
y_ref = se3_spline(x0(1:3), y_target, Tsteps, car.dscale)';

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
    mpc_horizon = min(car.mpc_horizon, steps_left);
    car.nlobj.PredictionHorizon = mpc_horizon;
    car.nlobj.ControlHorizon = mpc_horizon;
    if size(car.nlopt.MV0, 1) > mpc_horizon
        car.nlopt.MV0 = car.nlopt.MV0(1:mpc_horizon, :);
    end
        
    [uk, car.nlopt, info] = nlmpcmove(car.nlobj, xk, u_prev, y_ref(k+1:k+mpc_horizon, :), [], car.nlopt);
    u_history(:, k) = uk;
    u_prev = uk;
    
    ODEFUN = @(t, xk) ackerman_dynamics(xk, uk, car.length, car.M1, car.M2);
    [TOUT, YOUT] = ode45(ODEFUN,[0 Ts], xk');
    x_history(:, k+1) = YOUT(end, :);
    waitbar(k/Tsteps, hbar);
end
close(hbar)

y_ref = y_ref';

end