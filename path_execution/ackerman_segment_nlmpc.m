function [x_history, u_history, y_ref] = ackerman_segment_nlmpc(car, x0, y_target, waitbar_enabled)

if nargin < 4
    waitbar_enabled = false;
end

Ts = car.nlobj.Ts;
dist = se2_dist(x0(1:3), y_target);
duration = dist / car.se2_speed;
Tsteps = ceil(duration / Ts);

x_history = zeros(5, Tsteps + 1);
x_history(:, 1) = x0;
u_history = zeros(2, Tsteps);
u_prev = [0; 0];

% Create reference targets along the line segment to avoid overshooting
% http://www.cs.cornell.edu/courses/cs4620/2013fa/lectures/16spline-curves.pdf
y_ref = se2_spline(x0(1:3), y_target, Tsteps, car.dscale);

if waitbar_enabled
    hbar = waitbar(0,'Simulation Progress');
end
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
    
    [uk, car.nlopt, ~] = nlmpcmove(car.nlobj, xk, u_prev, y_ref(:, k+1:k+mpc_horizon)', [], car.nlopt);
    u_history(:, k) = uk;
    u_prev = uk;
    
    ODEFUN = @(t, xk) ackerman_dynamics(xk, uk, car.length, car.M1, car.M2);
    [~, YOUT] = ode45(ODEFUN,[0 Ts], xk');
    x_history(:, k+1) = YOUT(end, :);
    if waitbar_enabled
        waitbar(k/Tsteps, hbar);
    end
end
if waitbar_enabled
    close(hbar)
end

end