clc; clear;

% Vaguely like a 2006 Honda Civic?
% l = 2.7; % Wheelbase length
% w = 1.5; % Distance between wheels
% slow = 1; % m/s (3.6 kph, 2.240 mph)
% med = 7; % m/s (27 kph, 16.777 mph)
% fast = 15; % m/s (54 kph, 33.554 mph)
% dscale = 50; % used by se3_spline to define "curvyness" of generated reference splines
% 
% M = 1330; % kilograms
% M1 = M/2;
% M2 = M/2;
% duration = 30;

% Generates somewhat reasonable figures
l = 1.0; % Wheelbase length
w = 0.5; % Distance between wheels
slow = 1; % m/s (3.6 kph, 2.240 mph)
med = 7; % m/s (27 kph, 16.777 mph)
fast = 15; % m/s (54 kph, 33.554 mph)
dscale = 5; % used by se2_spline to define "curvyness" of generated reference splines

M = 1;
M1 = M/2;
M2 = M/2;

nx = 5; % State dimension
ny = 3; % Output dimension
nu = 2; % Control dimension
Ts = 0.05; % Sample time
mpc_horizon = 10;

nlobj = nlmpc(nx, ny, nu);
nlobj.Ts = Ts;
nlobj.PredictionHorizon = mpc_horizon;
nlobj.ControlHorizon = mpc_horizon;

nlobj.Model.IsContinuousTime = true;
nlobj.Model.NumberOfParameters = 3;
nlobj.Model.StateFcn = @ackerman_dynamics;
nlobj.Model.OutputFcn = @(state, control, l, M1, M2) state(1:3);

nlobj.States(1).Name = 'x';
nlobj.States(1).Units = 'Meters';

nlobj.States(2).Name = 'y';
nlobj.States(2).Units = 'Meters';

nlobj.States(3).Name = 'theta';
nlobj.States(3).Units = 'Radians';

nlobj.States(4).Name = 'phi';
nlobj.States(4).Units = 'Radians';
nlobj.States(4).Min = -deg2rad(75);
nlobj.States(4).Max = deg2rad(75);

nlobj.States(5).Name = 'v';
nlobj.States(5).Units = 'Meters / Second';
nlobj.States(5).Min = 0.0;
nlobj.States(5).Max = 20.0;

nlobj.ManipulatedVariables(1).Name = 'F';
nlobj.ManipulatedVariables(1).Units = 'Newtons';
nlobj.ManipulatedVariables(1).Min = -80;
nlobj.ManipulatedVariables(1).Max = 20;

nlobj.ManipulatedVariables(2).Name = 'Steering Change';
nlobj.ManipulatedVariables(2).Units = 'Radians / Second';

nlobj.Jacobian.StateFcn = @ackerman_state_jacobian;
nlobj.Jacobian.OutputFcn = @(state, control, l, M1, M2) [[1 0 0 0 0]', [0 1 0 0 0]', [0 0 1 0 0]']';

validateFcns(nlobj, zeros(5, 1), zeros(2, 1), [], {l, M1, M2});

nlopt = nlmpcmoveopt;
nlopt.Parameters = {l, M1, M2};
nlopt.OutputWeights = [10, 10, 10];
nlopt.MVWeights = [0.1, 0.01];
nlopt.MVRateWeights = [1, 1];

%% Experiment 1 Setup
ex1.l = l;
ex1.w = w;
ex1.obstacle.bl = [3, 0];
ex1.obstacle.tr = [6, 3];
ex1.y0 = [0; 0; pi/1.2];
ex1.y1 = [2; 5; pi/2];
ex1.slow.x0 = [ex1.y0; 0; slow];
ex1.med.x0 = [ex1.y0; 0; med];
ex1.fast.x0 = [ex1.y0; 0; fast];

fprintf(sprintf('Ex 1 Start, Goal dist: %g\n', se2_dist(ex1.y0, ex1.y1)))

ex1.ref_color = 'k';
ex1.slow.color = 'b';
ex1.med.color = 'm';
ex1.fast.color = 'r';

duration = se2_dist(ex1.y0, ex1.y1)/ 5.0;
Tsteps = ceil(duration / Ts);

ex1.y_ref = se2_spline(ex1.y0, ex1.y1, Tsteps, dscale);
ex1.lifted_y_ref = [ex1.y_ref; zeros(2, Tsteps + 1)];

%% Experiment 2 Setup
ex2.l = l;
ex2.w = w;
ex2.obstacle.bl = [1, 3];
ex2.obstacle.tr = [5, 4];
ex2.y0 = [0; 0; pi/2];
ex2.y1 = [4; 0; -pi/2];
ex2.slow.x0 = [ex2.y0; 0; slow];
ex2.med.x0 = [ex2.y0; 0; med];
ex2.fast.x0 = [ex2.y0; 0; fast];

fprintf(sprintf('Ex 2 Start, Goal dist: %g\n', se2_dist(ex2.y0, ex2.y1)))

ex2.ref_color = 'k';
ex2.slow.color = 'b';
ex2.med.color = 'm';
ex2.fast.color = 'r';

duration = se2_dist(ex2.y0, ex2.y1)/ 5.0;
Tsteps = ceil(duration / Ts);

ex2.y_ref = se2_spline(ex2.y0, ex2.y1, Tsteps, dscale);
ex2.lifted_y_ref = [ex2.y_ref; zeros(2, Tsteps + 1)];

%%
figure(1); clf
plot_env(ex1, ex2);

%% Ex1 - Slow
[ex1.slow.x_history, ex1.slow.u_history] = ...
    ackerman_segment_nlmpc(nlobj, nlopt, ex1.slow.x0, ex1.y1, l, M1, M2, dscale);
%%
subplot(2, 3, 1);
plot_car_traj(ex1.slow.x_history, l, w, ex1.slow.color);
title(sprintf('Slow start v = %g', ex1.slow.x0(end)))
drawnow
%% Ex1 - Medium
[ex1.med.x_history, ex1.med.u_history] = ...
    ackerman_segment_nlmpc(nlobj, nlopt, ex1.med.x0, ex1.y1, l, M1, M2, dscale);
%%
subplot(2, 3, 2);
plot_car_traj(ex1.med.x_history, l, w, ex1.med.color);
title(sprintf('Medium start v = %g', ex1.med.x0(end)))
drawnow
%% Ex1 - Fast
[ex1.fast.x_history, ex1.fast.u_history] = ...
    ackerman_segment_nlmpc(nlobj, nlopt, ex1.fast.x0, ex1.y1, l, M1, M2, dscale);
%%
subplot(2, 3, 3);
plot_car_traj(ex1.fast.x_history, l, w, ex1.fast.color);
title(sprintf('Fast start v = %g', ex1.fast.x0(end)))
drawnow
%% Ex2 - Slow
[ex2.slow.x_history, ex2.slow.u_history] = ...
    ackerman_segment_nlmpc(nlobj, nlopt, ex2.slow.x0, ex2.y1, l, M1, M2, dscale);
%%
subplot(2, 3, 4);
plot_car_traj(ex2.slow.x_history, l, w, ex2.slow.color);
title(sprintf('Slow start v = %g', ex2.slow.x0(end)))
drawnow
%% Ex2 - Medium
[ex2.med.x_history, ex2.med.u_history] = ...
    ackerman_segment_nlmpc(nlobj, nlopt, ex2.med.x0, ex2.y1, l, M1, M2, dscale);
%%
subplot(2, 3, 5);
plot_car_traj(ex2.med.x_history, l, w, ex2.med.color);
title(sprintf('Medium start v = %g', ex2.med.x0(end)))
drawnow
%% Ex2 - Fast
[ex2.fast.x_history, ex2.fast.u_history] = ...
    ackerman_segment_nlmpc(nlobj, nlopt, ex2.fast.x0, ex2.y1, l, M1, M2, dscale);
%%
subplot(2, 3, 6);
plot_car_traj(ex2.fast.x_history, l, w, ex2.fast.color);
title(sprintf('Fast start v = %g', ex2.fast.x0(end)))
drawnow
%%
fprintf('Experiment 1 final velocities\n');
fprintf('Slow:   %g\n', ex1.slow.x_history(5, end));
fprintf('Medium: %g\n', ex1.med.x_history(5, end));
fprintf('Fast:   %g\n', ex1.fast.x_history(5, end));
fprintf('\nExperiment 2 final velocities\n');
fprintf('Slow:   %g\n', ex2.slow.x_history(5, end));
fprintf('Medium: %g\n', ex2.med.x_history(5, end));
fprintf('Fast:   %g\n', ex2.fast.x_history(5, end));
%%
figure(2); clf;
play_trajectories(ex1, ex2);