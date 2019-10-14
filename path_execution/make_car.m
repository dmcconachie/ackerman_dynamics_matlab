function car = make_car(validate)

if nargin < 1
    validate = true;
end

length = 1.0; % Wheelbase length
width = 0.5;  % Distance between wheels

car.length = length;
car.width = width;

M = 1;
M1 = M/2;
M2 = M/2;
car.M = M;
car.M1 = M1;
car.M2 = M2;

nx = 5; % State dimension
ny = 3; % Output dimension
nu = 2; % Control dimension

car.Ts = 0.05; % Sample time
car.default_action_duration = 2;
car.default_Tsteps = ceil(car.default_action_duration / car.Ts);
car.mpc_horizon = 10;
car.se2_speed = 5;
car.dscale = car.se2_speed; % used by se2_spline to define "curvyness" of generated reference splines

car.nlobj = nlmpc(nx, ny, nu);
car.nlobj.Ts = car.Ts;
car.nlobj.PredictionHorizon = car.mpc_horizon;
car.nlobj.ControlHorizon = car.mpc_horizon;

car.nlobj.Model.IsContinuousTime = true;
car.nlobj.Model.NumberOfParameters = 3;
car.nlobj.Model.StateFcn = @ackerman_dynamics;
car.nlobj.Model.OutputFcn = @(state, control, length, M1, M2) state(1:3);

car.nlobj.States(1).Name = 'x';
car.nlobj.States(1).Units = 'Meters';

car.nlobj.States(2).Name = 'y';
car.nlobj.States(2).Units = 'Meters';

car.nlobj.States(3).Name = 'theta';
car.nlobj.States(3).Units = 'Radians';

car.nlobj.States(4).Name = 'phi';
car.nlobj.States(4).Units = 'Radians';
car.nlobj.States(4).Min = -deg2rad(75);
car.nlobj.States(4).Max = deg2rad(75);

car.nlobj.States(5).Name = 'v';
car.nlobj.States(5).Units = 'Meters / Second';
car.nlobj.States(5).Min = 0.0;
car.nlobj.States(5).Max = 20.0;

car.nlobj.ManipulatedVariables(1).Name = 'F';
car.nlobj.ManipulatedVariables(1).Units = 'Newtons';
car.nlobj.ManipulatedVariables(1).Min = -80;
car.nlobj.ManipulatedVariables(1).Max = 20;

car.nlobj.ManipulatedVariables(2).Name = 'Steering Change';
car.nlobj.ManipulatedVariables(2).Units = 'Radians / Second';

car.nlobj.Jacobian.StateFcn = @ackerman_state_jacobian;
car.nlobj.Jacobian.OutputFcn = @(state, control, length, M1, M2) [[1 0 0 0 0]', [0 1 0 0 0]', [0 0 1 0 0]']';

if validate
    validateFcns(car.nlobj, zeros(5, 1), zeros(2, 1), [], {length, M1, M2});
end

car.nlopt = nlmpcmoveopt;
car.nlopt.Parameters = {length, M1, M2};
car.nlopt.OutputWeights = [10, 10, 10];
car.nlopt.MVWeights = [0.1, 0.01];
car.nlopt.MVRateWeights = [1, 1];

end