function dstate = ackerman_dynamics(state, control, l, M1, M2)
%ACKERMAN_DYNAMICS - Taken from https://ieeexplore.ieee.org/document/7075182
%   params(1) = l       Length of vehicle
%   params(2) = M1      Mass on rear axle of the vehicle
%   params(3) = M2      Mass on front axle of the vehicle
%   state(1) = x        Position of the vehicle in world X-coords (x1 in paper)
%   state(2) = y        Position of the vechile in world Y-coords (x2 in paper)
%   state(3) = theta    Orientation of the vehicle in world θ-coords
%   state(4) = phi      Turn of the steering wheel (relative to θ)
%   state(5) = v        Velocity at the center of the rear axle (u1 in paper - conflated with control input 1 in paper)
%   control(1) = F      Propulsion of the motor
%   control(2) = u      Change in steering angle

% l = params(1);
% M1 = params(2);
% M2 = params(3);

x = state(1);
y = state(2);
theta = state(3);
phi = state(4);
v = state(5);

F = control(1);
u = control(2);

dx = v * cos(theta);
dy = v * sin(theta);
dtheta = v/l * tan(phi);
dphi = u;
dv = (F - M2 * dphi * sin(phi) / cos(phi)^3) / (M1 + M2 + M2 * tan(phi)^2);

dstate = [dx; dy; dtheta; dphi; dv];

end

