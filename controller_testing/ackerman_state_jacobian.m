function [A, B] = ackerman_state_jacobian(state, control, l, M1, M2)

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

A = [[ 0, 0, -v*sin(theta),                                                                                                                  0, cos(theta)]
     [ 0, 0,  v*cos(theta),                                                                                                                  0, sin(theta)]
     [ 0, 0,             0,                                                                                             (v*(tan(phi)^2 + 1))/l, tan(phi)/l]
     [ 0, 0,             0,                                                                                                                  0,          0]
     [ 0, 0,             0, -(M2*(M2*u + 3*M1*u*cos(phi)^2 - 2*M1*u*cos(phi)^4 + 2*F*cos(phi)^3*sin(phi)))/(cos(phi)^2*(M1*cos(phi)^2 + M2)^2),          0]];

B = [[                           0,                                            0]
     [                           0,                                            0]
     [                           0,                                            0]
     [                           0,                                            1]
     [ 1/(M2*tan(phi)^2 + M1 + M2), -(M2*sin(phi))/(M1*cos(phi)^3 + M2*cos(phi))]];

end