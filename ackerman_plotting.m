function ackerman_plotting(t, state_history, control_history, l, w)

x = state_history(1, :);
y = state_history(2, :);
theta = state_history(3, :);
phi = state_history(4, :);
v = state_history(5, :);

F = control_history(1, :);
dphi = control_history(2, :);

figure(1)
clf;
subplot(3, 2, 1);
plot(t, [x; y]');
xlabel('Time')
ylabel('Position (m)')
legend('X', 'Y')

subplot(3, 2, 2);
plot(t, [theta; phi]');
xlabel('Time')
ylabel('Orientation (rad)');
legend('Theta', 'Phi');

subplot(3, 2, 3);
plot(t, v);
xlabel('Time');
ylabel('Velocity (m/s)');

subplot(3, 2, 5);
plot(t(2:end), F);
xlabel('Time');
ylabel('Input Force (N)');

subplot(3, 2, 6);
plot(t(2:end), dphi);
xlabel('Time');
ylabel('Steering Angle Change (rad/s)');

subplot(3, 2, 4);
hold on
plot_car_traj(state_history, l, w);
hold off
axis([min(x)-l*1.5, max(x)+l*1.5, min(y)-l*1.5, max(y)+l*1.5]);
axis equal
xlabel('X (m)');
ylabel('Y (m)');

end