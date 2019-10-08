function plot_waypoints(car, waypoints, color)

if nargin < 3
    color = 'y';
end

x = waypoints(:, 1);
y = waypoints(:, 2);
theta = waypoints(:, 3);
phi = 0;

hold_state = ishold;
hold on
num_waypoints = size(waypoints, 1);
alpha = linspace(0.1, 0.5, num_waypoints);
for idx = 1:num_waypoints
    plot_car(x(idx), y(idx), theta(idx), phi, car.length, car.width, color, alpha(idx));
end
plot(x, y, color);
if ~hold_state
    hold off
end

end