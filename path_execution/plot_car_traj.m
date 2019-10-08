function plot_car_traj(state_history, l, w, color, substep)

if nargin < 4
    color = 'y';
end
if nargin < 5
    substep = 8;
end

n = size(state_history, 2);
idx_to_plot = 1:substep:n;
if idx_to_plot(end) ~= n
    idx_to_plot = [idx_to_plot, n];
end

x = state_history(1, idx_to_plot);
y = state_history(2, idx_to_plot);
theta = state_history(3, idx_to_plot);
phi = state_history(4, idx_to_plot);

hold_state = ishold;
hold on
for idx = 1:length(idx_to_plot)
    plot_car(x(idx), y(idx), theta(idx), phi(idx), l, w, color);
end
plot(state_history(1, :), state_history(2, :), color);

if ~hold_state
    hold off
end

end