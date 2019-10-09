function [obstacles, waypoints, waypoint_traj_indices, x_trajectory, u_trajectory, y_trajectory] = ...
    execute_path(car, obstacles_file, waypoints_file, plotting_on)

if nargin < 3
    plotting_on = true;
end

obstacles = to_obstacles(obstacles_file);
waypoints = load(waypoints_file)';

if plotting_on
    plot_obstacles(obstacles);
    axis square
    hold on
    plot_waypoints(car, waypoints, 'b')
end

x0 = [waypoints(:, 1); 0; 5];
x_trajectory = x0;
y_trajectory = x0(1:3);
u_trajectory = [];
num_waypoints = size(waypoints, 2);
waypoint_traj_indices = zeros(1, num_waypoints);
waypoint_traj_indices(1) = 1;
for idx = 2:num_waypoints
    [x_history, u_history, y_ref] = ackerman_segment_nlmpc(car, x0, waypoints(:, idx), plotting_on);
    if plotting_on
        plot(y_ref(1, :), y_ref(2, :), 'k');
        plot_car_traj(x_history, car.length, car.width, 'r', 4);
    end
    x0 = x_history(:, end);
    x_trajectory = [x_trajectory, x_history(:, 2:end)];
    y_trajectory = [y_trajectory, y_ref(:, 2:end)];
    u_trajectory = [u_trajectory, u_history];
    waypoint_traj_indices(idx) = size(x_trajectory, 2);
end
if plotting_on
    hold off
    drawnow
end

end

function obstacles = to_obstacles(obstacles_file)

obs.center = [0; 0];
obs.theta = 0;
obs.half_extents = [0; 0];

obstacles_data = load(obstacles_file);
num_obstacles = size(obstacles_data, 1);

obstacles(1:num_obstacles) = obs;
for idx = 1:num_obstacles
    obstacles(idx).center = obstacles_data(idx, 1:2)';
    obstacles(idx).theta = obstacles_data(idx, 3);
    obstacles(idx).half_extents = obstacles_data(idx, 5:-1:4)' / 2;
end

end