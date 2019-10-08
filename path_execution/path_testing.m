clc;clear;clf;
[car, obstacles, waypoints, waypoint_traj_indices, x_trajectory, u_trajectory, y_trajectory] = ...
    execute_path('../../planning_logs/planned_path__2019-10-08__11-49-36__obstacles.csv', ...
                 '../../planning_logs/planned_path__2019-10-08__11-49-36__path.csv', ...
                 true);
%%
hold on
% plot_car_traj(x_trajectory, car.length, car.width, 'g', 1);

edges_valid = collision_check_edges(car, obstacles, waypoints, waypoint_traj_indices, x_trajectory);
last_valid_waypoint_idx = find(edges_valid, 1, 'last') + 1;
last_valid_traj_state_idx = waypoint_traj_indices(last_valid_waypoint_idx);
valid_traj = x_trajectory(:, 1:last_valid_traj_state_idx);

first_invalid_waypoint_idx = find(~edges_valid, 1, 'first') + 1;
first_invalid_traj_state_idx = waypoint_traj_indices(first_invalid_waypoint_idx - 1);
invalid_traj = x_trajectory(:, first_invalid_traj_state_idx:end);

%%
clf;
hold on
plot_obstacles(obstacles);
axis square
plot_waypoints(car, waypoints, 'b')
% plot(y_trajectory(1, :), y_trajectory(2, :), 'k');
if ~isempty(valid_traj)
    plot_car_traj(valid_traj, car.length, car.width, 'y', 4);
end
if ~isempty(invalid_traj)
    plot_car_traj(invalid_traj, car.length, car.width, 'r', 4);end
hold off
drawnow

%%
generate_features(car, obstacles, waypoints, edges_valid);