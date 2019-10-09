clc;clear;clf;
[car, obstacles, waypoints, waypoint_traj_indices, x_trajectory, u_trajectory, y_trajectory] = ...
    execute_path('../../planning_logs/planned_path__2019-10-08__16-42-35__obstacles.csv', ...
                 '../../planning_logs/planned_path__2019-10-08__16-42-35__path.csv', ...
                 true);
%%
hold on
% plot_car_traj(x_trajectory, car.length, car.width, 'g', 1);

edge_valid = collision_check_edges(car, obstacles, waypoints, waypoint_traj_indices, x_trajectory);
last_valid_waypoint_idx = find(edge_valid, 1, 'last') + 1;
last_valid_traj_state_idx = waypoint_traj_indices(last_valid_waypoint_idx);
valid_traj = x_trajectory(:, 1:last_valid_traj_state_idx);

first_invalid_waypoint_idx = find(~edge_valid, 1, 'first') + 1;
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
    plot_car_traj(invalid_traj, car.length, car.width, 'r', 4);
end
hold off
drawnow

%%
planned_waypoints = waypoints(:, 1:last_valid_waypoint_idx);
executed_states = x_trajectory(:, waypoint_traj_indices(1:last_valid_waypoint_idx));
features = generate_features(car, obstacles, planned_waypoints);
transition_distances = generate_distances(obstacles, planned_waypoints, executed_states);

%%
clf
row1 = 0;
row2 = length(features);
row3 = 2 * length(features);
row4 = 3 * length(features);
for idx = 1:length(features)
    subplot(4, length(features), row1 + idx); 
    imagesc(features(idx).start_car); axis square
    
    subplot(4, length(features), row2 + idx); 
    imagesc(features(idx).end_car); axis square
    
    subplot(4, length(features), row3 + idx); 
    imagesc(features(idx).environment); axis square
    
    subplot(4, length(features), row4 + idx);
    image(:, :, 1) = double(features(idx).environment);
    image(:, :, 2) = double(features(idx).start_car);
    image(:, :, 3) = double(features(idx).end_car);
    imagesc(image); axis square
    
%     title(sprintf("Valid: %g\n", edge_valid(idx)));
%     pause
end