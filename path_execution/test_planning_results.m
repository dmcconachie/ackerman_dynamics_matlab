clc; clear;
basedir = "/home/dmcconac/Dropbox/car_planning_classification_experiments/planning_logs/";
outputdir = "/home/dmcconac/Dropbox/car_planning_classification_experiments/planning_logs/parsed_output/";
[~, ~, ~] = mkdir(outputdir);
files = dir(append(basedir, "*path.csv"));

car = make_car();
aggregated_transition_distances = [];
for file = sort({files.name})
    path_file = append(basedir, file{1});
    obstacles_file = append(basedir, file{1}(1:end-8), "obstacles.csv");
    fprintf("Parsing %s ... ", file{1});
    
    [obstacles, waypoints, waypoint_traj_indices, x_trajectory, u_trajectory, y_trajectory] = ...
        execute_path(car, obstacles_file, path_file, false);
    
    edge_valid = collision_check_edges(car, obstacles, waypoints, waypoint_traj_indices, x_trajectory);
    last_valid_waypoint_idx = find(edge_valid, 1, 'last') + 1;
    
    planned_waypoints = waypoints(:, 1:last_valid_waypoint_idx);
    executed_states = x_trajectory(:, waypoint_traj_indices(1:last_valid_waypoint_idx));
    features = generate_features(car, obstacles, planned_waypoints);
    transition_distances = generate_distances(obstacles, planned_waypoints, executed_states);
   
    for idx = 1:length(features)
        outfile_name = append(outputdir, sprintf("%d.png", size(aggregated_transition_distances, 2) + idx - 1));
        save_to_file(features(idx), outfile_name);
    end
    aggregated_transition_distances = [aggregated_transition_distances, transition_distances];
    fprintf("total transitions %d\n", size(aggregated_transition_distances, 2));
end
writematrix(aggregated_transition_distances', append(outputdir, "distances.csv"));