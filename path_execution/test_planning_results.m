clc; clear;
basedir = "/mnt/big_narstie_data/dmcconac/car_planning_classification_experiments/planning_logs/";
outputdir = "/mnt/big_narstie_data/dmcconac/car_planning_classification_experiments/planning_logs/parsed_output/";
[~, ~, ~] = mkdir(outputdir);
files = dir(append(basedir, "*path.csv"));
filenames = sort({files.name});

car = make_car(false);

transition_data = cell(2, length(filenames));

tic
parfor idx = 1:length(filenames)
    file = filenames(idx);
    path_file = append(basedir, file{1});
    obstacles_file = append(basedir, file{1}(1:end-8), "obstacles.csv");
    
    [obstacles, waypoints, waypoint_traj_indices, x_trajectory, u_trajectory, y_trajectory] = ...
        execute_path(car, obstacles_file, path_file, false);

    last_valid_waypoint_idx = find(waypoint_traj_indices ~= 0, 1, 'last');
    assert(size(x_trajectory, 2) == waypoint_traj_indices(last_valid_waypoint_idx));
    
    executed_waypoints = waypoints(:, 1:last_valid_waypoint_idx);
    executed_states = x_trajectory(:, waypoint_traj_indices(1:last_valid_waypoint_idx));
    features = generate_features(car, obstacles, executed_waypoints);
    transition_distances = generate_distances(obstacles, executed_waypoints, executed_states);
    
    transition_data(:, idx) = {features, transition_distances};

    fprintf("Parsing %s, path contains %d waypoints, result contains %d valid waypoints\n", ...
        file{1}(length(basedir):end), length(waypoints), last_valid_waypoint_idx);
end
toc

%%
save(append(outputdir, 'transition_data.mat'), 'transition_data');
aggregated_transitions = [transition_data{1, :}];
aggregated_transition_distances = [transition_data{2, :}];
clearvars transition_data

parfor idx = 1:length(aggregated_transitions)
    outfile_name = append(outputdir, sprintf("%d.png", idx - 1));    
    save_to_file(aggregated_transitions(idx), outfile_name);
end

writematrix(aggregated_transition_distances', append(outputdir, "distances.csv"));