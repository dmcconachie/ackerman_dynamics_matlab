clc; clear;
basedir = "/mnt/big_narstie_data/dmcconac/car_planning_classification_experiments/planning_logs/";
intermediatedir = "/mnt/big_narstie_data/dmcconac/car_planning_classification_experiments/planning_logs/intermediate_output/";
outputdir = "/mnt/big_narstie_data/dmcconac/car_planning_classification_experiments/planning_logs/parsed_output/";
[~, ~, ~] = mkdir(intermediatedir);
[~, ~, ~] = mkdir(outputdir);

input_files = dir(append(basedir, "*path.csv"));
input_files = sort({input_files.name});
already_parsed_files = dir(append(intermediatedir, "*.mat"));
already_parsed_files = sort({already_parsed_files.name});

%%
intermediate_files = cell(size(input_files));
for idx = 1:length(input_files)
    tmpfile = append(input_files{idx}(1:end-8), "trajectory.mat");
    intermediate_files{idx} = convertStringsToChars(tmpfile);
end
already_parsed_flag = ismember(intermediate_files, already_parsed_files);
filenames = input_files(~already_parsed_flag);

%%
car = make_car(false);
parfor idx = 1:length(filenames)
    experiment_name = filenames{idx}(1:end-8);
    path_file       = append(basedir,         experiment_name, "path.csv");
    obstacles_file  = append(basedir,         experiment_name, "obstacles.csv");
    trajectory_file = append(intermediatedir, experiment_name, "trajectory.mat");
    
    [obstacles, waypoints, waypoint_traj_indices, x_trajectory, u_trajectory, y_trajectory] = ...
        execute_path(car, obstacles_file, path_file, false);
    parsave(trajectory_file, ...
        obstacles, waypoints, waypoint_traj_indices, ...
        x_trajectory, u_trajectory, y_trajectory);

    fprintf("Parsing %s, path contains %d waypoints, result contains %d valid waypoints\n", ...
        experiment_name(1:end-2), length(waypoints), last_valid_waypoint_idx);
end

%%
transition_data = cell(2, length(filenames));
parfor idx = 1:length(intermediate_files)
    experiment_name = filenames{idx}(1:end-14);
    path_file       = append(basedir,         experiment_name, "path.csv");
    obstacles_file  = append(basedir,         experiment_name, "obstacles.csv");
    trajectory_file = append(intermediatedir, experiment_name, "trajectory.mat");
    
    [obstacles, waypoints, waypoint_traj_indices, ...
        x_trajectory, u_trajectory, y_trajectory] = parload(trajectory_file);
    
    last_valid_waypoint_idx = find(waypoint_traj_indices ~= 0, 1, 'last');
    assert(size(x_trajectory, 2) == waypoint_traj_indices(last_valid_waypoint_idx));

    executed_waypoints = waypoints(:, 1:last_valid_waypoint_idx);
    executed_states = x_trajectory(:, waypoint_traj_indices(1:last_valid_waypoint_idx));
    features = generate_features(car, obstacles, executed_waypoints);
    transition_distances = generate_distances(obstacles, executed_waypoints, executed_states);

    transition_data(:, idx) = {features, transition_distances};
end

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
