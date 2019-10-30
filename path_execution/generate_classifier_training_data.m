clc; clear;
fprintf("Loading file lists ...\n"); tic;
basedir = "/mnt/big_narstie_data/dmcconac/car_planning_classification_experiments/planning_logs/";
intermediatedir = "/mnt/big_narstie_data/dmcconac/car_planning_classification_experiments/planning_logs/intermediate_output/";
outputdir = "/mnt/big_narstie_data/dmcconac/car_planning_classification_experiments/planning_logs/parsed_output/";
[~, ~, ~] = mkdir(intermediatedir);
[~, ~, ~] = mkdir(outputdir);

input_files = dir(append(basedir, "*path.csv"));
input_files = sort({input_files.name});
already_parsed_files = dir(append(intermediatedir, "*.mat"));
already_parsed_files = sort({already_parsed_files.name});

intermediate_files = cell(size(input_files));
for idx = 1:length(input_files)
    tmpfile = append(input_files{idx}(1:end-8), "trajectory.mat");
    intermediate_files{idx} = convertStringsToChars(tmpfile);
end
already_parsed_flag = ismember(intermediate_files, already_parsed_files);
trajectories_to_generate = input_files(~already_parsed_flag);
fprintf("Done in %g seconds\n", toc);

%%
fprintf("Generating trajectories ...\n"); tic;
car = make_car(false);
parfor idx = 1:length(trajectories_to_generate)
    experiment_name = trajectories_to_generate{idx}(1:end-8);
    path_file       = append(basedir,         experiment_name, "path.csv");
    obstacles_file  = append(basedir,         experiment_name, "obstacles.csv");
    trajectory_file = append(intermediatedir, experiment_name, "trajectory.mat");
    
    plot_traj = false;
    [obstacles, waypoints, waypoint_traj_indices, x_trajectory, u_trajectory, y_trajectory] = ...
        execute_path(car, obstacles_file, path_file, plot_traj);
    parsave(trajectory_file, ...
        obstacles, waypoints, waypoint_traj_indices, ...
        x_trajectory, u_trajectory, y_trajectory);

    last_valid_waypoint_idx = find(waypoint_traj_indices ~= 0, 1, 'last');
    fprintf("Parsing %s, path contains %d waypoints, result contains %d valid waypoints\n", ...
        experiment_name(1:end-2), length(waypoints), last_valid_waypoint_idx);
end
fprintf("Done in %g seconds\n", toc);

%%
fprintf("Generating transition data ...\n"); tic;
obstacles_data = cell(1, length(intermediate_files));
waypoint_data = cell(1, length(intermediate_files));
states_data = cell(1, length(intermediate_files));
distances = cell(1, length(intermediate_files));
parfor idx = 1:length(intermediate_files)
    experiment_name = intermediate_files{idx}(1:end-14);
%     path_file       = append(basedir,         experiment_name, "path.csv");
%     obstacles_file  = append(basedir,         experiment_name, "obstacles.csv");
    trajectory_file = append(intermediatedir, experiment_name, "trajectory.mat");
    
    [obstacles, waypoints, waypoint_traj_indices, ...
        x_trajectory, u_trajectory, y_trajectory] = parload(trajectory_file);
    
    last_valid_waypoint_idx = find(waypoint_traj_indices ~= 0, 1, 'last');
    assert(size(x_trajectory, 2) == waypoint_traj_indices(last_valid_waypoint_idx));

    obstacles_data{idx} = obstacles;
    waypoint_data{idx} = waypoints(:, 1:last_valid_waypoint_idx);
    states_data{idx} = x_trajectory(:, waypoint_traj_indices(1:last_valid_waypoint_idx));
    distances{idx} = generate_distances(obstacles, waypoint_data{idx}, states_data{idx});
end
fprintf("Done in %g seconds\n", toc);
fprintf("Saving data to file ...\n"); tic;
save(append(intermediatedir, "transition_data.mat"), "obstacles_data", "waypoint_data", "states_data", "-v7.3");
save(append(intermediatedir, "distances.mat"), "distances", "-v7.3");
writematrix([distances{:}]', append(outputdir, "distances.csv"));
fprintf("Done in %g seconds\n", toc);

%% Regenerate features from cached data
% load(append(intermediatedir, "transition_data.mat"), "obstacles_data", "waypoint_data");
fprintf("Generating transition features ...\n"); tic;
% template_features = generate_features(car, obstacles_data{1}, waypoint_data{1});
% features = repmat({template_features}, 1, length(obstacles_data));
features_data = cell(1, length(intermediate_files));
features_data_images = cell(1, length(intermediate_files));
parfor idx = 1:length(intermediate_files)
    features = generate_features(car, obstacles_data{idx}, waypoint_data{idx});
    features_data(idx) = {[features.relative]};
    features_data_images(idx) = {features};
end
fprintf("Done in %g seconds\n", toc);
fprintf("Saving data to file ...\n"); tic;
save(append(intermediatedir, "features_data.mat"), "features_data", "-v7.3");
save(append(intermediatedir, "features_data_images.mat"), "features_data_images", "-v7.3");
aggregated_features = [features_data{:}];
features_matrix = [[aggregated_features.end_state];
                   [aggregated_features.start_dist_and_bearing];
                   [aggregated_features.end_dist_and_bearing]];
writematrix(features_matrix', append(outputdir, "features.csv"));
fprintf("Done in %g seconds\n", toc);

%%
fprintf("Writing transition data to image files ...\n"); tic;
aggregated_features_images = [features_data_images{:}];
parfor idx = 1:length(aggregated_features_images)
    outfile_name = append(outputdir, sprintf("%d.png", idx - 1));    
    save_to_file(aggregated_features_images(idx), outfile_name);
end
fprintf("Done in %g seconds\n", toc);

%%
transitions_per_example = cellfun('length', features_data);
transitions_per_example_cumulative = cumsum([0, transitions_per_example]);

transitions_per_example_cumulative(1:3)
global_to_indiv = @(idx) deal(find(transitions_per_example_cumulative >= idx, 1) - 1, ...
                              idx - transitions_per_example_cumulative(find(transitions_per_example_cumulative >= idx, 1) - 1));

%%
%{
matlab_indicies = [3728 34330 43896];
for idx = 1:length(matlab_indicies)
    matlab_idx = matlab_indicies(idx);
    [test_idx, trans_idx] = global_to_indiv(matlab_idx);
%     features_matrix(:, matlab_idx)
%     [features_data{test_idx}(trans_idx).end_state; 
%      features_data{test_idx}(trans_idx).start_dist_and_bearing;
%      features_data{test_idx}(trans_idx).end_dist_and_bearing]
    figure(1)
    subplot(length(matlab_indicies), 1, idx);
    imshow(features_to_image(features_data_images{test_idx}(trans_idx)));
    
    experiment_name = input_files{test_idx}(1:end-8);
    path_file       = append(basedir,         experiment_name, "path.csv");
    obstacles_file  = append(basedir,         experiment_name, "obstacles.csv");

    figure(2);
    plot_traj = true;
    [obstacles, waypoints, waypoint_traj_indices, x_trajectory, u_trajectory, y_trajectory] = ...
        execute_path(car, obstacles_file, path_file, plot_traj);
    
    last_valid_waypoint_idx = find(waypoint_traj_indices ~= 0, 1, 'last');
    fprintf("Parsing %s, path contains %d waypoints, result contains %d valid waypoints\n", ...
        experiment_name(1:end-2), length(waypoints), last_valid_waypoint_idx);
    
    pause
end
%}
