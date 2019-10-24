clc; clear;
car = make_car();
% obstacles_file = "/mnt/big_narstie_data/dmcconac/car_planning_classification_experiments/planning_logs/problem_result__2019-10-15_16:47:25.314__obstacles.csv";
% obstacles = to_obstacles(obstacles_file);

obstacles.center = [5.5; 4.5];
obstacles.theta = 0;
obstacles.half_extents = [2; 2];

figure(1); clf;
plot_obstacles(obstacles); hold on

x0 = [0; 0; pi/2; 0.0; 4.0];
plot_car(x0(1), x0(2), x0(3), x0(4), car.length, car.width, 'k', 0.5);
axis([-9, 9, -9, 9]);
axis equal
%%
% n_test = 15;
% x_test = zeros(5, n_test);
% for idx=1:n_test
%     [X, Y] = ginput(2);
%     x_test(1:2, idx) = [X(1); Y(1)];
%     x_test(3, idx) = atan2(diff(Y), diff(X));
%     plot_car(x_test(1, idx), x_test(2, idx), x_test(3, idx), x_test(4, idx), car.length, car.width, 'm', 0.5);
%     se2_dist(x0, x_test(:, idx))
% end
% save("x_test.mat", "x_test");

%%
figure(1);
load("x_test.mat");
n_test = size(x_test, 2);
for idx=1:n_test
    plot_car(x_test(1, idx), x_test(2, idx), x_test(3, idx), x_test(4, idx), car.length, car.width, 'm', 0.5);
    text(x_test(1, idx), x_test(2, idx), sprintf("%d", idx));
end

%%
figure(2);
clearvars image
transition_data = cell(2, n_test);
for idx = 1:n_test
    waypoints = [x0(1:3), x_test(1:3, idx)];
    states = [x0, x_test(:, idx)];
    
    tic;
    features = generate_features(car, obstacles, waypoints);
    transition_distances = generate_distances(obstacles, waypoints, states);
    transition_data(:, idx) = {features, transition_distances};
    toc;
    
    subplot(4, 4, idx);
    imshow(features_to_image(features));
    title(sprintf("Index: %d", idx));
end

%%
while true
    figure(1);
    test_idx = input("Enter test index: ");
    if ~isnumeric(test_idx) || test_idx < 1 || test_idx > n_test
        break
    end


    [X, Y] = ginput(2);
    theta = atan2(diff(Y), diff(X));
    x_new = [X(1); Y(1); theta; 0; 0];    
    x_test(:, test_idx) = x_new;
    save("x_test.mat", "x_test");
    distance = se2_dist(x0, x_new);
    fprintf("Distance: %g\n", distance);

    clf; plot_obstacles(obstacles); hold on
    plot_car(x0(1), x0(2), x0(3), x0(4), car.length, car.width, 'k', 0.5);
    for idx=1:n_test
        plot_car(x_test(1, idx), x_test(2, idx), x_test(3, idx), x_test(4, idx), car.length, car.width, 'm', 0.5);
        text(x_test(1, idx), x_test(2, idx), sprintf("%d", idx));
    end
    axis([-9, 9, -9, 9]);
    axis equal
    drawnow

    waypoints = [x0(1:3), x_new(1:3)];
    states = [x0, x_new];
    
    features = generate_features(car, obstacles, waypoints);
    transition_distances = generate_distances(obstacles, waypoints, states);

    transition_data(:, test_idx) = {features, transition_distances};
    
    figure(2);
    subplot(4, 4, test_idx);
    image(:, :, 1) = double(features.environment | features.start_car.arrow | features.end_car.arrow);
    image(:, :, 2) = double(features.start_car.box);
    image(:, :, 3) = double(features.end_car.box);
    imshow(image)
    title(sprintf("Index: %d", test_idx));
    drawnow
end

%%
aggregated_transitions = [transition_data{1, :}];
aggregated_transition_distances = [transition_data{2, :}];
outputdir = "/mnt/big_narstie_data/dmcconac/car_planning_classification_experiments/classifier_figure_tests/";
[~, ~, ~] = mkdir(outputdir);
for idx = 1:n_test
    outfile_name = append(outputdir, sprintf("%d.png", idx - 1));    
    save_to_file(aggregated_transitions(idx), outfile_name);
end