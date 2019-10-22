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