function features = generate_features(car, obstacles, waypoints, edges_valid)

image_size = 64;
capture_width = 10;

next_feature.start_car = [];
next_feature.end_car = [];
next_feature.environment = [];

features = [];

for edge_idx = 1:length(edges_valid)
    valid = edges_valid(edge_idx);
    start_state = waypoints(edge_idx, :)';
    end_state = waypoints(edge_idx + 1, :)';
    
    center = (start_state(1:2) + end_state(1:2)) / 2;
    linspace_x = linspace(center(1) - capture_width / 2, center(1) + capture_width / 2, image_size);
    linspace_y = linspace(center(2) - capture_width / 2, center(2) + capture_width / 2, image_size);
    [pixels_x, pixels_y] = meshgrid(linspace_x, flip(linspace_y));
        
    next_feature.start_car = rasterize_car(car, start_state, pixels_x, pixels_y);
    next_feature.end_car = rasterize_car(car, end_state, pixels_x, pixels_y);
    next_feature.environment = rasterize_obstacles(obstacles, pixels_x, pixels_y);
    
    features = [features, next_feature];
    
    if ~valid
        break
    end
end

end

function rasterized = rasterize_car(car, state, pixels_x, pixels_y)
    rect = car_to_rect(car, state);
    inside_fn = @(x, y) point_inside_rect(rect, [x; y]);
    rasterized = arrayfun(inside_fn, pixels_x, pixels_y);
end

function rasterized = rasterize_obstacles(obstacles, pixels_x, pixels_y)
    rasterized = false(size(pixels_x));
    for obs = obstacles
        inside_fn = @(x, y) point_inside_rect(obs, [x; y]);
        rasterized_obs = arrayfun(inside_fn, pixels_x, pixels_y);
        rasterized = rasterized | rasterized_obs;
    end
end
