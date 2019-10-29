function features = generate_features(car, obstacles, waypoints)

    image_size = 64;
    capture_width = 10;

    feature.environment = [];
    % feature.start_car.box = [];
    % feature.start_car.arrow = [];
    feature.start_car.triangle = [];
    feature.start_dist_and_bearing = [];
    % feature.end_car.box = [];
    % feature.end_car.arrow = [];
    feature.end_car.triangle = [];
    feature.end_dist_and_bearing = [];

    feature.relative.end_state = [];
    feature.relative.start_dist_and_bearing = [];
    feature.relative.end_dist_and_bearing = [];

    num_edges = size(waypoints, 2) - 1;
    features(1:num_edges) = feature;

    for edge_idx = 1:num_edges
        start_state = waypoints(:, edge_idx);
        end_state = waypoints(:, edge_idx + 1);

        center = (start_state(1:2) + end_state(1:2)) / 2;
        linspace_x = linspace(center(1) - capture_width / 2, center(1) + capture_width / 2, image_size);
        linspace_y = linspace(center(2) - capture_width / 2, center(2) + capture_width / 2, image_size);
        [pixels_x, pixels_y] = meshgrid(linspace_x, flip(linspace_y));

        features(edge_idx).environment = rasterize_obstacles(obstacles, pixels_x, pixels_y);
        features(edge_idx).start_car = rasterize_car(car, start_state, pixels_x, pixels_y);
        features(edge_idx).end_car = rasterize_car(car, end_state, pixels_x, pixels_y);

        start_db = nearest_dist_and_bearing(obstacles, start_state(1:2));
        end_db = nearest_dist_and_bearing(obstacles, end_state(1:2));
        features(edge_idx).start_dist_and_bearing = start_db;
        features(edge_idx).end_dist_and_bearing = end_db;

        features(edge_idx).relative.end_state = se2_delta(start_state(1:3), end_state(1:3));
        features(edge_idx).relative.start_dist_and_bearing = [
                start_db(1);
                revolute_delta(start_state(3), start_db(2))];
        features(edge_idx).relative.end_dist_and_bearing = [
                end_db(1);
                revolute_delta(end_state(3), end_db(2))];
    end

end

function rasterized = rasterize_car(car, state, pixels_x, pixels_y)
%     rect = car_to_rect(car, state);
%     inside_fn = @(x, y) point_inside_rect(rect, [x; y]);
%     rasterized.box = arrayfun(inside_fn, pixels_x, pixels_y);
%     rasterized.arrow = rasterize_arrow(car, state, pixels_x, pixels_y);
    rasterized.box = [];
    rasterized.arrow = [];
    rasterized.triangle = rasterize_triangle(car, state, pixels_x, pixels_y);
end

function rasterized = rasterize_obstacles(obstacles, pixels_x, pixels_y)
    rasterized = false(size(pixels_x));
    for obs = obstacles
        inside_fn = @(x, y) point_inside_rect(obs, [x; y]);
        rasterized_obs = arrayfun(inside_fn, pixels_x, pixels_y);
        rasterized = rasterized | rasterized_obs;
    end
end

function rasterized = rasterize_triangle(car, state, pixels_x, pixels_y)

    points = car_to_triangle(car, state);
    inside = @(x, y) point_inside_triangle(points, [x; y]);
    rasterized = arrayfun(inside, pixels_x, pixels_y);

end

function rasterized = rasterize_arrow(car, state, pixels_x, pixels_y)
    points = car_to_arrow(car, state, deg2rad(30), car.width);
    rasterized = rasterize_segment(points(:, 1), points(:, 2), pixels_x, pixels_y) | ...
                 rasterize_segment(points(:, 2), points(:, 3), pixels_x, pixels_y) | ...
                 rasterize_segment(points(:, 2), points(:, 4), pixels_x, pixels_y);
end

function rasterized = rasterize_segment(p1, p2, pixels_x, pixels_y)
    segment_length = norm(p2 - p1);
    a = p2(2) - p1(2);
    b = p2(1) - p1(1);
    c = p2(1)*p1(2) - p2(2)*p1(1);

    dist_p1 = pdist2(p1', [pixels_x(:), pixels_y(:)])';
    dist_p2 = pdist2(p2', [pixels_x(:), pixels_y(:)])';
    dist_line = abs(a * pixels_x(:) - b * pixels_y(:) + c) / segment_length;

    pixel_size = abs(pixels_y(1) - pixels_y(2));
    rasterized = reshape(dist_p1 <= segment_length & ...
                         dist_p2 <= segment_length & ...
                         dist_line < pixel_size/2, size(pixels_x));
end

function db = nearest_dist_and_bearing(obstacles, point)

%     dbs = arrayfun(@(obs) distance_and_bearing(obs, point), obstacles);
    dbs = zeros(2, length(obstacles));
    for idx = 1:length(obstacles)
        dbs(:, idx) = distance_and_bearing(obstacles(idx), point);
    end
    [~, idx] = min(dbs(1, :));
    db = dbs(:, idx);

end