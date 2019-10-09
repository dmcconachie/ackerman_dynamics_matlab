function transition_distances = generate_distances(obstacles, planned_waypoints, executed_states)

% Note that this function assumes that the waypoints are not inside the
% obstacles (needed for the line-rectangle intersection check)

assert(size(planned_waypoints, 1) == 3);
assert(size(executed_states, 1) == 5);
assert(size(planned_waypoints, 2) == size(executed_states, 2));

num_states = size(executed_states, 2);

transition_distances = zeros(4, num_states - 1);
for idx = 1:num_states-1
    % Before Moving
    transition_distances(1, idx) = se3_dist(planned_waypoints(:, idx), executed_states(1:3, idx));
    transition_distances(2, idx) = ~obstacle_line_collision(obstacles, planned_waypoints(1:2, idx), executed_states(1:2, idx));
    % After Moving
    transition_distances(3, idx) = se3_dist(planned_waypoints(:, idx + 1), executed_states(1:3, idx +1));
    transition_distances(4, idx) = ~obstacle_line_collision(obstacles, planned_waypoints(1:2, idx + 1), executed_states(1:2, idx + 1));
end

end

function collision = obstacle_line_collision(obstacles, p1, p2)

    collision = 0;
    for obs = obstacles
        collision = oar_line_intersection(obs, p1, p2);
        if collision
            break
        end
    end

end