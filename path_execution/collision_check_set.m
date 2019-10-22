function collision = collision_check_set(car, obstacles, states)

    collision = zeros(1, size(states, 2));
    for idx = 1:length(collision)
        collision(idx) = collision_check(car, obstacles, states(:, idx));
    end

end