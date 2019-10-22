function collision = collision_check(car, obstacles, state)

    car_rect = car_to_rect(car, state);
    valid = true;
    for obs = obstacles
        valid = ~aar_oar_intersection(to_points(obs), car_rect);
        if ~valid
            break;
        end
    end
    collision = ~valid;

end