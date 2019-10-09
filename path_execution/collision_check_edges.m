function valid_edges = collision_check_edges(car, obstacles, ...
                            waypoints, waypoint_traj_indices, x_trajectory, plotting_on)
                        
    if nargin < 6
        plotting_on = false;
    end

    num_waypoints = size(waypoints, 2);
    valid_edges = zeros(1, num_waypoints - 1);

    assert(isvector(waypoint_traj_indices));
    assert(length(waypoint_traj_indices) == num_waypoints);
    assert(all(sort(waypoint_traj_indices) == waypoint_traj_indices));
    assert(waypoint_traj_indices(1) == 1);
    assert(waypoint_traj_indices(end) == size(x_trajectory, 2));


    for waypoint_idx = 2:num_waypoints
        traj_start_idx = waypoint_traj_indices(waypoint_idx - 1);
        traj_end_idx = waypoint_traj_indices(waypoint_idx);

        valid = true;
        for traj_idx = traj_start_idx:traj_end_idx
            state = x_trajectory(:, traj_idx);
            valid = collision_check(car, obstacles, state(1:3));
            if ~valid
                if plotting_on
                    plot_car(state(1), state(2), state(3), state(4), car.length, car.width, 'm', 0.2);
                end
                break
            else
                if plotting_on
                    plot_car(state(1), state(2), state(3), state(4), car.length, car.width, 'y', 0.2);
                end
            end
        end
        valid_edges(waypoint_idx - 1) = valid;
        if ~valid
            break
        end
    end

end

function valid = collision_check(car, obstacles, state)

    car_rect = car_to_rect(car, state);
    valid = true;
    for obs = obstacles
        valid = ~aar_oar_intersection(to_points(obs), car_rect);
        if ~valid
            break;
        end
    end

end
