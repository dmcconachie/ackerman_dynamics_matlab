function parsave(trajectory_file, ...
    obstacles, waypoints, waypoint_traj_indices, ...
    x_trajectory, u_trajectory, y_trajectory)

save(trajectory_file, ...
    'obstacles', 'waypoints', 'waypoint_traj_indices', ...
    'x_trajectory', 'u_trajectory', 'y_trajectory');

end