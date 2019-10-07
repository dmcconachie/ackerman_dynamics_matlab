function intersect = aar_oar_intersection(aar_points, object_aligned_rect)

% Iterate through all lines that define the rectangles and look for a
% separating line
% https://stackoverflow.com/questions/10962379/how-to-check-intersection-between-2-rotated-rectangles

% Or just look for overlap, because we have rectangles, not just polygons
% https://github.com/gszauer/GamePhysicsCookbook/blob/master/Code/Geometry2D.cpp

oar_axis1 = [cos(object_aligned_rect.theta); -sin(object_aligned_rect.theta)];
oar_axis2 = [-oar_axis1(2); oar_axis1(1)];

axes_to_test = [[1; 0], [0; 1], oar_axis1, oar_axis2];

aar_projections = axes_to_test' * aar_points;
aar_interval_min = min(aar_projections, [], 2);
aar_interval_max = max(aar_projections, [], 2);

oar_projections = axes_to_test' * to_points(object_aligned_rect);
oar_interval_min = min(oar_projections, [], 2);
oar_interval_max = max(oar_projections, [], 2);

separating_axis = aar_interval_max < oar_interval_min | ...
                  oar_interval_max < aar_interval_min;

intersect = ~any(separating_axis);

end