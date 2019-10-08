function intersect = aar_oar_intersection(aar_points, oa_rect)

% Iterate through all lines that define the rectangles and look for a
% separating line
% https://stackoverflow.com/questions/10962379/how-to-check-intersection-between-2-rotated-rectangles

% Or just look for overlap, because we have rectangles, not just polygons
% https://github.com/gszauer/GamePhysicsCookbook/blob/master/Code/Geometry2D.cpp

oar_axis1 = [cos(oa_rect.theta); sin(oa_rect.theta)];
oar_axis2 = [-oar_axis1(2); oar_axis1(1)];

axes_to_test = [[1; 0], [0; 1], oar_axis1, oar_axis2];

aar_projections = axes_to_test' * aar_points;
aar_interval_min = min(aar_projections, [], 2);
aar_interval_max = max(aar_projections, [], 2);

oar_points = to_points(oa_rect);
oar_projections = axes_to_test' * oar_points;
oar_interval_min = min(oar_projections, [], 2);
oar_interval_max = max(oar_projections, [], 2);

separating_axis = aar_interval_max < oar_interval_min | ...
                  oar_interval_max < aar_interval_min;

intersect = ~any(separating_axis);
% if intersect
%     clf;
%     patch(aar_points(1, :), aar_points(2, :), 'k', 'FaceAlpha', 0.2, 'LineStyle', ':');
%     patch(oar_points(1, :), oar_points(2, :), 'm', 'FaceAlpha', 0.2, 'LineStyle', ':');
%     axis square
%     drawnow;
%     pause
% end

end