function h = plot_obstacle(obstacle)
% obstacle is an axis aligned rectangular patch, specifed by 2 points

bl = obstacle.bl;
br = [obstacle.tr(1), obstacle.bl(2)];
tr = obstacle.tr;
tl = [obstacle.bl(1), obstacle.tr(2)];

h = patch([bl(1) br(1) tr(1) tl(1)], [bl(2) br(2) tr(2) tl(2)], 'k', 'FaceAlpha', 0.2, 'LineStyle', ':');

end