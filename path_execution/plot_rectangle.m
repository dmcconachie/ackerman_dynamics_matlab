function h = plot_rectangle(rect)

points = to_points(rect);
h = patch(points(1, :), points(2, :), 'k', 'FaceAlpha', 0.2, 'LineStyle', ':');

end