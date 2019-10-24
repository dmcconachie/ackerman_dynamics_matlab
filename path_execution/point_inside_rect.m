function inside = point_inside_rect(rect, point)

    rect_points = to_points(rect);
    rect_area = rect.half_extents(1) * rect.half_extents(2) * 4;
    a1 = triangle_area(rect_points(:, 1), rect_points(:, 2), point);
    a2 = triangle_area(rect_points(:, 2), rect_points(:, 3), point);
    a3 = triangle_area(rect_points(:, 3), rect_points(:, 4), point);
    a4 = triangle_area(rect_points(:, 4), rect_points(:, 1), point);
    
    inside = a1 + a2 + a3 + a4 <= rect_area * 1.00001;

end