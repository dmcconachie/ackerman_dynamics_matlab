function inside = point_inside_rect(rect, point)

    rect_points = to_points(rect);
    rect_area = rect.half_extents(1) * rect.half_extents(2) * 4;
    a1 = triangle_area(rect_points(:, 1), rect_points(:, 2), point);
    a2 = triangle_area(rect_points(:, 2), rect_points(:, 3), point);
    a3 = triangle_area(rect_points(:, 3), rect_points(:, 4), point);
    a4 = triangle_area(rect_points(:, 4), rect_points(:, 1), point);
    
    inside = a1 + a2 + a3 + a4 <= rect_area * 1.00001;

end

function area = triangle_area(p1, p2, p3)

    area = abs((p1(1) * (p2(2) - p3(2)) + ...
                p2(1) * (p3(2) - p1(2)) + ...
                p3(1) * (p1(2) - p2(2))) / 2.0);

end