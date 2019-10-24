function inside = point_inside_triangle(triangle_points, point)

    a_tri = triangle_area(triangle_points(:, 1), triangle_points(:, 2), triangle_points(:, 3));
    a1 = triangle_area(triangle_points(:, 1), triangle_points(:, 2), point);
    a2 = triangle_area(triangle_points(:, 2), triangle_points(:, 3), point);
    a3 = triangle_area(triangle_points(:, 3), triangle_points(:, 1), point);
    
    inside = a1 + a2 + a3 <= a_tri * 1.00001;

end