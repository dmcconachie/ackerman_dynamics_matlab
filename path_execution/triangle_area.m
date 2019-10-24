function area = triangle_area(p1, p2, p3)

    area = abs((p1(1) * (p2(2) - p3(2)) + ...
                p2(1) * (p3(2) - p1(2)) + ...
                p3(1) * (p1(2) - p2(2))) / 2.0);

end