function db = distance_and_bearing(rect, point)

    delta = rect.center - point;
    offset = min(abs(delta), rect.half_extents);
    vector_to_rect = delta - sign(delta) .* offset;
    db = [norm(vector_to_rect); atan2(vector_to_rect(2), vector_to_rect(1))];

end