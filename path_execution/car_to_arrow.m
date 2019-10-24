function points = car_to_arrow(car, state, head_spread, head_length)
    
    rot = [cos(state(3)), -sin(state(3));
           sin(state(3)),  cos(state(3))];
    arrow_start = state(1:2);
    arrow_tip = state(1:2) + rot * [car.length; 0];

    phi = state(3) + pi - head_spread;
    rot = [cos(phi), -sin(phi);
           sin(phi),  cos(phi)];
    arrow_left = arrow_tip + rot * [head_length; 0];

    phi = state(3) + pi + head_spread;
    rot = [cos(phi), -sin(phi);
           sin(phi),  cos(phi)];
    arrow_right = arrow_tip + rot * [head_length; 0];

    points = [arrow_start, arrow_tip, arrow_left, arrow_right];

end