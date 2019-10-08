function rect = car_to_rect(car, state)

    rect.theta = state(3);
    rot = [cos(rect.theta), -sin(rect.theta);
           sin(rect.theta),  cos(rect.theta)];
    rect.center = state(1:2) + rot * [car.length / 2; 0];
    rect.half_extents = [car.length; car.width] / 2 * 1.6;

end