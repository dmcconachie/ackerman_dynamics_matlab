function points = car_to_triangle(car, state)
    
    rot = [cos(state(3)), -sin(state(3));
           sin(state(3)),  cos(state(3))];
    triangle_left =  state(1:2) + rot * [-0.3*car.length;  car.width * 0.8];
    triangle_right = state(1:2) + rot * [-0.3*car.length; -car.width * 0.8];
    triangle_tip =   state(1:2) + rot * [ 1.3*car.length; 0];

    points = [triangle_left, triangle_right, triangle_tip];

end