function points = to_points(rect)

rot = [cos(rect.theta), -sin(rect.theta);
       sin(rect.theta),  cos(rect.theta)];

he_x = rect.half_extents(1);
he_y = rect.half_extents(2);
   
p1 = rect.center + rot * [ he_x;  he_y];
p2 = rect.center + rot * [-he_x;  he_y];
p3 = rect.center + rot * [-he_x; -he_y];
p4 = rect.center + rot * [ he_x; -he_y];

points = [p1, p2, p3, p4];

end