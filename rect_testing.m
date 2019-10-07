clc;clear;
rect1.half_extents = [2; 1];
rect1.theta = 0;
rect1.center = [0; 0];

rect2.half_extents = [4; 1];
rect2.theta = pi/6;
rect2.center = [7.5; 2.5];

figure(1); clf;
hold on
plot_rectangle(rect1);
plot_rectangle(rect2);
axis equal

aar_oar_intersection(to_points(rect1), rect2)