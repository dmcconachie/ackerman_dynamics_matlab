function plot_car(x, y, theta, phi, l, w, color)

if nargin < 7
    color = 'y';
end

scatter(x, y, ['+', color]);

wheel_len = 0.25 * l;
offset_w = 0.8 * w;
offset_l_rear = 0.3 * l;
offset_l_front = 1.3 * l;

rot = [cos(theta), -sin(theta);
       sin(theta),  cos(theta)];

lf = [x; y] + rot * [ offset_l_front;  offset_w];
rf = [x; y] + rot * [ offset_l_front; -offset_w];
lr = [x; y] + rot * [-offset_l_rear;   offset_w];
rr = [x; y] + rot * [-offset_l_rear;  -offset_w];

patch([lf(1) rf(1) rr(1) lr(1)], [lf(2) rf(2) rr(2) lr(2)], color, 'FaceAlpha', 0.2, 'LineStyle', ':');
line([lf(1) rf(1)], [lf(2) rf(2)], 'color', 'r');

offset_w = 0.5 * w;
offset_l_rear = 0;
offset_l_front = l;

wheel_rear_l = [x; y] + rot * [ offset_l_rear;  offset_w];
wheel_rear_r = [x; y] + rot * [ offset_l_rear; -offset_w];

wheel_rear_ls = wheel_rear_l + rot * [-wheel_len/2; 0];
wheel_rear_le = wheel_rear_l + rot * [ wheel_len/2; 0];

wheel_rear_rs = wheel_rear_r + rot * [-wheel_len/2; 0];
wheel_rear_re = wheel_rear_r + rot * [ wheel_len/2; 0];

line([wheel_rear_ls(1), wheel_rear_le(1)], [wheel_rear_ls(2), wheel_rear_le(2)], 'color', 'k');
line([wheel_rear_rs(1), wheel_rear_re(1)], [wheel_rear_rs(2), wheel_rear_re(2)], 'color', 'k');

wheel_front_l = [x; y] + rot * [ offset_l_front;  offset_w];
wheel_front_r = [x; y] + rot * [ offset_l_front; -offset_w];

rot_phi = [cos(phi), -sin(phi);
           sin(phi),  cos(phi)];

wheel_front_ls = wheel_front_l + rot * rot_phi * [-wheel_len/2; 0];
wheel_front_le = wheel_front_l + rot * rot_phi * [ wheel_len/2; 0];

wheel_front_rs = wheel_front_r + rot * rot_phi * [-wheel_len/2; 0];
wheel_front_re = wheel_front_r + rot * rot_phi * [ wheel_len/2; 0];

line([wheel_front_ls(1), wheel_front_le(1)], [wheel_front_ls(2), wheel_front_le(2)], 'color', 'k');
line([wheel_front_rs(1), wheel_front_re(1)], [wheel_front_rs(2), wheel_front_re(2)], 'color', 'k');

end