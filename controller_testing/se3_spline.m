function points = se3_spline(q0, q1, steps, dscale)

% http://www.cs.cornell.edu/courses/cs4620/2013fa/lectures/16spline-curves.pdf

p0 = q0(1:2)';
p1 = q1(1:2)';
t0 = [cos(q0(3)), sin(q0(3))] * dscale;
t1 = [cos(q1(3)), sin(q1(3))] * dscale;
f = @(s) [s^3 s^2 s 1] * [2 -2 1 1; -3 3 -2 -1; 0 0 1 0; 1 0 0 0] * [p0; p1; t0; t1];
df = @(s) p1*(- 6*s^2 + 6*s) - p0*(- 6*s^2 + 6*s) - t1*(- 3*s^2 + 2*s) + t0*(3*s^2 - 4*s + 1);

points = zeros(3, steps);
s = 0:1/steps:1;
for idx = 1:length(s)
    points(1:2, idx) = f(s(idx));
    dxdy = df(s(idx));
    points(3, idx) = atan2(dxdy(2), dxdy(1));
end

end