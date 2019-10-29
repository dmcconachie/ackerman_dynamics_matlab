function delta = se2_delta(q1, q2)

invrot = [cos(q1(3)), sin(q1(3)); -sin(q1(3)), cos(q1(3))];
delta = [invrot * (q2(1:2) - q1(1:2)); revolute_delta(q1(3), q2(3))];

end