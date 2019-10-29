function delta = se2_delta(q1, q2)

delta = q2 - q1;
delta(3) = revolute_delta(q1(3), q2(3));

end