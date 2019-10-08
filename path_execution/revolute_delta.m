function delta = revolute_delta(r1, r2)

delta = r2 - r1;
while delta < -pi
    delta = delta + 2 * pi;
end
while delta > pi
    delta = delta - 2 * pi;
end

end