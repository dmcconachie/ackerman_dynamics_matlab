function dist = se2_dist(q1, q2, lambda)

if nargin < 3
    lambda = 5;
end

delta = se2_delta(q1, q2);
dist = sqrt(delta(1:2)' * delta(1:2) + lambda * delta(3) * delta(3));

end