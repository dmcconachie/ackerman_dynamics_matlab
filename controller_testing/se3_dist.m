function dist = se3_dist(q1, q2, lambda)

if nargin < 3
    lambda = 1;
end

delta = se3_delta(q1, q2);
dist = sqrt(delta(1:2)' * delta(1:2) + lambda * delta(3) * delta(3));

end