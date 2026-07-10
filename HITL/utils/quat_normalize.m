function q = quat_normalize(q)
%QUAT_NORMALIZE Normalize a [w x y z] quaternion column.
q = q(:);
n = norm(q);
if n <= eps
    q = [1; 0; 0; 0];
else
    q = q / n;
end
end
