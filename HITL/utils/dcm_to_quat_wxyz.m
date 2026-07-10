function q = dcm_to_quat_wxyz(DCM_be)
%DCM_TO_QUAT_WXYZ Convert earth-to-body DCM to body-to-earth [w x y z].
R = DCM_be.';
tr = trace(R);
if tr > 0
    s = sqrt(tr + 1.0) * 2;
    qw = 0.25 * s;
    qx = (R(3,2) - R(2,3)) / s;
    qy = (R(1,3) - R(3,1)) / s;
    qz = (R(2,1) - R(1,2)) / s;
elseif R(1,1) > R(2,2) && R(1,1) > R(3,3)
    s = sqrt(1.0 + R(1,1) - R(2,2) - R(3,3)) * 2;
    qw = (R(3,2) - R(2,3)) / s;
    qx = 0.25 * s;
    qy = (R(1,2) + R(2,1)) / s;
    qz = (R(1,3) + R(3,1)) / s;
elseif R(2,2) > R(3,3)
    s = sqrt(1.0 + R(2,2) - R(1,1) - R(3,3)) * 2;
    qw = (R(1,3) - R(3,1)) / s;
    qx = (R(1,2) + R(2,1)) / s;
    qy = 0.25 * s;
    qz = (R(2,3) + R(3,2)) / s;
else
    s = sqrt(1.0 + R(3,3) - R(1,1) - R(2,2)) * 2;
    qw = (R(2,1) - R(1,2)) / s;
    qx = (R(1,3) + R(3,1)) / s;
    qy = (R(2,3) + R(3,2)) / s;
    qz = 0.25 * s;
end
q = quat_normalize([qw; qx; qy; qz]);
end
