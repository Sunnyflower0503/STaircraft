function DCM_be = quat_to_dcm_be(q)
%QUAT_TO_DCM_BE Convert body-to-earth quaternion to earth-to-body DCM.
q = quat_normalize(q);
w = q(1); x = q(2); y = q(3); z = q(4);
R_eb = [1 - 2*(y*y + z*z), 2*(x*y - z*w),     2*(x*z + y*w);
        2*(x*y + z*w),     1 - 2*(x*x + z*z), 2*(y*z - x*w);
        2*(x*z - y*w),     2*(y*z + x*w),     1 - 2*(x*x + y*y)];
DCM_be = R_eb.';
end
