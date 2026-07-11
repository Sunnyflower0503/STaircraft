function q = euler_to_quat_wxyz(roll, pitch, yaw)
%EULER_TO_QUAT_WXYZ Convert ZYX Euler angles to [qw; qx; qy; qz].
cy = cos(yaw * 0.5); sy = sin(yaw * 0.5);
cp = cos(pitch * 0.5); sp = sin(pitch * 0.5);
cr = cos(roll * 0.5); sr = sin(roll * 0.5);
q = [cr*cp*cy + sr*sp*sy;
     sr*cp*cy - cr*sp*sy;
     cr*sp*cy + sr*cp*sy;
     cr*cp*sy - sr*sp*cy];
q = quat_normalize(q);
end
