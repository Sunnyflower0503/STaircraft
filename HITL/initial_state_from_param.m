function x0 = initial_state_from_param(param, cfg)
%INITIAL_STATE_FROM_PARAM Build the 13-state initial condition.

p_e = [0; 0; -cfg.init.AMSL];
v_b = zeros(3, 1);
euler = zeros(3, 1);
pqr = zeros(3, 1);

if isfield(param, "InitData")
    init = param.InitData;
    if isfield(init, "Xe"), p_e = init.Xe(:); end
    if isfield(init, "Vb"), v_b = init.Vb(:); end
    if isfield(init, "Euler"), euler = init.Euler(:); end
    if isfield(init, "pqr"), pqr = init.pqr(:); end
end

q_eb = eul_zyx_to_quat_wxyz(euler(3), euler(2), euler(1));
R_eb = quat_to_rotm_body_to_earth(q_eb);
v_e = R_eb * v_b;
x0 = [p_e; v_e; q_eb; pqr];
end

function q = eul_zyx_to_quat_wxyz(yaw, pitch, roll)
cy = cos(yaw * 0.5); sy = sin(yaw * 0.5);
cp = cos(pitch * 0.5); sp = sin(pitch * 0.5);
cr = cos(roll * 0.5); sr = sin(roll * 0.5);
q = [cr*cp*cy + sr*sp*sy;
     sr*cp*cy - cr*sp*sy;
     cr*sp*cy + sr*cp*sy;
     cr*cp*sy - sr*sp*cy];
q = quat_normalize(q);
end

function R = quat_to_rotm_body_to_earth(q)
R = quat_to_dcm_be(q).';
end
