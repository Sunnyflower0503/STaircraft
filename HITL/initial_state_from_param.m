function x0 = initial_state_from_param(param, cfg)
%INITIAL_STATE_FROM_PARAM Build the 13-state initial condition.
% Default HITL geodetic origin is cfg.init; p_e starts at local NED zero.

p_e = [0; 0; 0];
v_b = zeros(3, 1);
euler = [0; 0; deg2rad(cfg.init.heading_deg)];
pqr = zeros(3, 1);

if isfield(param, "InitData")
    init = param.InitData;
    % Keep optional initial velocity/rates from the plant parameter file, but
    % keep HITL local position and heading tied to cfg.init by default.
    if isfield(init, "Vb"), v_b = init.Vb(:); end
    if isfield(init, "pqr"), pqr = init.pqr(:); end
end

q_eb = euler_to_quat_wxyz(euler(1), euler(2), euler(3));
R_eb = quat_to_rotm_body_to_earth(q_eb);
v_e = R_eb * v_b;
x0 = [p_e; v_e; q_eb; pqr];
end

function R = quat_to_rotm_body_to_earth(q)
R = quat_to_dcm_be(q).';
end

