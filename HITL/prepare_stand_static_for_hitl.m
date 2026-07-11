function [param, x_stand, u0, meta] = prepare_stand_static_for_hitl(param, cfg)
%PREPARE_STAND_STATIC_FOR_HITL Prepare the validated 40 deg stand-static state.
% The stand geometry and settling logic are adapted from run_takeoff_throttle_sweep.m.

hitl_dir = fileparts(mfilename("fullpath"));
addpath(fullfile(hitl_dir, "utils"));
addpath(fullfile(fileparts(hitl_dir), "matlab_model"));

cache_file = cfg.stand.cache_file;
u0 = zeros(12, 1);
if cfg.stand.use_cached_settled_state && isfile(cache_file)
    s = load(cache_file, "param", "x_stand", "u0", "meta");
    param = s.param;
    x_stand = s.x_stand;
    u0 = s.u0;
    meta = s.meta;
    meta.cache_used = true;
    validate_stand_state(x_stand, u0, meta);
    return;
end

param.ground.enable = true;
dt = cfg.dt;
settle_time = cfg.stand.settle_time_s;
theta0 = cfg.stand.angle_deg * param.D2R;

q0 = [cos(theta0 / 2); 0; sin(theta0 / 2); 0];
R_eb0 = quat_to_rotm_local(q0);
r_fc = param.ground.contact_points_b(:, 2);
r_rear = mean(param.ground.contact_points_b(:, 4:6), 2);
z_fc_rel = R_eb0 * r_fc; z_fc_rel = z_fc_rel(3);
z_rear_rel = R_eb0 * r_rear; z_rear_rel = z_rear_rel(3);
stand_height = z_rear_rel - z_fc_rel;
stand_top_z = param.ground.z - stand_height;

p0 = [0; 0; param.ground.z - z_rear_rel];
x0 = [p0; zeros(3, 1); q0; zeros(3, 1)];
stand_cfg = struct("enabled", true, "r_b", r_fc, "top_z", stand_top_z, ...
    "k", get_ground_scalar(param.ground.k, 1), ...
    "c", get_ground_scalar(param.ground.c, 1));

t = 0:dt:settle_time;
[~, z] = Runge_Kutta4(@(tt, xx) dynamics_with_stand(tt, xx, u0, param, stand_cfg), t, x0);
x_stand = z(:, end);
x_stand(7:10) = quat_normalize(x_stand(7:10));
euler_deg = quat_to_euler_zyx_local(x_stand(7:10)) * param.R2D;

meta = struct();
meta.mode = "stand_static";
meta.cache_used = false;
meta.cache_file = cache_file;
meta.position_ned = x_stand(1:3);
meta.euler_deg = euler_deg;
meta.velocity_norm = norm(x_stand(4:6));
meta.angular_rate_norm = norm(x_stand(11:13));
meta.stand_angle_deg = cfg.stand.angle_deg;
meta.settle_time_s = settle_time;
meta.dt = dt;
meta.stand_height = stand_height;
meta.stand_top_z = stand_top_z;

validate_stand_state(x_stand, u0, meta);

cache_dir = fileparts(cache_file);
if ~exist(cache_dir, "dir")
    mkdir(cache_dir);
end
save(cache_file, "param", "x_stand", "u0", "meta");
end

function dx = dynamics_with_stand(t, x, u, param, stand_cfg)
dx = tandem_zx_dynamics(t, x, u, param);
q = quat_normalize(x(7:10));
R_eb = quat_to_rotm_local(q);
[f_stand_b, m_stand_b] = stand_contact_force(x(1:3), x(4:6), R_eb, x(11:13), stand_cfg);
dx(4:6) = dx(4:6) + R_eb * f_stand_b / param.m;
dx(11:13) = dx(11:13) + param.J \ m_stand_b;
end

function [f_b, m_b] = stand_contact_force(p_e, v_e, R_eb, omega_b, stand_cfg)
R_be = R_eb';
r_b = stand_cfg.r_b;
contact_pos_e = p_e + R_eb * r_b;
v_contact_b = R_be * v_e + cross(omega_b, r_b);
v_contact_e = R_eb * v_contact_b;
penetration = contact_pos_e(3) - stand_cfg.top_z;
normal = 0;
force_e = zeros(3, 1);
if penetration > 0
    normal = max(0, stand_cfg.k * penetration + stand_cfg.c * v_contact_e(3));
    force_e(3) = -normal;
end
f_b = R_be * force_e;
m_b = cross(r_b, f_b);
end

function validate_stand_state(x_stand, u0, meta)
if ~isequal(size(x_stand), [13, 1])
    error("prepare_stand_static_for_hitl:BadStateSize", "x_stand must be 13x1.");
end
if ~isequal(size(u0), [12, 1])
    error("prepare_stand_static_for_hitl:BadInputSize", "u0 must be 12x1.");
end
if any(~isfinite(x_stand)) || any(~isfinite(u0))
    error("prepare_stand_static_for_hitl:NonFinite", "Stand state or input contains NaN/Inf.");
end
if meta.velocity_norm > 1e-3
    error("prepare_stand_static_for_hitl:VelocityNotSettled", "Stand linear velocity norm is %.3g.", meta.velocity_norm);
end
if meta.angular_rate_norm > 1e-3
    error("prepare_stand_static_for_hitl:RateNotSettled", "Stand angular-rate norm is %.3g.", meta.angular_rate_norm);
end
if meta.euler_deg(2) <= 20
    error("prepare_stand_static_for_hitl:PitchTooSmall", "Stand pitch %.3f deg is not above 20 deg.", meta.euler_deg(2));
end
end

function R = quat_to_rotm_local(q)
q = quat_normalize(q);
qw = q(1); qx = q(2); qy = q(3); qz = q(4);
R = [1 - 2*(qy^2 + qz^2), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw);
     2*(qx*qy + qz*qw), 1 - 2*(qx^2 + qz^2), 2*(qy*qz - qx*qw);
     2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx^2 + qy^2)];
end

function euler = quat_to_euler_zyx_local(q)
R = quat_to_rotm_local(q);
pitch = asin(-R(3, 1));
roll = atan2(R(3, 2), R(3, 3));
yaw = atan2(R(2, 1), R(1, 1));
euler = [roll; pitch; yaw];
end

function value = get_ground_scalar(value, idx)
value = value(:);
if isscalar(value)
    value = value(1);
else
    value = value(idx);
end
end
