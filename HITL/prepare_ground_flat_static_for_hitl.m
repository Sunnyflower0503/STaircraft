function [param, x_flat, u0, meta] = prepare_ground_flat_static_for_hitl(param, cfg)
%PREPARE_GROUND_FLAT_STATIC_FOR_HITL Prepare a nose-forward, level ground pose.

hitl_dir = fileparts(mfilename("fullpath"));
addpath(fullfile(hitl_dir, "utils"));
addpath(fullfile(fileparts(hitl_dir), "matlab_model"));

param.ground.enable = true;
u0 = zeros(12, 1);
euler_deg = [0; 0; cfg.init.heading_deg];
q0 = euler_to_quat_wxyz(euler_deg(1) * param.D2R, euler_deg(2) * param.D2R, euler_deg(3) * param.D2R);
R_eb0 = quat_to_rotm_local(q0);
contact_rel_e = R_eb0 * param.ground.contact_points_b;
touching = abs(contact_rel_e(3, :) - max(contact_rel_e(3, :))) < 1e-9;
touch_count = max(1, nnz(touching));
static_penetration = estimate_static_penetration(param, touch_count);
p0 = [0; 0; param.ground.z - max(contact_rel_e(3, :)) + static_penetration];
x_flat = [p0; zeros(3, 1); q0(:); zeros(3, 1)];

meta = struct();
meta.mode = "ground_flat_static";
meta.cache_used = false;
meta.cache_file = "";
meta.position_ned = x_flat(1:3);
meta.q_eb = x_flat(7:10);
meta.euler_deg = euler_deg;
meta.velocity_norm = 0;
meta.angular_rate_norm = 0;
meta.contact_z_rel = contact_rel_e(3, :);
meta.ground_z = param.ground.z;
meta.static_penetration = static_penetration;
meta.touch_count = touch_count;

if any(~isfinite(x_flat)) || any(~isfinite(u0))
    error("prepare_ground_flat_static_for_hitl:NonFinite", "Flat ground state or input contains NaN/Inf.");
end
end

function penetration = estimate_static_penetration(param, touch_count)
ground_k = param.ground.k(:);
if isscalar(ground_k)
    k_total = ground_k(1) * touch_count;
else
    contact_rel_e = param.ground.contact_points_b;
    touching = abs(contact_rel_e(3, :) - max(contact_rel_e(3, :))) < 1e-9;
    k_total = sum(ground_k(touching));
end
g = 9.8;
if isfield(param, "g")
    g = param.g;
elseif isfield(param, "gravity")
    g = param.gravity;
end
penetration = param.m * g / max(k_total, eps);
penetration = min(max(penetration, 1e-4), 0.02);
end

function R = quat_to_rotm_local(q)
q = quat_normalize(q);
qw = q(1); qx = q(2); qy = q(3); qz = q(4);
R = [1 - 2*(qy^2 + qz^2), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw);
     2*(qx*qy + qz*qw), 1 - 2*(qx^2 + qz^2), 2*(qy*qz - qx*qw);
     2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx^2 + qy^2)];
end
