clear; clc;

repo_dir = fileparts(mfilename('fullpath'));
model_dir = fullfile(repo_dir, 'matlab_model');
addpath(model_dir);

param = init_param_zx();
param.ground.enable = true;

dt = 0.001;
drop_height = 0.03;
t = 0:dt:2.5;
u = zeros(12, 1);

[theta_ground, q_eb0, p_contact] = natural_ground_pose(param);
x0 = [p_contact - [0; 0; drop_height]; zeros(3, 1); q_eb0; zeros(3, 1)];

[~, z] = Runge_Kutta4(@(tt, xx) tandem_zx_dynamics(tt, xx, u, param), t, x0);
metrics = ground_drop_metrics(z, t, param);

fprintf('Ground drop test passed.\n');
fprintf('theta_ground = %.9f deg\n', theta_ground * param.R2D);
fprintf('first contact time = %.9g s\n', metrics.first_contact_time);
fprintf('max penetration = %.9g m\n', metrics.max_penetration);
fprintf('max total normal = %.9g N\n', metrics.max_total_normal);
fprintf('max point normal = %.9g N\n', metrics.max_point_normal);
fprintf('first rebound height = %.9g m\n', metrics.first_rebound_height);
fprintf('final roll deg = %.9g\n', metrics.final_euler(1) * param.R2D);
fprintf('final pitch deg = %.9g\n', metrics.final_euler(2) * param.R2D);
fprintf('final velocity norm = %.9g m/s\n', norm(z(4:6, end)));
fprintf('final angular velocity norm = %.9g rad/s\n', norm(z(11:13, end)));

assert(metrics.first_contact_time > 0, 'Drop test did not detect first contact.');
assert(metrics.max_total_normal > 0, 'Ground never produced normal force.');
assert(metrics.max_penetration > 0 && metrics.max_penetration < 0.08, ...
    'Drop penetration is outside the expected range.');
assert(metrics.min_total_normal >= -1e-12, 'Ground model produced tensile normal force.');
assert(all(isfinite(z(:))), 'Drop trajectory contains non-finite values.');
assert(abs(metrics.final_euler(1)) < 2 * param.D2R, 'Symmetric drop created excessive roll.');
assert(metrics.final_energy < metrics.initial_energy, 'Damping should reduce mechanical energy.');

function [theta, q_eb, p_e] = natural_ground_pose(param)
points = param.ground.contact_points_b;
p = polyfit(points(1, :).', points(3, :).', 1);
theta = atan(p(1));
R_eb = pitch_dcm_be(theta);
contact_rel_e = R_eb * points;
p_e = [0; 0; param.ground.z - mean(contact_rel_e(3, :))];
q_eb = rotm_to_quat_local(R_eb);
end

function metrics = ground_drop_metrics(z, t, param)
num_samples = size(z, 2);
max_pen = 0;
max_total_normal = 0;
max_point_normal = 0;
min_total_normal = inf;
first_contact_idx = [];
contact_z = zeros(1, num_samples);

for i = 1:num_samples
    q = z(7:10, i) / norm(z(7:10, i));
    R_eb = quat_to_rotm_local(q);
    [~, ~, info] = zx_ground_contact_force(z(1:3, i), z(4:6, i), R_eb, z(11:13, i), param);
    total_normal = sum(info.normal_force);
    max_pen = max(max_pen, max(info.penetration));
    max_total_normal = max(max_total_normal, total_normal);
    max_point_normal = max(max_point_normal, max(info.normal_force));
    min_total_normal = min(min_total_normal, total_normal);
    contact_z(i) = max(info.contact_pos_e(3, :));
    if isempty(first_contact_idx) && any(info.active)
        first_contact_idx = i;
    end
end

if isempty(first_contact_idx)
    first_contact_time = NaN;
    first_rebound_height = NaN;
else
    first_contact_time = t(first_contact_idx);
    after = first_contact_idx:num_samples;
    clearance = param.ground.z - contact_z(after);
    first_rebound_height = max(clearance);
end

qf = z(7:10, end) / norm(z(7:10, end));
final_euler = rotm_to_euler_zyx(quat_to_rotm_local(qf));
metrics = struct();
metrics.first_contact_time = first_contact_time;
metrics.max_penetration = max_pen;
metrics.max_total_normal = max_total_normal;
metrics.max_point_normal = max_point_normal;
metrics.min_total_normal = min_total_normal;
metrics.first_rebound_height = first_rebound_height;
metrics.final_euler = final_euler;
metrics.initial_energy = energy_like(z(:, 1), param);
metrics.final_energy = energy_like(z(:, end), param);
end

function E = energy_like(x, param)
v = x(4:6);
w = x(11:13);
E = 0.5 * param.m * dot(v, v) + 0.5 * w.' * param.J * w - param.m * param.g * x(3);
end

function R = pitch_dcm_be(theta)
R = [cos(theta), 0, sin(theta); 0, 1, 0; -sin(theta), 0, cos(theta)];
end

function q = rotm_to_quat_local(R)
tr = trace(R);
s = sqrt(tr + 1) * 2;
q = [0.25 * s; (R(3, 2) - R(2, 3)) / s; (R(1, 3) - R(3, 1)) / s; (R(2, 1) - R(1, 2)) / s];
q = q / norm(q);
end

function R = quat_to_rotm_local(q)
qw = q(1); qx = q(2); qy = q(3); qz = q(4);
R = [1 - 2*(qy^2 + qz^2), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw);
     2*(qx*qy + qz*qw), 1 - 2*(qx^2 + qz^2), 2*(qy*qz - qx*qw);
     2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx^2 + qy^2)];
end

function euler = rotm_to_euler_zyx(R)
pitch = asin(-R(3, 1));
roll = atan2(R(3, 2), R(3, 3));
yaw = atan2(R(2, 1), R(1, 2)*0 + R(1, 1));
euler = [roll; pitch; yaw];
end
