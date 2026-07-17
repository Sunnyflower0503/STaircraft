clear; clc;

repo_dir = fileparts(mfilename('fullpath'));
model_dir = fullfile(repo_dir, 'matlab_model');
addpath(model_dir);

param = init_param_zx();
param.ground.enable = true;

dt_list = [0.002, 0.001, 0.0005];
drop_height = 0.03;
t_end = 2.0;
u = zeros(12, 1);

[~, q_eb0, p_contact] = natural_ground_pose(param);
x0 = [p_contact - [0; 0; drop_height]; zeros(3, 1); q_eb0; zeros(3, 1)];

rows = {};
metrics_list = cell(numel(dt_list), 1);
for idt = 1:numel(dt_list)
    dt = dt_list(idt);
    t = 0:dt:t_end;
    [~, z] = Runge_Kutta4(@(tt, xx) tandem_zx_dynamics(tt, xx, u, param), t, x0);
    metrics = ground_drop_metrics(z, t, param);
    metrics.dt = dt;
    metrics_list{idt} = metrics;
    rows(end + 1, :) = metric_row(metrics); %#ok<SAGROW>
end

ref = metrics_list{end};
for idt = 1:numel(dt_list)-1
    metrics_list{idt}.rel_max_penetration = rel_diff(metrics_list{idt}.max_penetration, ref.max_penetration);
    metrics_list{idt}.rel_max_total_normal = rel_diff(metrics_list{idt}.max_total_normal, ref.max_total_normal);
    metrics_list{idt}.rel_max_point_normal = rel_diff(metrics_list{idt}.max_point_normal, ref.max_point_normal);
    metrics_list{idt}.rel_rebound_height = rel_diff(metrics_list{idt}.first_rebound_height, ref.first_rebound_height);
end
metrics_list{end}.rel_max_penetration = 0;
metrics_list{end}.rel_max_total_normal = 0;
metrics_list{end}.rel_max_point_normal = 0;
metrics_list{end}.rel_rebound_height = 0;

rows = {};
for idt = 1:numel(dt_list)
    rows(end + 1, :) = metric_row(metrics_list{idt}); %#ok<SAGROW>
end

result = cell2table(rows, 'VariableNames', { ...
    'dt_s', 'max_penetration_m', 'max_total_normal_N', 'max_point_normal_N', ...
    'first_rebound_height_m', 'final_pitch_deg', ...
    'final_x_m', 'final_y_m', 'final_z_m', ...
    'final_vx_mps', 'final_vy_mps', 'final_vz_mps', ...
    'has_nonfinite', ...
    'rel_max_penetration', 'rel_max_total_normal', 'rel_max_point_normal', 'rel_rebound_height'});

out_csv = fullfile(repo_dir, 'ground_timestep_convergence.csv');
writetable(result, out_csv);

disp(result);
fprintf('Saved %s\n', out_csv);
fprintf('Ground timestep convergence test passed.\n');

assert(~any(result.has_nonfinite), 'Convergence run produced NaN or Inf.');
assert(result.rel_max_penetration(2) < 0.15, 'dt=0.001 penetration differs too much from dt=0.0005.');
assert(result.rel_max_total_normal(2) < 0.20, 'dt=0.001 total normal differs too much from dt=0.0005.');

function row = metric_row(m)
row = {m.dt, m.max_penetration, m.max_total_normal, m.max_point_normal, ...
    m.first_rebound_height, m.final_euler(2) * 180 / pi, ...
    m.final_position(1), m.final_position(2), m.final_position(3), ...
    m.final_velocity(1), m.final_velocity(2), m.final_velocity(3), ...
    m.has_nonfinite, ...
    getfield_default(m, 'rel_max_penetration', NaN), ...
    getfield_default(m, 'rel_max_total_normal', NaN), ...
    getfield_default(m, 'rel_max_point_normal', NaN), ...
    getfield_default(m, 'rel_rebound_height', NaN)};
end

function value = getfield_default(s, name, default_value)
if isfield(s, name)
    value = s.(name);
else
    value = default_value;
end
end

function d = rel_diff(value, ref)
d = abs(value - ref) / max(abs(ref), 1e-12);
end

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
    contact_z(i) = max(info.contact_pos_e(3, :));
    if isempty(first_contact_idx) && any(info.active)
        first_contact_idx = i;
    end
end

if isempty(first_contact_idx)
    first_rebound_height = NaN;
else
    first_rebound_height = max(param.ground.z - contact_z(first_contact_idx:end));
end

qf = z(7:10, end) / norm(z(7:10, end));
metrics = struct();
metrics.max_penetration = max_pen;
metrics.max_total_normal = max_total_normal;
metrics.max_point_normal = max_point_normal;
metrics.first_rebound_height = first_rebound_height;
metrics.final_euler = rotm_to_euler_zyx(quat_to_rotm_local(qf));
metrics.final_position = z(1:3, end);
metrics.final_velocity = z(4:6, end);
metrics.has_nonfinite = any(~isfinite(z(:)));
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
yaw = atan2(R(2, 1), R(1, 1));
euler = [roll; pitch; yaw];
end
