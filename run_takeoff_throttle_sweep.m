clear; clc;

repo_dir = fileparts(mfilename('fullpath'));
model_dir = fullfile(repo_dir, 'matlab_model');
addpath(model_dir);

param = init_param_zx();
param.ground.enable = true;

dt = 0.001;
settle_time = 20.0;
case_time = 10.0;
theta0 = 40 * param.D2R;
throttle_list = 0.05:0.05:1.0;
tau_motor = get_param_field(param, 'tau_motor', 0.15);
motor_a = exp(-dt / tau_motor);
contact_loss_hold = 0.05;

timestamp = datestr(now, 'yyyymmdd_HHMMSS');
out_dir = fullfile(repo_dir, 'result', ['takeoff_throttle_sweep_' timestamp]);
fig_dir = fullfile(out_dir, 'figures');
data_dir = fullfile(out_dir, 'data');
if ~exist(fig_dir, 'dir'), mkdir(fig_dir); end
if ~exist(data_dir, 'dir'), mkdir(data_dir); end

q0 = [cos(theta0 / 2); 0; sin(theta0 / 2); 0];
R_eb0 = quat2rotm(q0.');
r_fc = param.ground.contact_points_b(:, 2);
r_rear = mean(param.ground.contact_points_b(:, 4:6), 2);
z_fc_rel = R_eb0 * r_fc; z_fc_rel = z_fc_rel(3);
z_rear_rel = R_eb0 * r_rear; z_rear_rel = z_rear_rel(3);
stand_height = z_rear_rel - z_fc_rel;
stand_top_z = param.ground.z - stand_height;

p0 = [0; 0; param.ground.z - z_rear_rel];
x0 = [p0; zeros(3, 1); q0; zeros(3, 1)];
u_zero = zeros(12, 1);
stand_cfg = struct('enabled', true, 'r_b', r_fc, 'top_z', stand_top_z, ...
    'k', get_ground_scalar(param.ground.k, 1), ...
    'c', get_ground_scalar(param.ground.c, 1));

fprintf('Takeoff throttle sweep\n');
fprintf('Output: %s\n', out_dir);
fprintf('stand_height = %.9g m\n', stand_height);
fprintf('stand_top_z = %.9g m\n', stand_top_z);

assert(abs(stand_height - 0.1818358) < 5e-5, 'Unexpected stand height.');

settle = simulate_case(x0, u_zero, u_zero, settle_time, dt, param, stand_cfg, ...
    motor_a, contact_loss_hold, true, x0(1), false);
x_settle = settle.x_final;
settle_euler = quat_to_euler_zyx(x_settle(7:10));

fprintf('Settled state after %.1f s:\n', settle_time);
fprintf('  position = [%.9g %.9g %.9g]\n', x_settle(1:3));
fprintf('  Euler deg = [%.9g %.9g %.9g]\n', settle_euler * param.R2D);
fprintf('  velocity = [%.9g %.9g %.9g]\n', x_settle(4:6));
fprintf('  omega = [%.9g %.9g %.9g]\n', x_settle(11:13));

assert(norm(x_settle(4:6)) < 1e-3, 'Settling linear velocity is not near zero.');
assert(norm(x_settle(11:13)) < 1e-3, 'Settling angular velocity is not near zero.');
assert(abs(settle_euler(2) - theta0) < 3 * param.D2R, 'Settled pitch is not close to 40 deg.');

summary_rows = {};
case_results = cell(numel(throttle_list), 1);
min_liftoff_throttle = NaN;
min_sustained_throttle = NaN;

for i = 1:numel(throttle_list)
    throttle = throttle_list(i);
    u_cmd = zeros(12, 1);
    u_cmd(1:8) = throttle;
    result = simulate_case(x_settle, zeros(12, 1), u_cmd, case_time, dt, param, stand_cfg, ...
        motor_a, contact_loss_hold, true, x_settle(1), true);

    if strcmp(result.status, 'sustained_takeoff') && isnan(min_sustained_throttle)
        min_sustained_throttle = throttle;
    end
    if ~isnan(result.liftoff_time) && isnan(min_liftoff_throttle)
        min_liftoff_throttle = throttle;
    end

    if throttle < 0.4
        assert(abs(result.x_ned(end) - x_settle(1)) < 1e-4, ...
            'Ground x displacement should be near zero for throttle < 0.4.');
    end
    assert(~result.has_nonfinite, 'Case trajectory contains NaN or Inf.');

    case_results{i} = result;
    save(fullfile(data_dir, sprintf('case_throttle_%04.2f.mat', throttle)), 'result', 'throttle');
    plot_case(result, fig_dir, throttle);

    summary_rows(end + 1, :) = {throttle, string(result.status), ...
        result.liftoff_time, result.touchdown_time, result.max_height, ...
        result.final_pitch_deg, result.max_normal_force}; %#ok<SAGROW>

    fprintf('throttle %.2f: %-17s liftoff=%7.3f touchdown=%7.3f max_height=%.4f final_pitch=%.3f maxN=%.2f\n', ...
        throttle, result.status, result.liftoff_time, result.touchdown_time, ...
        result.max_height, result.final_pitch_deg, result.max_normal_force);
end

summary = cell2table(summary_rows, 'VariableNames', { ...
    'throttle', 'status', 'liftoff_time', 'touchdown_time', ...
    'max_height', 'final_pitch', 'max_normal_force'});
writetable(summary, fullfile(out_dir, 'summary.csv'));
save(fullfile(out_dir, 'config.mat'), 'param', 'dt', 'settle_time', 'case_time', ...
    'theta0', 'throttle_list', 'tau_motor', 'stand_height', 'stand_top_z', ...
    'x_settle', 'min_liftoff_throttle', 'min_sustained_throttle');
plot_overview(summary, fig_dir);

fprintf('\nMinimum liftoff throttle: %.3g\n', min_liftoff_throttle);
fprintf('Minimum sustained takeoff throttle: %.3g\n', min_sustained_throttle);
fprintf('Saved summary: %s\n', fullfile(out_dir, 'summary.csv'));

function result = simulate_case(x0, u_actual0, u_command, tf, dt, param, stand_cfg, ...
    motor_a, contact_loss_hold, constrain_x, x_lock, use_events)
num_steps = round(tf / dt);
sample_stride = 10;
max_samples = floor(num_steps / sample_stride) + 1;

x = x0(:);
u_actual = u_actual0(:);
stand_enabled = stand_cfg.enabled;
loss_time = 0;
liftoff_time = NaN;
touchdown_time = NaN;
status = "no_takeoff";

time = zeros(max_samples, 1);
u_cmd_hist = zeros(max_samples, 1);
u_act_hist = zeros(max_samples, 1);
x_hist = zeros(max_samples, 1);
height_hist = zeros(max_samples, 1);
vel_hist = zeros(max_samples, 3);
euler_hist = zeros(max_samples, 3);
normal_hist = zeros(max_samples, 7);
contact_hist = false(max_samples, 7);
sample_idx = 1;

[perm_info, stand_info] = contact_info(x, param, stand_cfg, stand_enabled);
write_sample();

max_height = -inf;
max_normal_force = 0;
has_nonfinite = false;

for k = 1:num_steps
    t = (k - 1) * dt;
    u_actual = motor_a * u_actual + (1 - motor_a) * u_command;
    dyn = @(tt, xx) dynamics_with_stand(tt, xx, u_actual, param, stand_cfg, stand_enabled);
    [~, z_step] = Runge_Kutta4(dyn, [t, t + dt], x);
    x = z_step(:, end);
    x(7:10) = x(7:10) / norm(x(7:10));

    [perm_info, stand_info] = contact_info(x, param, stand_cfg, stand_enabled);
    any_contact = any(perm_info.active) || stand_info.active;
    if constrain_x && any_contact
        x(1) = x_lock;
        x(4) = 0;
    end

    [perm_info, stand_info] = contact_info(x, param, stand_cfg, stand_enabled);
    any_contact = any(perm_info.active) || stand_info.active;
    normal_total = sum(perm_info.normal_force) + stand_info.normal_force;
    max_normal_force = max(max_normal_force, normal_total);
    max_height = max(max_height, ground_clearance(x, param));

    if use_events
        if isnan(liftoff_time)
            if any_contact
                loss_time = 0;
            else
                loss_time = loss_time + dt;
                if loss_time >= contact_loss_hold
                    liftoff_time = t + dt - contact_loss_hold;
                    stand_enabled = false;
                    status = "liftoff";
                end
            end
        else
            stand_enabled = false;
            [perm_info, stand_info] = contact_info(x, param, stand_cfg, stand_enabled);
            if any(perm_info.active)
                touchdown_time = t + dt;
                status = "touchdown";
                if mod(k, sample_stride) ~= 0
                    write_sample();
                end
                break;
            end
        end
    end

    has_nonfinite = has_nonfinite || any(~isfinite(x)) || any(~isfinite(u_actual));
    if mod(k, sample_stride) == 0 || k == num_steps
        write_sample();
    end
end

if use_events && ~isnan(liftoff_time) && isnan(touchdown_time)
    status = "sustained_takeoff";
end

num_written = sample_idx - 1;
time = time(1:num_written);
u_cmd_hist = u_cmd_hist(1:num_written);
u_act_hist = u_act_hist(1:num_written);
x_hist = x_hist(1:num_written);
height_hist = height_hist(1:num_written);
vel_hist = vel_hist(1:num_written, :);
euler_hist = euler_hist(1:num_written, :);
normal_hist = normal_hist(1:num_written, :);
contact_hist = contact_hist(1:num_written, :);

result = struct();
result.status = char(status);
result.liftoff_time = liftoff_time;
result.touchdown_time = touchdown_time;
result.max_height = max_height;
result.final_pitch_deg = quat_to_euler_zyx(x(7:10)).' * [0; param.R2D; 0];
result.max_normal_force = max_normal_force;
result.has_nonfinite = has_nonfinite;
result.x_final = x;
result.t = time;
result.u_command = u_cmd_hist;
result.u_actual = u_act_hist;
result.x_ned = x_hist;
result.height = height_hist;
result.velocity = vel_hist;
result.euler_deg = euler_hist;
result.normal_forces = normal_hist;
result.contact_active = contact_hist;

    function write_sample()
        if sample_idx > numel(time)
            return;
        end
        q = x(7:10) / norm(x(7:10));
        eul = quat_to_euler_zyx(q) * param.R2D;
        [pii, sii] = contact_info(x, param, stand_cfg, stand_enabled);
        time(sample_idx) = min((k_exists() - 1) * dt, tf);
        u_cmd_hist(sample_idx) = mean(u_command(1:8));
        u_act_hist(sample_idx) = mean(u_actual(1:8));
        x_hist(sample_idx) = x(1);
        height_hist(sample_idx) = ground_clearance(x, param);
        vel_hist(sample_idx, :) = x(4:6).';
        euler_hist(sample_idx, :) = eul.';
        normal_hist(sample_idx, :) = [pii.normal_force, sii.normal_force];
        contact_hist(sample_idx, :) = [pii.active, sii.active];
        sample_idx = sample_idx + 1;
    end

    function kk = k_exists()
        if exist('k', 'var')
            kk = k;
        else
            kk = 1;
        end
    end
end

function dx = dynamics_with_stand(t, x, u, param, stand_cfg, stand_enabled)
dx = tandem_zx_dynamics(t, x, u, param);
if ~stand_enabled
    return;
end
q = x(7:10) / norm(x(7:10));
R_eb = quat2rotm(q.');
[f_stand_b, m_stand_b] = stand_contact_force(x(1:3), x(4:6), R_eb, x(11:13), stand_cfg);
dx(4:6) = dx(4:6) + R_eb * f_stand_b / param.m;
dx(11:13) = dx(11:13) + param.J \ m_stand_b;
end

function [f_b, m_b, info] = stand_contact_force(p_e, v_e, R_eb, omega_b, stand_cfg)
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
info = struct('active', normal > 0, 'normal_force', normal, ...
    'penetration', max(penetration, 0), 'contact_pos_e', contact_pos_e);
end

function [perm_info, stand_info] = contact_info(x, param, stand_cfg, stand_enabled)
q = x(7:10) / norm(x(7:10));
R_eb = quat2rotm(q.');
[~, ~, perm_info] = zx_ground_contact_force(x(1:3), x(4:6), R_eb, x(11:13), param);
if stand_enabled
    [~, ~, stand_info] = stand_contact_force(x(1:3), x(4:6), R_eb, x(11:13), stand_cfg);
else
    stand_info = struct('active', false, 'normal_force', 0, 'penetration', 0);
end
end

function clearance = ground_clearance(x, param)
q = x(7:10) / norm(x(7:10));
R_eb = quat2rotm(q.');
contact_pos = x(1:3) + R_eb * param.ground.contact_points_b;
clearance = param.ground.z - max(contact_pos(3, :));
end

function euler = quat_to_euler_zyx(q)
q = q(:) / norm(q);
R = quat2rotm(q.');
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

function value = get_param_field(s, name, default_value)
if isstruct(s) && isfield(s, name)
    value = s.(name);
else
    value = default_value;
end
end

function plot_case(result, fig_dir, throttle)
fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1200 900]);
t = result.t;
subplot(3, 2, 1); plot(t, result.u_command, '--', t, result.u_actual, '-'); grid on;
ylabel('Throttle'); legend('command', 'actual');
subplot(3, 2, 2); plot(t, [result.x_ned, result.height]); grid on;
ylabel('Position/height (m)'); legend('NED x', 'clearance');
subplot(3, 2, 3); plot(t, result.velocity); grid on;
ylabel('Velocity (m/s)'); legend('v_N', 'v_E', 'v_D');
subplot(3, 2, 4); plot(t, result.euler_deg); grid on;
ylabel('Euler (deg)'); legend('roll', 'pitch', 'yaw');
subplot(3, 2, 5); plot(t, result.normal_forces); grid on;
ylabel('Normal force (N)'); xlabel('Time (s)');
legend('p1','p2','p3','p4','p5','p6','stand');
subplot(3, 2, 6); stairs(t, result.contact_active); grid on;
ylabel('Contact active'); xlabel('Time (s)');
title(sprintf('Throttle %.2f: %s', throttle, result.status));
saveas(fig, fullfile(fig_dir, sprintf('case_throttle_%04.2f.png', throttle)));
close(fig);
end

function plot_overview(summary, fig_dir)
fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1000 800]);
status_code = zeros(height(summary), 1);
for i = 1:height(summary)
    if summary.status(i) == "sustained_takeoff"
        status_code(i) = 2;
    elseif summary.status(i) == "touchdown"
        status_code(i) = 1;
    else
        status_code(i) = 0;
    end
end
subplot(3, 1, 1); stem(summary.throttle, status_code, 'filled'); grid on;
ylabel('status'); yticks([0 1 2]); yticklabels({'no','touch','sustain'});
subplot(3, 1, 2); plot(summary.throttle, summary.liftoff_time, '-o'); grid on;
ylabel('liftoff time (s)');
subplot(3, 1, 3); plot(summary.throttle, summary.max_height, '-o'); grid on;
ylabel('max clearance (m)'); xlabel('Throttle');
saveas(fig, fullfile(fig_dir, 'overview.png'));
close(fig);
end
