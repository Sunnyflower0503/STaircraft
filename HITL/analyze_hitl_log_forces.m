function out_dir = analyze_hitl_log_forces(log_file)
%ANALYZE_HITL_LOG_FORCES Plot force, PWM, position, and attitude from HITL MAT logs.
%
% Usage:
%   analyze_hitl_log_forces
%   analyze_hitl_log_forces('D:/.../HITL/logs/run_hitl_stand_takeoff_yyyymmdd_HHMMSS.mat')

if nargin < 1 || strlength(string(log_file)) == 0
    log_file = latest_hitl_log();
end
log_file = char(log_file);

hitl_dir = fileparts(mfilename("fullpath"));
root_dir = fileparts(hitl_dir);
addpath(hitl_dir);
addpath(fullfile(hitl_dir, "utils"));
addpath(fullfile(root_dir, "matlab_model"));

s = load(log_file, "stats");
if ~isfield(s, "stats")
    error("analyze_hitl_log_forces:MissingStats", "MAT file does not contain variable 'stats'.");
end
stats = s.stats;
history = get_history(stats);

timestamp = string(datetime("now", "Format", "yyyyMMdd_HHmmSS"));
[~, log_name] = fileparts(log_file);
out_dir = fullfile(root_dir, "result", "hitl_log_analysis_" + log_name + "_" + timestamp);
fig_dir = fullfile(out_dir, "figures");
data_dir = fullfile(out_dir, "data");
if ~exist(fig_dir, "dir"), mkdir(fig_dir); end
if ~exist(data_dir, "dir"), mkdir(data_dir); end

param = get_param_snapshot(stats);
cfg = get_cfg_snapshot(stats);
meta = get_meta_snapshot(stats);

diag = compute_force_history(history, param, meta);
summary = build_summary_table(history, diag);

writetable(summary, fullfile(out_dir, "summary_timeseries.csv"));
writetable(summary, fullfile(data_dir, "summary_timeseries.csv"));
save(fullfile(out_dir, "analysis_data.mat"), "stats", "history", "diag", "param", "cfg", "meta", "log_file");
save(fullfile(data_dir, "analysis_data.mat"), "stats", "history", "diag", "param", "cfg", "meta", "log_file");

plot_pwm(history, fig_dir);
plot_position_velocity(history, diag, fig_dir);
plot_attitude(history, diag, fig_dir);
plot_forces(history, diag, fig_dir);
plot_moments(history, diag, fig_dir);
plot_force_norms(history, diag, fig_dir);

fprintf("HITL log analysis complete.\n");
fprintf("  log_file: %s\n", log_file);
fprintf("  out_dir : %s\n", out_dir);
fprintf("  samples : %d\n", numel(history.t));
if ~isempty(history.t)
    fprintf("  time    : %.3f to %.3f s\n", history.t(1), history.t(end));
end
end

function log_file = latest_hitl_log()
hitl_dir = fileparts(mfilename("fullpath"));
logs = dir(fullfile(hitl_dir, "logs", "*.mat"));
if isempty(logs)
    error("analyze_hitl_log_forces:NoLogs", "No HITL MAT logs found under %s.", fullfile(hitl_dir, "logs"));
end
[~, idx] = max([logs.datenum]);
log_file = fullfile(logs(idx).folder, logs(idx).name);
end

function history = get_history(stats)
if ~isfield(stats, "history") || isempty(stats.history)
    error("analyze_hitl_log_forces:MissingHistory", ...
        "Log has no stats.history. Re-run HITL with the updated logger.");
end
h = stats.history;
if isfield(h, "plant_time_s")
    t = double(h.plant_time_s(:)).';
else
    t = double(h.wall_time_s(:)).';
end
if ~isfield(h, "x_state") || ~isfield(h, "u")
    error("analyze_hitl_log_forces:IncompleteHistory", ...
        "Log history lacks x_state/u. Re-run HITL with the updated logger.");
end
history = struct();
history.t = t;
history.wall_time_s = double(h.wall_time_s(:)).';
history.x = double(h.x_state);
history.u = double(h.u);
history.servo_raw = get_history_matrix(h, "servo_raw", 8, numel(t), NaN);
history.force_enable = get_history_row(h, "force_enable", numel(t), NaN);
history.position_ned = history.x(1:3, :);
history.velocity_ned = history.x(4:6, :);
history.q_eb = history.x(7:10, :);
history.omega_b = history.x(11:13, :);
end

function mat = get_history_matrix(h, name, rows, cols, fill_value)
if isfield(h, name)
    mat = double(h.(name));
else
    mat = fill_value * ones(rows, cols);
end
end

function row = get_history_row(h, name, cols, fill_value)
if isfield(h, name)
    row = double(h.(name)(:)).';
else
    row = fill_value * ones(1, cols);
end
end

function param = get_param_snapshot(stats)
if isfield(stats, "param_snapshot")
    param = stats.param_snapshot;
else
    param = init_param_zx();
end
end

function cfg = get_cfg_snapshot(stats)
if isfield(stats, "cfg_snapshot")
    cfg = stats.cfg_snapshot;
else
    cfg = hitl_config();
end
end

function meta = get_meta_snapshot(stats)
if isfield(stats, "meta")
    meta = stats.meta;
else
    meta = struct();
end
end

function diag = compute_force_history(history, param, meta)
N = numel(history.t);
names = ["rotor", "aero", "gravity", "ground", "stand", "total"];
for name = names
    diag.force_b.(name) = zeros(3, N);
    diag.force_e.(name) = zeros(3, N);
    diag.moment_b.(name) = zeros(3, N);
end
diag.euler_dbg_deg = zeros(3, N);
diag.accel_e = zeros(3, N);
diag.angular_accel_b = zeros(3, N);
diag.force_norm = zeros(numel(names), N);
diag.force_names = cellstr(names);

for k = 1:N
    x = history.x(:, k);
    u = history.u(:, k);
    q = quat_normalize(x(7:10));
    R_eb = quat_to_dcm_be(q).';
    R_be = R_eb';
    v_e = x(4:6);
    omega_b = x(11:13);

    delta_t = [u(1:8); u(9); u(10)];
    [f_r, m_r] = tandem_rotor_fm(delta_t, v_e, R_eb, param);

    if get_optional_field(param, "aero_body_enable", true)
        [f_a, m_a] = tandem_aero_fm(v_e, R_eb, omega_b, u(11), u(12), u(1:8), param);
    else
        f_a = zeros(3, 1);
        m_a = zeros(3, 1);
    end

    f_g_b = R_be * [0; 0; param.m * param.g];
    m_g_b = zeros(3, 1);

    f_ground_b = zeros(3, 1);
    m_ground_b = zeros(3, 1);
    if isfield(param, "ground") && isfield(param.ground, "enable") && param.ground.enable
        [f_ground_b, m_ground_b] = zx_ground_contact_force(x(1:3), v_e, R_eb, omega_b, param);
    end

    [f_stand_b, m_stand_b] = stand_force_from_meta(x, R_eb, omega_b, meta);

    f_total_b = f_r + f_a + f_g_b + f_ground_b + f_stand_b;
    m_total_b = m_r + m_a + m_ground_b + m_stand_b;

    diag = assign_component(diag, "rotor", k, f_r, m_r, R_eb);
    diag = assign_component(diag, "aero", k, f_a, m_a, R_eb);
    diag = assign_component(diag, "gravity", k, f_g_b, m_g_b, R_eb);
    diag = assign_component(diag, "ground", k, f_ground_b, m_ground_b, R_eb);
    diag = assign_component(diag, "stand", k, f_stand_b, m_stand_b, R_eb);
    diag = assign_component(diag, "total", k, f_total_b, m_total_b, R_eb);

    diag.euler_dbg_deg(:, k) = quat_to_euler_deg_local(q);
    diag.accel_e(:, k) = R_eb * f_total_b / param.m;
    diag.angular_accel_b(:, k) = param.J \ (m_total_b - cross(omega_b, param.J * omega_b));
end

for i = 1:numel(names)
    diag.force_norm(i, :) = vecnorm(diag.force_b.(names(i)), 2, 1);
end
end

function diag = assign_component(diag, name, k, f_b, m_b, R_eb)
diag.force_b.(name)(:, k) = f_b;
diag.force_e.(name)(:, k) = R_eb * f_b;
diag.moment_b.(name)(:, k) = m_b;
end

function [f_b, m_b] = stand_force_from_meta(x, R_eb, omega_b, meta)
f_b = zeros(3, 1);
m_b = zeros(3, 1);
if ~isfield(meta, "stand_cfg")
    return;
end
stand_cfg = meta.stand_cfg;
R_be = R_eb';
r_b = stand_cfg.r_b;
contact_pos_e = x(1:3) + R_eb * r_b;
v_contact_b = R_be * x(4:6) + cross(omega_b, r_b);
v_contact_e = R_eb * v_contact_b;
penetration = contact_pos_e(3) - stand_cfg.top_z;
force_e = zeros(3, 1);
if penetration > 0
    normal = max(0, stand_cfg.k * penetration + stand_cfg.c * v_contact_e(3));
    force_e(3) = -normal;
end
f_b = R_be * force_e;
m_b = cross(r_b, f_b);
end

function summary = build_summary_table(history, diag)
t = history.t(:);
summary = table(t, 'VariableNames', {'time_s'});
summary.wall_time_s = history.wall_time_s(:);
summary.force_enable = history.force_enable(:);
for i = 1:8
    summary.(sprintf("pwm%d", i)) = history.servo_raw(i, :).';
end
for i = 1:12
    summary.(sprintf("u%d", i)) = history.u(i, :).';
end
labels = ["x_n", "y_e", "z_d"];
for i = 1:3
    summary.(labels(i)) = history.position_ned(i, :).';
end
labels = ["vx_n", "vy_e", "vz_d"];
for i = 1:3
    summary.(labels(i)) = history.velocity_ned(i, :).';
end
labels = ["qw", "qx", "qy", "qz"];
for i = 1:4
    summary.(labels(i)) = history.q_eb(i, :).';
end
labels = ["roll_dbg_deg", "pitch_dbg_deg", "yaw_dbg_deg"];
for i = 1:3
    summary.(labels(i)) = diag.euler_dbg_deg(i, :).';
end
for name = string(diag.force_names(:)).'
    f = diag.force_b.(name);
    summary.(name + "_Fx_b") = f(1, :).';
    summary.(name + "_Fy_b") = f(2, :).';
    summary.(name + "_Fz_b") = f(3, :).';
    fe = diag.force_e.(name);
    summary.(name + "_Fn_e") = fe(1, :).';
    summary.(name + "_Fe_e") = fe(2, :).';
    summary.(name + "_Fd_e") = fe(3, :).';
    m = diag.moment_b.(name);
    summary.(name + "_Mx_b") = m(1, :).';
    summary.(name + "_My_b") = m(2, :).';
    summary.(name + "_Mz_b") = m(3, :).';
end
summary.ax_n = diag.accel_e(1, :).';
summary.ay_e = diag.accel_e(2, :).';
summary.az_d = diag.accel_e(3, :).';
end

function plot_pwm(history, fig_dir)
fig = new_fig("PWM and actuator commands");
t = history.t;
tiledlayout(2, 1);
nexttile; plot(t, history.servo_raw.'); grid on; ylabel("PWM"); title("SERVO_OUTPUT_RAW"); legend(compose("servo%d", 1:8), "Location", "eastoutside");
nexttile; plot(t, history.u.'); grid on; ylabel("u"); xlabel("time [s]"); title("Model actuator input"); legend(compose("u%d", 1:12), "Location", "eastoutside");
save_fig(fig, fig_dir, "pwm_and_actuators.png");
end

function plot_position_velocity(history, diag, fig_dir)
fig = new_fig("Position velocity acceleration");
t = history.t;
tiledlayout(3, 1);
nexttile; plot(t, history.position_ned.'); grid on; ylabel("m"); title("Position NED"); legend("N", "E", "D");
nexttile; plot(t, history.velocity_ned.'); grid on; ylabel("m/s"); title("Velocity NED"); legend("N", "E", "D");
nexttile; plot(t, diag.accel_e.'); grid on; ylabel("m/s^2"); xlabel("time [s]"); title("Acceleration NED"); legend("N", "E", "D");
save_fig(fig, fig_dir, "position_velocity_acceleration.png");
end

function plot_attitude(history, diag, fig_dir)
fig = new_fig("Attitude");
t = history.t;
tiledlayout(3, 1);
nexttile; plot(t, history.q_eb.'); grid on; ylabel("q"); title("Quaternion q_eb [w x y z]"); legend("qw", "qx", "qy", "qz");
nexttile; plot(t, diag.euler_dbg_deg.'); grid on; ylabel("deg"); title("Euler_dbg only"); legend("roll", "pitch", "yaw");
nexttile; plot(t, history.x(11:13, :).'); grid on; ylabel("rad/s"); xlabel("time [s]"); title("Body rates p q r"); legend("p", "q", "r");
save_fig(fig, fig_dir, "attitude_quaternion_eulerdbg_rates.png");
end

function plot_forces(history, diag, fig_dir)
t = history.t;
for frame = ["body", "earth"]
    fig = new_fig("Forces " + frame);
    tiledlayout(3, 1);
    components = ["rotor", "aero", "gravity", "ground", "stand", "total"];
    labels = ["X", "Y", "Z"];
    for axis_idx = 1:3
        nexttile; hold on; grid on;
        for component = components
            if frame == "body"
                y = diag.force_b.(component)(axis_idx, :);
            else
                y = diag.force_e.(component)(axis_idx, :);
            end
            plot(t, y);
        end
        ylabel(labels(axis_idx) + " [N]");
        if axis_idx == 1
            title("Force components in " + frame + " frame");
        end
        if axis_idx == 3
            xlabel("time [s]");
        end
        legend(components, "Location", "eastoutside");
    end
    save_fig(fig, fig_dir, "forces_" + frame + ".png");
end
end

function plot_moments(history, diag, fig_dir)
fig = new_fig("Moments body");
t = history.t;
tiledlayout(3, 1);
components = ["rotor", "aero", "ground", "stand", "total"];
labels = ["Mx", "My", "Mz"];
for axis_idx = 1:3
    nexttile; hold on; grid on;
    for component = components
        plot(t, diag.moment_b.(component)(axis_idx, :));
    end
    ylabel(labels(axis_idx) + " [Nm]");
    if axis_idx == 1
        title("Moment components in body frame");
    end
    if axis_idx == 3
        xlabel("time [s]");
    end
    legend(components, "Location", "eastoutside");
end
save_fig(fig, fig_dir, "moments_body.png");
end

function plot_force_norms(history, diag, fig_dir)
fig = new_fig("Force norms");
plot(history.t, diag.force_norm.'); grid on;
xlabel("time [s]"); ylabel("||F_b|| [N]");
title("Force norm by component");
legend(diag.force_names, "Location", "eastoutside");
save_fig(fig, fig_dir, "force_norms.png");
end

function fig = new_fig(name)
fig = figure("Name", name, "Color", "w", "Visible", "off");
fig.Position(3:4) = [1200 800];
end

function save_fig(fig, fig_dir, filename)
exportgraphics(fig, fullfile(fig_dir, filename), "Resolution", 160);
close(fig);
end

function value = get_optional_field(s, name, default_value)
if isfield(s, name)
    value = s.(name);
else
    value = default_value;
end
end

function euler_deg = quat_to_euler_deg_local(q)
R_eb = quat_to_dcm_be(q).';
pitch = asin(max(-1, min(1, -R_eb(3, 1))));
roll = atan2(R_eb(3, 2), R_eb(3, 3));
yaw = atan2(R_eb(2, 1), R_eb(1, 1));
euler_deg = rad2deg([roll; pitch; yaw]);
end
