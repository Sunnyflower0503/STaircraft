function stats = hitl_static_pose_loop(title_text, mode_name, x, u, param, cfg, meta)
%HITL_STATIC_POSE_LOOP Send a HITL pose and optionally integrate dynamics.

fprintf("========================================\n");
fprintf("GJ Aircraft HITL - %s\n", title_text);
fprintf("========================================\n");
fprintf("USB    : Nora/PX4 -> QGC\n");
fprintf("Serial : Nora/PX4 -> MATLAB %s\n", cfg.serial.port);
fprintf("Mode   : %s\n", mode_name);
fprintf("Force  : follows runtime_control.txt force_enable\n");
fprintf("RX     : SERVO_OUTPUT_RAW\n");
fprintf("TX     : HIL_STATE_QUATERNION\n");
fprintf("Stop   : Ctrl+C in MATLAB\n");
fprintf("========================================\n");

uav0 = state_to_uavdata_like(0, x, u, param, cfg);
contact_diag = hitl_ground_contact_diagnostics(x, param);
cfg = update_runtime_control(cfg, 0, true);
fprintf("[HITL STATIC] Runtime control file: %s\n", cfg.runtime_control.file);
fprintf("[HITL STATIC] Initial force_enable from runtime control: %d\n", cfg.model.force_enable);

fprintf("[HITL STATIC] Pose prepared.\n");
fprintf("  q_eb [wxyz]   : [%.9f %.9f %.9f %.9f]\n", meta.q_eb(1), meta.q_eb(2), meta.q_eb(3), meta.q_eb(4));
fprintf("  Euler deg dbg : [%.6f %.6f %.6f]\n", meta.euler_deg(1), meta.euler_deg(2), meta.euler_deg(3));
fprintf("  position NED  : [%.6f %.6f %.6f] m\n", x(1), x(2), x(3));
fprintf("  velocity norm : %.3g m/s\n", norm(x(4:6)));
fprintf("  omega norm    : %.3g rad/s\n", norm(x(11:13)));
if isfield(meta, "stand_height")
    fprintf("  stand_height  : %.9f m\n", meta.stand_height);
    fprintf("  stand_top_z   : %.9f m\n", meta.stand_top_z);
end
if isfield(meta, "cache_used")
    fprintf("  cache_used    : %d\n", logical(meta.cache_used));
end
if isfield(meta, "cache_file")
    fprintf("  cache_file    : %s\n", string(meta.cache_file));
end
fprintf("  lat/lon/AMSL  : %.6f %.6f %.0f\n", uav0.lat_deg, uav0.lon_deg, uav0.AMSL);
fprintf("  contacts      : %d/%d active\n", contact_diag.active_contact_count, contact_diag.contact_count_total);
if isfield(meta, "user_initial_conditions")
    fprintf("[HITL STATIC] User config: loaded=%d mode=%s enable_override=%d applied=[%s]\n", ...
        logical(meta.user_initial_conditions.config_loaded), ...
        meta.user_initial_conditions.mode, logical(meta.user_initial_conditions.enable_override), ...
        strjoin(meta.user_initial_conditions.applied_fields, ", "));
end

stats = init_static_stats(cfg, mode_name, meta, x, uav0);
last_servo_raw = nan(1, 8);
last_servo_rx_s = NaN;
last_print_s = 0;
last_wall_time_s = 0;
plant_time_s = 0;
first_servo_reported = false;
no_servo_warning_printed = false;
ser = [];
cleanup_obj = onCleanup(@() cleanup_static_run(fileparts(mfilename("fullpath")), stats, ser));

try
    ser = serial_open(cfg);
catch ME
    fprintf(2, "\n[HITL STATIC] Failed to open %s @ %d baud.\n", cfg.serial.port, cfg.serial.baudrate);
    fprintf(2, "Check:\n");
    fprintf(2, "- 检查串口线是否插好；\n");
    fprintf(2, "- 检查 %s 是否正确；\n", cfg.serial.port);
    fprintf(2, "- 检查 QGC 是否占用了该串口；\n");
    fprintf(2, "- 检查波特率是否为 %d。\n", cfg.serial.baudrate);
    rethrow(ME);
end

fprintf("[HITL STATIC] Serial opened: %s @ %d. Entering frozen-pose loop.\n", ...
    cfg.serial.port, cfg.serial.baudrate);

t_start = tic;
while true
    loop_tic = tic;
    elapsed_s = toc(t_start);
    state_dt_s = max(0, elapsed_s - last_wall_time_s);
    last_wall_time_s = elapsed_s;
    cfg = update_runtime_control(cfg, elapsed_s);

    bytes = serial_read_bytes(ser);
    stats.rx_bytes_total = stats.rx_bytes_total + numel(bytes);

    try
        servo_msg = mavlink_decode_servo_output_raw(bytes, cfg);
    catch ME
        stats.decode_error_count = stats.decode_error_count + 1;
        servo_msg = empty_servo_msg();
        fprintf(2, "[HITL STATIC] decode error at t=%.3fs: %s\n", elapsed_s, ME.message);
    end

    if servo_msg.is_new
        stats.servo_output_raw_count = stats.servo_output_raw_count + 1;
        if ~isnan(last_servo_rx_s)
            stats.max_rx_gap_s = max(stats.max_rx_gap_s, elapsed_s - last_servo_rx_s);
        end
        last_servo_rx_s = elapsed_s;
        last_servo_raw = servo_to_row(servo_msg);
        stats.last_servo_raw = last_servo_raw;
        u = actuator_from_servo_output_raw(servo_msg, u, cfg);

        if ~first_servo_reported
            fprintf("HITL serial link is alive.\n");
            first_servo_reported = true;
        end
    end

    if cfg.model.force_enable == 1
        plant_step_s = min(state_dt_s, cfg.model.max_runtime_step_s);
        x = integrate_static_pose_step(plant_time_s, x, u, param, meta, plant_step_s);
        plant_time_s = plant_time_s + plant_step_s;
        contact_diag = hitl_ground_contact_diagnostics(x, param);
    end

    uav = state_to_uavdata_like(plant_time_s, x, u, param, cfg);
    payload = uavdata_to_hil_state_quaternion_payload(uav, cfg);
    tx_bytes = mavlink_encode_hil_state_quaternion(payload, cfg);
    serial_write_bytes(ser, tx_bytes);

    stats.tx_bytes_total = stats.tx_bytes_total + numel(tx_bytes);
    stats.hil_state_quaternion_tx_count = stats.hil_state_quaternion_tx_count + 1;
    stats.lat_deg = uav.lat_deg;
    stats.lon_deg = uav.lon_deg;
    stats.AMSL = uav.AMSL;
    stats.duration_s_actual = elapsed_s;
    stats.force_enable = cfg.model.force_enable;
    stats.position_ned = x(1:3);
    stats.velocity_ned = x(4:6);
    stats.q_eb = quat_normalize(x(7:10));
    stats.euler_deg = quat_to_euler_deg_local(x(7:10));

    if ~first_servo_reported && ~no_servo_warning_printed && elapsed_s >= 5
        fprintf(2, "No SERVO_OUTPUT_RAW received yet.\n");
        fprintf(2, "Check COM port, QGC ownership, MAVLink stream, and baudrate.\n");
        no_servo_warning_printed = true;
    end

    if elapsed_s - last_print_s >= 1
        fprintf("[HITL STATIC] wall_t=%.1fs plant_t=%.3fs mode=%s force_enable=%d servo_msgs=%d tx_msgs=%d max_gap=%.3fs lat=%.6f lon=%.6f AMSL=%.0f pos=[%.2f %.2f %.2f] vel=[%.2f %.2f %.2f] q=[%.5f %.5f %.5f %.5f] Euler_dbg=[%.2f %.2f %.2f] contacts=%d/%d last_servo=[%s]\n", ...
            elapsed_s, plant_time_s, mode_name, cfg.model.force_enable, stats.servo_output_raw_count, stats.hil_state_quaternion_tx_count, ...
            stats.max_rx_gap_s, stats.lat_deg, stats.lon_deg, stats.AMSL, ...
            x(1), x(2), x(3), x(4), x(5), x(6), ...
            stats.q_eb(1), stats.q_eb(2), stats.q_eb(3), stats.q_eb(4), ...
            stats.euler_deg(1), stats.euler_deg(2), stats.euler_deg(3), ...
            contact_diag.active_contact_count, contact_diag.contact_count_total, sprintf("%.0f ", last_servo_raw));
        last_print_s = elapsed_s;
    end

    loop_time_s = toc(loop_tic);
    stats.max_loop_time_s = max(stats.max_loop_time_s, loop_time_s);
    if loop_time_s > cfg.sample_time
        stats.loop_overrun_count = stats.loop_overrun_count + 1;
    else
        pause(cfg.sample_time - loop_time_s);
    end
end
end

function x_next = integrate_static_pose_step(t, x, u, param, meta, step_s)
if step_s <= 0
    x_next = x;
    return;
end
[~, z] = Runge_Kutta4(@(tt, xx) static_pose_dynamics(tt, xx, u, param, meta), [t t + step_s], x);
x_next = z(:, end);
x_next(7:10) = quat_normalize(x_next(7:10));
end

function dx = static_pose_dynamics(t, x, u, param, meta)
dx = tandem_zx_dynamics(t, x, u, param);
if isfield(meta, "stand_cfg")
    q = quat_normalize(x(7:10));
    R_eb = quat_to_dcm_be(q).';
    [f_stand_b, m_stand_b] = stand_contact_force(x(1:3), x(4:6), R_eb, x(11:13), meta.stand_cfg);
    dx(4:6) = dx(4:6) + R_eb * f_stand_b / param.m;
    dx(11:13) = dx(11:13) + param.J \ m_stand_b;
end
end

function [f_b, m_b] = stand_contact_force(p_e, v_e, R_eb, omega_b, stand_cfg)
R_be = R_eb';
r_b = stand_cfg.r_b;
contact_pos_e = p_e + R_eb * r_b;
v_contact_b = R_be * v_e + cross(omega_b, r_b);
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

function euler_deg = quat_to_euler_deg_local(q)
q = quat_normalize(q);
R = quat_to_dcm_be(q).';
pitch = asin(max(-1, min(1, -R(3, 1))));
roll = atan2(R(3, 2), R(3, 3));
yaw = atan2(R(2, 1), R(1, 1));
euler_deg = [roll; pitch; yaw] * 180 / pi;
end

function stats = init_static_stats(cfg, mode_name, meta, x, uav0)
stats = struct();
stats.started_at = string(datetime("now", "Format", "yyyy-MM-dd HH:mm:ss"));
stats.serial_port = cfg.serial.port;
stats.serial_baudrate = cfg.serial.baudrate;
stats.mode = mode_name;
stats.force_enable = cfg.model.force_enable;
stats.rx_msg = cfg.mavlink.rx_msg;
stats.tx_msg = cfg.mavlink.tx_msg;
stats.rx_bytes_total = 0;
stats.tx_bytes_total = 0;
stats.servo_output_raw_count = 0;
stats.hil_state_quaternion_tx_count = 0;
stats.decode_error_count = 0;
stats.max_rx_gap_s = 0;
stats.loop_overrun_count = 0;
stats.max_loop_time_s = 0;
stats.last_servo_raw = nan(1, 8);
stats.q_eb = meta.q_eb;
stats.euler_deg = meta.euler_deg;
stats.position_ned = x(1:3);
stats.lat_deg = uav0.lat_deg;
stats.lon_deg = uav0.lon_deg;
stats.AMSL = uav0.AMSL;
stats.duration_s_actual = 0;
stats.log_file = "";
end

function msg = empty_servo_msg()
msg = struct("is_new", false, "timestamp", []);
for k = 1:8
    msg.(sprintf("servo%d_raw", k)) = uint16(0);
end
end

function row = servo_to_row(msg)
row = double([msg.servo1_raw, msg.servo2_raw, msg.servo3_raw, msg.servo4_raw, ...
    msg.servo5_raw, msg.servo6_raw, msg.servo7_raw, msg.servo8_raw]);
end

function cleanup_static_run(hitl_dir, stats, ser)
try
    if ~isempty(ser)
        clear ser;
    end
catch
end
try
    logs_dir = fullfile(hitl_dir, "logs");
    if ~exist(logs_dir, "dir")
        mkdir(logs_dir);
    end
    timestamp = string(datetime("now", "Format", "yyyyMMdd_HHmmss"));
    stats.log_file = fullfile(logs_dir, string(stats.mode) + "_" + timestamp + ".mat");
    save(stats.log_file, "stats");
    fprintf("\n[HITL STATIC] Saved run log: %s\n", stats.log_file);
catch ME
    fprintf(2, "\n[HITL STATIC] Failed to save run log: %s\n", ME.message);
end
end
