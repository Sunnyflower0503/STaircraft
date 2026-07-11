%RUN_HITL_STAND_STATIC One-click stand-static HITL communication runner.
% Run this file directly, like pressing Run in the former Simulink HITL model.

clearvars -except ans; clc;

hitl_dir = fileparts(mfilename("fullpath"));
root_dir = fileparts(hitl_dir);
addpath(hitl_dir);
addpath(fullfile(hitl_dir, "utils"));
addpath(fullfile(hitl_dir, "mavlink_backend"));
addpath(fullfile(root_dir, "matlab_model"));

fprintf("========================================\n");
fprintf("GJ Aircraft HITL - Stand Static Mode\n");
fprintf("========================================\n");
fprintf("USB    : Nora/PX4 -> QGC\n");
fprintf("Serial : Nora/PX4 -> MATLAB COM4\n");
fprintf("Mode   : stand_static\n");
fprintf("Force  : disabled / frozen stand state\n");
fprintf("RX     : SERVO_OUTPUT_RAW\n");
fprintf("TX     : HIL_STATE_QUATERNION\n");
fprintf("Stop   : Ctrl+C in MATLAB\n");
fprintf("========================================\n");

cfg = hitl_config();
cfg.model.force_enable = 0;
cfg.model.init_mode = "stand_static";

param = init_param_zx();
[param, x, u, meta] = prepare_stand_static_for_hitl(param, cfg);
[x, u, meta] = apply_user_initial_conditions(x, u, cfg, param, meta);
uav0 = state_to_uavdata_like(0, x, u, param, cfg);

fprintf("[HITL RUN] Stand state prepared.\n");
fprintf("  Euler deg     : [%.6f %.6f %.6f]\n", meta.euler_deg(1), meta.euler_deg(2), meta.euler_deg(3));
fprintf("  velocity norm : %.3g m/s\n", meta.velocity_norm);
fprintf("  omega norm    : %.3g rad/s\n", meta.angular_rate_norm);
fprintf("  cache_used    : %d\n", logical(meta.cache_used));
fprintf("  cache_file    : %s\n", string(meta.cache_file));
fprintf("  lat/lon/AMSL  : %.6f %.6f %.0f\n", uav0.lat_deg, uav0.lon_deg, uav0.AMSL);
fprintf("[HITL RUN] User config: loaded=%d mode=%s enable_override=%d applied=[%s]\n", ...
    logical(meta.user_initial_conditions.config_loaded), ...
    meta.user_initial_conditions.mode, logical(meta.user_initial_conditions.enable_override), ...
    strjoin(meta.user_initial_conditions.applied_fields, ", "));

stats = init_run_stats(cfg, meta, x, uav0);
last_servo_raw = nan(1, 8);
last_servo_rx_s = NaN;
last_print_s = 0;
first_servo_reported = false;
no_servo_warning_printed = false;
ser = [];
cleanup_obj = onCleanup(@() cleanup_run(hitl_dir, stats, ser));

try
    ser = serial_open(cfg);
catch ME
    fprintf(2, "\n[HITL RUN] Failed to open %s @ %d baud.\n", cfg.serial.port, cfg.serial.baudrate);
    fprintf(2, "Check:\n");
    fprintf(2, "- 检查串口线是否插好；\n");
    fprintf(2, "- 检查 COM4 是否正确；\n");
    fprintf(2, "- 检查 QGC 是否占用了 COM4；\n");
    fprintf(2, "- 检查波特率是否为 115200。\n");
    rethrow(ME);
end

fprintf("[HITL RUN] Serial opened: %s @ %d. Entering stand-static loop.\n", cfg.serial.port, cfg.serial.baudrate);

t_start = tic;
while true
    loop_tic = tic;
    elapsed_s = toc(t_start);

    bytes = serial_read_bytes(ser);
    stats.rx_bytes_total = stats.rx_bytes_total + numel(bytes);

    try
        servo_msg = mavlink_decode_servo_output_raw(bytes, cfg);
    catch ME
        stats.decode_error_count = stats.decode_error_count + 1;
        servo_msg = empty_servo_msg();
        fprintf(2, "[HITL RUN] decode error at t=%.3fs: %s\n", elapsed_s, ME.message);
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

    % Stand-static runner freezes x. Do not call Runge_Kutta4 or tandem_zx_dynamics.
    uav = state_to_uavdata_like(elapsed_s, x, u, param, cfg);
    payload = uavdata_to_hil_state_quaternion_payload(uav, cfg);
    tx_bytes = mavlink_encode_hil_state_quaternion(payload, cfg);
    serial_write_bytes(ser, tx_bytes);

    stats.tx_bytes_total = stats.tx_bytes_total + numel(tx_bytes);
    stats.hil_state_quaternion_tx_count = stats.hil_state_quaternion_tx_count + 1;
    stats.lat_deg = uav.lat_deg;
    stats.lon_deg = uav.lon_deg;
    stats.AMSL = uav.AMSL;
    stats.duration_s_actual = elapsed_s;

    if ~first_servo_reported && ~no_servo_warning_printed && elapsed_s >= 5
        fprintf(2, "No SERVO_OUTPUT_RAW received yet.\n");
        fprintf(2, "Check:\n");
        fprintf(2, "- QGC 是否占用了 COM4；\n");
        fprintf(2, "- COM4 是否正确；\n");
        fprintf(2, "- PX4 MAVLink stream 是否输出 SERVO_OUTPUT_RAW；\n");
        fprintf(2, "- 波特率是否为 115200。\n");
        no_servo_warning_printed = true;
    end

    if elapsed_s - last_print_s >= 1
        fprintf("[HITL RUN] t=%.1fs servo_msgs=%d tx_msgs=%d max_gap=%.3fs lat=%.6f lon=%.6f AMSL=%.0f Euler=[%.2f %.2f %.2f] last_servo=[%s]\n", ...
            elapsed_s, stats.servo_output_raw_count, stats.hil_state_quaternion_tx_count, ...
            stats.max_rx_gap_s, stats.lat_deg, stats.lon_deg, stats.AMSL, ...
            stats.euler_deg(1), stats.euler_deg(2), stats.euler_deg(3), sprintf("%.0f ", last_servo_raw));
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

function stats = init_run_stats(cfg, meta, x, uav0)
stats = struct();
stats.started_at = string(datetime("now", "Format", "yyyy-MM-dd HH:mm:ss"));
stats.serial_port = cfg.serial.port;
stats.serial_baudrate = cfg.serial.baudrate;
stats.mode = "stand_static";
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

function cleanup_run(hitl_dir, stats, ser)
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
    stats.log_file = fullfile(logs_dir, "run_hitl_stand_static_" + string(timestamp) + ".mat");
    save(stats.log_file, "stats");
    fprintf("\n[HITL RUN] Saved run log: %s\n", stats.log_file);
catch ME
    fprintf(2, "\n[HITL RUN] Failed to save run log: %s\n", ME.message);
end
end

