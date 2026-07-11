%RUN_HITL_STAND_TAKEOFF One-click stand-release takeoff HITL runner.
% Manual arm/throttle in PX4/QGC; MATLAB only releases the model stand.

clearvars -except ans; clc;

hitl_dir = fileparts(mfilename("fullpath"));
root_dir = fileparts(hitl_dir);
addpath(hitl_dir);
addpath(fullfile(hitl_dir, "utils"));
addpath(fullfile(hitl_dir, "mavlink_backend"));
addpath(fullfile(root_dir, "matlab_model"));

fprintf("========================================\n");
fprintf("GJ Aircraft HITL - Stand Takeoff Mode\n");
fprintf("========================================\n");
fprintf("USB    : Nora/PX4 -> QGC\n");
fprintf("Serial : Nora/PX4 -> MATLAB COM4\n");
fprintf("Mode   : stand_takeoff\n");
fprintf("Stand  : hold until throttle release\n");
fprintf("RX     : SERVO_OUTPUT_RAW\n");
fprintf("TX     : HIL_STATE_QUATERNION\n");
fprintf("Stop   : landing detected or Ctrl+C\n");
fprintf("========================================\n");

cfg = hitl_config();
cfg.model.force_enable = 0;
cfg.model.init_mode = "stand_static";

param = init_param_zx();
[param, x, u, meta] = prepare_stand_static_for_hitl(param, cfg);
[x, u, meta] = apply_user_initial_conditions(x, u, cfg, param, meta);
param.ground.enable = true;

uav0 = state_to_uavdata_like(0, x, u, param, cfg);
contact_diag = hitl_ground_contact_diagnostics(x, param);

fprintf("[HITL TAKEOFF] Stand state prepared.\n");
fprintf("  Euler deg     : [%.6f %.6f %.6f]\n", meta.euler_deg(1), meta.euler_deg(2), meta.euler_deg(3));
fprintf("  velocity norm : %.3g m/s\n", meta.velocity_norm);
fprintf("  omega norm    : %.3g rad/s\n", meta.angular_rate_norm);
fprintf("  cache_used    : %d\n", logical(meta.cache_used));
fprintf("  cache_file    : %s\n", string(meta.cache_file));
fprintf("  lat/lon/AMSL  : %.6f %.6f %.0f\n", uav0.lat_deg, uav0.lon_deg, uav0.AMSL);
fprintf("  contacts      : %d/%d active\n", contact_diag.active_contact_count, contact_diag.contact_count_total);
fprintf("[HITL TAKEOFF] User config: loaded=%d mode=%s enable_override=%d applied=[%s]\n", ...
    logical(meta.user_initial_conditions.config_loaded), ...
    meta.user_initial_conditions.mode, logical(meta.user_initial_conditions.enable_override), ...
    strjoin(meta.user_initial_conditions.applied_fields, ", "));

stats = init_takeoff_stats(cfg, meta, x, uav0);
state = initial_stand_takeoff_state();
last_servo_raw = nan(1, 8);
last_servo_rx_s = NaN;
last_print_s = 0;
last_state_update_s = 0;
first_servo_reported = false;
no_servo_warning_printed = false;
stop_after_landing = false;
ser = [];
cleanup_obj = onCleanup(@() cleanup_run(hitl_dir, stats, ser));

try
    ser = serial_open(cfg);
catch ME
    fprintf(2, "\n[HITL TAKEOFF] Failed to open %s @ %d baud.\n", cfg.serial.port, cfg.serial.baudrate);
    fprintf(2, "Check:\n");
    fprintf(2, "- 检查串口线是否插好；\n");
    fprintf(2, "- 检查 COM4 是否正确；\n");
    fprintf(2, "- 检查 QGC 是否占用了 COM4；\n");
    fprintf(2, "- 检查波特率是否为 115200。\n");
    rethrow(ME);
end

fprintf("[HITL TAKEOFF] Serial opened: %s @ %d. Manual arm only; no MAV_CMD_COMPONENT_ARM_DISARM will be sent.\n", ...
    cfg.serial.port, cfg.serial.baudrate);

t_start = tic;
while ~stop_after_landing
    loop_tic = tic;
    elapsed_s = toc(t_start);
    state_dt_s = max(0, elapsed_s - last_state_update_s);
    last_state_update_s = elapsed_s;

    bytes = serial_read_bytes(ser);
    stats.rx_bytes_total = stats.rx_bytes_total + numel(bytes);

    try
        servo_msg = mavlink_decode_servo_output_raw(bytes, cfg);
    catch ME
        stats.decode_error_count = stats.decode_error_count + 1;
        servo_msg = empty_servo_msg();
        fprintf(2, "[HITL TAKEOFF] decode error at t=%.3fs: %s\n", elapsed_s, ME.message);
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

    main_throttle = mean(u(1:8));

    if state.stand_released
        [~, z] = Runge_Kutta4(@(tt, xx) tandem_zx_dynamics(tt, xx, u, param), ...
            [elapsed_s elapsed_s + cfg.dt], x);
        x = z(:, end);
        x(7:10) = quat_normalize(x(7:10));
    end

    contact_diag = hitl_ground_contact_diagnostics(x, param);
    state = stand_takeoff_state_step(state, main_throttle, contact_diag.active_contact_count, state_dt_s, cfg);

    if state.just_released
        fprintf("[HITL] Stand released: throttle=%.3f t=%.3f\n", main_throttle, elapsed_s);
    end
    if state.just_liftoff_confirmed
        fprintf("[HITL] Liftoff confirmed at t=%.3f\n", elapsed_s);
    end
    if state.just_landing_confirmed
        fprintf("[HITL] Landing confirmed: active_contact_count=%d/6 at t=%.3f\n", ...
            contact_diag.active_contact_count, elapsed_s);
        fprintf("[HITL] Simulation stopped after landing.\n");
        stop_after_landing = true;
    end

    uav = state_to_uavdata_like(elapsed_s, x, u, param, cfg);
    payload = uavdata_to_hil_state_quaternion_payload(uav, cfg);
    tx_bytes = mavlink_encode_hil_state_quaternion(payload, cfg);
    serial_write_bytes(ser, tx_bytes);

    stats.tx_bytes_total = stats.tx_bytes_total + numel(tx_bytes);
    stats.hil_state_quaternion_tx_count = stats.hil_state_quaternion_tx_count + 1;
    stats.duration_s_actual = elapsed_s;
    stats.phase = state.phase;
    stats.stand_released = state.stand_released;
    stats.liftoff_confirmed = state.liftoff_confirmed;
    stats.active_contact_count = contact_diag.active_contact_count;
    stats.main_throttle = main_throttle;
    stats.position_ned = x(1:3);
    stats.velocity_ned = x(4:6);
    stats.euler_deg = quat_to_euler_deg_local(x(7:10));
    stats.lat_deg = uav.lat_deg;
    stats.lon_deg = uav.lon_deg;
    stats.AMSL = uav.AMSL;

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
        fprintf("[HITL TAKEOFF] t=%.1fs phase=%s throttle=%.3f stand_released=%d liftoff=%d contacts=%d/6 servo=[%s] u1_8=[%s] pos=[%.2f %.2f %.2f] vel=[%.2f %.2f %.2f] Euler=[%.2f %.2f %.2f]\n", ...
            elapsed_s, string(state.phase), main_throttle, logical(state.stand_released), ...
            logical(state.liftoff_confirmed), contact_diag.active_contact_count, ...
            sprintf("%.0f ", last_servo_raw), sprintf("%.3f ", u(1:8)), ...
            x(1), x(2), x(3), x(4), x(5), x(6), ...
            stats.euler_deg(1), stats.euler_deg(2), stats.euler_deg(3));
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

function stats = init_takeoff_stats(cfg, meta, x, uav0)
stats = struct();
stats.started_at = string(datetime("now", "Format", "yyyy-MM-dd HH:mm:ss"));
stats.serial_port = cfg.serial.port;
stats.serial_baudrate = cfg.serial.baudrate;
stats.mode = "stand_takeoff";
stats.rx_msg = cfg.mavlink.rx_msg;
stats.tx_msg = cfg.mavlink.tx_msg;
stats.phase = "STAND_HOLD";
stats.stand_released = false;
stats.liftoff_confirmed = false;
stats.active_contact_count = 0;
stats.main_throttle = 0;
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
stats.velocity_ned = x(4:6);
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

function euler_deg = quat_to_euler_deg_local(q_eb)
R_eb = quat_to_dcm_be(q_eb).';
pitch = asin(-R_eb(3, 1));
roll = atan2(R_eb(3, 2), R_eb(3, 3));
yaw = atan2(R_eb(2, 1), R_eb(1, 1));
euler_deg = rad2deg([roll; pitch; yaw]);
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
    stats.log_file = fullfile(logs_dir, "run_hitl_stand_takeoff_" + string(timestamp) + ".mat");
    save(stats.log_file, "stats");
    fprintf("\n[HITL TAKEOFF] Saved run log: %s\n", stats.log_file);
catch ME
    fprintf(2, "\n[HITL TAKEOFF] Failed to save run log: %s\n", ME.message);
end
end
