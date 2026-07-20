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
fprintf("TX     : HIL_STATE_QUATERNION only\n");
fprintf("Stop   : landing detected or Ctrl+C\n");
fprintf("========================================\n");

cfg = hitl_config();
cfg.model.init_mode = "stand_static";
cfg_flight = cfg;
max_wall_time_s = str2double(getenv("HITL_MAX_WALL_TIME_S"));
if ~isfinite(max_wall_time_s) || max_wall_time_s <= 0
    max_wall_time_s = Inf;
end
if isfinite(max_wall_time_s)
    fprintf("[HITL TAKEOFF] Automatic stop after %.1f s.\n", max_wall_time_s);
end

fprintf("[HITL TAKEOFF] Runtime control file: %s\n", cfg.runtime_control.file);
fprintf("[HITL TAKEOFF] Set it to force_enable=1 to enable dynamics, force_enable=0 to freeze.\n");
cfg = update_runtime_control(cfg, 0, true);
cfg_flight.model.force_enable = cfg.model.force_enable;
fprintf("[HITL TAKEOFF] Initial force_enable from runtime control: %d\n", cfg.model.force_enable);

param = init_param_zx();
param = apply_hitl_model_switches(param, cfg);
[param, x, u, meta] = prepare_stand_static_for_hitl(param, cfg);
param = apply_hitl_model_switches(param, cfg);
[x, u, meta] = apply_user_initial_conditions(x, u, cfg, param, meta);
u_commanded = u;
actuator_delay_state = [];
[u, actuator_delay_state] = apply_actuator_transport_delay(0, u_commanded, actuator_delay_state, cfg);
param.ground.enable = true;

uav0 = state_to_uavdata_like(0, x, u, param, cfg);
contact_diag = hitl_ground_contact_diagnostics(x, param);

fprintf("[HITL TAKEOFF] Stand state prepared.\n");
fprintf("  q_eb [wxyz]   : [%.9f %.9f %.9f %.9f]\n", meta.q_eb(1), meta.q_eb(2), meta.q_eb(3), meta.q_eb(4));
fprintf("  Euler deg dbg : [%.6f %.6f %.6f]\n", meta.euler_deg(1), meta.euler_deg(2), meta.euler_deg(3));
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
stats.cfg_snapshot = cfg;
stats.param_snapshot = param;
stats.meta = meta;
stats.log_file = create_log_file(hitl_dir);
fprintf("[HITL TAKEOFF] Log autosave file: %s\n", stats.log_file);
fprintf("[HITL TAKEOFF] Actuator delays: motors=%.3f s, elevons=%.3f s.\n", ...
    cfg.actuator_delay.motor_s, cfg.actuator_delay.elevon_s);
fprintf("[HITL TAKEOFF] Actuator first-order tau: motors=%.3f s, elevons=%.3f s.\n", ...
    cfg.actuator_delay.motor_tau_s, cfg.actuator_delay.elevon_tau_s);
save_stats_snapshot(stats, "[HITL TAKEOFF] Created initial log");
state = initial_stand_takeoff_state();
history = init_history();
history_sample_period_s = 0.1;
last_history_sample_s = -Inf;
autosave_period_s = 2.0;
last_autosave_s = -Inf;
last_servo_raw = nan(1, 8);
last_servo_rx_s = NaN;
last_print_s = 0;
last_wall_time_s = 0;
plant_time_s = 0;
flight_wall_time_s = 0;
first_servo_reported = false;
no_servo_warning_printed = false;
stop_after_landing = false;
landing_confirmed_wall_s = NaN;
ser = [];
runtime = containers.Map("KeyType", "char", "ValueType", "any");
runtime("stats") = stats;
runtime("ser") = ser;
cleanup_obj = onCleanup(@() cleanup_run(hitl_dir, runtime));

try
    ser = serial_open(cfg);
    runtime("ser") = ser;
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
    wall_time_s = toc(t_start);
    if wall_time_s >= max_wall_time_s
        fprintf("[HITL TAKEOFF] Maximum wall time reached; stopping with the last state frozen.\n");
        break;
    end
    state_dt_s = max(0, wall_time_s - last_wall_time_s);
    last_wall_time_s = wall_time_s;
    cfg = update_runtime_control(cfg, wall_time_s);
    cfg_flight.model.force_enable = cfg.model.force_enable;

    bytes = serial_read_bytes(ser);
    stats.rx_bytes_total = stats.rx_bytes_total + numel(bytes);

    try
        servo_msg = mavlink_decode_servo_output_raw(bytes, cfg);
    catch ME
        stats.decode_error_count = stats.decode_error_count + 1;
        servo_msg = empty_servo_msg();
        fprintf(2, "[HITL TAKEOFF] decode error at wall_t=%.3fs: %s\n", wall_time_s, ME.message);
    end

    if servo_msg.is_new
        stats.servo_output_raw_count = stats.servo_output_raw_count + 1;
        if ~isnan(last_servo_rx_s)
            stats.max_rx_gap_s = max(stats.max_rx_gap_s, wall_time_s - last_servo_rx_s);
        end
        last_servo_rx_s = wall_time_s;
        last_servo_raw = servo_to_row(servo_msg);
        stats.last_servo_raw = last_servo_raw;
        u_commanded = actuator_from_servo_output_raw(servo_msg, u_commanded, cfg);

        if ~first_servo_reported
            fprintf("HITL serial link is alive.\n");
            first_servo_reported = true;
        end
    end

    [u, actuator_delay_state] = apply_actuator_transport_delay( ...
        wall_time_s, u_commanded, actuator_delay_state, cfg);

    main_throttle = mean(u(1:8));

    plant_step_s = 0;
    if state.stand_released && string(state.phase) ~= "LANDED"
        flight_wall_time_s = flight_wall_time_s + state_dt_s;
        plant_step_s = min(state_dt_s, cfg.model.max_runtime_step_s);
        x = integrate_aircraft_step(plant_time_s, x, u, param, cfg_flight, plant_step_s);
        plant_time_s = plant_time_s + plant_step_s;
    end

    contact_diag = hitl_ground_contact_diagnostics(x, param);
    rear_contact = numel(contact_diag.active) >= 6 && all(contact_diag.active(4:6));
    gentle_rear_contact = rear_contact ...
        && norm(x(4:5)) <= cfg.landing.rear_contact_max_xy_speed ...
        && x(6) <= cfg.landing.rear_contact_max_down_speed;
    if state.stand_released
        state_step_s = plant_step_s;
    else
        state_step_s = state_dt_s;
    end
    state = stand_takeoff_state_step(state, main_throttle, contact_diag.active_contact_count, ...
        state_step_s, cfg, gentle_rear_contact);

    if state.just_released
        fprintf("[HITL] Stand released: throttle=%.3f t=%.3f\n", main_throttle, wall_time_s);
    end
    if state.just_liftoff_confirmed
        fprintf("[HITL] Liftoff confirmed at t=%.3f\n", plant_time_s);
    end
    if state.just_landing_confirmed
        % A tailsitter is supported by its three rear points before the front
        % points touch. Once that contact is continuously gentle, impose the
        % intended static-ground terminal constraint instead of continuing to
        % integrate an armed position controller against a stiff spring.
        x(4:6) = 0;
        x(11:13) = 0;
        fprintf("[HITL] Landing confirmed: active_contact_count=%d/6 at t=%.3f\n", ...
            contact_diag.active_contact_count, plant_time_s);
        fprintf("[HITL] Holding the ground model for 2 s to verify rear-contact protection.\n");
        landing_confirmed_wall_s = wall_time_s;
    end
    if isfinite(landing_confirmed_wall_s) ...
            && wall_time_s - landing_confirmed_wall_s >= cfg.landing.post_confirm_hold_s
        fprintf("[HITL] Simulation stopped after the post-landing protection window.\n");
        stop_after_landing = true;
    end

    cfg_sensor = cfg;
    if ~state.stand_released
        % The external stand still supports the aircraft even if dynamics
        % have been enabled. Keep the IMU consistent with a static pose
        % until the stand-release gate actually opens.
        cfg_sensor.model.force_enable = 0;
    end
    uav = state_to_uavdata_like(wall_time_s, x, u, param, cfg_sensor);
    payload = uavdata_to_hil_state_quaternion_payload(uav, cfg);
    tx_bytes = mavlink_encode_hil_state_quaternion(payload, cfg, rear_contact);
    serial_write_bytes(ser, tx_bytes);

    stats.tx_bytes_total = stats.tx_bytes_total + numel(tx_bytes);
    stats.hil_state_quaternion_tx_count = stats.hil_state_quaternion_tx_count + 1;
    stats.duration_s_actual = wall_time_s;
    stats.flight_wall_time_s = flight_wall_time_s;
    stats.plant_time_s = plant_time_s;
    stats.plant_time_lag_s = max(0, flight_wall_time_s - plant_time_s);
    stats.max_runtime_step_s = cfg.model.max_runtime_step_s;
    stats.phase = state.phase;
    stats.stand_released = state.stand_released;
    stats.liftoff_confirmed = state.liftoff_confirmed;
    stats.active_contact_count = contact_diag.active_contact_count;
    stats.main_throttle = main_throttle;
    stats.force_enable = cfg.model.force_enable;
    stats.position_ned = x(1:3);
    stats.velocity_ned = x(4:6);
    stats.q_eb = quat_normalize(x(7:10));
    stats.euler_deg = quat_to_euler_deg_local(x(7:10));
    stats.x_state = x;
    stats.u = u;
    stats.u_commanded = u_commanded;
    stats.lat_deg = uav.lat_deg;
    stats.lon_deg = uav.lon_deg;
    stats.AMSL = uav.AMSL;

    if wall_time_s - last_history_sample_s >= history_sample_period_s
        history = append_history(history, stats);
        stats.history = history;
        runtime("stats") = stats;
        last_history_sample_s = wall_time_s;
    end

    if wall_time_s - last_autosave_s >= autosave_period_s
        runtime("stats") = stats;
        save_stats_snapshot(stats, "[HITL TAKEOFF] Autosaved log");
        last_autosave_s = wall_time_s;
    end

    if ~first_servo_reported && ~no_servo_warning_printed && wall_time_s >= 5
        fprintf(2, "No SERVO_OUTPUT_RAW received yet.\n");
        fprintf(2, "Check:\n");
        fprintf(2, "- QGC 是否占用了 COM4；\n");
        fprintf(2, "- COM4 是否正确；\n");
        fprintf(2, "- PX4 MAVLink stream 是否输出 SERVO_OUTPUT_RAW；\n");
        fprintf(2, "- 波特率是否为 115200。\n");
        no_servo_warning_printed = true;
    end

    if wall_time_s - last_print_s >= 1
        fprintf("[HITL TAKEOFF] wall_t=%.3fs plant_t=%.3fs lag=%.3fs phase=%s force_enable=%d throttle=%.3f stand_released=%d liftoff=%d contacts=%d/6 servo=[%s] u1_8=[%s] pos=[%.2f %.2f %.2f] vel=[%.2f %.2f %.2f] q=[%.5f %.5f %.5f %.5f] Euler_dbg=[%.2f %.2f %.2f]\n", ...
            wall_time_s, plant_time_s, stats.plant_time_lag_s, string(state.phase), ...
            cfg.model.force_enable, main_throttle, logical(state.stand_released), logical(state.liftoff_confirmed), contact_diag.active_contact_count, ...
            sprintf("%.0f ", last_servo_raw), sprintf("%.3f ", u(1:8)), ...
            x(1), x(2), x(3), x(4), x(5), x(6), ...
            stats.q_eb(1), stats.q_eb(2), stats.q_eb(3), stats.q_eb(4), ...
            stats.euler_deg(1), stats.euler_deg(2), stats.euler_deg(3));
        last_print_s = wall_time_s;
    end

    loop_time_s = toc(loop_tic);
    stats.max_loop_time_s = max(stats.max_loop_time_s, loop_time_s);
    if loop_time_s > cfg.sample_time
        stats.loop_overrun_count = stats.loop_overrun_count + 1;
    else
        pause(cfg.sample_time - loop_time_s);
    end
end

stats.history = history;
runtime("stats") = stats;

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
stats.force_enable = cfg.model.force_enable;
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
stats.q_eb = meta.q_eb;
stats.position_ned = x(1:3);
stats.velocity_ned = x(4:6);
stats.x_state = x;
stats.u = zeros(12, 1);
stats.u_commanded = zeros(12, 1);
stats.lat_deg = uav0.lat_deg;
stats.lon_deg = uav0.lon_deg;
stats.AMSL = uav0.AMSL;
stats.duration_s_actual = 0;
stats.flight_wall_time_s = 0;
stats.plant_time_s = 0;
stats.plant_time_lag_s = 0;
stats.max_runtime_step_s = cfg.model.max_runtime_step_s;
stats.history_sample_period_s = 0.1;
stats.history = init_history();
stats.log_file = "";
end

function history = init_history()
history = struct();
history.wall_time_s = zeros(1, 0);
history.plant_time_s = zeros(1, 0);
history.position_ned = zeros(3, 0);
history.velocity_ned = zeros(3, 0);
history.q_eb = zeros(4, 0);
history.x_state = zeros(13, 0);
history.u = zeros(12, 0);
history.u_commanded = zeros(12, 0);
history.servo_raw = zeros(8, 0);
history.force_enable = zeros(1, 0);
history.main_throttle = zeros(1, 0);
history.active_contact_count = zeros(1, 0);
history.phase = strings(1, 0);
end

function history = append_history(history, stats)
history.wall_time_s(end + 1) = stats.duration_s_actual;
history.plant_time_s(end + 1) = stats.plant_time_s;
history.position_ned(:, end + 1) = stats.position_ned(:);
history.velocity_ned(:, end + 1) = stats.velocity_ned(:);
history.q_eb(:, end + 1) = stats.q_eb(:);
history.x_state(:, end + 1) = stats.x_state(:);
history.u(:, end + 1) = stats.u(:);
history.u_commanded(:, end + 1) = stats.u_commanded(:);
history.servo_raw(:, end + 1) = stats.last_servo_raw(:);
history.force_enable(end + 1) = stats.force_enable;
history.main_throttle(end + 1) = stats.main_throttle;
history.active_contact_count(end + 1) = stats.active_contact_count;
history.phase(end + 1) = string(stats.phase);
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

function cleanup_run(hitl_dir, runtime)
stats = runtime("stats");
try
    ser = runtime("ser");
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
    if strlength(string(stats.log_file)) == 0
        stats.log_file = create_log_file(hitl_dir);
    end
    save(stats.log_file, "stats");
    fprintf("\n[HITL TAKEOFF] Saved run log: %s\n", stats.log_file);
catch ME
    fprintf(2, "\n[HITL TAKEOFF] Failed to save run log: %s\n", ME.message);
end
end

function log_file = create_log_file(hitl_dir)
logs_dir = fullfile(hitl_dir, "logs");
if ~exist(logs_dir, "dir")
    mkdir(logs_dir);
end
timestamp = string(datetime("now", "Format", "yyyyMMdd_HHmmss"));
log_file = fullfile(logs_dir, "run_hitl_stand_takeoff_" + string(timestamp) + ".mat");
end

function save_stats_snapshot(stats, label)
try
    save(stats.log_file, "stats");
    fprintf("%s: %s\n", label, stats.log_file);
catch ME
    fprintf(2, "%s failed: %s\n", label, ME.message);
end
end
