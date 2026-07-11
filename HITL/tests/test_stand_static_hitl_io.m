function stats = test_stand_static_hitl_io(duration_s)
%TEST_STAND_STATIC_HITL_IO Test stand-static state plus COM4 MAVLink IO.

if nargin < 1
    duration_s = 30;
end

this_dir = fileparts(mfilename("fullpath"));
hitl_dir = fileparts(this_dir);
addpath(hitl_dir);
addpath(fullfile(hitl_dir, "utils"));
addpath(fullfile(hitl_dir, "mavlink_backend"));
addpath(fullfile(fileparts(hitl_dir), "matlab_model"));

cfg = hitl_config();
cfg.model.force_enable = 0;
cfg.model.init_mode = "stand_static";
param = init_param_zx();
[param, x, u, meta] = prepare_stand_static_for_hitl(param, cfg);

ser = serial_open(cfg);
cleanup = onCleanup(@() clear("ser")); %#ok<NASGU>

stats = init_stats(duration_s, cfg, meta, x);
t_start = tic;
last_print_s = 0;
last_servo_rx_s = NaN;
last_servo_raw = nan(1, 8);

while true
    loop_tic = tic;
    elapsed_s = toc(t_start);
    if elapsed_s >= duration_s
        break;
    end

    bytes = serial_read_bytes(ser);
    stats.rx_bytes_total = stats.rx_bytes_total + numel(bytes);

    try
        servo_msg = mavlink_decode_servo_output_raw(bytes, cfg);
    catch ME
        stats.decode_error_count = stats.decode_error_count + 1;
        servo_msg = empty_servo_msg();
        fprintf("[STAND STATIC HITL] decode error at t=%.3fs: %s\n", elapsed_s, ME.message);
    end

    if servo_msg.is_new
        stats.servo_output_raw_count = stats.servo_output_raw_count + 1;
        if ~isnan(last_servo_rx_s)
            stats.max_rx_gap_s = max(stats.max_rx_gap_s, elapsed_s - last_servo_rx_s);
        end
        last_servo_rx_s = elapsed_s;
        last_servo_raw = servo_to_row(servo_msg);
        u = actuator_from_servo_output_raw(servo_msg, u, cfg);
    end

    % Stand-static HITL freezes x. Do not call Runge_Kutta4 or tandem_zx_dynamics here.
    uav = state_to_uavdata_like(elapsed_s, x, u, param, cfg);
    payload = uavdata_to_hil_state_quaternion_payload(uav, cfg);
    tx_bytes = mavlink_encode_hil_state_quaternion(payload, cfg);
    serial_write_bytes(ser, tx_bytes);
    stats.tx_bytes_total = stats.tx_bytes_total + numel(tx_bytes);
    stats.hil_state_quaternion_tx_count = stats.hil_state_quaternion_tx_count + 1;
    stats.lat_deg = uav.lat_deg;
    stats.lon_deg = uav.lon_deg;
    stats.AMSL = uav.AMSL;

    loop_time_s = toc(loop_tic);
    if loop_time_s > cfg.sample_time
        stats.loop_overrun_count = stats.loop_overrun_count + 1;
    else
        pause(cfg.sample_time - loop_time_s);
    end

    elapsed_s = toc(t_start);
    if elapsed_s - last_print_s >= 1
        fprintf("[STAND STATIC HITL] t=%.1fs servo_msgs=%d tx_msgs=%d lat=%.6f lon=%.6f AMSL=%.0f Euler=[%.2f %.2f %.2f] last_servo=[%s]\n", ...
            elapsed_s, stats.servo_output_raw_count, stats.hil_state_quaternion_tx_count, ...
            stats.lat_deg, stats.lon_deg, stats.AMSL, ...
            stats.euler_deg(1), stats.euler_deg(2), stats.euler_deg(3), sprintf("%.0f ", last_servo_raw));
        last_print_s = elapsed_s;
    end
end

stats.duration_s_actual = toc(t_start);
if stats.servo_output_raw_count == 0
    stats.max_rx_gap_s = stats.duration_s_actual;
elseif ~isnan(last_servo_rx_s)
    stats.max_rx_gap_s = max(stats.max_rx_gap_s, stats.duration_s_actual - last_servo_rx_s);
end
stats.last_servo_raw = last_servo_raw;
stats.log_file = save_stats(hitl_dir, stats);

fprintf("[STAND STATIC HITL] done: rx_bytes=%d tx_bytes=%d servo_msgs=%d tx_msgs=%d decode_errors=%d max_gap=%.3fs log=%s\n", ...
    stats.rx_bytes_total, stats.tx_bytes_total, stats.servo_output_raw_count, ...
    stats.hil_state_quaternion_tx_count, stats.decode_error_count, stats.max_rx_gap_s, stats.log_file);
end

function stats = init_stats(duration_s, cfg, meta, x)
stats = struct();
stats.started_at = datestr(now, "yyyy-mm-dd HH:MM:SS");
stats.requested_duration_s = duration_s;
stats.duration_s_actual = 0;
stats.serial_port = cfg.serial.port;
stats.serial_baudrate = cfg.serial.baudrate;
stats.rx_bytes_total = 0;
stats.tx_bytes_total = 0;
stats.servo_output_raw_count = 0;
stats.hil_state_quaternion_tx_count = 0;
stats.decode_error_count = 0;
stats.max_rx_gap_s = 0;
stats.loop_overrun_count = 0;
stats.last_servo_raw = nan(1, 8);
stats.euler_deg = meta.euler_deg;
stats.position_ned = x(1:3);
stats.lat_deg = NaN;
stats.lon_deg = NaN;
stats.AMSL = NaN;
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

function log_file = save_stats(hitl_dir, stats)
logs_dir = fullfile(hitl_dir, "logs");
if ~exist(logs_dir, "dir")
    mkdir(logs_dir);
end
timestamp = datestr(now, "yyyymmdd_HHMMSS");
log_file = fullfile(logs_dir, "stand_static_hitl_" + string(timestamp) + ".mat");
save(log_file, "stats");
end
