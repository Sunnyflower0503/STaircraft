function stats = test_hitl_link_health(duration_s)
%TEST_HITL_LINK_HEALTH Check COM4 MAVLink link smoothness without dynamics.

if nargin < 1
    duration_s = 30;
end

this_dir = fileparts(mfilename("fullpath"));
hitl_dir = fileparts(this_dir);
addpath(hitl_dir);
addpath(fullfile(hitl_dir, "utils"));
addpath(fullfile(hitl_dir, "mavlink_backend"));

cfg = hitl_config();
cfg.model.force_enable = 0;
cfg.sample_time = 0.01;

payload = fixed_hil_state_payload();
ser = serial_open(cfg);
cleanup = onCleanup(@() clear("ser")); %#ok<NASGU>

stats = init_stats(duration_s, cfg);
t_start = tic;
last_print_s = 0;
last_servo_rx_s = NaN;
last_servo_msg = [];

while true
    loop_tic = tic;
    elapsed_s = toc(t_start);
    if elapsed_s >= duration_s
        break;
    end

    bytes = serial_read_bytes(ser);
    n_rx = numel(bytes);
    stats.rx_bytes_total = stats.rx_bytes_total + n_rx;
    if n_rx == 0
        stats.empty_read_count = stats.empty_read_count + 1;
    end

    try
        servo_msg = mavlink_decode_servo_output_raw(bytes, cfg);
    catch ME
        stats.decode_error_count = stats.decode_error_count + 1;
        servo_msg = empty_servo_msg();
        fprintf("[HITL LINK] decode error at t=%.3fs: %s\n", elapsed_s, ME.message);
    end

    if servo_msg.is_new
        stats.servo_output_raw_count = stats.servo_output_raw_count + 1;
        if ~isnan(last_servo_rx_s)
            stats.max_rx_gap_s = max(stats.max_rx_gap_s, elapsed_s - last_servo_rx_s);
        end
        last_servo_rx_s = elapsed_s;
        last_servo_msg = servo_msg;
    end

    payload.time_usec = uint64(elapsed_s * 1e6);
    tx_bytes = mavlink_encode_hil_state_quaternion(payload, cfg);
    serial_write_bytes(ser, tx_bytes);
    stats.tx_bytes_total = stats.tx_bytes_total + numel(tx_bytes);
    stats.hil_state_quaternion_tx_count = stats.hil_state_quaternion_tx_count + 1;

    loop_time_s = toc(loop_tic);
    stats.max_loop_time_s = max(stats.max_loop_time_s, loop_time_s);
    if loop_time_s > cfg.sample_time
        stats.loop_overrun_count = stats.loop_overrun_count + 1;
    else
        pause(cfg.sample_time - loop_time_s);
    end

    elapsed_s = toc(t_start);
    if elapsed_s - last_print_s >= 1
        stats = update_rates(stats, elapsed_s);
        fprintf("[HITL LINK] t=%.1fs rx_bytes=%d tx_msgs=%d servo_msgs=%d servo_rate=%.1fHz max_gap=%.2fs overruns=%d\n", ...
            elapsed_s, stats.rx_bytes_total, stats.hil_state_quaternion_tx_count, ...
            stats.servo_output_raw_count, stats.mean_rx_rate_hz, stats.max_rx_gap_s, stats.loop_overrun_count);
        if ~isempty(last_servo_msg)
            print_servo_msg(last_servo_msg);
        end
        last_print_s = elapsed_s;
    end
end

stats.duration_s_actual = toc(t_start);
stats = update_rates(stats, stats.duration_s_actual);
if stats.servo_output_raw_count <= 1 && ~isnan(last_servo_rx_s)
    stats.max_rx_gap_s = max(stats.max_rx_gap_s, stats.duration_s_actual - last_servo_rx_s);
elseif stats.servo_output_raw_count == 0
    stats.max_rx_gap_s = stats.duration_s_actual;
end

print_final_result(stats);
stats.log_file = save_stats(hitl_dir, stats);
fprintf("Saved link health stats: %s\n", stats.log_file);
end

function stats = init_stats(duration_s, cfg)
stats = struct();
stats.started_at = datestr(now, "yyyy-mm-dd HH:MM:SS");
stats.requested_duration_s = duration_s;
stats.duration_s_actual = 0;
stats.serial_port = cfg.serial.port;
stats.serial_baudrate = cfg.serial.baudrate;
stats.mavlink_backend = cfg.mavlink.backend;
stats.rx_bytes_total = 0;
stats.tx_bytes_total = 0;
stats.servo_output_raw_count = 0;
stats.hil_state_quaternion_tx_count = 0;
stats.decode_error_count = 0;
stats.empty_read_count = 0;
stats.max_rx_gap_s = 0;
stats.mean_rx_rate_hz = 0;
stats.mean_tx_rate_hz = 0;
stats.loop_overrun_count = 0;
stats.max_loop_time_s = 0;
stats.log_file = "";
end

function stats = update_rates(stats, elapsed_s)
if elapsed_s <= 0
    return;
end
stats.mean_rx_rate_hz = stats.servo_output_raw_count / elapsed_s;
stats.mean_tx_rate_hz = stats.hil_state_quaternion_tx_count / elapsed_s;
end

function payload = fixed_hil_state_payload()
payload = struct();
payload.time_usec = uint64(0);
payload.attitude_quaternion = single([1 0 0 0]);
payload.rollspeed = single(0);
payload.pitchspeed = single(0);
payload.yawspeed = single(0);
payload.lat = int32(0);
payload.lon = int32(0);
payload.alt = int32(0);
payload.vx = int16(0);
payload.vy = int16(0);
payload.vz = int16(0);
payload.ind_airspeed = uint16(0);
payload.true_airspeed = uint16(0);
payload.xacc = int16(0);
payload.yacc = int16(0);
payload.zacc = int16(0);
end

function msg = empty_servo_msg()
msg = struct();
msg.is_new = false;
msg.timestamp = [];
for k = 1:8
    msg.(sprintf("servo%d_raw", k)) = uint16(0);
end
end

function print_servo_msg(msg)
fprintf("  latest SERVO_OUTPUT_RAW: servo1_raw=%d servo2_raw=%d servo3_raw=%d servo4_raw=%d servo5_raw=%d servo6_raw=%d servo7_raw=%d servo8_raw=%d\n", ...
    msg.servo1_raw, msg.servo2_raw, msg.servo3_raw, msg.servo4_raw, ...
    msg.servo5_raw, msg.servo6_raw, msg.servo7_raw, msg.servo8_raw);
end

function print_final_result(stats)
if stats.servo_output_raw_count == 0
    fprintf("No SERVO_OUTPUT_RAW received.\n");
    fprintf("Check:\n");
    fprintf("- QGC 是否占用了 COM4\n");
    fprintf("- COM4 是否真的是串口线端口\n");
    fprintf("- PX4 MAVLink instance 是否绑定到该串口\n");
    fprintf("- 波特率是否为 115200\n");
    fprintf("- PX4 是否输出 SERVO_OUTPUT_RAW stream\n");
    fprintf("- 飞控是否处于 HITL/仿真相关状态\n");
    fprintf("- 是否需要解锁或进入对应模式才有 servo output\n");
end

if stats.servo_output_raw_count > 0 && stats.max_rx_gap_s < 1.0
    fprintf("HITL serial link is alive.\n");
elseif stats.servo_output_raw_count == 0 && stats.rx_bytes_total > 0
    fprintf("Serial bytes received, but no SERVO_OUTPUT_RAW decoded. Check MAVLink stream/message type.\n");
elseif stats.rx_bytes_total == 0
    fprintf("No serial bytes received. Check COM port, wiring, baudrate, and whether QGC has occupied the port.\n");
else
    fprintf("SERVO_OUTPUT_RAW received, but max receive gap is %.2fs. Check link smoothness and PX4 stream rate.\n", stats.max_rx_gap_s);
end

fprintf("Summary: rx_bytes=%d tx_bytes=%d servo_msgs=%d tx_msgs=%d decode_errors=%d empty_reads=%d mean_servo_rate=%.2fHz mean_tx_rate=%.2fHz max_gap=%.3fs overruns=%d max_loop=%.4fs\n", ...
    stats.rx_bytes_total, stats.tx_bytes_total, stats.servo_output_raw_count, ...
    stats.hil_state_quaternion_tx_count, stats.decode_error_count, stats.empty_read_count, ...
    stats.mean_rx_rate_hz, stats.mean_tx_rate_hz, stats.max_rx_gap_s, ...
    stats.loop_overrun_count, stats.max_loop_time_s);
end

function log_file = save_stats(hitl_dir, stats)
logs_dir = fullfile(hitl_dir, "logs");
if ~exist(logs_dir, "dir")
    mkdir(logs_dir);
end
timestamp = datestr(now, "yyyymmdd_HHMMSS");
log_file = fullfile(logs_dir, "link_health_" + string(timestamp) + ".mat");
save(log_file, "stats");
end
