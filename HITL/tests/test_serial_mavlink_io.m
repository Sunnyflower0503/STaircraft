function test_serial_mavlink_io(duration_s)
%TEST_SERIAL_MAVLINK_IO Exercise COM serial IO and MAVLink encode/decode only.

if nargin < 1
    duration_s = 10;
end

this_dir = fileparts(mfilename("fullpath"));
root = fileparts(this_dir);
addpath(root); addpath(fullfile(root, "utils")); addpath(fullfile(root, "mavlink_backend"));

cfg = hitl_config();
ser = serial_open(cfg);
cleanup = onCleanup(@() clear("ser")); %#ok<NASGU>

t0 = tic;
received = false;
while toc(t0) < duration_s
    now_s = toc(t0);
    bytes = serial_read_bytes(ser);
    servo_msg = mavlink_decode_servo_output_raw(bytes, cfg);
    if servo_msg.is_new
        received = true;
        fprintf("[t=%.3f] SERVO_OUTPUT_RAW:\n", now_s);
        fprintf("s1=%d s2=%d s3=%d s4=%d s5=%d s6=%d s7=%d s8=%d\n", ...
            servo_msg.servo1_raw, servo_msg.servo2_raw, servo_msg.servo3_raw, servo_msg.servo4_raw, ...
            servo_msg.servo5_raw, servo_msg.servo6_raw, servo_msg.servo7_raw, servo_msg.servo8_raw);
    end

    payload = minimal_hil_state_payload(now_s);
    tx_bytes = mavlink_encode_hil_state_quaternion(payload, cfg);
    serial_write_bytes(ser, tx_bytes);
    pause(cfg.sample_time);
end

if received
    fprintf("SERVO_OUTPUT_RAW received successfully.\n");
else
    fprintf("No SERVO_OUTPUT_RAW received. Check PX4 MAVLink stream, COM port, baudrate, and HITL configuration.\n");
end
end

function payload = minimal_hil_state_payload(t)
payload = struct();
payload.time_usec = uint64(t * 1e6);
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
