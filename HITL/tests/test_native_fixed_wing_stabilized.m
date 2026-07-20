function result = test_native_fixed_wing_stabilized(external_moment_body_nm, pulse_duration_s, recovery_duration_s, cruise_throttle_milli, cruise_airspeed_m_s, manual_xyr_milli)
%TEST_NATIVE_FIXED_WING_STABILIZED Fixed-wing Stabilized disturbance test.
% COM_RC_IN_MODE must be 1 before this test and restored to 0 afterward.
% The plant starts airborne in level flight, remains outside Mission mode,
% and receives one body-axis moment pulse after an eight-second baseline.

if nargin < 1 || isempty(external_moment_body_nm)
    external_moment_body_nm = zeros(3, 1);
end
if nargin < 2 || isempty(pulse_duration_s)
    pulse_duration_s = 0.4;
end
if nargin < 3 || isempty(recovery_duration_s)
    recovery_duration_s = 10.0;
end
if nargin < 4 || isempty(cruise_throttle_milli)
    cruise_throttle_milli = 590;
end
if nargin < 5 || isempty(cruise_airspeed_m_s)
    cruise_airspeed_m_s = 13.0;
end
if nargin < 6 || isempty(manual_xyr_milli)
    manual_xyr_milli = zeros(3, 1);
end

validateattributes(external_moment_body_nm, {'numeric'}, {'vector', 'numel', 3, 'finite', 'real'});
validateattributes(pulse_duration_s, {'numeric'}, {'scalar', 'finite', '>=', 0, '<=', 2});
validateattributes(recovery_duration_s, {'numeric'}, {'scalar', 'finite', '>=', 5, '<=', 40});
validateattributes(cruise_throttle_milli, {'numeric'}, {'scalar', 'integer', '>=', 0, '<=', 1000});
validateattributes(cruise_airspeed_m_s, {'numeric'}, {'scalar', 'finite', '>=', 10, '<=', 20});
validateattributes(manual_xyr_milli, {'numeric'}, {'vector', 'numel', 3, 'finite', 'integer', '>=', -1000, '<=', 1000});
external_moment_body_nm = double(external_moment_body_nm(:));
manual_xyr_milli = double(manual_xyr_milli(:));

sample_time_s = 0.02;
hil_warmup_s = 3.0;
transition_s = hil_warmup_s;
mode_s = transition_s + 0.8;
arm_s = mode_s + 1.0;
plant_release_s = arm_s + 2.0;
baseline_duration_s = 8.0;
pulse_start_s = plant_release_s + baseline_duration_s;
pulse_end_s = pulse_start_s + pulse_duration_s;
test_end_s = pulse_end_s + recovery_duration_s;

tests_dir = fileparts(mfilename("fullpath"));
hitl_dir = fileparts(tests_dir);
root_dir = fileparts(hitl_dir);
addpath(hitl_dir);
addpath(fullfile(hitl_dir, "utils"));
addpath(fullfile(hitl_dir, "mavlink_backend"));
addpath(fullfile(root_dir, "matlab_model"));

cfg = hitl_config();
cfg.serial.port = "COM9";
cfg.sample_time = sample_time_s;
cfg.model.force_enable = 1;
cfg.runtime_control.enable_file_control = false;
cfg.mavlink.sysid = 245;
cfg.mavlink.compid = 190;
command_cfg = cfg;
command_cfg.serial.port = "COM5";

param = apply_hitl_model_switches(init_param_zx(), cfg);
param.ground.enable = false;
q_initial = euler_to_quat_wxyz(0, 0, 0);
x = [0; 0; -100; cruise_airspeed_m_s; 0; 0; q_initial(:); 0; 0; 0];
u = zeros(12, 1);
u_commanded = u;
actuator_delay_state = [];

manual_low = pymavlink_encode_manual_control(1, 0, 0, 0, 0, 0, cfg);
manual_cruise = pymavlink_encode_manual_control(1, 0, 0, cruise_throttle_milli, 0, 0, cfg);
manual_pulse = pymavlink_encode_manual_control(1, manual_xyr_milli(1), manual_xyr_milli(2), ...
    cruise_throttle_milli, manual_xyr_milli(3), 0, cfg);
cmd_transition_fw = pymavlink_encode_command_long(1, 0, 3000, [4 0 0 0 0 0 0], cfg);
cmd_stabilized = pymavlink_encode_command_long(1, 0, 176, [81 7 0 0 0 0 0], cfg);
cmd_arm = pymavlink_encode_command_long(1, 0, 400, [1 21196 0 0 0 0 0], cfg);
cmd_force_disarm = pymavlink_encode_command_long(1, 0, 400, [0 21196 0 0 0 0 0], cfg);
gcs_heartbeat = pymavlink_encode_gcs_heartbeat(cfg);

ser = serial_open(cfg);
command_ser = serial_open(command_cfg);
cleanup_obj = onCleanup(@() safe_disarm(command_ser, cmd_force_disarm, manual_low)); %#ok<NASGU>

history = init_history();
last_servo = nan(1, 8);
last_step_wall = NaN;
last_print_s = -Inf;
last_heartbeat_s = -Inf;
last_transition_s = -Inf;
last_mode_s = -Inf;
last_arm_s = -Inf;
plant_time_s = 0;
latest_vtol_state = 0;
latest_main_mode = 0;
latest_armed = false;
aborted = false;
abort_reason = "";

fprintf("[FW STAB] start: V=%.1f m/s throttle=%d/1000 Mext=[%+.3f %+.3f %+.3f] Nm pulse=%.2f s recovery=%.1f s\n", ...
    cruise_airspeed_m_s, cruise_throttle_milli, external_moment_body_nm, pulse_duration_s, recovery_duration_s);

t0 = tic;
next_tick_s = sample_time_s;
while true
    loop_tic = tic;
    wall_t = toc(t0);
    if wall_t >= test_end_s
        break;
    end

    if wall_t - last_heartbeat_s >= 0.5
        serial_write_bytes(command_ser, gcs_heartbeat);
        last_heartbeat_s = wall_t;
    end

    bytes = serial_read_bytes(ser);
    command_bytes = serial_read_bytes(command_ser);
    servo_msg = mavlink_decode_servo_output_raw(bytes, cfg);
    diagnostics = pymavlink_decode_diagnostics([bytes; command_bytes], cfg);

    if diagnostics.extended_sys_state_new
        latest_vtol_state = diagnostics.vtol_state;
    end
    if diagnostics.heartbeat_new
        latest_main_mode = double(bitand(bitshift(uint32(diagnostics.custom_mode), -16), uint32(255)));
        latest_armed = diagnostics.armed;
    end
    if diagnostics.statustext_new
        fprintf("[FW STAB] STATUSTEXT severity=%d text=%s\n", diagnostics.severity, diagnostics.text);
        if contains(lower(string(diagnostics.text)), "failsafe") || contains(lower(string(diagnostics.text)), "lockdown")
            aborted = true;
            abort_reason = "PX4 reported " + string(diagnostics.text);
            break;
        end
    end

    if servo_msg.is_new
        u_commanded = actuator_from_servo_output_raw(servo_msg, u_commanded, cfg);
        last_servo = double([servo_msg.servo1_raw servo_msg.servo2_raw servo_msg.servo3_raw servo_msg.servo4_raw ...
            servo_msg.servo5_raw servo_msg.servo6_raw servo_msg.servo7_raw servo_msg.servo8_raw]);
    end
    [u, actuator_delay_state] = apply_actuator_transport_delay(wall_t, u_commanded, actuator_delay_state, cfg);

    if wall_t >= transition_s && latest_vtol_state ~= 4 && wall_t - last_transition_s >= 0.5
        serial_write_bytes(command_ser, cmd_transition_fw);
        last_transition_s = wall_t;
    elseif wall_t >= mode_s && latest_vtol_state == 4 && latest_main_mode ~= 7 && wall_t - last_mode_s >= 0.5
        serial_write_bytes(command_ser, cmd_stabilized);
        last_mode_s = wall_t;
    elseif wall_t >= arm_s && latest_vtol_state == 4 && latest_main_mode == 7 && ~latest_armed && wall_t - last_arm_s >= 0.7
        serial_write_bytes(command_ser, cmd_arm);
        last_arm_s = wall_t;
    end

    if wall_t < arm_s
        serial_write_bytes(command_ser, manual_low);
    elseif wall_t >= pulse_start_s && wall_t < pulse_end_s
        serial_write_bytes(command_ser, manual_pulse);
    else
        serial_write_bytes(command_ser, manual_cruise);
    end

    if wall_t >= plant_release_s
        if isnan(last_step_wall)
            step_s = sample_time_s;
        else
            step_s = min(max(wall_t - last_step_wall, 0), cfg.model.max_runtime_step_s);
        end
        last_step_wall = wall_t;
        external_moment_now = zeros(3, 1);
        if wall_t >= pulse_start_s && wall_t < pulse_end_s
            external_moment_now = external_moment_body_nm;
        end
        x = integrate_midpoint_step(plant_time_s, x, u, param, step_s, external_moment_now);
        plant_time_s = plant_time_s + step_s;
    else
        external_moment_now = zeros(3, 1);
    end

    uav = state_to_uavdata_like(wall_t, x, u, param, cfg);
    payload = uavdata_to_hil_state_quaternion_payload(uav, cfg);
    serial_write_bytes(ser, mavlink_encode_hil_state_quaternion(payload, cfg));

    euler_deg = quat_to_euler_deg(x(7:10));
    history = append_history(history, wall_t, plant_time_s, x, euler_deg, last_servo, external_moment_now);

    if wall_t >= plant_release_s + 1
        if abs(euler_deg(1)) > 55 || abs(euler_deg(2)) > 40
            aborted = true;
            abort_reason = sprintf("attitude safety limit roll=%.1f pitch=%.1f deg", euler_deg(1), euler_deg(2));
            break;
        elseif norm(x(11:13)) > 2.0
            aborted = true;
            abort_reason = sprintf("rate safety limit %.2f rad/s", norm(x(11:13)));
            break;
        elseif norm(x(4:6)) < 7 || norm(x(4:6)) > 22
            aborted = true;
            abort_reason = sprintf("airspeed safety limit %.1f m/s", norm(x(4:6)));
            break;
        elseif x(3) > -20
            aborted = true;
            abort_reason = sprintf("altitude safety limit z=%.1f m", x(3));
            break;
        end
    end

    if wall_t - last_print_s >= 0.5
        fprintf("[FW STAB] wall=%.2f plant=%.2f phase=%s armed=%d vtol=%d mode=%d pos=[%+.1f %+.1f %+.1f] V=%.2f euler=[%+.2f %+.2f %+.2f] pqr=[%+.3f %+.3f %+.3f] servo56=[%.0f %.0f] diff=%+.0f\n", ...
            wall_t, plant_time_s, phase_name(wall_t, plant_release_s, pulse_start_s, pulse_end_s), ...
            latest_armed, latest_vtol_state, latest_main_mode, x(1:3), norm(x(4:6)), euler_deg, x(11:13), ...
            last_servo(5), last_servo(6), last_servo(6) - last_servo(5));
        last_print_s = wall_t;
    end

    elapsed_s = toc(loop_tic);
    now_s = toc(t0);
    if elapsed_s > sample_time_s || now_s > next_tick_s
        next_tick_s = now_s;
    else
        remaining_s = next_tick_s - now_s;
        if remaining_s > 0.012
            pause(remaining_s - 0.012);
        end
        while toc(t0) < next_tick_s
        end
    end
    next_tick_s = next_tick_s + sample_time_s;
end

safe_disarm(command_ser, cmd_force_disarm, manual_low);
clear ser command_ser;

result = evaluate_result(history, plant_release_s, pulse_start_s, pulse_end_s, test_end_s, external_moment_body_nm);
result.aborted = aborted;
result.abort_reason = abort_reason;
result.passed = result.passed && ~aborted && latest_vtol_state == 4 && latest_main_mode == 7;
result.external_moment_body_nm = external_moment_body_nm;
result.pulse_duration_s = pulse_duration_s;
result.recovery_duration_s = recovery_duration_s;
result.cruise_throttle_milli = cruise_throttle_milli;
result.cruise_airspeed_m_s = cruise_airspeed_m_s;
result.manual_xyr_milli = manual_xyr_milli;
result.history = history;

logs_dir = fullfile(hitl_dir, "logs");
if ~isfolder(logs_dir), mkdir(logs_dir); end
axis_name = disturbance_name(external_moment_body_nm);
result.log_file = fullfile(logs_dir, "native_fw_stabilized_" + axis_name + "_" + ...
    string(datetime("now", "Format", "yyyyMMdd_HHmmss")) + ".mat");
save(result.log_file, "result", "-v7.3");

fprintf("[FW STAB] result passed=%d aborted=%d baseline_rp_peak=%.2fdeg baseline_rate_peak=%.3frad/s disturbance_peak=[%.2f %.2f %.2f]deg recovery_rp=%.2fdeg recovery_rate=%.3frad/s log=%s\n", ...
    result.passed, result.aborted, result.baseline_roll_pitch_peak_deg, result.baseline_rate_peak_radps, ...
    result.disturbance_peak_deg, result.recovery_roll_pitch_error_deg, result.recovery_rate_norm_radps, result.log_file);
if aborted
    fprintf(2, "[FW STAB] ABORT: %s\n", abort_reason);
end
end

function result = evaluate_result(history, plant_release_s, pulse_start_s, pulse_end_s, test_end_s, moment)
baseline = history.wall_time_s >= plant_release_s + 2 & history.wall_time_s < pulse_start_s;
recovery = history.wall_time_s >= max(pulse_end_s, test_end_s - 2);
disturbance = history.wall_time_s >= pulse_start_s;
baseline_euler = median(history.euler_deg(:, baseline), 2);
delta_euler = wrap_deg(history.euler_deg - baseline_euler);

result.baseline_euler_deg = baseline_euler;
result.baseline_roll_pitch_peak_deg = max(abs(delta_euler(1:2, baseline)), [], "all");
result.baseline_rate_peak_radps = max(vecnorm(history.pqr_radps(:, baseline)));
result.disturbance_peak_deg = max(abs(delta_euler(:, disturbance)), [], 2);
result.disturbance_peak_rate_radps = max(abs(history.pqr_radps(:, disturbance)), [], 2);
result.recovery_roll_pitch_error_deg = max(abs(median(delta_euler(1:2, recovery), 2)));
result.recovery_rate_norm_radps = norm(median(history.pqr_radps(:, recovery), 2));
result.recovery_yaw_offset_deg = median(delta_euler(3, recovery));
result.minimum_speed_mps = min(history.speed_mps(history.wall_time_s >= plant_release_s));
result.maximum_speed_mps = max(history.speed_mps(history.wall_time_s >= plant_release_s));
result.altitude_change_m = -(history.position_ned_m(3, end) - median(history.position_ned_m(3, baseline)));

baseline_ok = result.baseline_roll_pitch_peak_deg <= 3.0 && result.baseline_rate_peak_radps <= 0.25;
recovery_ok = result.recovery_roll_pitch_error_deg <= 2.5 && result.recovery_rate_norm_radps <= 0.08;
if norm(moment) > 0
    disturbed_axis = find(abs(moment) == max(abs(moment)), 1);
    disturbance_was_effective = result.disturbance_peak_deg(disturbed_axis) >= 2.0;
else
    disturbance_was_effective = true;
end
result.passed = baseline_ok && recovery_ok && disturbance_was_effective;
end

function history = init_history()
history.wall_time_s = zeros(1, 0);
history.plant_time_s = zeros(1, 0);
history.position_ned_m = zeros(3, 0);
history.speed_mps = zeros(1, 0);
history.euler_deg = zeros(3, 0);
history.pqr_radps = zeros(3, 0);
history.servo = zeros(8, 0);
history.external_moment_body_nm = zeros(3, 0);
end

function history = append_history(history, wall_t, plant_t, x, euler_deg, servo, moment)
history.wall_time_s(end + 1) = wall_t;
history.plant_time_s(end + 1) = plant_t;
history.position_ned_m(:, end + 1) = x(1:3);
history.speed_mps(end + 1) = norm(x(4:6));
history.euler_deg(:, end + 1) = euler_deg(:);
history.pqr_radps(:, end + 1) = x(11:13);
history.servo(:, end + 1) = servo(:);
history.external_moment_body_nm(:, end + 1) = moment(:);
end

function euler_deg = quat_to_euler_deg(q_eb)
R_eb = quat_to_dcm_be(q_eb).';
euler_deg = rad2deg([atan2(R_eb(3, 2), R_eb(3, 3)); asin(-R_eb(3, 1)); atan2(R_eb(2, 1), R_eb(1, 1))]);
end

function value = wrap_deg(value)
value = mod(value + 180, 360) - 180;
end

function x_next = integrate_midpoint_step(t, x, u, param, step_s, external_moment_body_nm)
dx1 = tandem_zx_dynamics(t, x, u, param);
dx1(11:13) = dx1(11:13) + param.J \ external_moment_body_nm;
x_mid = x + 0.5 * step_s * dx1;
dx2 = tandem_zx_dynamics(t + 0.5 * step_s, x_mid, u, param);
dx2(11:13) = dx2(11:13) + param.J \ external_moment_body_nm;
x_next = x + step_s * dx2;
x_next(7:10) = x_next(7:10) / norm(x_next(7:10));
end

function name = phase_name(t, release_s, pulse_start_s, pulse_end_s)
if t < release_s, name = "setup";
elseif t < pulse_start_s, name = "baseline";
elseif t < pulse_end_s, name = "pulse";
else, name = "recovery";
end
end

function name = disturbance_name(moment)
if norm(moment) == 0
    name = "neutral";
else
    labels = ["roll", "pitch", "yaw"];
    [~, index] = max(abs(moment));
    direction = "pos";
    if moment(index) < 0, direction = "neg"; end
    name = labels(index) + "_" + direction;
end
end

function safe_disarm(ser, disarm_bytes, manual_low)
if isempty(ser) || ~isvalid(ser), return; end
for k = 1:5
    serial_write_bytes(ser, manual_low);
    serial_write_bytes(ser, disarm_bytes);
    pause(0.1);
end
end
