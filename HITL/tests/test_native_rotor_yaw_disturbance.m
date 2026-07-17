function result = test_native_rotor_yaw_disturbance(sample_time_s, yaw_pulse_milli, throttle_milli, virtual_xz_gain, pulse_duration_s, external_moment_body_nm, recovery_duration_s, flight_mode, external_force_ned_n)
%TEST_NATIVE_ROTOR_YAW_DISTURBANCE Native MATLAB HITL yaw pulse test.
% COM_RC_IN_MODE must already be 1 (direct MAVLink joystick). The caller
% must restore it to 0 afterward.
% Set yaw_pulse_milli=0 and external_moment_body_nm=[Mx;My;Mz] to test
% recovery from an independent body-axis moment disturbance.

if nargin < 1 || isempty(sample_time_s)
    sample_time_s = 0.02; % Editable default: 50 Hz.
end
if nargin < 2 || isempty(yaw_pulse_milli)
    yaw_pulse_milli = 30; % 3% yaw stick; increase only after checking the log.
end
if nargin < 3 || isempty(throttle_milli)
    % Direct joystick mode: this maps through MPC_THR_HOVER=0.685 to about
    % 0.656 actuator thrust, matching the 3.2 kg model hover point.
    throttle_milli = 476;
end
if nargin < 4 || isempty(virtual_xz_gain)
    virtual_xz_gain = 0; % Zero is the real firmware path.
end
if nargin < 5 || isempty(pulse_duration_s)
    pulse_duration_s = 0.4;
end
if nargin < 6 || isempty(external_moment_body_nm)
    external_moment_body_nm = zeros(3, 1);
end
if nargin < 7 || isempty(recovery_duration_s)
    recovery_duration_s = 5.0;
end
if nargin < 8 || isempty(flight_mode)
    flight_mode = "stabilized";
end
if nargin < 9 || isempty(external_force_ned_n)
    external_force_ned_n = zeros(3, 1);
end
validateattributes(sample_time_s, {'numeric'}, {'scalar', 'finite', 'positive'});
validateattributes(yaw_pulse_milli, {'numeric'}, ...
    {'scalar', 'finite', 'integer', '>=', -1000, '<=', 1000});
validateattributes(throttle_milli, {'numeric'}, ...
    {'scalar', 'finite', 'integer', '>=', 0, '<=', 1000});
validateattributes(virtual_xz_gain, {'numeric'}, {'scalar', 'finite'});
validateattributes(pulse_duration_s, {'numeric'}, ...
    {'scalar', 'finite', '>=', 0.1, '<=', 2.0});
validateattributes(external_moment_body_nm, {'numeric'}, ...
    {'vector', 'numel', 3, 'finite', 'real'});
external_moment_body_nm = double(external_moment_body_nm(:));
flight_mode = lower(string(flight_mode));
mustBeMember(flight_mode, ["stabilized", "altitude", "position"]);
validateattributes(external_force_ned_n, {'numeric'}, ...
    {'vector', 'numel', 3, 'finite', 'real'});
external_force_ned_n = double(external_force_ned_n(:));
validateattributes(recovery_duration_s, {'numeric'}, ...
    {'scalar', 'finite', '>=', 3.0, '<=', 15.0});

if nargin >= 8 && lower(string(flight_mode)) == "position"
    pulse_start_s = 10.0;
else
    pulse_start_s = 6.5;
end
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
cfg.sample_time = double(sample_time_s);
cfg.model.force_enable = 1;
cfg.runtime_control.enable_file_control = false;
cfg.hover.Euler_deg = [0; 90; 0];
cfg.mavlink.sysid = 245;
cfg.mavlink.compid = 190;

param = init_param_zx();
param = apply_hitl_model_switches(param, cfg);
[param, x, u, meta] = prepare_airborne_nose_up_hover_for_hitl(param, cfg);
q_initial = x(7:10);

manual_low = pymavlink_encode_manual_control(1, 0, 0, 0, 0, 0, cfg);
manual_hover = pymavlink_encode_manual_control(1, 0, 0, throttle_milli, 0, 0, cfg);
manual_pulse = pymavlink_encode_manual_control(1, 0, 0, throttle_milli, yaw_pulse_milli, 0, cfg);
manual_takeoff = pymavlink_encode_manual_control(1, 0, 0, 700, 0, 0, cfg);
cmd_transition_mc = pymavlink_encode_command_long(1, 0, 3000, [3 0 0 0 0 0 0], cfg);
mode_number = struct("stabilized", 7, "altitude", 2, "position", 3);
cmd_control_mode = pymavlink_encode_command_long(1, 0, 176, ...
    [81 mode_number.(flight_mode) 0 0 0 0 0], cfg);
cmd_altitude_mode = pymavlink_encode_command_long(1, 0, 176, [81 2 0 0 0 0 0], cfg);
cmd_arm = pymavlink_encode_command_long(1, 0, 400, [1 21196 0 0 0 0 0], cfg);
cmd_force_disarm = pymavlink_encode_command_long(1, 0, 400, [0 21196 0 0 0 0 0], cfg);

ser = serial_open(cfg);
cleanup_obj = onCleanup(@() safe_disarm_and_close(ser, cmd_force_disarm, manual_low));

history = init_history();
last_servo = nan(1, 8);
last_print = -inf;
last_step_wall = NaN;
plant_time = 0;
loop_count = 0;
overrun_count = 0;
aborted = false;
abort_reason = "";
diagnostic_events = strings(0, 1);
transition_sent = false;
control_mode_sent = false;
position_mode_sent = false;
last_arm_attempt_s = -inf;

fprintf("[NATIVE DIST] COM9 native MATLAB loop started. mode=%s Target dt=%.3f s, throttle=%d/1000, yaw r=%d/1000, pulse=%.2f s, Mext=[%+.3f %+.3f %+.3f] Nm Fned=[%+.2f %+.2f %+.2f] N\n", ...
    flight_mode, cfg.sample_time, throttle_milli, yaw_pulse_milli, pulse_duration_s, ...
    external_moment_body_nm, external_force_ned_n);
t0 = tic;
next_tick_s = cfg.sample_time;
while true
    loop_tic = tic;
    wall_t = toc(t0);
    if wall_t >= test_end_s
        break;
    end

    bytes = serial_read_bytes(ser);
    servo_msg = mavlink_decode_servo_output_raw(bytes, cfg);
    diagnostics = pymavlink_decode_diagnostics(bytes, cfg);
    if diagnostics.command_ack_new
        event = sprintf("wall=%.2f COMMAND_ACK command=%d result=%d", ...
            wall_t, diagnostics.command, diagnostics.ack_result);
        diagnostic_events(end + 1, 1) = event; %#ok<AGROW>
        fprintf("[NATIVE DIST] %s\n", event);
    end
    if diagnostics.statustext_new
        event = sprintf("wall=%.2f STATUSTEXT severity=%d text=%s", ...
            wall_t, diagnostics.severity, diagnostics.text);
        diagnostic_events(end + 1, 1) = event; %#ok<AGROW>
        fprintf("[NATIVE DIST] %s\n", event);
    end
    if servo_msg.is_new
        u = actuator_from_servo_output_raw(servo_msg, u, cfg);
        last_servo = [servo_msg.servo1_raw servo_msg.servo2_raw servo_msg.servo3_raw servo_msg.servo4_raw ...
                      servo_msg.servo5_raw servo_msg.servo6_raw servo_msg.servo7_raw servo_msg.servo8_raw];
    end

    if wall_t >= 4.8 && (all(isnan(last_servo)) || max(last_servo(1:4)) < 1000)
        aborted = true;
        abort_reason = "arming/output confirmation failed";
        break;
    end

    external_moment_now = zeros(3, 1);
    external_force_now = zeros(3, 1);
    if wall_t >= pulse_start_s && wall_t < pulse_end_s
        external_moment_now = external_moment_body_nm;
        external_force_now = external_force_ned_n;
    end

    if wall_t >= 5.0
        if isnan(last_step_wall)
            step_s = cfg.sample_time;
        else
            step_s = min(max(wall_t - last_step_wall, 0), cfg.model.max_runtime_step_s);
        end
        last_step_wall = wall_t;
        u_model = apply_virtual_xz_decoupling(u, virtual_xz_gain);
        x = integrate_midpoint_step(plant_time, x, u_model, param, step_s, ...
            external_moment_now, external_force_now);
        plant_time = plant_time + step_s;
    end

    uav = state_to_uavdata_like(plant_time, x, u, param, cfg);
    payload = uavdata_to_hil_state_quaternion_payload(uav, cfg);
    serial_write_bytes(ser, mavlink_encode_hil_state_quaternion(payload, cfg));

    if wall_t < 3.0
        serial_write_bytes(ser, manual_low);
    elseif flight_mode ~= "stabilized" && wall_t < 5.0
        % The HIL plant is frozen during this interval. A short climb command
        % lets the PX4 takeoff state machine leave the landed/ramp state;
        % center the stick when dynamics are released at wall_t=5 s.
        serial_write_bytes(ser, manual_takeoff);
    elseif wall_t < pulse_start_s || wall_t >= pulse_end_s
        serial_write_bytes(ser, manual_hover);
    else
        serial_write_bytes(ser, manual_pulse);
    end

    if wall_t >= 0.4 && ~transition_sent
        serial_write_bytes(ser, cmd_transition_mc);
        transition_sent = true;
    elseif wall_t >= 1.2 && ~control_mode_sent
        if flight_mode == "position"
            % Horizontal position is not valid early enough for POSCTL.
            % Enter ALTCTL for arming/takeoff and switch after HIL aiding is valid.
            serial_write_bytes(ser, cmd_altitude_mode);
        else
            serial_write_bytes(ser, cmd_control_mode);
        end
        control_mode_sent = true;
    elseif wall_t >= 2.2 && wall_t < 4.6 ...
            && (all(isnan(last_servo)) || max(last_servo(1:4)) < 1000) ...
            && wall_t - last_arm_attempt_s >= 0.7
        serial_write_bytes(ser, cmd_arm);
        last_arm_attempt_s = wall_t;
    end

    if flight_mode == "position" && wall_t >= 5.8 && ~position_mode_sent
        serial_write_bytes(ser, cmd_control_mode);
        position_mode_sent = true;
    end

    attitude_error_deg = quaternion_distance_deg(x(7:10), q_initial);
    rate_norm = norm(x(11:13));
    history = append_history(history, wall_t, plant_time, x, last_servo, ...
        attitude_error_deg, q_initial, external_moment_now, external_force_now);
    if attitude_error_deg > 30
        aborted = true;
        abort_reason = sprintf("attitude error %.2f deg", attitude_error_deg);
        break;
    elseif rate_norm > 2
        aborted = true;
        abort_reason = sprintf("rate norm %.3f rad/s", rate_norm);
        break;
    elseif norm(x(4:6)) > 20
        aborted = true;
        abort_reason = sprintf("speed norm %.2f m/s", norm(x(4:6)));
        break;
    elseif norm(x(1:2)) > 100 || abs(x(3) - meta.position_ned(3)) > 50
        aborted = true;
        abort_reason = sprintf("position excursion [%.1f %.1f %.1f] m", x(1), x(2), x(3));
        break;
    end

    if wall_t - last_print >= 0.5
        phase = phase_name(wall_t, pulse_start_s, pulse_end_s);
        spin_deg = history.relative_rotation_deg(1, end);
        tilt_deg = norm(history.relative_rotation_deg(2:3, end));
        fprintf("[NATIVE DIST] wall=%.2f plant=%.2f phase=%s pos=[%+.2f %+.2f %+.2f] vel=[%+.2f %+.2f %+.2f] pqr=[%+.4f %+.4f %+.4f] err=%.2f spin=%+.2f tilt=%.2f tip=%g/%g servo=[%s]\n", ...
            wall_t, plant_time, phase, x(1:3), x(4:6), ...
            x(11), x(12), x(13), attitude_error_deg, ...
            spin_deg, tilt_deg, last_servo(7), last_servo(8), sprintf("%.0f ", last_servo));
        last_print = wall_t;
    end

    loop_count = loop_count + 1;
    elapsed = toc(loop_tic);
    now_s = toc(t0);
    if elapsed > cfg.sample_time || now_s > next_tick_s
        overrun_count = overrun_count + 1;
        % Drop a missed deadline instead of running catch-up iterations.
        next_tick_s = now_s;
    else
        % Windows pause() can oversleep at 20 ms periods. Sleep most of the
        % remainder, then use an absolute deadline for a stable editable
        % 50 Hz MATLAB loop.
        remaining_s = next_tick_s - now_s;
        if remaining_s > 0.012
            pause(remaining_s - 0.012);
        end
        while toc(t0) < next_tick_s
        end
    end
    next_tick_s = next_tick_s + cfg.sample_time;
end

safe_disarm_and_close(ser, cmd_force_disarm, manual_low);
clear ser;

result = struct();
result.history = history;
result.aborted = aborted;
result.abort_reason = abort_reason;
result.loop_count = loop_count;
result.overrun_count = overrun_count;
result.wall_duration_s = toc(t0);
active_duration_s = history.wall_time_s(end) - history.wall_time_s(1);
result.mean_loop_hz = (loop_count - 1) / max(active_duration_s, eps);
result.final_state = x;
result.final_servo = last_servo;
result.yaw_pulse_milli = yaw_pulse_milli;
result.throttle_milli = throttle_milli;
result.virtual_xz_gain = virtual_xz_gain;
result.pulse_duration_s = pulse_duration_s;
result.external_moment_body_nm = external_moment_body_nm;
result.external_force_ned_n = external_force_ned_n;
result.recovery_duration_s = recovery_duration_s;
result.flight_mode = flight_mode;
result.diagnostic_events = diagnostic_events;
result.cfg_snapshot = cfg;
result.param_snapshot = param;
result.meta = meta;

logs_dir = fullfile(hitl_dir, "logs");
if ~isfolder(logs_dir), mkdir(logs_dir); end
if flight_mode ~= "stabilized"
    log_prefix = "native_rotor_" + flight_mode + "_";
elseif any(external_moment_body_nm ~= 0) || any(external_force_ned_n ~= 0)
    log_prefix = "native_rotor_external_moment_";
else
    log_prefix = "native_rotor_yaw_disturbance_";
end
result.log_file = fullfile(logs_dir, log_prefix + string(datetime("now", "Format", "yyyyMMdd_HHmmss")) + ".mat");
save(result.log_file, "result", "-v7.3");
fprintf("[NATIVE DIST] done: hz=%.1f overruns=%d/%d aborted=%d reason=%s log=%s\n", ...
    result.mean_loop_hz, overrun_count, loop_count, aborted, abort_reason, result.log_file);
end

function history = init_history()
history.wall_time_s = zeros(1, 0);
history.plant_time_s = zeros(1, 0);
history.pqr_radps = zeros(3, 0);
history.attitude_error_deg = zeros(1, 0);
history.quaternion = zeros(4, 0);
history.relative_rotation_deg = zeros(3, 0);
history.servo = zeros(8, 0);
history.external_moment_body_nm = zeros(3, 0);
history.external_force_ned_n = zeros(3, 0);
history.position_ned_m = zeros(3, 0);
history.velocity_ned_mps = zeros(3, 0);
end

function history = append_history(history, wall_t, plant_t, x, servo, attitude_error_deg, q_initial, external_moment_body_nm, external_force_ned_n)
history.wall_time_s(end + 1) = wall_t;
history.plant_time_s(end + 1) = plant_t;
history.pqr_radps(:, end + 1) = x(11:13);
history.attitude_error_deg(end + 1) = attitude_error_deg;
history.quaternion(:, end + 1) = x(7:10);
history.relative_rotation_deg(:, end + 1) = relative_rotation_vector_deg(x(7:10), q_initial);
history.servo(:, end + 1) = servo(:);
history.external_moment_body_nm(:, end + 1) = external_moment_body_nm(:);
history.external_force_ned_n(:, end + 1) = external_force_ned_n(:);
history.position_ned_m(:, end + 1) = x(1:3);
history.velocity_ned_mps(:, end + 1) = x(4:6);
end

function rotation_deg = relative_rotation_vector_deg(q, q0)
q = q(:) / norm(q);
q0 = q0(:) / norm(q0);
q0_conj = [q0(1); -q0(2:4)];
q_rel = [q0_conj(1) * q(1) - dot(q0_conj(2:4), q(2:4)); ...
         q0_conj(1) * q(2:4) + q(1) * q0_conj(2:4) + cross(q0_conj(2:4), q(2:4))];
q_rel = q_rel / norm(q_rel);
if q_rel(1) < 0
    q_rel = -q_rel;
end
v_norm = norm(q_rel(2:4));
if v_norm < 1e-12
    rotation_deg = zeros(3, 1);
else
    angle = 2 * atan2(v_norm, q_rel(1));
    rotation_deg = rad2deg(angle) * q_rel(2:4) / v_norm;
end
end

function u_model = apply_virtual_xz_decoupling(u, gain)
u_model = u(:);
if gain == 0
    return;
end

% u(9:10) are left/right wingtip throttles. Positive (left-right) tip
% differential needs positive mixer yaw0: MAIN1/4 down, MAIN2/3 up.
tip_spin = 0.5 * (u_model(9) - u_model(10));
r_comp = gain * tip_spin;
u_model(3:4) = min(max(u_model(3:4) - r_comp, 0), 1); % MAIN1
u_model(5:6) = min(max(u_model(5:6) + r_comp, 0), 1); % MAIN2
u_model(1:2) = min(max(u_model(1:2) + r_comp, 0), 1); % MAIN3
u_model(7:8) = min(max(u_model(7:8) - r_comp, 0), 1); % MAIN4
end

function name = phase_name(t, pulse_start_s, pulse_end_s)
if t < 3.0, name = "setup";
elseif t < 5.0, name = "prespool";
elseif t < pulse_start_s, name = "baseline";
elseif t < pulse_end_s, name = "pulse";
else, name = "recovery";
end
end

function angle_deg = quaternion_distance_deg(q, q0)
q = q(:) / norm(q);
q0 = q0(:) / norm(q0);
angle_deg = rad2deg(2 * acos(min(max(abs(dot(q, q0)), -1), 1)));
end

function x_next = integrate_midpoint_step(t, x, u, param, step_s, external_moment_body_nm, external_force_ned_n)
dx1 = tandem_zx_dynamics(t, x, u, param);
dx1(4:6) = dx1(4:6) + external_force_ned_n / param.m;
dx1(11:13) = dx1(11:13) + param.J \ external_moment_body_nm;
x_mid = x + 0.5 * step_s * dx1;
dx2 = tandem_zx_dynamics(t + 0.5 * step_s, x_mid, u, param);
dx2(4:6) = dx2(4:6) + external_force_ned_n / param.m;
dx2(11:13) = dx2(11:13) + param.J \ external_moment_body_nm;
x_next = x + step_s * dx2;
x_next(7:10) = x_next(7:10) / norm(x_next(7:10));
end

function safe_disarm_and_close(ser, disarm_bytes, manual_low)
if isempty(ser) || ~isvalid(ser), return; end
for k = 1:5
    serial_write_bytes(ser, manual_low);
    serial_write_bytes(ser, disarm_bytes);
    pause(0.1);
end
try
    flush(ser);
catch
end
end
