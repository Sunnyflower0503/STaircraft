function result = run_hitl_direct_landing()
%RUN_HITL_DIRECT_LANDING Dynamic no-route HITL plant for a vertical landing.
% PX4 is initialized in an airborne nose-up pose. A USB-side controller selects
% multicopter AUTO.LAND, arms, and releases dynamics through runtime_control.txt.

hitl_dir = fileparts(mfilename("fullpath"));
root_dir = fileparts(hitl_dir);
addpath(hitl_dir);
addpath(fullfile(hitl_dir, "utils"));
addpath(fullfile(hitl_dir, "mavlink_backend"));
addpath(fullfile(root_dir, "matlab_model"));

cfg = hitl_config();
cfg.model.init_mode = "airborne_nose_up_hover";
cfg.model.force_enable = 0;

landing_altitude_m = str2double(getenv("HITL_LANDING_ALT_M"));
if ~isfinite(landing_altitude_m) || landing_altitude_m <= 0
    landing_altitude_m = 8;
end
cfg.hover.altitude_m = landing_altitude_m;

max_wall_time_s = str2double(getenv("HITL_MAX_WALL_TIME_S"));
if ~isfinite(max_wall_time_s) || max_wall_time_s <= 0
    max_wall_time_s = 90;
end

cfg = update_runtime_control(cfg, 0, true);
param = init_param_zx();
param = apply_hitl_model_switches(param, cfg);
[param, x, u, meta] = prepare_airborne_nose_up_hover_for_hitl(param, cfg);
param.ground.enable = true;
airborne_start_state = x;
reference_time_s = str2double(getenv("HITL_REFERENCE_TIME_S"));
if ~isfinite(reference_time_s) || reference_time_s < 1
    reference_time_s = 3;
end
% HIL_STATE initializes PX4's local altitude reference from the first sample.
% Establish that reference at the configured ground AMSL before presenting the
% frozen airborne state; otherwise an 8 m airborne start appears as local z=0.
x(3) = 0;
airborne_initialized = false;
u_commanded = u;
actuator_delay_state = [];
[u, actuator_delay_state] = apply_actuator_transport_delay(0, u_commanded, actuator_delay_state, cfg);

log_file = string(getenv("HITL_EXPERIMENT_LOG"));
if strlength(log_file) == 0
    logs_dir = fullfile(hitl_dir, "logs");
    if ~isfolder(logs_dir), mkdir(logs_dir); end
    log_file = fullfile(logs_dir, "run_hitl_direct_landing_" + ...
        string(datetime("now", "Format", "yyyyMMdd_HHmmss")) + ".mat");
end

fprintf("========================================\n");
fprintf("Tandem tailsitter direct landing HITL\n");
fprintf("TELEM2 : %s @ %d\n", cfg.serial.port, cfg.serial.baudrate);
fprintf("Initial: %.1f m airborne, nose-up, dynamics frozen\n", landing_altitude_m);
fprintf("Origin : %.1f s ground-reference phase before frozen airborne state\n", reference_time_s);
fprintf("Contact: TD_CNTCT bitmask; rear=0x38, all=0x3F\n");
fprintf("Log    : %s\n", log_file);
fprintf("========================================\n");

ser = serial_open(cfg);
cleanup_obj = onCleanup(@() close_serial(ser)); %#ok<NASGU>
history = init_history();
last_servo = nan(1, 8);
last_servo_rx_s = NaN;
last_print_s = -inf;
last_history_s = -inf;
last_wall_s = 0;
plant_time_s = 0;
all_contact_timer_s = 0;
landing_confirmed = false;
landing_confirmed_wall_s = NaN;
all_contact_latched = false;
all_contact_first_wall_s = NaN;
rear_complete_seen = false;
rear_complete_wall_s = NaN;
first_contact_mask = uint8(0);
first_contact_wall_s = NaN;
front_before_rear = false;
disarmed_output_since_s = NaN;
aborted = false;
abort_reason = "";

t0 = tic;
while toc(t0) < max_wall_time_s
    loop_tic = tic;
    wall_s = toc(t0);
    wall_dt_s = max(0, wall_s - last_wall_s);
    last_wall_s = wall_s;
    cfg = update_runtime_control(cfg, wall_s);

    if ~airborne_initialized && wall_s >= reference_time_s
        x = airborne_start_state;
        airborne_initialized = true;
        fprintf("[DIRECT LAND] local origin established; presenting frozen %.1f m airborne state\n", ...
            landing_altitude_m);
    end

    bytes = serial_read_bytes(ser);
    servo_msg = mavlink_decode_servo_output_raw(bytes, cfg);
    if servo_msg.is_new
        last_servo = double([servo_msg.servo1_raw servo_msg.servo2_raw ...
            servo_msg.servo3_raw servo_msg.servo4_raw servo_msg.servo5_raw ...
            servo_msg.servo6_raw servo_msg.servo7_raw servo_msg.servo8_raw]);
        u_commanded = actuator_from_servo_output_raw(servo_msg, u_commanded, cfg);
        last_servo_rx_s = wall_s;
    end
    [u, actuator_delay_state] = apply_actuator_transport_delay( ...
        wall_s, u_commanded, actuator_delay_state, cfg);

    plant_step_s = 0;
    if cfg.model.force_enable == 1 && ~all_contact_latched
        plant_step_s = min(wall_dt_s, cfg.model.max_runtime_step_s);
        x = integrate_aircraft_step(plant_time_s, x, u, param, cfg, plant_step_s);
        plant_time_s = plant_time_s + plant_step_s;
    end

    if airborne_initialized
        contact_diag = hitl_ground_contact_diagnostics(x, param);
        contact_mask = hitl_contact_bitmask(contact_diag.active);
    else
        contact_mask = uint8(0);
    end
    rear_complete = bitand(contact_mask, uint8(56)) == uint8(56);
    any_front = bitand(contact_mask, uint8(7)) ~= 0;
    all_contact = contact_mask == uint8(63);

    if all_contact && ~all_contact_latched
        all_contact_latched = true;
        all_contact_first_wall_s = wall_s;
        x(4:6) = 0;
        x(11:13) = 0;
        fprintf("[DIRECT LAND] first all 6/6 contact at wall=%.3f s; " + ...
            "terminal contact pose latched\n", wall_s);
    end

    if contact_mask ~= 0 && first_contact_mask == 0
        first_contact_mask = contact_mask;
        first_contact_wall_s = wall_s;
        fprintf("[DIRECT LAND] first contact mask=0x%02X at wall=%.3f s\n", ...
            contact_mask, wall_s);
    end
    if any_front && ~rear_complete_seen
        front_before_rear = true;
    end
    if rear_complete && ~rear_complete_seen
        rear_complete_seen = true;
        rear_complete_wall_s = wall_s;
        fprintf("[DIRECT LAND] rear 3/3 complete at wall=%.3f s, mask=0x%02X, " + ...
            "vel=[%+.3f %+.3f %+.3f] m/s\n", wall_s, contact_mask, x(4:6));
    end

    if all_contact
        all_contact_timer_s = all_contact_timer_s + max(plant_step_s, wall_dt_s);
    else
        all_contact_timer_s = 0;
    end
    if ~landing_confirmed && all_contact_timer_s >= cfg.landing.confirm_s
        landing_confirmed = true;
        landing_confirmed_wall_s = wall_s;
        x(4:6) = 0;
        x(11:13) = 0;
        fprintf("[DIRECT LAND] all 6/6 confirmed for %.2f s at wall=%.3f s\n", ...
            cfg.landing.confirm_s, wall_s);
    end

    uav = state_to_uavdata_like(wall_s, x, u, param, cfg);
    payload = uavdata_to_hil_state_quaternion_payload(uav, cfg);
    serial_write_bytes(ser, mavlink_encode_hil_state_quaternion(payload, cfg, contact_mask));

    euler_deg = quat_to_euler_deg_local(x(7:10));
    if wall_s - last_history_s >= 0.02
        history = append_history(history, wall_s, plant_time_s, x, euler_deg, ...
            last_servo, contact_mask, cfg.model.force_enable);
        last_history_s = wall_s;
    end

    if cfg.model.force_enable == 1
        if ~airborne_initialized
            aborted = true;
            abort_reason = "dynamics released before local-origin initialization completed";
        end
        if isfinite(last_servo_rx_s) && wall_s - last_servo_rx_s > 1.0
            aborted = true;
            abort_reason = "SERVO_OUTPUT_RAW stale for more than 1 s";
        elseif norm(x(4:6)) > 8
            aborted = true;
            abort_reason = sprintf("speed hard limit exceeded: %.2f m/s", norm(x(4:6)));
        elseif norm(x(11:13)) > 3
            aborted = true;
            abort_reason = sprintf("body-rate hard limit exceeded: %.2f rad/s", norm(x(11:13)));
        elseif -x(3) > landing_altitude_m + 4 || x(3) > 3
            aborted = true;
            abort_reason = sprintf("altitude hard limit exceeded: z=%.2f m", x(3));
        end
    end
    if aborted
        fprintf(2, "[DIRECT LAND] ABORT: %s\n", abort_reason);
        break;
    end

    disarmed_outputs = all(isfinite(last_servo([1:4 7:8]))) ...
        && max(last_servo([1:4 7:8])) <= 950;
    if landing_confirmed && disarmed_outputs
        if ~isfinite(disarmed_output_since_s)
            disarmed_output_since_s = wall_s;
        elseif wall_s - disarmed_output_since_s >= 0.5
            fprintf("[DIRECT LAND] disarmed outputs held for 0.5 s; stopping.\n");
            break;
        end
    else
        disarmed_output_since_s = NaN;
    end

    if wall_s - last_print_s >= 0.5
        fprintf("[DIRECT LAND] wall=%5.1f plant=%5.1f force=%d mask=0x%02X " + ...
            "pos=[%+.2f %+.2f %+.2f] vel=[%+.2f %+.2f %+.2f] " + ...
            "rpy=[%+.1f %+.1f %+.1f] MAIN=[%s]\n", ...
            wall_s, plant_time_s, cfg.model.force_enable, contact_mask, ...
            x(1:3), x(4:6), euler_deg, sprintf("%.0f ", last_servo));
        last_print_s = wall_s;
    end

    loop_elapsed_s = toc(loop_tic);
    if loop_elapsed_s < cfg.sample_time
        pause(cfg.sample_time - loop_elapsed_s);
    end
end

result = struct();
result.started_at = string(datetime("now", "Format", "yyyy-MM-dd HH:mm:ss"));
result.log_file = log_file;
result.cfg_snapshot = cfg;
result.param_snapshot = param;
result.meta = meta;
result.history = history;
result.final_state = x;
result.final_servo = last_servo;
result.first_contact_mask = first_contact_mask;
result.first_contact_wall_s = first_contact_wall_s;
result.rear_complete_seen = rear_complete_seen;
result.rear_complete_wall_s = rear_complete_wall_s;
result.front_before_rear = front_before_rear;
result.all_contacts_confirmed = landing_confirmed;
result.all_contacts_wall_s = landing_confirmed_wall_s;
result.all_contact_latched = all_contact_latched;
result.all_contact_first_wall_s = all_contact_first_wall_s;
result.aborted = aborted;
result.abort_reason = abort_reason;
result.final_force_enable = cfg.model.force_enable;
result.reference_time_s = reference_time_s;
result.airborne_initialized = airborne_initialized;
save(log_file, "result", "-v7.3");
fprintf("[DIRECT LAND] saved %s\n", log_file);

if aborted
    error("run_hitl_direct_landing:Aborted", "%s", abort_reason);
end
end

function history = init_history()
history.wall_time_s = zeros(1, 0);
history.plant_time_s = zeros(1, 0);
history.position_ned_m = zeros(3, 0);
history.velocity_ned_mps = zeros(3, 0);
history.euler_deg = zeros(3, 0);
history.pqr_radps = zeros(3, 0);
history.servo_raw = zeros(8, 0);
history.contact_mask = zeros(1, 0, "uint8");
history.force_enable = zeros(1, 0);
end

function history = append_history(history, wall_s, plant_s, x, euler_deg, servo, mask, force_enable)
history.wall_time_s(end + 1) = wall_s;
history.plant_time_s(end + 1) = plant_s;
history.position_ned_m(:, end + 1) = x(1:3);
history.velocity_ned_mps(:, end + 1) = x(4:6);
history.euler_deg(:, end + 1) = euler_deg(:);
history.pqr_radps(:, end + 1) = x(11:13);
history.servo_raw(:, end + 1) = servo(:);
history.contact_mask(end + 1) = mask;
history.force_enable(end + 1) = force_enable;
end

function euler_deg = quat_to_euler_deg_local(q_eb)
R_eb = quat_to_dcm_be(q_eb).';
pitch = asin(-R_eb(3, 1));
roll = atan2(R_eb(3, 2), R_eb(3, 3));
yaw = atan2(R_eb(2, 1), R_eb(1, 1));
euler_deg = rad2deg([roll; pitch; yaw]);
end

function close_serial(ser)
try
    if ~isempty(ser)
        clear ser;
    end
catch
end
end
