function hitl_main(max_time_s)
%HITL_MAIN Run the MATLAB HITL bridge loop.

if nargin < 1
    max_time_s = inf;
end

hitl_dir = fileparts(mfilename("fullpath"));
root_dir = fileparts(hitl_dir);
addpath(hitl_dir);
addpath(fullfile(hitl_dir, "utils"));
addpath(fullfile(hitl_dir, "mavlink_backend"));
addpath(fullfile(root_dir, "matlab_model"));

cfg = hitl_config();
param = init_param_zx();
if string(cfg.model.init_mode) == "stand_static"
    [param, x, u, meta] = prepare_stand_static_for_hitl(param, cfg);
else
    x = initial_state_from_param(param, cfg);
    u = zeros(12, 1);
    meta = struct("mode", string(cfg.model.init_mode), "euler_deg", [NaN; NaN; NaN]);
end

uav0 = state_to_uavdata_like(0, x, u, param, cfg);
fprintf("[HITL] Initial geodetic position:\nlat=%.6f lon=%.6f AMSL=%.0f heading=%.0f\n", ...
    uav0.lat_deg, uav0.lon_deg, uav0.AMSL, cfg.init.heading_deg);
fprintf("[HITL] Initial mode:\nforce_enable=%d init_mode=%s stand_euler=[%.2f %.2f %.2f]\n", ...
    cfg.model.force_enable, string(cfg.model.init_mode), meta.euler_deg(1), meta.euler_deg(2), meta.euler_deg(3));

ser = serial_open(cfg);
cleanup = onCleanup(@() clear("ser")); %#ok<NASGU>

pacer_state = real_time_pacer([], 0, cfg);
t = 0;
last_print_t = -inf;

while t < max_time_s
    cfg = update_runtime_control(cfg, t);
    bytes = serial_read_bytes(ser);
    servo_msg = mavlink_decode_servo_output_raw(bytes, cfg);
    u = actuator_from_servo_output_raw(servo_msg, u, cfg);

    x = integrate_aircraft_step(t, x, u, param, cfg);

    uav = state_to_uavdata_like(t, x, u, param, cfg);
    payload = uavdata_to_hil_state_quaternion_payload(uav, cfg);
    tx_bytes = mavlink_encode_hil_state_quaternion(payload, cfg);
    serial_write_bytes(ser, tx_bytes);

    if t - last_print_t >= 1
        fprintf("t=%.2f is_new=%d u_throttle=[%s] lat=%.7f lon=%.7f AMSL=%.2f TAS=%.2f EAS=%.2f force_enable=%d\n", ...
            t, logical(servo_msg.is_new), sprintf("%.2f ", u(1:10)), ...
            uav.lat_deg, uav.lon_deg, uav.AMSL, uav.TAS, uav.EAS, cfg.model.force_enable);
        last_print_t = t;
    end

    pacer_state = real_time_pacer(pacer_state, t + cfg.sample_time, cfg);
    t = t + cfg.sample_time;
end
end
