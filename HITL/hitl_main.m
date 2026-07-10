function hitl_main(max_time_s)
%HITL_MAIN Run the MATLAB HITL bridge loop.

if nargin < 1
    max_time_s = inf;
end

hitl_dir = fileparts(mfilename("fullpath"));
root_dir = fileparts(hitl_dir);
addpath(hitl_dir);
addpath(fullfile(hitl_dir, "utils"));
addpath(fullfile(root_dir, "matlab_model"));

cfg = hitl_config();
param = init_param_zx();
x = initial_state_from_param(param, cfg);
u = zeros(12, 1);
ser = serial_open(cfg);
cleanup = onCleanup(@() clear("ser")); %#ok<NASGU>

pacer_state = real_time_pacer([], 0, cfg);
t = 0;
last_print_t = -inf;

while t < max_time_s
    bytes = serial_read_bytes(ser);
    servo_msg = mavlink_decode_servo_output_raw(bytes, cfg);
    u = actuator_from_servo_output_raw(servo_msg, u, cfg);

    [~, z] = Runge_Kutta4(@(tt, xx) tandem_zx_dynamics(tt, xx, u, param), [t t + cfg.dt], x);
    x = z(:, end);
    x(7:10) = quat_normalize(x(7:10));

    uav = state_to_uavdata_like(t, x, u, param, cfg);
    payload = uavdata_to_hil_state_quaternion_payload(uav, cfg);
    tx_bytes = mavlink_encode_hil_state_quaternion(payload, cfg);
    serial_write_bytes(ser, tx_bytes);

    if t - last_print_t >= 1
        fprintf("t=%.2f is_new=%d u_throttle=[%s] lat=%.7f lon=%.7f AMSL=%.2f TAS=%.2f EAS=%.2f\n", ...
            t, logical(servo_msg.is_new), sprintf("%.2f ", u(1:10)), ...
            uav.lat_deg, uav.lon_deg, uav.AMSL, uav.TAS, uav.EAS);
        last_print_t = t;
    end

    pacer_state = real_time_pacer(pacer_state, t + cfg.dt, cfg);
    t = t + cfg.dt;
end
end
