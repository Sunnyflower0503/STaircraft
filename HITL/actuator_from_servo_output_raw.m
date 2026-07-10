function u = actuator_from_servo_output_raw(servo_msg, u_prev, cfg)
%ACTUATOR_FROM_SERVO_OUTPUT_RAW Convert PX4 PWM outputs to model input u.

if ~isfield(servo_msg, "is_new") || ~servo_msg.is_new
    u = u_prev(:);
    return;
end

if isempty(cfg.elevon_pwm_breakpoints) || isempty(cfg.elevon_deg_table)
    error("Missing elevon lookup table from Simulink 1-D Lookup Table.");
end

s1 = double(servo_msg.servo1_raw);
s2 = double(servo_msg.servo2_raw);
s3 = double(servo_msg.servo3_raw);
s4 = double(servo_msg.servo4_raw);
s5 = double(servo_msg.servo5_raw);
s6 = double(servo_msg.servo6_raw);
s7 = double(servo_msg.servo7_raw);
s8 = double(servo_msg.servo8_raw);

dt1  = sat01((s3 - 1000) * 0.001);
dt2  = sat01((s3 - 1000) * 0.001);
dt3  = sat01((s1 - 1000) * 0.001);
dt4  = sat01((s1 - 1000) * 0.001);
dt5  = sat01((s2 - 1000) * 0.001);
dt6  = sat01((s2 - 1000) * 0.001);
dt7  = sat01((s4 - 1000) * 0.001);
dt8  = sat01((s4 - 1000) * 0.001);
dt9  = sat01((s7 - 1000) * 0.001);
dt10 = sat01((s8 - 1000) * 0.001);

daeL_deg = -interp1(cfg.elevon_pwm_breakpoints, cfg.elevon_deg_table, s5, "linear", "extrap");
daeR_deg = -interp1(cfg.elevon_pwm_breakpoints, cfg.elevon_deg_table, s6, "linear", "extrap");

daeL = deg2rad(daeL_deg);
daeR = deg2rad(daeR_deg);

if ~isempty(cfg.elevon.min_rad) && ~isempty(cfg.elevon.max_rad)
    daeL = min(max(daeL, cfg.elevon.min_rad), cfg.elevon.max_rad);
    daeR = min(max(daeR, cfg.elevon.min_rad), cfg.elevon.max_rad);
end

u = [dt1; dt2; dt3; dt4; dt5; dt6; dt7; dt8; dt9; dt10; daeL; daeR];
end
