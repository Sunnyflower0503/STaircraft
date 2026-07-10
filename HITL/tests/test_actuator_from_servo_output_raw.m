function test_actuator_from_servo_output_raw()
root = fileparts(fileparts(mfilename("fullpath")));
addpath(root); addpath(fullfile(root, "utils"));
cfg = hitl_config();
cfg.elevon_pwm_breakpoints = [1000 1500 2000];
cfg.elevon_deg_table = [-20 0 20];
msg = struct("is_new", true, "servo1_raw", 1200, "servo2_raw", 1300, ...
    "servo3_raw", 1400, "servo4_raw", 1500, "servo5_raw", 1500, ...
    "servo6_raw", 1500, "servo7_raw", 1600, "servo8_raw", 1700, "timestamp", 0);
u = actuator_from_servo_output_raw(msg, zeros(12, 1), cfg);
expected = [0.4; 0.4; 0.2; 0.2; 0.3; 0.3; 0.5; 0.5; 0.6; 0.7; 0; 0];
assert(max(abs(u - expected)) < 1e-12, "Actuator mapping mismatch.");
msg.is_new = false;
u_prev = (1:12).';
assert(isequal(actuator_from_servo_output_raw(msg, u_prev, cfg), u_prev), "Previous actuator command was not held.");
end
