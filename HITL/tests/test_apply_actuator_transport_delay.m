function test_apply_actuator_transport_delay()
cfg.actuator_delay.motor_s = 0.3;
cfg.actuator_delay.elevon_s = 0.2;
state = [];

u0 = zeros(12, 1);
u0(11:12) = [-0.1; 0.1];
[~, state] = apply_actuator_transport_delay(0.0, u0, state, cfg);

u1 = ones(12, 1);
u1(11:12) = [0.2; -0.2];
[u, state] = apply_actuator_transport_delay(0.1, u1, state, cfg);
assert(max(abs(u - u0)) < 1e-12);

[u, state] = apply_actuator_transport_delay(0.21, u1, state, cfg);
assert(max(abs(u(1:10) - u0(1:10))) < 1e-12);
assert(max(abs(u(11:12) - u0(11:12))) < 1e-12);

[u, ~] = apply_actuator_transport_delay(0.31, u1, state, cfg);
assert(max(abs(u(1:10) - u0(1:10))) < 1e-12);
assert(max(abs(u(11:12) - u1(11:12))) < 1e-12);

[u, ~] = apply_actuator_transport_delay(0.41, u1, state, cfg);
assert(max(abs(u - u1)) < 1e-12);
end
