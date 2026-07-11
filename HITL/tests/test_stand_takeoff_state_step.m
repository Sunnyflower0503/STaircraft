function test_stand_takeoff_state_step()
root = fileparts(fileparts(mfilename("fullpath")));
addpath(root); addpath(fullfile(root, "utils")); addpath(fullfile(fileparts(root), "matlab_model"));

cfg = hitl_config();
cfg.stand.release_throttle = 0.4;
cfg.stand.release_hold_s = 0.1;
cfg.landing.liftoff_confirm_s = 0.05;
cfg.landing.min_active_contacts = 5;
cfg.landing.confirm_s = 0.1;

dt = 0.01;

state = initial_stand_takeoff_state();
for k = 1:20
    state = stand_takeoff_state_step(state, 0.40, 6, dt, cfg);
end
assert(~state.stand_released, "Throttle exactly 0.40 must not release the stand.");
assert(state.phase == "STAND_HOLD", "0.40 throttle should stay in STAND_HOLD.");

state = initial_stand_takeoff_state();
for k = 1:9
    state = stand_takeoff_state_step(state, 0.41, 6, dt, cfg);
end
assert(~state.stand_released, "0.41 throttle shorter than 0.1 s must not release.");

state = initial_stand_takeoff_state();
release_count = 0;
for k = 1:10
    state = stand_takeoff_state_step(state, 0.41, 6, dt, cfg);
    release_count = release_count + double(state.just_released);
end
assert(state.stand_released, "0.41 throttle held for 0.1 s should release.");
assert(state.phase == "FLIGHT", "Released stand should enter FLIGHT.");
assert(release_count == 1, "Stand release event should occur exactly once.");

state = stand_takeoff_state_step(state, 0.0, 6, dt, cfg);
assert(state.stand_released, "Stand must not recover after throttle is lowered.");
assert(state.phase == "FLIGHT", "Released stand must remain in FLIGHT before landing.");

state = initial_stand_takeoff_state();
state.phase = "FLIGHT";
state.stand_released = true;
for k = 1:20
    state = stand_takeoff_state_step(state, 0.0, 5, dt, cfg);
end
assert(~state.liftoff_confirmed, "Contact before liftoff should not confirm liftoff.");
assert(state.phase == "FLIGHT", "5/6 contact before liftoff must not end the run.");

state = initial_stand_takeoff_state();
state.phase = "FLIGHT";
state.stand_released = true;
for k = 1:20
    state = stand_takeoff_state_step(state, 0.0, 6, dt, cfg);
end
assert(~state.liftoff_confirmed, "6/6 contact before liftoff should not confirm liftoff.");
assert(state.phase == "FLIGHT", "6/6 contact before liftoff must not end the run.");

state = make_liftoff_state(cfg, dt);
for k = 1:20
    state = stand_takeoff_state_step(state, 0.0, 4, dt, cfg);
end
assert(state.phase == "FLIGHT", "4/6 contacts after liftoff should not count as landed.");

state = make_liftoff_state(cfg, dt);
for k = 1:10
    state = stand_takeoff_state_step(state, 0.0, 5, dt, cfg);
end
assert(state.phase == "LANDED", "5/6 contacts held for 0.1 s should land.");
assert(state.just_landing_confirmed, "Landing confirmation event should be raised.");

state = make_liftoff_state(cfg, dt);
for k = 1:10
    state = stand_takeoff_state_step(state, 0.0, 6, dt, cfg);
end
assert(state.phase == "LANDED", "6/6 contacts held for 0.1 s should land.");

state = make_liftoff_state(cfg, dt);
for k = 1:5
    state = stand_takeoff_state_step(state, 0.0, 5, dt, cfg);
end
assert(state.landing_timer_s > 0, "Landing timer should accumulate during active contact.");
state = stand_takeoff_state_step(state, 0.0, 0, dt, cfg);
assert(state.landing_timer_s == 0, "Landing timer should reset after contact is lost.");
assert(state.phase == "FLIGHT", "Brief contact shorter than 0.1 s should not land.");
end

function state = make_liftoff_state(cfg, dt)
state = initial_stand_takeoff_state();
state.phase = "FLIGHT";
state.stand_released = true;
steps = ceil(cfg.landing.liftoff_confirm_s / dt);
for k = 1:steps
    state = stand_takeoff_state_step(state, 0.0, 0, dt, cfg);
end
assert(state.liftoff_confirmed, "Test setup failed to confirm liftoff.");
end
