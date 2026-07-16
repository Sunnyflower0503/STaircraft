# HITL Time Alignment And Logging Fix 20260711

This note records the runtime fix applied to the MATLAB HITL stand-takeoff runner.

Root cause:

- `cfg.sample_time = 0.01 s` is the HITL outer-loop target period.
- `cfg.dt = 0.001 s` is a high-resolution integration step used by offline routines.
- Advancing the online plant by only `cfg.dt` once per outer loop makes position evolve about ten times slower than wall time.

Implemented design:

- `integrate_aircraft_step(t, x, u, param, cfg, step_s)` accepts an optional runtime step.
- When `step_s` is omitted, the default step is `cfg.sample_time`, preserving existing five-argument calls while fixing the default online time scale.
- `run_hitl_stand_takeoff.m` keeps `plant_time_s` separate from wall-clock time.
- In `STAND_HOLD`, the stand state remains frozen and `plant_time_s` stays at zero.
- In `FLIGHT`, the plant advances by the measured wall-clock loop interval, capped by `cfg.model.max_runtime_step_s = 0.05 s`.
- The console prints `wall_t`, `plant_t`, and `lag` so hardware runs can show whether the plant is keeping up with real time.
- Runtime logs are stored through a mutable `containers.Map` so `onCleanup` saves the latest stats rather than the initial stats snapshot.
- Logs include a 10 Hz `history` struct containing wall time, plant time, NED position, NED velocity, throttle, contact count, and phase.

Validation:

- `HITL/tests/run_all_hitl_tests.m` includes regression coverage for the new default and explicit `step_s` behavior in `integrate_aircraft_step`.
- Hardware validation is still required: during `FLIGHT`, `plant_t` should grow with wall time and `lag` should remain close to zero unless the runtime loop stalls.
