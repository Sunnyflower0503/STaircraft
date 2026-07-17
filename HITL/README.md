# HITL adapter layer

This folder currently focuses on one target only:

`40 deg stand-static state -> HIL_STATE_QUATERNION` plus `SERVO_OUTPUT_RAW` receive on `COM4@115200`.

The plant model remains in `matlab_model/`. HITL code handles serial I/O, MAVLink encode/decode, state conversion, and stand-static communication tests.

## Current Scope

This stage tests:

- 40 deg stand-static state generation
- COM4@115200 serial communication
- RX: MAVLink v2 `SERVO_OUTPUT_RAW`
- TX: MAVLink v2 `HIL_STATE_QUATERNION`
- QGC display of the aircraft near the configured initial geodetic position with stand attitude

This stage does not test:

- takeoff
- `force_enable=1`
- full closed-loop control
- ground taxi
- airborne flight
- throttle sweep

## MAVLink Backend

Current backend:

```matlab
cfg.mavlink.backend = "pymavlink";
```

MATLAB uses Python. Install the backend package into the same Python environment MATLAB reports from `pyenv`:

```powershell
D:\Python\Python3.12.4\python.exe -m pip install pymavlink
```

Verify from MATLAB:

```matlab
pyenv
py.importlib.import_module("pymavlink")
```

## Initial Geodetic Position

Current HITL initial position:

```text
lat = 34.021511 deg
lon = 108.757100 deg
AMSL = 500 m
heading = 0 deg
```

`state_to_uavdata_like` treats `cfg.init` as the default geodetic reference origin.

## User Editable Configuration / 用户可编辑配置

Edit `HITL/user_hitl_config.m` for normal lab setup changes. This avoids changing internal HITL scripts.

- `user.serial.port` and `user.serial.baudrate`: MATLAB serial link settings.
- `user.init.lat_deg`, `lon_deg`, `AMSL`, and `heading_deg`: initial HIL geodetic reference.
- `user.ic.Euler_deg`, `Vb_mps`, and `pqr_radps`: editable initial attitude, body velocity, and body rates.

The default `user.ic.mode = "stand_cache"` preserves the validated cached stand-static state. To replace only selected cached values, set `user.ic.enable_override = true` and the matching `override_position`, `override_velocity`, `override_attitude`, `override_rates`, or `override_u0` flag to `true`.

Set `user.ic.mode = "manual"` to construct the complete initial state from `Xe_NED_m`, `Vb_mps`, `Euler_deg`, `pqr_radps`, and `u0`. `Vb_mps` is body-frame velocity; HITL converts it to the NED velocity state using the configured attitude. Quaternion ordering remains `[qw; qx; qy; qz]`.

## One-Click Run Script / 一键运行脚本

Use this script like pressing Run in the former Simulink HITL model:

```matlab
run('D:/D_zx/26WORK/ShengTai/0710HITL_ST/STaircraft/HITL/run_hitl_stand_static.m')
```

or open `HITL/run_hitl_stand_static.m` in MATLAB and click Run.

Run order:

1. USB: connect Nora/PX4 to QGC.
2. Serial: connect Nora/PX4 to MATLAB `COM4`.
3. Make sure QGC does not occupy `COM4`.
4. Run `run_hitl_stand_static.m` in MATLAB.
5. Wait for the aircraft to appear in QGC.
6. Manually arm and move throttle/control sticks as needed.
7. Watch MATLAB for changing `SERVO_OUTPUT_RAW` values.
8. Press Ctrl+C in MATLAB to stop. The script will try to save a log under `HITL/logs/run_hitl_stand_static_yyyymmdd_HHMMSS.mat`.

The script freezes the prepared stand-static state. It does not call `Runge_Kutta4` or `tandem_zx_dynamics` inside the runtime loop.

## Stand Takeoff Runner / 支架起飞运行脚本

`HITL/run_hitl_stand_takeoff.m` starts from the same cached stand-static state, then waits for PX4 servo output throttle before releasing the model stand:

```matlab
run('D:/D_zx/26WORK/ShengTai/0710HITL_ST/STaircraft/HITL/run_hitl_stand_takeoff.m')
```

Runtime phases:

- `STAND_HOLD`: MATLAB keeps receiving `SERVO_OUTPUT_RAW`, updating actuator `u`, and sending `HIL_STATE_QUATERNION`; `x` stays frozen and the aircraft cannot move on the ground while `mean(u(1:8)) <= 0.4`.
- `FLIGHT`: if `mean(u(1:8)) > 0.4` continuously for `0.1 s`, the stand is released once and never restored. MATLAB then advances the model with `Runge_Kutta4(@tandem_zx_dynamics, ...)`.
- `LANDED`: after stand release, liftoff is confirmed only when all six permanent contact points are off the ground for `0.05 s`. After that, `5/6` or `6/6` active contacts held for `0.1 s` confirms landing, saves the log, and stops the loop.

The script does not send `MAV_CMD_COMPONENT_ARM_DISARM`; PX4 arming remains manual in QGC or RC. Ground/landing decisions use the existing six permanent contact-point diagnostics from `zx_ground_contact_force(..., info.active)`, not height estimates. The removable stand is only part of the cached initial condition and disappears permanently after release.

Runtime time alignment:

- `cfg.sample_time = 0.01 s` is the HITL communication/main-loop target period.
- `cfg.dt = 0.001 s` remains available for high-resolution offline stand settling and model studies.
- Online takeoff does not integrate only `cfg.dt` per main loop. After stand release, it measures the actual wall-clock loop interval and advances plant dynamics by that interval.
- A single runtime integration step is capped by `cfg.model.max_runtime_step_s = 0.05 s` to avoid a large RK4 step after an OS, serial, or Python stall.
- `run_hitl_stand_takeoff.m` also polls `HITL/runtime_control.txt`: `force_enable=0` freezes model dynamics, and `force_enable=1` enables force integration after stand release.
- Console output prints `wall_t`, `plant_t`, and `lag`. In `STAND_HOLD`, `plant_t=0` is expected. In `FLIGHT`, `lag` should usually stay near zero; sustained growth means the loop is not keeping up.
- Run logs include a 10 Hz `history` struct with `wall_time_s`, `plant_time_s`, `position_ned`, `velocity_ned`, `main_throttle`, `active_contact_count`, and `phase`.

Key configuration in `hitl_config.m`:

```matlab
cfg.stand.release_throttle = 0.4;
cfg.stand.release_hold_s = 0.1;
cfg.landing.liftoff_confirm_s = 0.05;
cfg.landing.min_active_contacts = 5;
cfg.landing.confirm_s = 0.1;
cfg.model.max_runtime_step_s = 0.05;
```

Each second the runner prints `wall_t`, `plant_t`, `lag`, phase, throttle, stand/liftoff flags, active contact count, `servo1`-`servo8`, `u(1:8)`, position, velocity, and Euler angles. Logs are saved under:

```text
HITL/logs/run_hitl_stand_takeoff_yyyymmdd_HHMMSS.mat
```
## Stand Static HITL Test / 支架静止通信测试

Purpose:

- validate that the 40 deg stand-static state can be generated or loaded from cache
- validate that COM4@115200 can receive `SERVO_OUTPUT_RAW`
- validate that MATLAB continuously sends `HIL_STATE_QUATERNION`
- validate that QGC shows the aircraft near `lat=34.021511`, `lon=108.757100`, `AMSL=500 m`
- validate that the displayed attitude is the stand-static attitude, not airborne or takeoff motion

Run order:

1. Connect USB: Nora/PX4 -> QGC.
2. Connect serial line: Nora/PX4 -> MATLAB `COM4`.
3. Keep QGC on USB only. Do not let QGC occupy `COM4`.
4. Run the no-hardware tests:

```matlab
cd('D:/D_zx/26WORK/ShengTai/0710HITL_ST/STaircraft/HITL/tests')
run_all_hitl_tests
```

5. Run the stand-static communication test:

```matlab
stats = test_stand_static_hitl_io(30)
```

6. In QGC, check:

- the aircraft appears
- location is near `34.021511, 108.757100`
- AMSL is about `500 m`
- attitude is the stand-static attitude, typically pitch about `37 deg`
- `SERVO_OUTPUT_RAW` is received and changes if PX4 outputs change

If `test_stand_static_hitl_io` receives `SERVO_OUTPUT_RAW` and QGC display is correct, the current target, “stand-static + HITL communication”, is complete.

## Additional Static HITL Poses / 其他静态半物理姿态

Additional pose runners are available for quick PX4/QGC mode checks.
They use the same serial, MAVLink, geodetic origin, and `HIL_STATE_QUATERNION`
path as `run_hitl_stand_static.m`; neither runner changes the 13-state dynamics
or the six permanent ground-contact geometry.

For runners based on `hitl_static_pose_loop.m`, dynamics follows
`HITL/runtime_control.txt`:

```text
force_enable=0   % hold/freeze the prepared pose
force_enable=1   % integrate dynamics with current SERVO_OUTPUT_RAW commands
```

Attitude interface rule:

- The model state and `HIL_STATE_QUATERNION` message use `q_eb = [qw qx qy qz]`
  directly.
- Euler angles are printed only as `Euler_dbg` for human debugging and must not
  be treated as the HITL attitude interface, especially near pitch `90 deg`.
- The payload builder validates that the quaternion sent to PX4/QGC is finite
  and normalized.

### 90 deg Nose-Up Stand / 90 度机头朝上支架

```matlab
run('D:/D_zx/26WORK/ShengTai/0710HITL_ST/STaircraft/HITL/run_hitl_nose_up_90_stand_static.m')
```

- Sets `cfg.stand.angle_deg = 90`.
- Uses the existing rear-row permanent contact points and a removable vertical
  stand under the original front-center point.
- Settles the state with the ground model and stand force, then either freezes
  or integrates from that pose according to `runtime_control.txt`.
- The 90 deg geometry gives `stand_height = 0.700000000 m`; with the current
  finite ground/stand stiffness, a 5 s smoke test settled near pitch
  `87.35 deg`.

### Ground Flat Nose-Forward / 平放地面、机头朝前

```matlab
run('D:/D_zx/26WORK/ShengTai/0710HITL_ST/STaircraft/HITL/run_hitl_ground_flat_static.m')
```

- Sets Euler angle to `[roll, pitch, yaw] = [0, 0, cfg.init.heading_deg]`.
- Places the aircraft nose-forward on the NED ground model with a small static
  compression preload so the front-row ground contacts are active.
- Intended for checking PX4 fixed-wing and multicopter/rotor mode behavior in
  QGC while MATLAB either freezes or integrates the model from the flat pose.

### Airborne Nose-Up Hover / 空中机头竖直向上定点

```matlab
run('D:/D_zx/26WORK/ShengTai/0710HITL_ST/STaircraft/HITL/run_hitl_airborne_nose_up_hover_static.m')
```

- Initializes the aircraft already airborne with zero velocity and zero angular
  rate.
- Default NED position is `[0, 0, -20] m`, i.e. `20 m` above the local origin.
- Default Euler angle is `[roll, pitch, yaw] = [0, 90, cfg.init.heading_deg]`.
- Ground contact is disabled for this initial mode.
- Use `force_enable=0` to hold a fixed hover display pose, or
  `force_enable=1` to let the model integrate from that initial state.

## User Model Switches / 用户模型开关

The file `HITL/user_hitl_config.m` now contains model switches applied after
`init_param_zx()` in all HITL runners:

```matlab
user.model.slipstream_enable = true;     % false: disable 8-main-rotor slipstream aero panels
user.model.slipstream_ff_enable = true;  % false: keep slipstream velocity but set f_s = 1
user.model.aero_body_enable = true;      % false: disable all body/wing aero force and moment
```

For temporary tests, set for example:

```matlab
user.model.slipstream_enable = false;
user.model.aero_body_enable = false;
```

The airborne nose-up hover initial pose can also be edited there:

```matlab
user.hover.altitude_m = 20;
user.hover.Euler_deg = [0; 90; user.init.heading_deg];
user.hover.u0 = zeros(12, 1);
```

## HITL Log Force Analysis / HITL 日志力分解后处理

After running `run_hitl_airborne_nose_up_hover_static.m`,
`run_hitl_nose_up_90_stand_static.m`, `run_hitl_ground_flat_static.m`, or
`run_hitl_stand_takeoff.m`, the runner prints a `Log autosave file` path and
updates that MAT log about every `2 s` under `HITL/logs/`. Stopping MATLAB with
Ctrl+C still performs one final save, but the log should already exist while the
script is running.

Analyze the latest log:

```matlab
analyze_hitl_log_forces
```

Analyze a specified log:

```matlab
analyze_hitl_log_forces('D:/D_zx/26WORK/ShengTai/0710HITL_ST/STaircraft/HITL/logs/run_hitl_stand_takeoff_yyyymmdd_HHMMSS.mat')
```

The analyzer creates:

```text
result/hitl_log_analysis_<log_name>_<timestamp>/
    figures/
        pwm_and_actuators.png
        position_velocity_acceleration.png
        attitude_quaternion_eulerdbg_rates.png
        forces_body.png
        forces_earth.png
        moments_body.png
        force_norms.png
    data/
    summary_timeseries.csv
    analysis_data.mat
```

The force decomposition is recomputed from the saved `x_state`, actuator input
`u`, PWM values, and the saved `param_snapshot`. It includes rotor force,
aerodynamic force, gravity, ground contact, removable stand force when present,
and total force/moment. Attitude plots use quaternion `q_eb` as the primary
attitude; `Euler_dbg` is included only for visual debugging.

## Cached Stand State

The stand-static preparation uses the validated 40 deg stand logic from `run_takeoff_throttle_sweep.m` and saves:

```text
HITL/cache/stand_static_settled_state.mat
```

Default stand settings:

```matlab
cfg.model.force_enable = 0;
cfg.model.init_mode = "stand_static";
cfg.stand.angle_deg = 40;
cfg.stand.settle_time_s = 20;
cfg.stand.use_cached_settled_state = true;
cfg.dt = 0.001;
```

With `force_enable=0`, `hitl_main` freezes the prepared stand-static state and keeps sending `HIL_STATE_QUATERNION` from that state.

## Common Issues

- `COM4` is occupied by QGroundControl or another program.
- The PX4 serial baudrate is not `115200`.
- PX4 is not outputting `SERVO_OUTPUT_RAW` on this MAVLink instance.
- QGC/PX4 is displaying another GPS or positioning source instead of the HITL state.
- MATLAB's Python environment does not have `pymavlink` installed.
- `HIL_STATE_QUATERNION` may not be enough for some PX4 configurations; `HIL_GPS` may be needed later, but it is intentionally out of scope for this stage.

