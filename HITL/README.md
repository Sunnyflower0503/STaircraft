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

Key configuration in `hitl_config.m`:

```matlab
cfg.stand.release_throttle = 0.4;
cfg.stand.release_hold_s = 0.1;
cfg.landing.liftoff_confirm_s = 0.05;
cfg.landing.min_active_contacts = 5;
cfg.landing.confirm_s = 0.1;
```

Each second the runner prints phase, throttle, stand/liftoff flags, active contact count, `servo1`-`servo8`, `u(1:8)`, position, velocity, and Euler angles. Logs are saved under:

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

