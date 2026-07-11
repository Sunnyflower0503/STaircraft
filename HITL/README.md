# HITL adapter layer

This folder reproduces the first MATLAB code version of the Simulink HITL chain:

`SERVO_OUTPUT_RAW -> Mixer1 -> matlab_model plant -> HIL_STATE_QUATERNION`

The plant model remains in `matlab_model/`. Serial I/O, MAVLink framing, actuator mapping, UAV-like state adaptation, payload packing, and real-time pacing live in this `HITL/` layer.

## Scope

Implemented for this phase:

- RX: MAVLink v2 `SERVO_OUTPUT_RAW`
- Mixer1 PWM-to-actuator mapping
- Open-loop calls into `tandem_zx_dynamics`
- TX MAVLink v2 `HIL_STATE_QUATERNION`
- Serial helpers for raw `uint8` bytes
- 1:1 wall-clock pacer
- Hardware-free MATLAB tests

Not implemented in this phase:

- `HIL_SENSOR`
- `HIL_GPS`
- `HIL_ACTUATOR_CONTROLS`

## MAVLink backend

Current backend:

```matlab
cfg.mavlink.backend = "pymavlink";
```

MATLAB is configured to use Python. Install the backend package into the same Python environment MATLAB reports from `pyenv`:

```powershell
D:\Python\Python3.12.4\python.exe -m pip install pymavlink
```

Or generally:

```powershell
pip install pymavlink
```

Verify from MATLAB:

```matlab
pyenv
py.importlib.import_module("pymavlink")
```

The bridge code lives in `HITL/mavlink_backend/`:

- `pymavlink_bridge.py`
- `mavlink_backend_init.m`
- `pymavlink_decode_servo_output_raw.m`
- `pymavlink_encode_hil_state_quaternion.m`

`stub` is still kept as a diagnostic mode; it reports a clear error and is not for real communication.

## User-supplied parameters still required

Confirm these in `hitl_config.m` before real hardware use:

- `cfg.mavlink.sysid`
- `cfg.mavlink.compid`
- Initial `lat_deg`, `lon_deg`, `AMSL`, and `heading_deg` in `cfg.init`
- Whether the serial port is still `COM4` at `115200`
- PX4/Nora is streaming `SERVO_OUTPUT_RAW` on that link
- The elevon lookup table. Current setting is:

```matlab
cfg.elevon_pwm_breakpoints = linspace(1000, 2000, 1000);
cfg.elevon_deg_table = linspace(-30, 30, 1000);
```

## Run order

1. Run the no-hardware tests:

```matlab
cd('D:/D_zx/26WORK/ShengTai/0710HITL_ST/STaircraft/HITL/tests')
run_all_hitl_tests
```

2. Test MAVLink encode only:

```matlab
cd('D:/D_zx/26WORK/ShengTai/0710HITL_ST/STaircraft/HITL/tests')
test_mavlink_encode_hil_state_quaternion
```

3. Test serial + MAVLink only, without the dynamics model:

```matlab
cd('D:/D_zx/26WORK/ShengTai/0710HITL_ST/STaircraft/HITL/tests')
test_serial_mavlink_io
```

4. After serial receive/transmit is confirmed, run the bounded HITL loop:

```matlab
cd('D:/D_zx/26WORK/ShengTai/0710HITL_ST/STaircraft/HITL')
hitl_main(10)
```

## Initial Geodetic Position

Current HITL initial position:

```text
lat = 34.021511 deg
lon = 108.757100 deg
AMSL = 500 m
heading = 0 deg
```

This comes from the new campus / lab runway reference point:

```matlab
% 新校区实验室
lat0_PHNL = 34.021511;
lon0_PHNL = 108.757100;
H_runway_PHNL = 500;
psi_runway_PHNL = 0;
```

`state_to_uavdata_like` treats `cfg.init` as the default geodetic reference origin. When the local NED state is `p_e = [0; 0; 0]`, the outgoing `HIL_STATE_QUATERNION` payload is sent at this lat/lon/AMSL. This remains true when `force_enable=0` and the state is frozen, so QGC should show the aircraft near this initial location as soon as HITL communication starts.

If QGC still shows the wrong position, check:

- `HIL_STATE_QUATERNION` is being sent continuously.
- `payload.lat`, `payload.lon`, and `payload.alt` are correct degE7/mm values.
- QGC/PX4 is not displaying another GPS or positioning source instead.
- PX4 accepts position from `HIL_STATE_QUATERNION` in the current mode.
- The current setup may still need `HIL_GPS` or another positioning message.
## Runtime force control

`hitl_main` can switch model forces on or off while MATLAB keeps running. Edit this file during runtime:

```text
HITL/runtime_control.txt
```

Set:

```text
force_enable=0
```

to disable model force integration. With the default:

```matlab
cfg.model.zero_force_mode = "freeze";
```

the aircraft state is held fixed.

Set:

```text
force_enable=1
```

to enable full dynamics force integration through `Runge_Kutta4(@tandem_zx_dynamics, ...)`.

After saving `runtime_control.txt`, `hitl_main` polls the file about every `0.2` seconds and prints a switch log such as:

```text
[HITL runtime] t=12.40 force_enable: 0 -> 1
```

Switching `force_enable` from `0` to `1` can produce a transient. It is better to confirm stable `SERVO_OUTPUT_RAW` reception before enabling full model forces.
## Common issues

- `COM4` is occupied by QGroundControl or another program.
- The PX4 serial baudrate is not `115200`.
- PX4 is not outputting `SERVO_OUTPUT_RAW` on this MAVLink instance.
- PX4 is not in the expected HITL/simulation configuration.
- `SYS_ID` or `COMP_ID` does not match the expected vehicle setup.
- MATLAB's Python environment does not have `pymavlink` installed.
- Bytes are arriving but no `SERVO_OUTPUT_RAW` is parsed; the stream may be disabled or the message type may be different.

## Notes and known limits

- The existing model directory is `matlab_model/`, not `model/`.
- The existing `Runge_Kutta4` interface is `Runge_Kutta4(fun, t_vector, X0)`, so `hitl_main` and the tests call it with `[t t+dt]` for a single step.
- `ab` is currently estimated from model acceleration minus gravity and transformed to body axes. TODO: confirm this matches the original Simulink specific-force convention exactly.
- Whether `HIL_STATE_QUATERNION` alone is sufficient for the current PX4 HITL setup must be confirmed on the real vehicle configuration.
- MATLAB real-time behavior is suitable for chain reproduction and validation, but it is not equivalent to a hard real-time C++ bridge.


