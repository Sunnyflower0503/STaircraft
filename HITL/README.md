# HITL adapter layer

This folder reproduces the first MATLAB code version of the Simulink HITL chain:

`SERVO_OUTPUT_RAW -> Mixer1 -> matlab_model plant -> HIL_STATE_QUATERNION`

The plant model remains in `matlab_model/`. Serial I/O, MAVLink framing, actuator mapping, UAV-like state adaptation, payload packing, and real-time pacing live in this `HITL/` layer.

## Scope

Implemented for this phase:

- RX: MAVLink v2 `SERVO_OUTPUT_RAW`
- Mixer1 PWM-to-actuator mapping
- Open-loop calls into `tandem_zx_dynamics`
- TX payload packing for `HIL_STATE_QUATERNION`
- Serial helpers for raw `uint8` bytes
- 1:1 wall-clock pacer
- Hardware-free MATLAB tests

Not implemented in this phase:

- `HIL_SENSOR`
- `HIL_GPS`
- `HIL_ACTUATOR_CONTROLS`

## User-supplied parameters still required

Fill these in `hitl_config.m` before real hardware use:

- `servo5_raw` and `servo6_raw` 1-D Lookup Table breakpoints and values:
  - `cfg.elevon_pwm_breakpoints`
  - `cfg.elevon_deg_table`
- Real `SYS_ID` and `COMP_ID`
- Initial `lat_deg`, `lon_deg`, and `AMSL`, unless available in `param.InitData`
- MAVLink backend choice: `pymavlink`, `mavlink_c`, MATLAB UAV Toolbox, or another adapter
- Confirm whether the serial port is still `COM4` at `115200`

The default backend is `stub`. Decode and encode functions intentionally report a clear error when real bytes must be parsed or produced.

## Run order

1. Run `HITL/tests/run_all_hitl_tests.m`.
2. Fill the elevon lookup table in `hitl_config.m`.
3. Set `cfg.mavlink.backend` and implement/connect the selected backend in the two MAVLink functions.
4. Confirm `COM4 @ 115200` is available.
5. Connect Nora/PX4.
6. Run `hitl_main.m`.

Example:

```matlab
cd('D:/D_zx/26WORK/ShengTai/0710HITL_ST/STaircraft/HITL/tests')
run_all_hitl_tests
```

For a bounded smoke run after the backend is connected:

```matlab
cd('D:/D_zx/26WORK/ShengTai/0710HITL_ST/STaircraft/HITL')
hitl_main(10)
```

## Notes and known limits

- The existing model directory is `matlab_model/`, not `model/`.
- The existing `Runge_Kutta4` interface is `Runge_Kutta4(fun, t_vector, X0)`, so `hitl_main` and the tests call it with `[t t+dt]` for a single step.
- `ab` is currently estimated from model acceleration minus gravity and transformed to body axes. TODO: confirm this matches the original Simulink specific-force convention exactly.
- Whether `HIL_STATE_QUATERNION` alone is sufficient for the current PX4 HITL setup must be confirmed on the real vehicle configuration.
- MATLAB real-time behavior is suitable for chain reproduction and validation, but it is not equivalent to a hard real-time C++ bridge.
