# GJ Aircraft Model MATLAB Copy

This directory contains a MATLAB implementation of the aircraft dynamics and
force/moment model extracted from the Simulink model:

```text
D:\D_zx\26WORK\ShengTai\0514transfer\SIMmodel\Tandem_zx_trans6.slx
```

The main Simulink reference block is:

```text
Tandem_zx_trans6/Aerocraft
```

The code is intended as an experimental copy for fixed-wing high angle-of-attack
takeoff, steep descent, and trajectory-planning verification. Prefer modifying
this copy first instead of changing the original `ZX` model directly.

## Simulink Interface Mapping

The Simulink `Aerocraft` block has three inputs and one output:

```text
Inputs:
  EnvData
  ActData
  VTOLMod

Output:
  UAVData
```

`ActData` is a bus with 12 actuator fields:

```text
daeL, daeR,
dt1, dt2, dt3, dt4, dt5, dt6, dt7, dt8,
dt9, dt10
```

In this MATLAB copy, the dynamics function uses a compact vector input:

```matlab
u = [dt1; dt2; dt3; dt4; dt5; dt6; dt7; dt8; dt9; dt10; daeL; daeR];
```

`dt1` to `dt8` are the main rotor throttle commands, `dt9` and `dt10`
are the left/right wingtip auxiliary propeller throttle commands, and `daeL`,
`daeR` are the left/right elevon commands.

`EnvData` contains environment quantities such as ground height, wind velocity,
wind angular rate, temperature, speed of sound, pressure, air density, and
gravity. This MATLAB copy currently uses the corresponding values from `param`,
especially `param.wind`, `param.rho`, and `param.g`.

`UAVData` in Simulink contains the aircraft state and flight-condition outputs:

```text
Ve, Xe, Euler, DCM_be, Vb, pqr, pqr_dot, Abb, Abe, mass,
lat_deg, lon_deg, AMSL, gamma, chi, alphaDot, alpha, beta,
TAS, pqr_air, Mach, DynamicPressure, EAS, T(K), Air_pressure,
g, Vb_air, Ve_Inertial
```

The MATLAB copy does not build the full `UAVData` bus. Instead, it integrates a
13-state vector and computes only the quantities needed by the force/moment
model.

## State Definition

The main state vector used by `tandem_zx_dynamics` is:

```text
x(1:3)    p_e      NED position [m]
x(4:6)    v_e      NED velocity [m/s]
x(7:10)   q_eb     body-to-earth quaternion [qw qx qy qz]
x(11:13)  omega_b  body angular rate [rad/s]
```

The dynamics are written in NED convention: positive earth `z` points downward.

## File Structure

### Main Dynamics

- `tandem_zx_dynamics.m`
  - Top-level 13-state 6DOF dynamics.
  - Calls rotor, aerodynamic, gravity, and optional ground-contact models.
  - Corresponds to the Simulink combination of `Aerocraft/6DOF (Quaternion)`
    and `Aerocraft/F&M`.

- `Runge_Kutta4.m`
  - Fixed-step fourth-order Runge-Kutta integration helper.

- `sat.m`
  - Saturation helper used for throttle and surface limits.

### Parameter Initialization

- `init_param_zx.m`
  - Defines mass, inertia, geometry, rotor layout, propeller spin directions,
    aerodynamic coefficient tables, actuator limits, wind, and ground-contact
    options.
  - Parameter values are mainly copied from the generated Simulink C code under
    `Tandem_zx_trans6_grt_rtw`.
  - Also builds optional full-angle aerodynamic tables for high angle-of-attack
    extrapolation.

- `aerodata1_tianshizhiyi_CFDslip.xlsx`
- `aerodata2_tianshizhiyi.xlsx`
  - Source aerodynamic data files kept with the model copy.

### Rotor and Propulsion Model

- `tandem_rotor_fm.m`
  - Computes the total body-axis force and moment from 8 main rotors plus 2
    wingtip auxiliary propellers.
  - Corresponds mainly to `Aerocraft/F&M/Thrust&M in body axis old`.

- `tandem_rotor_thrust.m`
  - Main rotor thrust, propeller torque, rotor speed, and thrust coefficient
    model.
  - Matches the generated C-code style polynomial model for `n`, `J`, `CT`, and
    `CM`.

- `tandem_addprop_fm.m`
  - Wingtip auxiliary propeller thrust and moment model.
  - Corresponds to `Addprop Left` and `Addprop Right`.

### Aerodynamic Model

- `tandem_aero_fm.m`
  - Computes aerodynamic force and moment in body axes.
  - Reproduces the main Simulink aerodynamic structure:
    - elevon force/moment increments,
    - `S_free` static lift/drag/pitching moment,
    - `Sw` dynamic derivative terms,
    - 8 `S_slip` force-panel contributions.

Current simplification:

- The Simulink model computes local slipstream angle of attack and dynamic
  pressure for each slipstream region.
- This MATLAB copy currently uses the common aircraft air-relative velocity for
  the slipstream panels.
- Slipstream force is included, but detailed slipstream-induced moments are not
  fully reproduced.

If slipstream accuracy is not required, the remaining 6DOF, rotor, auxiliary
propeller, gravity, and baseline aerodynamic paths are a close match to the
Simulink aircraft module.

### Ground Contact

- `zx_ground_contact_force.m`
  - Simple NED ground-contact model with normal force, damping, and friction.
  - Disabled by default through `param.ground.enable = false`.
  - This is not a block-for-block reproduction of the Simulink `ground Support`
    subsystem, so use caution for ground-roll or touchdown studies.

## Typical Usage

```matlab
addpath('GJ/model');

param = init_param_zx();

x0 = [
    param.InitData.Xe;
    [0.1; 0; 0];
    [1; 0; 0; 0];
    param.InitData.pqr
];

u = [
    0.3 * ones(8, 1);
    0;
    0;
    0;
    0
];

dx = tandem_zx_dynamics(0, x0, u, param);
```

## Known Matching Notes

- The actuator vector order in this MATLAB copy is different from the raw
  Simulink `ActData` bus order. Keep the mapping explicit when comparing
  signals.
- Check the sign convention of the inertia cross term `Ixz` before relying on
  roll/yaw coupling. The generated Simulink C data should be treated as the
  reference.
- Ground support and detailed slipstream effects are the main known differences.
- The model is most reliable for airborne fixed-wing or transition studies where
  detailed slipstream and ground-contact fidelity are not the dominant effects.

## Validation Status

This directory has previously been used by tests under:

```text
GJ/tests/
```

Existing validation has checked path resolution, high angle-of-attack
aerodynamic scans, ground-contact sanity behavior, and finite dynamics outputs.
Result artifacts may be found under:

```text
GJ/results/model_validation/
```
