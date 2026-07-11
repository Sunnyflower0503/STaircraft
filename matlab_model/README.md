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

Slipstream notes:

- The 8 `S_slip` panels support a paper-style slipstream model using
  propeller thrust, actuator-disk induced velocity, local slipstream dynamic
  pressure, local `alpha_s`/`beta_s`, and the `f_s` lift correction.
- `param.slipstream_enable = false` keeps the `S_slip` area in the free-stream
  aerodynamic model, but disables propeller-induced slipstream velocity.
- `param.slipstream_ff_enable = false` keeps induced velocity but forces
  `f_s = 1` for sensitivity checks.
- `param.slip_area_from_paper` selects the paper slipstream area formula;
  otherwise the configured `param.S_slip` areas are used.
- Slipstream force is included in `f_a`; slipstream-induced moment is
  intentionally ignored in this MATLAB copy.

If slipstream accuracy is not required, the remaining 6DOF, rotor, auxiliary
propeller, gravity, and baseline aerodynamic paths are a close match to the
Simulink aircraft module.

### Ground Contact

- `zx_ground_contact_force.m`
  - N-point unilateral spring-damper ground-contact model with regularized
    friction.
  - Disabled by default through `param.ground.enable = false`.
  - Contact point count is obtained from
    `size(param.ground.contact_points_b, 2)`, so the model is not hard-coded to
    four or six points.
  - Contact point positions are defined in body axes relative to the aircraft
    center of gravity.
  - Contact point velocity includes rigid-body rotation:
    `v_contact_b = R_be * v_e + cross(omega_b, r_contact_b)`.
  - NED convention is used throughout: earth `z` is positive downward, so
    `penetration = contact_pos_e(3) - param.ground.z`; ground normal force
    points along negative earth `z`.
  - Per-point force is transformed back to body axes and contributes moment
    `cross(r_contact_b, force_i_b)` about the center of gravity.
  - `k`, `c`, `mu`, and `xy_damping` may be scalars or length-`N` vectors.
  - The model returns a diagnostic `info` structure with per-point position,
    velocity, penetration, penetration rate, normal force, friction force,
    total contact force, moment, and active contact state.

The default six body-axis contact points are generated from the eight main
propeller positions and `param.MAC`:

```matlab
param.ground.contact_points_b = [
     0.0710,   0.0710,   0.0710,  -0.6290,  -0.6290,  -0.6290;
    -0.5845,   0,        0.5845,  -0.5845,   0,        0.5845;
     0.1750,   0.1750,   0.1750,  -0.1750,  -0.1750,  -0.1750
];
```

The six points are ordered as front-left, front-center, front-right,
rear-left, rear-center, and rear-right. Their front/rear height difference
gives a natural level-ground pitch angle of about `26.565051 deg`.

Recommended fixed steps:

- Airborne simulation with ground disabled: keep `param.ctrl_dt = 0.01 s`.
- Ground-contact simulation: use `dt = 0.001 s` for validation and contact
  studies.
- `dt = 0.002 s` may be used for speed-prioritized runs, but peak normal-force
  errors are larger.

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

This directory has been validated by repository-level MATLAB scripts under the
project root:

```text
verify_matlab_model.m
verify_propulsion_power_balance.m
test_ground_contact_model.m
test_ground_static_equilibrium.m
test_ground_drop.m
test_ground_timestep_convergence.m
test_slipstream_paper_model.m
test_slipstream_paper_251_trend.m
test_slipstream_validation.m
plot_slipstream_lift_alpha_sweep.m
```

Current ground-contact acceptance checks include:

```text
1. zero force with no penetration,
2. upward NED normal force under penetration,
3. unilateral push-only contact,
4. rigid-body angular velocity in contact-point velocity,
5. near-zero roll moment for symmetric left/right contacts,
6. independent enter/leave behavior for all six contact points,
7. finite outputs,
8. unchanged airborne dynamics when ground contact is disabled,
9. static equilibrium on level ground at the natural pitch angle,
10. low-height drop contact and damping behavior,
11. fixed-step convergence for dt = 0.002, 0.001, and 0.0005 s.
```

Recent acceptance results:

```text
test_ground_contact_model: passed
test_ground_static_equilibrium: passed
test_ground_drop: passed
test_ground_timestep_convergence: passed
verify_matlab_model: passed
```

Representative ground-contact values:

```text
natural ground pitch: 26.565051177 deg
static normal sum: 31.36 N
mass*g: 31.36 N
static normal error: about 3.5e-13 N
max static penetration: about 0.0122574 m
drop max penetration: about 0.0122574 m
drop max total normal: about 279 N for dt = 0.001 s
```

## 40 deg Stand Takeoff Throttle Sweep

The standalone experiment script
`../run_takeoff_throttle_sweep.m` evaluates fixed-throttle takeoff from a
removable 40 deg ground stand without changing the 13-state dynamics or the
six permanent ground contact points.

Run from MATLAB:

```matlab
run('D:\D_zx\26WORK\ShengTai\0710HITL_ST\STaircraft\run_takeoff_throttle_sweep.m')
```

Experiment setup:

- The stand supports the original front-center contact point
  `param.ground.contact_points_b(:,2)`.
- The rear support reference is the mean of rear-left, rear-center, and
  rear-right contact points `param.ground.contact_points_b(:,4:6)`.
- The 40 deg geometry is computed automatically from the contact-point
  coordinates; the expected stand height is about `0.1818358 m`.
- The model first settles for `20 s` with zero throttle, ground contact enabled,
  and the removable stand enabled.
- Each throttle case then starts from the same settled state and runs for up to
  `10 s` with fixed commands `0.05:0.05:1.0`.
- Main motor response is modeled only inside the experiment loop with
  `tau_motor = 0.15 s` and exact discrete first-order filtering.
- While any permanent contact point or the stand is touching, NED `x` is held
  fixed and `v_e(1)` is set to zero; forward motion is allowed only after all
  contacts are lost.
- Liftoff is declared only after all six permanent points and the stand remain
  out of contact for at least `0.05 s`; after liftoff the stand is permanently
  disabled for that case.

The script creates:

```text
result/takeoff_throttle_sweep_<timestamp>/
    figures/
    data/
    summary.csv
    config.mat
```

Each throttle case stores a MAT file and a PNG figure containing command and
actual throttle, NED position and clearance, velocity, Euler angles, six
permanent contact forces plus stand force, and contact states. The overview
figure summarizes throttle versus liftoff status, liftoff time, and maximum
height.

Latest run:

```text
result directory: result/takeoff_throttle_sweep_20260711_105005
stand height: 0.181835772 m
stand top NED z: -0.181835772 m
20 s settled position: [0, 0, -0.241892080] m
20 s settled Euler angle: [0, 37.593422493, 0] deg
20 s settled velocity norm: about 1.3e-14 m/s
20 s settled angular-rate norm: about 4.8e-14 rad/s
minimum instantaneous liftoff throttle: 0.85
minimum sustained takeoff throttle: 0.85
```

Latest sweep summary:

```text
throttle  status             liftoff_time_s  max_height_m  final_pitch_deg
0.05      no_takeoff         NaN             -0.0032       37.59
0.10      no_takeoff         NaN             -0.0032       37.59
0.15      no_takeoff         NaN             -0.0032       37.59
0.20      no_takeoff         NaN             -0.0032       37.59
0.25      no_takeoff         NaN             -0.0032       37.59
0.30      no_takeoff         NaN             -0.0032       37.59
0.35      no_takeoff         NaN             -0.0028       37.87
0.40      no_takeoff         NaN             -0.0024       38.21
0.45      no_takeoff         NaN             -0.0020       38.47
0.50      no_takeoff         NaN             -0.0017       38.71
0.55      no_takeoff         NaN             -0.0014       38.92
0.60      no_takeoff         NaN             -0.0012       39.12
0.65      no_takeoff         NaN             -0.0009       39.32
0.70      no_takeoff         NaN             -0.0007       39.50
0.75      no_takeoff         NaN             -0.0004       39.68
0.80      no_takeoff         NaN             -0.0002       39.86
0.85      sustained_takeoff  0.691           49.4000       56.06
0.90      sustained_takeoff  0.417           61.3095       74.98
0.95      sustained_takeoff  0.334           70.3334       67.39
1.00      sustained_takeoff  0.285           74.9716       39.49
```
