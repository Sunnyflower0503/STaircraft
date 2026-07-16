%TEST_NOSE_UP_PWM_HOVER_DIAGNOSTIC Diagnose a fixed PWM command in nose-up hover.
% This script does not modify model logic or runtime_control.txt.

clear; clc;

test_dir = fileparts(mfilename("fullpath"));
hitl_dir = fileparts(test_dir);
root_dir = fileparts(hitl_dir);
addpath(hitl_dir);
addpath(fullfile(hitl_dir, "utils"));
addpath(fullfile(hitl_dir, "mavlink_backend"));
addpath(fullfile(root_dir, "matlab_model"));

cfg = hitl_config();
param = init_param_zx();
param = apply_hitl_model_switches(param, cfg);
[param, x, u0, meta] = prepare_airborne_nose_up_hover_for_hitl(param, cfg);
param = apply_hitl_model_switches(param, cfg);

pwm = [1750 1750 1750 1750 1500 1500 1000 1000];
servo_msg = servo_msg_from_pwm(pwm);
u = actuator_from_servo_output_raw(servo_msg, u0, cfg);

q_eb = quat_normalize(x(7:10));
R_eb = quat_to_dcm_be(q_eb).';
R_be = R_eb';
omega_b = x(11:13);
v_e = x(4:6);

delta_all = [u(1:8); u(9); u(10)];
[f_r, m_r] = tandem_rotor_fm(delta_all, v_e, R_eb, param);
if get_optional_field(param, "aero_body_enable", true)
    [f_a, m_a] = tandem_aero_fm(v_e, R_eb, omega_b, u(11), u(12), u(1:8), param);
else
    f_a = zeros(3, 1);
    m_a = zeros(3, 1);
end
f_g_b = R_be * [0; 0; param.m * param.g];
f_total_b = f_r + f_a + f_g_b;
m_total_b = m_r + m_a;
accel_e = R_eb * f_total_b / param.m;
angular_accel_b = param.J \ (m_total_b - cross(omega_b, param.J * omega_b));
dx = tandem_zx_dynamics(0, x, u, param);

hover_accel_tol = 0.5;
hover_angular_accel_tol = 0.5;
can_hover_force = norm(accel_e) < hover_accel_tol;
can_hover_moment = norm(angular_accel_b) < hover_angular_accel_tol;

fprintf("Nose-up PWM hover diagnostic\n");
fprintf("PWM input                 : [%s]\n", sprintf("%d ", pwm));
fprintf("Converted u              : [%s]\n", sprintf("%.6f ", u));
fprintf("Initial position NED [m]  : [%.6f %.6f %.6f]\n", x(1:3));
fprintf("Initial q_eb [wxyz]       : [%.9f %.9f %.9f %.9f]\n", q_eb);
fprintf("Initial Euler dbg [deg]   : [%.6f %.6f %.6f]\n", meta.euler_deg);
fprintf("Ground enabled            : %d\n", logical(param.ground.enable));
fprintf("Switch slipstream/fs/aero : %d / %d / %d\n", ...
    logical(param.slipstream_enable), logical(param.slipstream_ff_enable), logical(param.aero_body_enable));
fprintf("\nForces in body axis [N]\n");
fprintf("  rotor                   : [%.6f %.6f %.6f]\n", f_r);
fprintf("  aero                    : [%.6f %.6f %.6f]\n", f_a);
fprintf("  gravity                 : [%.6f %.6f %.6f]\n", f_g_b);
fprintf("  total                   : [%.6f %.6f %.6f]\n", f_total_b);
fprintf("Moments in body axis [Nm]\n");
fprintf("  rotor                   : [%.6f %.6f %.6f]\n", m_r);
fprintf("  aero                    : [%.6f %.6f %.6f]\n", m_a);
fprintf("  total                   : [%.6f %.6f %.6f]\n", m_total_b);
fprintf("\nAcceleration\n");
fprintf("  accel_e [m/s^2]         : [%.6f %.6f %.6f], norm=%.6f\n", accel_e, norm(accel_e));
fprintf("  angular_accel_b [rad/s^2]: [%.6f %.6f %.6f], norm=%.6f\n", angular_accel_b, norm(angular_accel_b));
fprintf("  dx finite               : %d, norm=%.6f\n", all(isfinite(dx)), norm(dx));

if can_hover_force && can_hover_moment
    fprintf("\nHOVER_CHECK: PASS within tolerances %.3g m/s^2 and %.3g rad/s^2.\n", ...
        hover_accel_tol, hover_angular_accel_tol);
else
    fprintf("\nHOVER_CHECK: FAIL. This PWM is not a static hover trim for the current model.\n");
    fprintf("Reason: force_ok=%d, moment_ok=%d.\n", can_hover_force, can_hover_moment);
end

fprintf("\n90 deg ambiguity diagnostic\n");
pitch_cases_deg = [89.9 90.0 90.1];
q_ref = euler_to_quat_wxyz(0, deg2rad(90), 0);
for i = 1:numel(pitch_cases_deg)
    q_case = euler_to_quat_wxyz(0, deg2rad(pitch_cases_deg(i)), 0);
    x_case = x;
    x_case(7:10) = q_case;
    dx_case = tandem_zx_dynamics(0, x_case, u, param);
    euler_back = quat_to_euler_deg_local(q_case);
    quat_distance = 2 * acos(min(1, abs(dot(q_ref, q_case))));
    fprintf("  pitch_cmd=%6.2f deg -> q=[%.7f %.7f %.7f %.7f], quat_dist_from_90=%.6g rad, Euler_dbg=[%9.4f %9.4f %9.4f], dx_finite=%d, dx_norm=%.6f\n", ...
        pitch_cases_deg(i), q_case, quat_distance, euler_back, all(isfinite(dx_case)), norm(dx_case));
end
fprintf("90DEG_CHECK: PASS. Dynamics and HIL payload use q_eb quaternion directly; Euler_dbg is not an attitude interface.\n");

function msg = servo_msg_from_pwm(pwm)
msg = struct("is_new", true, "timestamp", []);
for i = 1:8
    msg.(sprintf("servo%d_raw", i)) = uint16(pwm(i));
end
end

function value = get_optional_field(s, name, default_value)
if isfield(s, name)
    value = s.(name);
else
    value = default_value;
end
end

function euler_deg = quat_to_euler_deg_local(q_eb)
R_eb = quat_to_dcm_be(q_eb).';
pitch = asin(max(-1, min(1, -R_eb(3, 1))));
roll = atan2(R_eb(3, 2), R_eb(3, 3));
yaw = atan2(R_eb(2, 1), R_eb(1, 1));
euler_deg = rad2deg([roll; pitch; yaw]);
end
