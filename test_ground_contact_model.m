clear; clc;

repo_dir = fileparts(mfilename('fullpath'));
model_dir = fullfile(repo_dir, 'matlab_model');
addpath(model_dir);

param = init_param_zx();
param.ground.enable = true;
param.ground.z = 0;
param.ground.k = 1000;
param.ground.c = 100;
param.ground.mu = 0.4;
param.ground.xy_damping = 20;
param.ground.friction_v_eps = 0.05;

expected_points = [
     0.071,   0.071,   0.071,  -0.629,  -0.629,  -0.629;
    -0.5845,  0,        0.5845, -0.5845, 0,        0.5845;
     0.175,   0.175,    0.175,  -0.175,  -0.175,  -0.175
];
assert(max(abs(param.ground.contact_points_b - expected_points), [], 'all') < 1e-12, ...
    'Six ground contact points do not match expected geometry.');

R_eb = eye(3);
R_be = R_eb';
omega_zero = zeros(3, 1);
v_e_zero = zeros(3, 1);

%% No penetration: all forces and moments are exactly zero.
p_no_contact = [0; 0; -1];
[f_b, m_b, info] = zx_ground_contact_force(p_no_contact, v_e_zero, R_eb, omega_zero, param);
assert(norm(f_b) == 0 && norm(m_b) == 0, 'No-penetration contact must be exactly zero.');
assert(~any(info.active), 'No-penetration active flags must be false.');

%% Penetration: normal force is upward in NED and unilateral.
param_one = param;
param_one.ground.contact_points_b = [0; 0; 0];
param_one.ground.k = 1000;
param_one.ground.c = 100;
param_one.ground.mu = 0;
p_pen = [0; 0; 0.01];
[f_b, m_b, info] = zx_ground_contact_force(p_pen, v_e_zero, R_eb, omega_zero, param_one);
f_e = R_eb * f_b;
assert(info.active(1), 'Penetrating point should be active.');
assert(f_e(3) < 0, 'Ground normal force must point upward in NED (-z).');
assert(abs(info.normal_force(1) - 10) < 1e-12, 'Normal force spring term mismatch.');
assert(norm(m_b) == 0, 'Center contact point should not produce moment.');

v_up_e = [0; 0; -10];
[f_pull_b, ~, info_pull] = zx_ground_contact_force(p_pen, v_up_e, R_eb, omega_zero, param_one);
assert(info_pull.normal_force(1) == 0 && norm(f_pull_b) == 0, ...
    'Unilateral contact must not pull when damping term overcomes spring term.');

%% Angular velocity contributes to contact point velocity and damping.
param_spin = param_one;
param_spin.ground.contact_points_b = [1; 0; 0];
param_spin.ground.k = 1000;
param_spin.ground.c = 100;
p_spin = [0; 0; 0.01];
omega_b = [0; -1; 0];
[~, ~, info_spin] = zx_ground_contact_force(p_spin, v_e_zero, R_eb, omega_b, param_spin);
expected_v_contact_b = cross(omega_b, [1; 0; 0]);
assert(norm(info_spin.contact_vel_b(:, 1) - expected_v_contact_b) < 1e-12, ...
    'Angular velocity term is missing from contact point velocity.');
assert(abs(info_spin.penetration_rate(1) - 1) < 1e-12, ...
    'Angular velocity contribution to penetration rate is incorrect.');
assert(abs(info_spin.normal_force(1) - 110) < 1e-12, ...
    'Angular velocity damping contribution is incorrect.');

%% Left/right symmetric contacts should produce near-zero roll moment.
param_sym = param;
param_sym.ground.contact_points_b = [0 0; -0.5 0.5; 0 0];
param_sym.ground.k = 1000;
param_sym.ground.c = 0;
param_sym.ground.mu = 0;
p_sym = [0; 0; 0.02];
[~, m_sym_b] = zx_ground_contact_force(p_sym, v_e_zero, R_eb, omega_zero, param_sym);
assert(abs(m_sym_b(1)) < 1e-12, 'Symmetric left/right contacts should have near-zero roll moment.');

%% Each of the six contact points can independently enter and leave contact.
for ip = 1:size(param.ground.contact_points_b, 2)
    param_single = param;
    param_single.ground.k = 1000;
    param_single.ground.c = 0;
    param_single.ground.mu = 0;
    r_i = param_single.ground.contact_points_b(:, ip);

    p_touch = -R_eb * r_i + [0; 0; 0.01];
    [f_touch_b, ~, info_touch] = zx_ground_contact_force(p_touch, v_e_zero, R_eb, omega_zero, param_single);
    assert(info_touch.active(ip), 'Expected selected contact point to be active.');
    assert(info_touch.normal_force(ip) > 0, 'Selected contact point normal force should be positive.');
    assert(all(isfinite([f_touch_b; info_touch.normal_force(:)])), 'Contact outputs must be finite.');

    p_clear = -R_eb * r_i + [0; 0; -0.01];
    [f_clear_b, m_clear_b, info_clear] = zx_ground_contact_force(p_clear, v_e_zero, R_eb, omega_zero, param_single);
    assert(~info_clear.active(ip), 'Expected selected contact point to leave contact.');
    assert(all(isfinite([f_clear_b; m_clear_b; info_clear.penetration(:)])), 'Clearance outputs must be finite.');
end

%% Airborne dynamics are unchanged when ground is disabled.
param_air = init_param_zx();
x0 = zeros(13, 1);
x0(3) = -30;
x0(4) = 12;
x0(7) = 1;
u = zeros(12, 1);
u(1:8) = 0.45;
dx_disabled = tandem_zx_dynamics(0, x0, u, param_air);
param_no_ground_field = rmfield(param_air, 'ground');
dx_no_ground_field = tandem_zx_dynamics(0, x0, u, param_no_ground_field);
assert(norm(dx_disabled - dx_no_ground_field) < 1e-12, ...
    'Airborne dynamics changed when ground contact is disabled.');
assert(all(isfinite(dx_disabled)), 'Airborne dynamics output must be finite.');

fprintf('Ground contact model tests passed.\n');
fprintf('Contact points [x;y;z] body frame:\n');
disp(param.ground.contact_points_b);
fprintf('Recommended fixed RK4 step: 0.001 to 0.002 s for ground-contact cases; ');
fprintf('0.01 s remains acceptable for airborne cases with ground disabled.\n');
