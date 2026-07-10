clear; clc;

repo_dir = fileparts(mfilename('fullpath'));
model_dir = fullfile(repo_dir, 'matlab_model');
addpath(model_dir);

param0 = init_param_zx();
param0.wind = [0; 0; 0];

Va = 2.87;
theta = 27 * param0.D2R;
dt = 0.6;

DCM_be = pitch_dcm_be(theta);
Ve = DCM_be * [Va; 0; 0];
omega_b = zeros(3, 1);
delta_t = dt * ones(8, 1);
delta_aeL = 0;
delta_aeR = 0;

modes = struct( ...
    'name', {'no_slipstream', 'fs_equal_1', 'full_fs'}, ...
    'slipstream_enable', {false, true, true}, ...
    'slipstream_ff_enable', {true, false, true});

summary_rows = {};
rotor_rows = {};
trend = struct();
trend.aero_body_up = zeros(numel(modes), 1);
trend.aero_earth_up = zeros(numel(modes), 1);
trend.prop_body_up = zeros(numel(modes), 1);
trend.prop_earth_up = zeros(numel(modes), 1);
trend.total_body_up = zeros(numel(modes), 1);
trend.total_earth_up = zeros(numel(modes), 1);

fprintf('Slipstream validation case\n');
fprintf('Va=%.6g m/s, theta=%.6g deg, dt=%.6g\n', Va, theta / param0.D2R, dt);
fprintf(['mode, aero_body_up, prop_body_up, total_body_up, ', ...
         'aero_earth_up, prop_earth_up, total_earth_up\n']);

for imode = 1:numel(modes)
    param = param0;
    param.slipstream_enable = modes(imode).slipstream_enable;
    param.slipstream_ff_enable = modes(imode).slipstream_ff_enable;

    [F_aero_body, M_aero_body, slip_diag] = tandem_aero_fm(Ve, DCM_be, omega_b, ...
        delta_aeL, delta_aeR, delta_t, param);

    [F_prop_body, prop_diag] = direct_propulsion_force(delta_t, Va, param);
    F_total_body = F_aero_body + F_prop_body;

    F_aero_earth = DCM_be * F_aero_body;
    F_prop_earth = DCM_be * F_prop_body;
    F_total_earth = DCM_be * F_total_body;

    aero_body_up = -F_aero_body(3);
    prop_body_up = -F_prop_body(3);
    total_body_up = -F_total_body(3);
    aero_earth_up = -F_aero_earth(3);
    prop_earth_up = -F_prop_earth(3);
    total_earth_up = -F_total_earth(3);

    trend.aero_body_up(imode) = aero_body_up;
    trend.aero_earth_up(imode) = aero_earth_up;
    trend.prop_body_up(imode) = prop_body_up;
    trend.prop_earth_up(imode) = prop_earth_up;
    trend.total_body_up(imode) = total_body_up;
    trend.total_earth_up(imode) = total_earth_up;

    fprintf('%s, %.9g, %.9g, %.9g, %.9g, %.9g, %.9g\n', ...
        modes(imode).name, aero_body_up, prop_body_up, total_body_up, ...
        aero_earth_up, prop_earth_up, total_earth_up);

    summary_rows(end + 1, :) = {string(modes(imode).name), Va, theta, dt, ...
        F_aero_body(1), F_aero_body(2), F_aero_body(3), aero_body_up, ...
        F_prop_body(1), F_prop_body(2), F_prop_body(3), prop_body_up, ...
        F_total_body(1), F_total_body(2), F_total_body(3), total_body_up, ...
        F_aero_earth(1), F_aero_earth(2), F_aero_earth(3), aero_earth_up, ...
        F_prop_earth(1), F_prop_earth(2), F_prop_earth(3), prop_earth_up, ...
        F_total_earth(1), F_total_earth(2), F_total_earth(3), total_earth_up, ...
        sum(prop_diag.T)}; %#ok<SAGROW>

    Rp = 0.5 * param.prop_D;
    for ip = 1:8
        Vi_ratio = safe_ratio(slip_diag.Vi_eff(ip), slip_diag.Vi0(ip));
        Rs_ratio = safe_ratio(slip_diag.Rs(ip), Rp);

        rotor_rows(end + 1, :) = {string(modes(imode).name), ip, ...
            prop_diag.T(ip), prop_diag.F_body(1, ip), prop_diag.F_body(2, ip), ...
            prop_diag.F_body(3, ip), -prop_diag.F_body(3, ip), ...
            slip_diag.Vi0(ip), slip_diag.Vi_eff(ip), Vi_ratio, ...
            slip_diag.Rs(ip), Rs_ratio, slip_diag.f_s(ip), slip_diag.S_slip_i(ip), ...
            slip_diag.alpha_s(ip), slip_diag.beta_s(ip), slip_diag.qbar_s(ip), ...
            slip_diag.CL_free(ip), slip_diag.CL_slip(ip), slip_diag.CD_slip(ip)}; %#ok<SAGROW>
    end
end

summary = cell2table(summary_rows, 'VariableNames', { ...
    'mode', 'Va_mps', 'theta_rad', 'dt', ...
    'F_aero_body_x_N', 'F_aero_body_y_N', 'F_aero_body_z_N', 'aero_body_up_N', ...
    'F_prop_body_x_N', 'F_prop_body_y_N', 'F_prop_body_z_N', 'prop_body_up_N', ...
    'F_total_body_x_N', 'F_total_body_y_N', 'F_total_body_z_N', 'total_body_up_N', ...
    'F_aero_earth_x_N', 'F_aero_earth_y_N', 'F_aero_earth_z_N', 'aero_earth_up_N', ...
    'F_prop_earth_x_N', 'F_prop_earth_y_N', 'F_prop_earth_z_N', 'prop_earth_up_N', ...
    'F_total_earth_x_N', 'F_total_earth_y_N', 'F_total_earth_z_N', 'total_earth_up_N', ...
    'sum_prop_T_N'});

rotor_diag = cell2table(rotor_rows, 'VariableNames', { ...
    'mode', 'rotor_index', ...
    'T_i_N', 'F_prop_body_i_x_N', 'F_prop_body_i_y_N', 'F_prop_body_i_z_N', ...
    'prop_body_i_up_N', ...
    'Vi0_mps', 'Vi_eff_mps', 'Vi_eff_over_Vi0', ...
    'Rs_m', 'Rs_over_Rp', 'f_s', 'S_slip_i_m2', ...
    'alpha_s_rad', 'beta_s_rad', 'qbar_s_Pa', ...
    'CL_free', 'CL_slip', 'CD_slip'});

summary_file = fullfile(repo_dir, 'slipstream_validation_summary.csv');
rotor_file = fullfile(repo_dir, 'slipstream_validation_rotor_diag.csv');
writetable(summary, summary_file);
writetable(rotor_diag, rotor_file);

checks = run_trend_checks(rotor_diag, trend);

fprintf('\nTrend checks:\n');
print_check('Vi_eff/Vi0 in 1.9~2.0', checks.vi_ratio);
print_check('Rs/Rp near 0.707', checks.rs_ratio);
print_check('Aero body-up: fs=1 > full_fs > no_slip', checks.aero_body_lift_order);
print_check('Aero earth-up: fs=1 > full_fs > no_slip', checks.aero_earth_lift_order);
print_check('full_fs aero body-up > no_slip', checks.full_fs_body_increment);
print_check('full_fs aero earth-up > no_slip', checks.full_fs_earth_increment);
print_check('Total body-up mainly from prop thrust', checks.body_total_prop_dominant);
if checks.body_total_prop_not_applicable
    fprintf('  NOTE: body-up prop check is N/A because prop_angle=0 gives no body -Z thrust.\n');
end
print_check('Total earth-up mainly from prop thrust', checks.earth_total_prop_dominant);

required_checks = rmfield(checks, 'body_total_prop_dominant');
required_checks = rmfield(required_checks, 'body_total_prop_not_applicable');
all_pass = all(struct2array(required_checks)) ...
        && (checks.body_total_prop_dominant || checks.body_total_prop_not_applicable);
if all_pass
    fprintf('\nPASS: slipstream validation passed.\n');
else
    fprintf('\nFAIL: slipstream validation failed; inspect CSV diagnostics.\n');
end

fprintf('Saved %s\n', summary_file);
fprintf('Saved %s\n', rotor_file);

assert(all_pass, 'Slipstream validation trend checks failed.');

function DCM_be = pitch_dcm_be(theta)
DCM_be = [cos(theta), 0, sin(theta); ...
          0,          1, 0; ...
         -sin(theta), 0, cos(theta)];
end

function [F_prop_body, diag] = direct_propulsion_force(delta_t, Va, param)
F_prop_body = zeros(3, 1);
diag.T = zeros(8, 1);
diag.F_body = zeros(3, 8);

alpha = 0;
beta = 0;
for ip = 1:8
    [T_i, ~, ~, ~] = tandem_rotor_thrust(delta_t(ip), alpha, beta, Va, ...
        param.prop_spin(ip), param, param.prop_angle(ip));

    prop_angle = param.prop_angle(ip);
    F_i = T_i * [cos(prop_angle); 0; -sin(prop_angle)];
    F_prop_body = F_prop_body + F_i;

    diag.T(ip) = T_i;
    diag.F_body(:, ip) = F_i;
end
end

function checks = run_trend_checks(rotor_diag, trend)
full_rows = strcmp(rotor_diag.mode, "full_fs");
vi_ratio = rotor_diag.Vi_eff_over_Vi0(full_rows);
rs_ratio = rotor_diag.Rs_over_Rp(full_rows);

checks = struct();
checks.vi_ratio = all(vi_ratio > 1.9 & vi_ratio < 2.0);
checks.rs_ratio = all(rs_ratio > 0.70 & rs_ratio < 0.72);
checks.aero_body_lift_order = trend.aero_body_up(2) > trend.aero_body_up(3) ...
                           && trend.aero_body_up(3) > trend.aero_body_up(1);
checks.aero_earth_lift_order = trend.aero_earth_up(2) > trend.aero_earth_up(3) ...
                            && trend.aero_earth_up(3) > trend.aero_earth_up(1);
checks.full_fs_body_increment = trend.aero_body_up(3) > trend.aero_body_up(1);
checks.full_fs_earth_increment = trend.aero_earth_up(3) > trend.aero_earth_up(1);
checks.body_total_prop_not_applicable = abs(trend.prop_body_up(3)) < 1e-9;
checks.body_total_prop_dominant = checks.body_total_prop_not_applicable ...
                               || (trend.prop_body_up(3) > trend.aero_body_up(3) ...
                               && trend.prop_body_up(3) / trend.total_body_up(3) > 0.5);
checks.earth_total_prop_dominant = trend.prop_earth_up(3) > trend.aero_earth_up(3) ...
                                && trend.prop_earth_up(3) / trend.total_earth_up(3) > 0.5;
end

function value = safe_ratio(num, den)
if abs(den) <= 1e-12
    value = 0;
else
    value = num / den;
end
end

function print_check(label, passed)
if passed
    status = 'PASS';
else
    status = 'FAIL';
end
fprintf('  %s: %s\n', status, label);
end
