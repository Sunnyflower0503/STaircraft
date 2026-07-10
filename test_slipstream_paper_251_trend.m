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
body_up = zeros(numel(modes), 1);
earth_up = zeros(numel(modes), 1);

fprintf('Paper 2.5.1 trend case\n');
fprintf('Va=%.6g m/s, theta=%.6g deg, dt=%.6g\n', Va, theta / param0.D2R, dt);
fprintf('mode, F_body_x, F_body_y, F_body_z, body_up_-Fz, earth_up_-Fez\n');

for imode = 1:numel(modes)
    param = param0;
    param.slipstream_enable = modes(imode).slipstream_enable;
    param.slipstream_ff_enable = modes(imode).slipstream_ff_enable;

    [F_body, M_body, slip_diag] = tandem_aero_fm(Ve, DCM_be, omega_b, ...
        delta_aeL, delta_aeR, delta_t, param);

    F_earth = DCM_be * F_body;
    body_up(imode) = -F_body(3);
    earth_up(imode) = -F_earth(3);

    fprintf('%s, %.9g, %.9g, %.9g, %.9g, %.9g\n', ...
        modes(imode).name, F_body(1), F_body(2), F_body(3), ...
        body_up(imode), earth_up(imode));

    summary_rows(end + 1, :) = {string(modes(imode).name), Va, theta, dt, ...
        F_body(1), F_body(2), F_body(3), body_up(imode), ...
        F_earth(1), F_earth(2), F_earth(3), earth_up(imode), ...
        M_body(1), M_body(2), M_body(3)}; %#ok<SAGROW>

    for ip = 1:8
        rotor_rows(end + 1, :) = {string(modes(imode).name), ip, ...
            slip_diag.T(ip), slip_diag.n_rps(ip), slip_diag.CT(ip), ...
            slip_diag.Vi0(ip), slip_diag.Vi_eff(ip), slip_diag.Rs(ip), ...
            slip_diag.f_s(ip), slip_diag.S_slip_i(ip), ...
            slip_diag.alpha_s(ip), slip_diag.beta_s(ip), slip_diag.qbar_s(ip), ...
            slip_diag.CL_free(ip), slip_diag.CL_slip(ip), slip_diag.CD_slip(ip), ...
            slip_diag.F_body(1, ip), slip_diag.F_body(2, ip), slip_diag.F_body(3, ip)}; %#ok<SAGROW>
    end
end

summary = cell2table(summary_rows, 'VariableNames', { ...
    'mode', 'Va_mps', 'theta_rad', 'dt', ...
    'F_body_x_N', 'F_body_y_N', 'F_body_z_N', 'body_up_N', ...
    'F_earth_x_N', 'F_earth_y_N', 'F_earth_z_N', 'earth_up_N', ...
    'M_body_x_Nm', 'M_body_y_Nm', 'M_body_z_Nm'});

rotor_diag = cell2table(rotor_rows, 'VariableNames', { ...
    'mode', 'rotor_index', ...
    'T_N', 'n_rps', 'CT', 'Vi0_mps', 'Vi_eff_mps', 'Rs_m', ...
    'f_s', 'S_slip_i_m2', 'alpha_s_rad', 'beta_s_rad', 'qbar_s_Pa', ...
    'CL_free', 'CL_slip', 'CD_slip', ...
    'F_slip_i_x_N', 'F_slip_i_y_N', 'F_slip_i_z_N'});

summary_file = fullfile(repo_dir, 'slipstream_paper_251_trend_summary.csv');
rotor_file = fullfile(repo_dir, 'slipstream_paper_251_trend_rotor_diag.csv');
writetable(summary, summary_file);
writetable(rotor_diag, rotor_file);

full_rows = strcmp(rotor_diag.mode, "full_fs");
Rp = 0.5 * param0.prop_D;
Vi_ratio = rotor_diag.Vi_eff_mps(full_rows) ./ rotor_diag.Vi0_mps(full_rows);
Rs_ratio = rotor_diag.Rs_m(full_rows) / Rp;

assert(all(isfinite(summary{:, setdiff(summary.Properties.VariableNames, {'mode'})}), 'all'), ...
    'Summary output contains non-finite values.');
assert(all(isfinite(rotor_diag{:, setdiff(rotor_diag.Properties.VariableNames, {'mode'})}), 'all'), ...
    'Rotor diagnostic output contains non-finite values.');
assert(all(Vi_ratio > 1.9 & Vi_ratio < 2.0), 'Vi_eff/Vi0 trend check failed.');
assert(all(Rs_ratio > 0.70 & Rs_ratio < 0.72), 'Rs/Rp trend check failed.');
assert(body_up(2) > body_up(3) && body_up(3) > body_up(1), ...
    'Body-axis lift trend check failed.');
assert(earth_up(2) > earth_up(3) && earth_up(3) > earth_up(1), ...
    'Earth vertical lift trend check failed.');
assert((body_up(3) - body_up(1)) / max(abs(body_up(1)), 1e-9) > 0.1, ...
    'Low-speed slipstream lift increase is not obvious enough.');

fprintf('\nPer-rotor full_fs mean Vi_eff/Vi0 = %.9g\n', mean(Vi_ratio));
fprintf('Per-rotor full_fs mean Rs/Rp = %.9g\n', mean(Rs_ratio));
fprintf('Body-up lift trend: no_slip=%.9g, fs_equal_1=%.9g, full_fs=%.9g\n', ...
    body_up(1), body_up(2), body_up(3));
fprintf('Earth-up lift trend: no_slip=%.9g, fs_equal_1=%.9g, full_fs=%.9g\n', ...
    earth_up(1), earth_up(2), earth_up(3));
fprintf('Saved %s\n', summary_file);
fprintf('Saved %s\n', rotor_file);
fprintf('Paper 2.5.1 trend test passed.\n');

function DCM_be = pitch_dcm_be(theta)
DCM_be = [cos(theta), 0, sin(theta); ...
          0,          1, 0; ...
         -sin(theta), 0, cos(theta)];
end
