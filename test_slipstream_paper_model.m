clear; clc;

repo_dir = fileparts(mfilename('fullpath'));
model_dir = fullfile(repo_dir, 'matlab_model');
addpath(model_dir);

param0 = init_param_zx();
param0.wind = [0; 0; 0];

Va_list = [0, 2, 6, 11];
dt_list = [0.5, 0.95];

modes = struct( ...
    'name', {'no_slipstream', 'fs_equal_1', 'full_fs'}, ...
    'slipstream_enable', {false, true, true}, ...
    'slipstream_ff_enable', {true, false, true});

DCM_be = eye(3);
omega_b = zeros(3, 1);
delta_aeL = 0;
delta_aeR = 0;

summary_rows = {};
rotor_rows = {};

fprintf('Paper slipstream model comparison\n');
fprintf('Va_mps, dt, mode, F_body_x, F_body_y, F_body_z, body_up_-Fz, earth_up_-Fez\n');

for iVa = 1:numel(Va_list)
    Va = Va_list(iVa);
    Ve = [Va; 0; 0];

    for idt = 1:numel(dt_list)
        dt = dt_list(idt);
        delta_t = dt * ones(8, 1);

        for imode = 1:numel(modes)
            param = param0;
            param.slipstream_enable = modes(imode).slipstream_enable;
            param.slipstream_ff_enable = modes(imode).slipstream_ff_enable;

            [F_body, M_body, slip_diag] = tandem_aero_fm(Ve, DCM_be, omega_b, ...
                delta_aeL, delta_aeR, delta_t, param);

            F_earth = DCM_be * F_body;
            body_up = -F_body(3);
            earth_up = -F_earth(3);

            fprintf('%.3g, %.3g, %s, %.9g, %.9g, %.9g, %.9g, %.9g\n', ...
                Va, dt, modes(imode).name, F_body(1), F_body(2), F_body(3), ...
                body_up, earth_up);

            summary_rows(end + 1, :) = {Va, dt, string(modes(imode).name), ...
                F_body(1), F_body(2), F_body(3), body_up, ...
                F_earth(1), F_earth(2), F_earth(3), earth_up, ...
                M_body(1), M_body(2), M_body(3)}; %#ok<SAGROW>

            for ip = 1:8
                rotor_rows(end + 1, :) = {Va, dt, string(modes(imode).name), ip, ...
                    slip_diag.T(ip), slip_diag.n_rps(ip), slip_diag.CT(ip), ...
                    slip_diag.Vi0(ip), slip_diag.Vi_eff(ip), slip_diag.Rs(ip), ...
                    slip_diag.f_s(ip), slip_diag.S_slip_i(ip), ...
                    slip_diag.alpha_s(ip), slip_diag.beta_s(ip), slip_diag.qbar_s(ip), ...
                    slip_diag.CL_free(ip), slip_diag.CL_slip(ip), slip_diag.CD_slip(ip), ...
                    slip_diag.F_body(1, ip), slip_diag.F_body(2, ip), slip_diag.F_body(3, ip)}; %#ok<SAGROW>
            end
        end
    end
end

summary = cell2table(summary_rows, 'VariableNames', { ...
    'Va_mps', 'dt', 'mode', ...
    'F_body_x_N', 'F_body_y_N', 'F_body_z_N', 'body_up_N', ...
    'F_earth_x_N', 'F_earth_y_N', 'F_earth_z_N', 'earth_up_N', ...
    'M_body_x_Nm', 'M_body_y_Nm', 'M_body_z_Nm'});

rotor_diag = cell2table(rotor_rows, 'VariableNames', { ...
    'Va_mps', 'dt', 'mode', 'rotor_index', ...
    'T_N', 'n_rps', 'CT', 'Vi0_mps', 'Vi_eff_mps', 'Rs_m', ...
    'f_s', 'S_slip_i_m2', 'alpha_s_rad', 'beta_s_rad', 'qbar_s_Pa', ...
    'CL_free', 'CL_slip', 'CD_slip', ...
    'F_slip_i_x_N', 'F_slip_i_y_N', 'F_slip_i_z_N'});

summary_file = fullfile(repo_dir, 'slipstream_paper_test_summary.csv');
rotor_file = fullfile(repo_dir, 'slipstream_paper_test_rotor_diag.csv');
writetable(summary, summary_file);
writetable(rotor_diag, rotor_file);

assert(all(isfinite(summary{:, setdiff(summary.Properties.VariableNames, {'mode'})}), 'all'), ...
    'Summary output contains non-finite values.');
assert(all(isfinite(rotor_diag{:, setdiff(rotor_diag.Properties.VariableNames, {'mode'})}), 'all'), ...
    'Rotor diagnostic output contains non-finite values.');

fprintf('\nSaved %s\n', summary_file);
fprintf('Saved %s\n', rotor_file);
fprintf('Paper slipstream model test passed.\n');
