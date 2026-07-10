clear; clc;

repo_dir = fileparts(mfilename('fullpath'));
model_dir = fullfile(repo_dir, 'matlab_model');
addpath(model_dir);

param = init_param_zx();
param.wind = [0; 0; 0];
param.slipstream_enable = true;
param.slipstream_ff_enable = true;

Va_list = [2, 6, 11];
alpha_deg_list = -6:1:12;
dt = 1.0;

DCM_be = eye(3);
omega_b = zeros(3, 1);
delta_t = dt * ones(8, 1);
delta_aeL = 0;
delta_aeR = 0;

rows = {};
lift_body = zeros(numel(alpha_deg_list), numel(Va_list));

for iVa = 1:numel(Va_list)
    Va = Va_list(iVa);

    for ia = 1:numel(alpha_deg_list)
        alpha = alpha_deg_list(ia) * param.D2R;
        Ve = [Va * cos(alpha); 0; Va * sin(alpha)];

        [F_aero_body, ~, slip_diag] = tandem_aero_fm(Ve, DCM_be, omega_b, ...
            delta_aeL, delta_aeR, delta_t, param);

        F_aero_earth = DCM_be * F_aero_body;
        lift_body(ia, iVa) = -F_aero_body(3);

        rows(end + 1, :) = {Va, alpha_deg_list(ia), dt, ...
            F_aero_body(1), F_aero_body(2), F_aero_body(3), -F_aero_body(3), ...
            F_aero_earth(1), F_aero_earth(2), F_aero_earth(3), -F_aero_earth(3), ...
            mean(slip_diag.Vi0), mean(slip_diag.Vi_eff), ...
            mean(slip_diag.f_s), mean(slip_diag.S_slip_i), ...
            mean(slip_diag.alpha_s) / param.D2R}; %#ok<SAGROW>
    end
end

result_table = cell2table(rows, 'VariableNames', { ...
    'Va_mps', 'alpha_deg', 'dt', ...
    'F_aero_body_x_N', 'F_aero_body_y_N', 'F_aero_body_z_N', 'lift_body_up_N', ...
    'F_aero_earth_x_N', 'F_aero_earth_y_N', 'F_aero_earth_z_N', 'lift_earth_up_N', ...
    'mean_Vi0_mps', 'mean_Vi_eff_mps', 'mean_f_s', ...
    'mean_S_slip_i_m2', 'mean_alpha_s_deg'});

csv_file = fullfile(repo_dir, 'slipstream_lift_alpha_sweep.csv');
png_file = fullfile(repo_dir, 'slipstream_lift_alpha_sweep.png');
fig_file = fullfile(repo_dir, 'slipstream_lift_alpha_sweep.fig');

writetable(result_table, csv_file);

fig = figure('Visible', 'off', 'Color', 'w');
hold on; grid on; box on;
colors = lines(numel(Va_list));
for iVa = 1:numel(Va_list)
    plot(alpha_deg_list, lift_body(:, iVa), '-o', ...
        'LineWidth', 1.5, 'Color', colors(iVa, :), ...
        'DisplayName', sprintf('Va = %.0f m/s', Va_list(iVa)));
end
xlabel('\alpha (deg)');
ylabel('Lift -F_{aero,z}^{body} (N)');
title(sprintf('Slipstream lift vs angle of attack, dt = %.2f, full f_s', dt));
legend('Location', 'northwest');
saveas(fig, png_file);
savefig(fig, fig_file);
close(fig);

assert(all(isfinite(result_table{:, 4:end}), 'all'), ...
    'Lift sweep output contains non-finite values.');

fprintf('Slipstream lift alpha sweep passed.\n');
fprintf('Saved %s\n', csv_file);
fprintf('Saved %s\n', png_file);
fprintf('Saved %s\n', fig_file);

for iVa = 1:numel(Va_list)
    fprintf('Va=%.0f m/s: lift range %.6g to %.6g N\n', ...
        Va_list(iVa), min(lift_body(:, iVa)), max(lift_body(:, iVa)));
end
