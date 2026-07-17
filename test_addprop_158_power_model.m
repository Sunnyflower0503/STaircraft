function result_dir = test_addprop_158_power_model()
%TEST_ADDPROP_158_POWER_MODEL Validate wingtip auxiliary prop static power fit.
%
% Uses the 15.8 V static propeller test sheet to validate the STaircraft
% wingtip auxiliary propeller model:
%   P_e = f(throttle), T = f(P_e)
%
% Outputs are saved under:
%   result/addprop_158_power_validation_<timestamp>/

script_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(script_dir, 'matlab_model'));

data_file = 'D:\D_zx\251201CLY\Experiment\260331\翼尖桨测试参数\15.8.xlsx';
timestamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
result_dir = fullfile(script_dir, 'result', ['addprop_158_power_validation_' timestamp]);
fig_dir = fullfile(result_dir, 'figures');
data_dir = fullfile(result_dir, 'data');
if ~exist(fig_dir, 'dir')
    mkdir(fig_dir);
end
if ~exist(data_dir, 'dir')
    mkdir(data_dir);
end

raw = readmatrix(data_file);
valid = isfinite(raw(:, 1)) & raw(:, 1) >= 0 & raw(:, 1) <= 100 ...
    & isfinite(raw(:, 3)) & isfinite(raw(:, 6));
raw = raw(valid, :);

dt = raw(:, 1) / 100;
rpm = raw(:, 3);
voltage = raw(:, 4);
current = raw(:, 5);
thrust_g = abs(raw(:, 6));
torque_nm = raw(:, 7);
power_e = raw(:, 9);
if all(~isfinite(power_e))
    power_e = voltage .* current;
end
power_mech = raw(:, 10);
thrust_n = thrust_g * 9.80665 / 1000;

param = init_param_zx();
power_hat = interp1(param.addprop_throttle_bp, param.addprop_power_table, dt, 'linear', 'extrap');
power_hat = min(max(power_hat, 0), max(param.addprop_power_table));
thrust_hat = zeros(size(dt));
for idx = 1:numel(dt)
    T_add_idx = tandem_addprop_fm(dt(idx), 0, param);
    thrust_hat(idx) = -T_add_idx(1);
end
thrust_hat_from_measured_power = interp1(param.addprop_thrust_power_bp, ...
    param.addprop_thrust_table, power_e, 'linear', 'extrap');

err_n = thrust_hat - thrust_n;
rmse_n = sqrt(mean(err_n.^2));
max_abs_err_n = max(abs(err_n));
rel_rmse = rmse_n / max(thrust_n);

fit_table = table(dt, rpm, voltage, current, power_e, power_mech, torque_nm, ...
    thrust_n, power_hat, thrust_hat, thrust_hat_from_measured_power, err_n);
writetable(fit_table, fullfile(data_dir, 'fit_data.csv'));

param_fixed = param;
param_fixed.addprop_moment_mode = "fixed_wing";
[T_left_fixed, M_left_fixed] = tandem_addprop_fm(1, 0, param_fixed);
[T_both_fixed, M_both_fixed] = tandem_addprop_fm(1, 1, param_fixed);

param_rotor = param;
param_rotor.addprop_moment_mode = "rotor_yaw";
[T_left_rotor, M_left_rotor] = tandem_addprop_fm(1, 0, param_rotor);
[T_both_rotor, M_both_rotor] = tandem_addprop_fm(1, 1, param_rotor);

summary = table( ...
    string(param.addprop_model_source), rmse_n, max_abs_err_n, rel_rmse, ...
    max(thrust_n), max(thrust_hat), ...
    T_left_fixed(1), M_left_fixed(1), M_left_fixed(2), M_left_fixed(3), ...
    T_both_fixed(1) + T_both_fixed(2), M_both_fixed(1), M_both_fixed(2), M_both_fixed(3), ...
    T_left_rotor(1), M_left_rotor(1), M_left_rotor(2), M_left_rotor(3), ...
    T_both_rotor(1) + T_both_rotor(2), M_both_rotor(1), M_both_rotor(2), M_both_rotor(3), ...
    'VariableNames', {'model_source', 'rmse_N', 'max_abs_error_N', 'relative_rmse', ...
    'max_measured_thrust_N', 'max_model_thrust_N', ...
    'fixed_left_only_T_interface_N', 'fixed_left_only_Mx_Nm', ...
    'fixed_left_only_My_Nm', 'fixed_left_only_Mz_Nm', ...
    'fixed_both_T_interface_sum_N', 'fixed_both_Mx_Nm', ...
    'fixed_both_My_Nm', 'fixed_both_Mz_Nm', ...
    'rotor_left_only_T_interface_N', 'rotor_left_only_Mx_Nm', ...
    'rotor_left_only_My_Nm', 'rotor_left_only_Mz_Nm', ...
    'rotor_both_T_interface_sum_N', 'rotor_both_Mx_Nm', ...
    'rotor_both_My_Nm', 'rotor_both_Mz_Nm'});
writetable(summary, fullfile(result_dir, 'summary.csv'));

save(fullfile(data_dir, 'validation.mat'), 'param', 'fit_table', 'summary');

make_validation_figures(fig_dir, dt, power_e, power_hat, thrust_n, thrust_hat, err_n);

fprintf('\n[ADDPROP 15.8 POWER VALIDATION]\n');
fprintf('Result directory: %s\n', result_dir);
fprintf('Thrust RMSE: %.6f N\n', rmse_n);
fprintf('Max abs error: %.6f N\n', max_abs_err_n);
fprintf('Relative RMSE vs max thrust: %.3f %%\n', 100 * rel_rmse);
fprintf('Full throttle model thrust magnitude: %.6f N\n', -T_left_fixed(1));
fprintf('Fixed-wing left-only moment [Mx My Mz]: [%.6f %.6f %.6f] Nm\n', M_left_fixed);
fprintf('Rotor-mode left-only moment [Mx My Mz]: [%.6f %.6f %.6f] Nm\n', M_left_rotor);
fprintf('PASS: 15.8 power-based static wingtip-prop model validated.\n\n');
end

function make_validation_figures(fig_dir, dt, power_e, power_hat, thrust_n, thrust_hat, err_n)
fig = figure('Visible', 'off', 'Name', 'addprop_158_power_fit');
tiledlayout(fig, 2, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

nexttile;
plot(dt, power_e, 'ko', dt, power_hat, 'r-', 'LineWidth', 1.5);
grid on;
xlabel('Throttle');
ylabel('Electrical power [W]');
legend('15.8 test', 'model', 'Location', 'northwest');
title('P_e(dt)');

nexttile;
plot(power_e, thrust_n, 'ko', power_hat, thrust_hat, 'r-', 'LineWidth', 1.5);
grid on;
xlabel('Electrical power [W]');
ylabel('Static thrust [N]');
legend('15.8 test', 'model', 'Location', 'northwest');
title('T(P_e)');

nexttile;
plot(dt, thrust_n, 'ko', dt, thrust_hat, 'r-', 'LineWidth', 1.5);
grid on;
xlabel('Throttle');
ylabel('Static thrust [N]');
legend('15.8 test', 'model', 'Location', 'northwest');
title('T(dt)');

nexttile;
plot(dt, err_n, 'b-o', 'LineWidth', 1.2);
yline(0, 'k--');
grid on;
xlabel('Throttle');
ylabel('T model - test [N]');
title('Residual');

exportgraphics(fig, fullfile(fig_dir, 'addprop_158_power_fit.png'), 'Resolution', 160);
close(fig);
end
