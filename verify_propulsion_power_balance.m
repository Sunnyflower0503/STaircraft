clear; clc;

repo_dir = fileparts(mfilename('fullpath'));
model_dir = fullfile(repo_dir, 'matlab_model');
data_file = fullfile(repo_dir, 'propulsion_data', 'ST建模.xlsx');
addpath(model_dir);

data = read_vehicle_raw_data(data_file);

old_rpm = max(polyval([-3424, 12810, -4], data.delta_t), 0);
new_rpm = zeros(size(data.rpm));

param = struct();
param.prop_angle = 0;
param.rho = 1.225;

for k = 1:numel(data.rpm)
    [~, ~, n_rps, ~] = tandem_rotor_thrust(data.delta_t(k), 0, 0, data.V0(k), 1, param, 0);
    new_rpm(k) = 60*n_rps;
end

old_metrics = metrics(data.rpm, old_rpm);
new_metrics = metrics(data.rpm, new_rpm);

validation_table = table(data.delta_t, data.V0, data.rpm, old_rpm, ...
    old_rpm - data.rpm, (old_rpm - data.rpm)./data.rpm*100, ...
    new_rpm, new_rpm - data.rpm, (new_rpm - data.rpm)./data.rpm*100, ...
    'VariableNames', {'delta_t','V0','RPM_measured','RPM_old_direct_poly', ...
    'old_error','old_error_percent','RPM_new_power_balance','new_error','new_error_percent'});

out_csv = fullfile(repo_dir, 'propulsion_power_balance_validation.csv');
writetable(validation_table, out_csv);

fprintf('\nOld direct throttle-to-RPM polynomial:\n');
print_metrics(old_metrics);
fprintf('\nNew power-balance model with measured V0:\n');
print_metrics(new_metrics);
fprintf('\nSaved %s\n', out_csv);

assert(new_metrics.R2_RPM > 0.99, 'Power-balance RPM R2 is lower than expected.');
assert(new_metrics.RMSE_RPM < 150, 'Power-balance RPM RMSE is higher than expected.');

function data = read_vehicle_raw_data(xlsx_file)
    raw = readcell(xlsx_file);

    header_row = [];
    for i = 1:size(raw, 1)
        row = raw(i, :);
        if any(strcmp(row, "油门")) && any(strcmp(row, "次数")) && any(strcmp(row, "台架空速（m/s）"))
            header_row = i;
            break;
        end
    end

    if isempty(header_row)
        error('verify_propulsion_power_balance:HeaderNotFound', ...
            'Cannot find vehicle raw data header in %s.', xlsx_file);
    end

    header = raw(header_row, :);
    idx_throttle = find(strcmp(header, "油门"), 1);
    idx_rpm = find(strcmp(header, "RPM"), 1);
    idx_v0 = find(strcmp(header, "台架空速（m/s）"), 1);

    throttle = [];
    rpm = [];
    V0 = [];
    last_throttle = NaN;

    for r = header_row + 2:size(raw, 1)
        row = raw(r, :);
        if all(cellfun(@is_empty_cell, row))
            break;
        end

        th = cell_to_double(row{idx_throttle});
        if ~isnan(th)
            last_throttle = th;
        else
            th = last_throttle;
        end

        rpm_i = cell_to_double(row{idx_rpm});
        v0_i = cell_to_double(row{idx_v0});

        if isnan(th) || isnan(rpm_i) || isnan(v0_i) || rpm_i <= 0 || v0_i <= 0
            continue;
        end

        throttle(end + 1, 1) = th/100; %#ok<AGROW>
        rpm(end + 1, 1) = rpm_i; %#ok<AGROW>
        V0(end + 1, 1) = v0_i; %#ok<AGROW>
    end

    data = struct();
    data.delta_t = throttle;
    data.rpm = rpm;
    data.V0 = V0;
end

function value = cell_to_double(x)
    if isnumeric(x)
        value = double(x);
    elseif ismissing(x) || isempty(x)
        value = NaN;
    else
        value = str2double(string(x));
    end
end

function tf = is_empty_cell(x)
    if ismissing(x) || isempty(x)
        tf = true;
    elseif isnumeric(x)
        tf = isnan(x);
    else
        tf = strlength(string(x)) == 0;
    end
end

function m = metrics(y, yhat)
    err = yhat - y;
    rel = err./y*100;
    ss_res = sum(err.^2);
    ss_tot = sum((y - mean(y)).^2);

    m = struct();
    m.RMSE_RPM = sqrt(mean(err.^2));
    m.MAE_RPM = mean(abs(err));
    m.MaxAbsRelError_percent = max(abs(rel));
    m.R2_RPM = 1 - ss_res/ss_tot;
    m.N = numel(y);
end

function print_metrics(m)
    fprintf('  N = %d\n', m.N);
    fprintf('  RMSE = %.2f RPM\n', m.RMSE_RPM);
    fprintf('  MAE = %.2f RPM\n', m.MAE_RPM);
    fprintf('  Max relative error = %.2f %%\n', m.MaxAbsRelError_percent);
    fprintf('  R2 = %.4f\n', m.R2_RPM);
end
