function [x, u, meta] = apply_user_initial_conditions(x, u, cfg, param, meta)
%APPLY_USER_INITIAL_CONDITIONS Apply editable initial-condition overrides.
% State convention: [Xe_NED; Ve_NED; qw qx qy qz; pqr_body].

arguments
    x (13, 1) double
    u (12, 1) double
    cfg struct
    param struct %#ok<INUSD>
    meta struct
end

ic = resolve_user_ic(cfg);
mode = string(ic.mode);
applied_fields = strings(0, 1);

switch mode
    case "stand_cache"
        if logical(ic.enable_override)
            if logical(ic.override_position)
                x(1:3) = column3(ic.Xe_NED_m, "Xe_NED_m");
                applied_fields(end + 1, 1) = "position";
            end
            if logical(ic.override_attitude)
                x(7:10) = euler_deg_to_quat(ic.Euler_deg);
                applied_fields(end + 1, 1) = "attitude";
            end
            if logical(ic.override_velocity)
                x(4:6) = body_to_ned_velocity(ic.Vb_mps, x(7:10));
                applied_fields(end + 1, 1) = "velocity";
            end
            if logical(ic.override_rates)
                x(11:13) = column3(ic.pqr_radps, "pqr_radps");
                applied_fields(end + 1, 1) = "rates";
            end
            if logical(ic.override_u0)
                u = column12(ic.u0, "u0");
                applied_fields(end + 1, 1) = "u0";
            end
        end

    case "manual"
        q_eb = euler_deg_to_quat(ic.Euler_deg);
        x = [column3(ic.Xe_NED_m, "Xe_NED_m"); ...
             body_to_ned_velocity(ic.Vb_mps, q_eb); ...
             q_eb; ...
             column3(ic.pqr_radps, "pqr_radps")];
        u = column12(ic.u0, "u0");
        applied_fields = ["position"; "velocity"; "attitude"; "rates"; "u0"];

    otherwise
        warning("apply_user_initial_conditions:UnknownMode", ...
            "Unknown user.ic.mode '%s'; keeping the prepared initial state.", mode);
end

meta.position_ned = x(1:3);
meta.euler_deg = quat_to_euler_deg(x(7:10));
meta.velocity_norm = norm(x(4:6));
meta.angular_rate_norm = norm(x(11:13));
meta.user_initial_conditions = struct( ...
    "config_loaded", isfield(cfg, "user") && isfield(cfg.user, "loaded") && logical(cfg.user.loaded), ...
    "config_file", get_config_file(cfg), ...
    "mode", mode, ...
    "enable_override", logical(ic.enable_override), ...
    "applied_fields", applied_fields);
end

function ic = resolve_user_ic(cfg)
ic = default_ic();
if isfield(cfg, "user") && isfield(cfg.user, "ic") && isstruct(cfg.user.ic)
    ic = merge_struct_fields(ic, cfg.user.ic);
elseif isfield(cfg, "ic") && isstruct(cfg.ic)
    ic = merge_struct_fields(ic, cfg.ic);
end
end

function ic = default_ic()
ic.enable_override = false;
ic.mode = "stand_cache";
ic.Xe_NED_m = [0; 0; 0];
ic.Vb_mps = [0; 0; 0];
ic.Euler_deg = [0; 37.593422; 0];
ic.pqr_radps = [0; 0; 0];
ic.u0 = zeros(12, 1);
ic.override_position = false;
ic.override_velocity = false;
ic.override_attitude = false;
ic.override_rates = false;
ic.override_u0 = false;
end

function result = merge_struct_fields(result, source)
names = fieldnames(source);
for k = 1:numel(names)
    name = names{k};
    if isfield(result, name)
        result.(name) = source.(name);
    end
end
end

function q_eb = euler_deg_to_quat(euler_deg)
euler_rad = deg2rad(column3(euler_deg, "Euler_deg"));
q_eb = euler_to_quat_wxyz(euler_rad(1), euler_rad(2), euler_rad(3));
end

function v_e = body_to_ned_velocity(v_b, q_eb)
R_eb = quat_to_dcm_be(q_eb).';
v_e = R_eb * column3(v_b, "Vb_mps");
end

function value = column3(value, name)
value = double(value(:));
if numel(value) ~= 3 || any(~isfinite(value))
    error("apply_user_initial_conditions:BadValue", ...
        "%s must contain three finite values.", name);
end
end

function value = column12(value, name)
value = double(value(:));
if numel(value) ~= 12 || any(~isfinite(value))
    error("apply_user_initial_conditions:BadValue", ...
        "%s must contain twelve finite values.", name);
end
end

function euler_deg = quat_to_euler_deg(q_eb)
R_eb = quat_to_dcm_be(q_eb).';
pitch = asin(-R_eb(3, 1));
roll = atan2(R_eb(3, 2), R_eb(3, 3));
yaw = atan2(R_eb(2, 1), R_eb(1, 1));
euler_deg = rad2deg([roll; pitch; yaw]);
end

function config_file = get_config_file(cfg)
config_file = "";
if isfield(cfg, "user") && isfield(cfg.user, "config_file")
    config_file = string(cfg.user.config_file);
end
end
