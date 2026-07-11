function cfg = hitl_config()
%HITL_CONFIG Configuration for the MATLAB HITL adapter layer.

cfg.sample_time = 0.01;
cfg.dt = 0.001;
cfg.pace = 1;

cfg.model.force_enable = 0;
cfg.model.init_mode = "stand_static";
cfg.model.zero_force_mode = "freeze";
cfg.model.max_runtime_step_s = 0.05;

cfg.runtime_control.enable_file_control = true;
cfg.runtime_control.file = fullfile(fileparts(mfilename("fullpath")), "runtime_control.txt");
cfg.runtime_control.check_period = 0.2;

cfg.serial.port = "COM4";
cfg.serial.baudrate = 115200;
cfg.serial.data_bits = 8;
cfg.serial.parity = "none";
cfg.serial.stop_bits = 1;
cfg.serial.byte_order = "little-endian";
cfg.serial.flow_control = "none";
cfg.serial.timeout = 10;

cfg.mavlink.version = 2;
cfg.mavlink.dialect = "common.xml";
cfg.mavlink.rx_msg = "SERVO_OUTPUT_RAW";
cfg.mavlink.tx_msg = "HIL_STATE_QUATERNION";
cfg.mavlink.sysid = 1;
cfg.mavlink.compid = 1;
cfg.mavlink.backend = "pymavlink";

% 新校区实验室 / 跑道初始点
% lat0_PHNL = 34.021511 deg
% lon0_PHNL = 108.757100 deg
% H_runway_PHNL = 500 m
% psi_runway_PHNL = 0 deg
cfg.init.lat_deg = 34.021511;
cfg.init.lon_deg = 108.757100;
cfg.init.AMSL = 500;
cfg.init.heading_deg = 0;
cfg.init.use_param_geodetic = false;

cfg.user.loaded = false;
cfg.user.config_file = fullfile(fileparts(mfilename("fullpath")), "user_hitl_config.m");
cfg.user.serial.port = cfg.serial.port;
cfg.user.serial.baudrate = cfg.serial.baudrate;
cfg.user.init.lat_deg = cfg.init.lat_deg;
cfg.user.init.lon_deg = cfg.init.lon_deg;
cfg.user.init.AMSL = cfg.init.AMSL;
cfg.user.init.heading_deg = cfg.init.heading_deg;
cfg.user.ic.enable_override = false;
cfg.user.ic.mode = "stand_cache";
cfg.user.ic.Xe_NED_m = [0; 0; 0];
cfg.user.ic.Vb_mps = [0; 0; 0];
cfg.user.ic.Euler_deg = [0; 37.593422; 0];
cfg.user.ic.pqr_radps = [0; 0; 0];
cfg.user.ic.u0 = zeros(12, 1);
cfg.user.ic.override_position = false;
cfg.user.ic.override_velocity = false;
cfg.user.ic.override_attitude = false;
cfg.user.ic.override_rates = false;
cfg.user.ic.override_u0 = false;
cfg.ic = cfg.user.ic;

cfg.stand.angle_deg = 40;
cfg.stand.settle_time_s = 20;
cfg.stand.use_cached_settled_state = true;
cfg.stand.cache_file = fullfile(fileparts(mfilename("fullpath")), ...
    "cache", "stand_static_settled_state.mat");
cfg.stand.release_throttle = 0.4;
cfg.stand.release_hold_s = 0.1;

cfg.landing.liftoff_confirm_s = 0.05;
cfg.landing.min_active_contacts = 5;
cfg.landing.confirm_s = 0.1;

cfg.env.rho0 = 1.225;
cfg.env.earth_radius = 6378137.0;
cfg.env.g = 9.80665;

cfg.throttle.min = 0;
cfg.throttle.max = 1;

% Simulink 1-D Lookup Table for servo5/servo6: pwm -> delta_t [deg].
cfg.elevon_pwm_breakpoints = linspace(1000, 2000, 1000);
cfg.elevon_deg_table = linspace(-30, 30, 1000);
cfg.elevon.min_rad = [];
cfg.elevon.max_rad = [];

cfg = apply_user_hitl_config(cfg);
end

function cfg = apply_user_hitl_config(cfg)
config_file = cfg.user.config_file;
if ~isfile(config_file)
    return;
end

try
    user = user_hitl_config();
catch ME
    warning("hitl_config:UserConfigReadFailed", ...
        "Could not read %s: %s. Using defaults.", config_file, ME.message);
    return;
end

if ~isstruct(user)
    warning("hitl_config:BadUserConfig", ...
        "user_hitl_config must return a struct. Using defaults.");
    return;
end

cfg.user = merge_user_struct(cfg.user, user);
cfg.user.loaded = true;
cfg.user.config_file = config_file;

if isfield(cfg.user, "serial")
    cfg.serial = merge_user_struct(cfg.serial, cfg.user.serial);
end
if isfield(cfg.user, "init")
    cfg.init = merge_user_struct(cfg.init, cfg.user.init);
end
cfg.ic = cfg.user.ic;
end

function target = merge_user_struct(target, source)
if ~isstruct(source)
    return;
end
names = fieldnames(source);
for k = 1:numel(names)
    name = names{k};
    if isfield(target, name) && isstruct(target.(name)) && isstruct(source.(name))
        target.(name) = merge_user_struct(target.(name), source.(name));
    elseif isfield(target, name) || ~isstruct(target)
        target.(name) = source.(name);
    else
        target.(name) = source.(name);
    end
end
end
