function cfg = hitl_config()
%HITL_CONFIG Configuration for the MATLAB HITL adapter layer.

cfg.sample_time = 0.01;
cfg.dt = 0.001;
cfg.pace = 1;

cfg.model.force_enable = 0;
cfg.model.init_mode = "stand_static";
cfg.model.zero_force_mode = "freeze";

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

cfg.stand.angle_deg = 40;
cfg.stand.settle_time_s = 20;
cfg.stand.use_cached_settled_state = true;
cfg.stand.cache_file = fullfile(fileparts(mfilename("fullpath")), ...
    "cache", "stand_static_settled_state.mat");

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
end
