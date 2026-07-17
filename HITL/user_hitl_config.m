function user = user_hitl_config()
%USER_HITL_CONFIG User-editable HITL serial and initial-condition settings.

user.serial.port = "COM9";
user.serial.baudrate = 115200;

user.init.lat_deg = 34.021511;
user.init.lon_deg = 108.757100;
user.init.AMSL = 500;
user.init.heading_deg = 0;

% Model switches used by all HITL runners after init_param_zx().
% slipstream_enable=false: disable the eight-main-rotor slipstream aero panels.
% slipstream_ff_enable=false: keep slipstream velocity but set f_s=1.
% aero_body_enable=false: disable all body/wing aerodynamic force and moment.
user.model.slipstream_enable = true;    % 滑流开关
user.model.slipstream_ff_enable = true;
user.model.aero_body_enable = true;     % 气动力开关

user.ic.enable_override = false;
user.ic.mode = "stand_cache";
user.ic.Xe_NED_m = [0; 0; 0];
user.ic.Vb_mps = [0; 0; 0];
user.ic.Euler_deg = [0; 37.593422; 0];
user.ic.pqr_radps = [0; 0; 0];
user.ic.u0 = zeros(12, 1);

user.ic.override_position = false;
user.ic.override_velocity = false;
user.ic.override_attitude = false;
user.ic.override_rates = false;
user.ic.override_u0 = false;

% Airborne nose-up hover initial pose.
user.hover.altitude_m = 20;
user.hover.Euler_deg = [0; 90; user.init.heading_deg];
user.hover.u0 = zeros(12, 1);
end
