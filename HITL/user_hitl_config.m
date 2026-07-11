function user = user_hitl_config()
%USER_HITL_CONFIG User-editable HITL serial and initial-condition settings.

user.serial.port = "COM4";
user.serial.baudrate = 115200;

user.init.lat_deg = 34.021511;
user.init.lon_deg = 108.757100;
user.init.AMSL = 500;
user.init.heading_deg = 0;

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
end
