function test_user_hitl_config()
root = fileparts(fileparts(mfilename("fullpath")));
addpath(root); addpath(fullfile(root, "utils"));

user = user_hitl_config();
cfg = hitl_config();

assert(isfield(user, "serial") && isfield(user.serial, "port") && isfield(user.serial, "baudrate"), ...
    "User serial settings are missing.");
assert(isfield(user, "init") && isfield(user.init, "lat_deg") && isfield(user.init, "lon_deg") && ...
    isfield(user.init, "AMSL") && isfield(user.init, "heading_deg"), ...
    "User initial geodetic settings are missing.");
assert(isfield(user, "ic") && isfield(user.ic, "mode") && isfield(user.ic, "Xe_NED_m") && ...
    isfield(user.ic, "Vb_mps") && isfield(user.ic, "Euler_deg") && isfield(user.ic, "pqr_radps") && isfield(user.ic, "u0"), ...
    "User initial-condition settings are missing.");
assert(isequal(size(user.ic.u0), [12, 1]), "User u0 must be 12x1.");
assert(logical(cfg.user.loaded), "Existing user_hitl_config.m was not loaded.");
assert(cfg.serial.port == user.serial.port, "User serial port was not applied to cfg.");
assert(cfg.serial.baudrate == user.serial.baudrate, "User baudrate was not applied to cfg.");
assert(cfg.init.lat_deg == user.init.lat_deg, "User latitude was not applied to cfg.");
assert(cfg.ic.mode == "stand_cache", "User initial-condition mode was not retained.");
end
