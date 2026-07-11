function test_initial_geodetic_position()
root = fileparts(fileparts(mfilename("fullpath")));
addpath(root); addpath(fullfile(root, "utils")); addpath(fullfile(fileparts(root), "matlab_model"));

cfg = hitl_config();
param = init_param_zx();
x = initial_state_from_param(param, cfg);
u = zeros(12, 1);
uav = state_to_uavdata_like(0, x, u, param, cfg);
payload = uavdata_to_hil_state_quaternion_payload(uav, cfg);

lat0 = 34.021511;
lon0 = 108.757100;
amsl0 = 500;

assert(abs(uav.lat_deg - lat0) < 1e-7, "Initial latitude mismatch.");
assert(abs(uav.lon_deg - lon0) < 1e-7, "Initial longitude mismatch.");
assert(abs(uav.AMSL - amsl0) < 1e-6, "Initial AMSL mismatch.");

assert(payload.lat == int32(lat0 * 1e7), "Payload latitude degE7 mismatch.");
assert(payload.lon == int32(lon0 * 1e7), "Payload longitude degE7 mismatch.");
assert(payload.alt == int32(amsl0 * 1000), "Payload AMSL mm mismatch.");

x2 = x;
x2(1:3) = [100; 100; -20];
uav2 = state_to_uavdata_like(0, x2, u, param, cfg);

assert(uav2.lat_deg > uav.lat_deg, "North offset should increase latitude.");
assert(uav2.lon_deg > uav.lon_deg, "East offset should increase longitude.");
assert(abs(uav2.AMSL - 520) < 1e-6, "Upward NED offset should increase AMSL to 520 m.");
end
