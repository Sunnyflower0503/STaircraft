function test_hil_state_quaternion_payload()
root = fileparts(fileparts(mfilename("fullpath")));
addpath(root); addpath(fullfile(root, "utils"));
cfg = hitl_config();
uav = struct("time_s", 1.25, "q_eb", [1;0;0;0], "pqr", [0.1;0.2;0.3], ...
    "lat_deg", 30.1, "lon_deg", 120.2, "AMSL", 50, "Ve", [1;2;3], ...
    "EAS", 4, "TAS", 5, "ab", [0.1;0.2;0.3]);
p = uavdata_to_hil_state_quaternion_payload(uav, cfg);
assert(isa(p.time_usec, "uint64"));
assert(isa(p.lat, "int32") && isa(p.lon, "int32") && isa(p.alt, "int32"));
assert(isa(p.vx, "int16") && isa(p.vy, "int16") && isa(p.vz, "int16"));
assert(isa(p.ind_airspeed, "uint16") && isa(p.true_airspeed, "uint16"));
assert(isa(p.xacc, "int16") && isa(p.yacc, "int16") && isa(p.zacc, "int16"));
assert(isa(p.attitude_quaternion, "single") && numel(p.attitude_quaternion) == 4);
end
