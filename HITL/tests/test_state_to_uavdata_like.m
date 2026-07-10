function test_state_to_uavdata_like()
root = fileparts(fileparts(mfilename("fullpath")));
addpath(root); addpath(fullfile(root, "utils")); addpath(fullfile(fileparts(root), "matlab_model"));
cfg = hitl_config(); cfg.init.AMSL = 100;
param = init_param_zx(); param.InitData.AMSL = 100; param.InitData.lat_deg = 30; param.InitData.lon_deg = 120;
x = [0; 0; -10; 5; 0; 0; 1; 0; 0; 0; 0; 0; 0];
uav = state_to_uavdata_like(1.5, x, zeros(12, 1), param, cfg);
assert(abs(uav.AMSL - 110) < 1e-12, "AMSL conversion failed.");
assert(all(~isnan(uav.Ve)), "Ve contains NaN.");
assert(uav.TAS >= 0 && uav.EAS >= 0, "Airspeed should be non-negative.");
assert(all(size(uav.DCM_be) == [3 3]), "DCM_be has wrong size.");
assert(abs(norm(uav.q_eb) - 1) < 1e-12, "Quaternion is not normalized.");
end
