function test_frozen_runner_specific_force()
%TEST_FROZEN_RUNNER_SPECIFIC_FORCE All static run poses must report one g.

root = fileparts(fileparts(mfilename("fullpath")));
addpath(root);
addpath(fullfile(root, "utils"));
addpath(fullfile(fileparts(root), "matlab_model"));

cfg = hitl_config();
cfg.model.force_enable = 0;
param = apply_hitl_model_switches(init_param_zx(), cfg);

[param_stand, x_stand, u_stand] = prepare_stand_static_for_hitl(param, cfg);
assert_frozen_specific_force("stand_static", x_stand, u_stand, param_stand, cfg);

[param_flat, x_flat, u_flat] = prepare_ground_flat_static_for_hitl(param, cfg);
assert_frozen_specific_force("ground_flat_static", x_flat, u_flat, param_flat, cfg);

[param_hover, x_hover, u_hover] = prepare_airborne_nose_up_hover_for_hitl(param, cfg);
assert_frozen_specific_force("airborne_nose_up_hover", x_hover, u_hover, param_hover, cfg);

cfg_90 = cfg;
cfg_90.stand.angle_deg = 90;
cfg_90.stand.cache_file = fullfile(root, "cache", "nose_up_90_stand_static_settled_state.mat");
[param_90, x_90, u_90] = prepare_stand_angle_static_for_hitl( ...
    param, cfg_90, 90, "nose_up_90_stand_static");
assert_frozen_specific_force("nose_up_90_stand_static", x_90, u_90, param_90, cfg_90);
end

function assert_frozen_specific_force(name, x, u, param, cfg)
uav = state_to_uavdata_like(1.0, x, u, param, cfg);
expected = -uav.DCM_be * [0; 0; cfg.env.g];
assert(norm(uav.ab - expected) < 1e-10, ...
    "%s frozen IMU direction is inconsistent with its attitude.", name);
assert(abs(norm(uav.ab) - cfg.env.g) < 1e-10, ...
    "%s frozen IMU must report one g.", name);
assert(norm(uav.Ve) < 1e-10, ...
    "%s frozen initial velocity must be zero.", name);
end
