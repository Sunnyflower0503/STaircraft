function stats = test_nose_up_90_static_hitl_io(duration_s)
%TEST_NOSE_UP_90_STATIC_HITL_IO Bounded, frozen R1 rotor HITL scenario.

if nargin < 1
    duration_s = 30;
end

tests_dir = fileparts(mfilename("fullpath"));
hitl_dir = fileparts(tests_dir);
root_dir = fileparts(hitl_dir);
addpath(hitl_dir);
addpath(fullfile(hitl_dir, "utils"));
addpath(fullfile(hitl_dir, "mavlink_backend"));
addpath(fullfile(root_dir, "matlab_model"));

cfg = hitl_config();
cfg.model.init_mode = "nose_up_90_stand_static";
cfg.model.force_enable = 0;
cfg.runtime_control.enable_file_control = false;
cfg.stand.angle_deg = 90;
cfg.stand.cache_file = fullfile(hitl_dir, "cache", "nose_up_90_stand_static_settled_state.mat");

param = init_param_zx();
param = apply_hitl_model_switches(param, cfg);
[param, x, u, meta] = prepare_stand_angle_static_for_hitl( ...
    param, cfg, cfg.stand.angle_deg, cfg.model.init_mode);
param = apply_hitl_model_switches(param, cfg);
[x, u, meta] = apply_user_initial_conditions(x, u, cfg, param, meta);
param.ground.enable = true;

stats = hitl_static_pose_loop( ...
    "R1 Nose-Up 90 deg Stand Static", cfg.model.init_mode, ...
    x, u, param, cfg, meta, duration_s);
end
