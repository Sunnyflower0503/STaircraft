%RUN_HITL_NOSE_UP_90_STAND_STATIC Frozen HITL pose with a 90 deg removable stand.
% Use this for semi-physical PX4/QGC checks where the aircraft appears nose-up.

clearvars -except ans; clc;

hitl_dir = fileparts(mfilename("fullpath"));
root_dir = fileparts(hitl_dir);
addpath(hitl_dir);
addpath(fullfile(hitl_dir, "utils"));
addpath(fullfile(hitl_dir, "mavlink_backend"));
addpath(fullfile(root_dir, "matlab_model"));

cfg = hitl_config();
cfg.model.init_mode = "nose_up_90_stand_static";
cfg.stand.angle_deg = 90;
cfg.stand.cache_file = fullfile(hitl_dir, "cache", "nose_up_90_stand_static_settled_state.mat");

param = init_param_zx();
param = apply_hitl_model_switches(param, cfg);
[param, x, u, meta] = prepare_stand_angle_static_for_hitl(param, cfg, cfg.stand.angle_deg, cfg.model.init_mode);
param = apply_hitl_model_switches(param, cfg);
[x, u, meta] = apply_user_initial_conditions(x, u, cfg, param, meta);
param.ground.enable = true;

hitl_static_pose_loop("Nose-Up 90 deg Stand Static", cfg.model.init_mode, x, u, param, cfg, meta);
