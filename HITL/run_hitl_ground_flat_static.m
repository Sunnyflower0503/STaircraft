%RUN_HITL_GROUND_FLAT_STATIC Frozen HITL pose with aircraft level and nose-forward.
% Use this for PX4/QGC fixed-wing and multicopter mode checks on the ground.

clearvars -except ans; clc;

hitl_dir = fileparts(mfilename("fullpath"));
root_dir = fileparts(hitl_dir);
addpath(hitl_dir);
addpath(fullfile(hitl_dir, "utils"));
addpath(fullfile(hitl_dir, "mavlink_backend"));
addpath(fullfile(root_dir, "matlab_model"));

cfg = hitl_config();
cfg.model.init_mode = "ground_flat_static";

param = init_param_zx();
param = apply_hitl_model_switches(param, cfg);
[param, x, u, meta] = prepare_ground_flat_static_for_hitl(param, cfg);
param = apply_hitl_model_switches(param, cfg);
[x, u, meta] = apply_user_initial_conditions(x, u, cfg, param, meta);
param.ground.enable = true;

hitl_static_pose_loop("Ground Flat Nose-Forward Static", cfg.model.init_mode, x, u, param, cfg, meta);
