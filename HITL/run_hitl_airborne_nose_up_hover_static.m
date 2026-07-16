%RUN_HITL_AIRBORNE_NOSE_UP_HOVER_STATIC HITL pose with aircraft airborne, nose-up, and stationary.
% The initial state is in the air, pitch 90 deg, zero velocity/rates.

clearvars -except ans; clc;

hitl_dir = fileparts(mfilename("fullpath"));
root_dir = fileparts(hitl_dir);
addpath(hitl_dir);
addpath(fullfile(hitl_dir, "utils"));
addpath(fullfile(hitl_dir, "mavlink_backend"));
addpath(fullfile(root_dir, "matlab_model"));

cfg = hitl_config();
cfg.model.init_mode = "airborne_nose_up_hover";

param = init_param_zx();
param = apply_hitl_model_switches(param, cfg);
[param, x, u, meta] = prepare_airborne_nose_up_hover_for_hitl(param, cfg);
param = apply_hitl_model_switches(param, cfg);
[x, u, meta] = apply_user_initial_conditions(x, u, cfg, param, meta);

hitl_static_pose_loop("Airborne Nose-Up Hover Static", cfg.model.init_mode, x, u, param, cfg, meta);
