function test_advance_hitl_dynamics_substeps()
root = fileparts(fileparts(mfilename("fullpath")));
addpath(root); addpath(fullfile(root, "utils")); addpath(fullfile(fileparts(root), "matlab_model"));

cfg = hitl_config();
cfg.sample_time = 0.01;
cfg.dt = 0.001;

param = init_param_zx();
x0 = initial_state_from_param(param, cfg);
u = zeros(12, 1);

[x_hold, sim_time, n_substeps] = advance_hitl_dynamics_substeps(x0, u, param, cfg, 0, false);
assert(n_substeps == 10, "One 0.01 s main loop should contain ten 0.001 s substeps.");
assert(abs(sim_time - 0.01) < 1e-12, "One main loop should advance sim_time by 0.01 s.");
assert(max(abs(x_hold - x0)) < 1e-12, "STAND_HOLD should advance sim_time but freeze x.");
end
