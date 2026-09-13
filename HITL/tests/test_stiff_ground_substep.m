function test_stiff_ground_substep()
root = fileparts(fileparts(mfilename("fullpath")));
addpath(root); addpath(fullfile(root, "utils")); addpath(fullfile(fileparts(root), "matlab_model"));

cfg = hitl_config();
cfg.model.force_enable = 1;
param = init_param_zx();
[param, x0, u] = prepare_ground_flat_static_for_hitl(param, cfg);

assert(param.ground.k >= 7000, "The landing surface must retain the stiff-contact calibration.");
assert(param.ground.c >= 180, "The stiff landing surface must retain its matched damping.");

% Drop the already-level vehicle by 20 mm at 0.3 m/s.  One outer 10 ms
% plant step must be identical to four explicit 2.5 ms near-ground steps.
x0(3) = x0(3) - 0.02;
x0(6) = 0.3;
x_outer = integrate_aircraft_step(0, x0, u, param, cfg, 0.01);
x_inner = x0;
for k = 1:4
    x_inner = integrate_aircraft_step((k - 1) * 0.0025, x_inner, u, param, cfg, 0.0025);
end

assert(all(isfinite(x_outer)), "Stiff ground integration produced NaN or Inf.");
assert(max(abs(x_outer - x_inner)) < 1e-10, ...
    "Near-ground integration did not use the configured stable substep.");
end
