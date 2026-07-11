function test_apply_user_initial_conditions()
root = fileparts(fileparts(mfilename("fullpath")));
addpath(root); addpath(fullfile(root, "utils")); addpath(fullfile(fileparts(root), "matlab_model"));

cfg = hitl_config();
param = init_param_zx();
x0 = [10; 20; -3; 1; 2; 3; 1; 0; 0; 0; 0.1; 0.2; 0.3];
u0 = (1:12).';
meta = struct("mode", "stand_static");

[x_cache, u_cache, meta_cache] = apply_user_initial_conditions(x0, u0, cfg, param, meta);
assert(isequal(x_cache, x0) && isequal(u_cache, u0), "Default stand_cache must preserve cached state.");
assert(isempty(meta_cache.user_initial_conditions.applied_fields), "Default stand_cache must apply no fields.");

cfg.user.ic.enable_override = true;
cfg.user.ic.override_attitude = true;
cfg.user.ic.override_velocity = true;
cfg.user.ic.Euler_deg = [0; 90; 0];
cfg.user.ic.Vb_mps = [2; 0; 0];
[x_override, ~, meta_override] = apply_user_initial_conditions(x0, u0, cfg, param, meta);
assert(max(abs(x_override(7:10) - [sqrt(0.5); 0; sqrt(0.5); 0])) < 1e-12, "Attitude override quaternion mismatch.");
assert(max(abs(x_override(4:6) - [0; 0; -2])) < 1e-12, "Vb to NED conversion mismatch.");
assert(isequal(meta_override.user_initial_conditions.applied_fields, ["attitude"; "velocity"]), "Override field summary mismatch.");

cfg.user.ic.mode = "manual";
cfg.user.ic.Xe_NED_m = [7; 8; -9];
cfg.user.ic.Euler_deg = [0; 0; 90];
cfg.user.ic.Vb_mps = [3; 0; 0];
cfg.user.ic.pqr_radps = [0.4; 0.5; 0.6];
cfg.user.ic.u0 = 0.25 * ones(12, 1);
[x_manual, u_manual] = apply_user_initial_conditions(x0, u0, cfg, param, meta);
assert(isequal(x_manual(1:3), [7; 8; -9]), "Manual position mismatch.");
assert(max(abs(x_manual(4:6) - [0; 3; 0])) < 1e-12, "Manual Vb to NED conversion mismatch.");
assert(max(abs(x_manual(7:10) - [sqrt(0.5); 0; 0; sqrt(0.5)])) < 1e-12, "Manual quaternion mismatch.");
assert(isequal(x_manual(11:13), [0.4; 0.5; 0.6]), "Manual rates mismatch.");
assert(isequal(u_manual, 0.25 * ones(12, 1)), "Manual u0 mismatch.");
end
