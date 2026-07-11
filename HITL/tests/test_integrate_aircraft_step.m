function test_integrate_aircraft_step()
root = fileparts(fileparts(mfilename("fullpath")));
addpath(root); addpath(fullfile(root, "utils")); addpath(fullfile(fileparts(root), "matlab_model"));

cfg = hitl_config();
param = init_param_zx();
x0 = initial_state_from_param(param, cfg);
u = [0.3 * ones(10, 1); 0; 0];

cfg.model.force_enable = 0;
cfg.model.zero_force_mode = "freeze";
x_freeze = integrate_aircraft_step(0, x0, u, param, cfg);
assert(max(abs(x_freeze - x0)) < 1e-12, "freeze mode should hold the aircraft state.");

cfg.model.force_enable = 1;
x_forced = integrate_aircraft_step(0, x0, u, param, cfg);
assert(numel(x_forced) == 13, "Forced integration changed state dimension.");
assert(all(isfinite(x_forced)), "Forced integration produced NaN or Inf.");
assert(abs(norm(x_forced(7:10)) - 1) < 1e-10, "Quaternion should remain normalized.");

[~, z_default] = Runge_Kutta4(@(tt, xx) tandem_zx_dynamics(tt, xx, u, param), [0 cfg.sample_time], x0);
x_default_expected = z_default(:, end);
x_default_expected(7:10) = quat_normalize(x_default_expected(7:10));
assert(max(abs(x_forced - x_default_expected)) < 1e-12, ...
    "Default integrate_aircraft_step call should advance by cfg.sample_time.");

step_s = 0.027;
x_step = integrate_aircraft_step(0, x0, u, param, cfg, step_s);
[~, z_step] = Runge_Kutta4(@(tt, xx) tandem_zx_dynamics(tt, xx, u, param), [0 step_s], x0);
x_step_expected = z_step(:, end);
x_step_expected(7:10) = quat_normalize(x_step_expected(7:10));
assert(max(abs(x_step - x_step_expected)) < 1e-12, ...
    "Explicit step_s should control the integration interval.");
end
