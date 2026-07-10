function test_openloop_model()
root = fileparts(fileparts(mfilename("fullpath")));
addpath(root); addpath(fullfile(root, "utils")); addpath(fullfile(fileparts(root), "matlab_model"));
cfg = hitl_config();
param = init_param_zx();
x = initial_state_from_param(param, cfg);
u = [0.3 * ones(10, 1); 0; 0];
t = 0; dt = cfg.dt;
while t < 5
    [~, z] = Runge_Kutta4(@(tt, xx) tandem_zx_dynamics(tt, xx, u, param), [t t + dt], x);
    x = z(:, end);
    x(7:10) = quat_normalize(x(7:10));
    t = t + dt;
end
assert(numel(x) == 13, "State dimension changed.");
assert(all(isfinite(x)), "State contains NaN or Inf.");
assert(abs(norm(x(7:10)) - 1) < 1e-8, "Quaternion norm drifted.");
end
