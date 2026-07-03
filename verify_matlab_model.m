clear; clc;

repo_dir = fileparts(mfilename('fullpath'));
model_dir = fullfile(repo_dir, 'matlab_model');
addpath(model_dir);

param = init_param_zx();

x0 = zeros(13, 1);
x0(3) = -30.0;
x0(4) = 12.0;
x0(7) = 1.0;

u = zeros(12, 1);
u(1:8) = 0.45;
u(9:10) = 0.0;
u(11:12) = 0.0;

dx = tandem_zx_dynamics(0.0, x0, u, param);
assert(isequal(size(dx), [13 1]), 'tandem_zx_dynamics output must be 13x1.');
assert(all(isfinite(dx)), 'tandem_zx_dynamics output contains non-finite values.');

t = 0:0.01:0.05;
[~, z, zdot] = Runge_Kutta4(@(tt, xx) tandem_zx_dynamics(tt, xx, u, param), t, x0);
assert(isequal(size(z), [13 numel(t)]), 'Runge_Kutta4 state output has unexpected size.');
assert(size(zdot, 1) == 13, 'Runge_Kutta4 derivative output has unexpected size.');
assert(all(isfinite(z(:))), 'Runge_Kutta4 state output contains non-finite values.');

fprintf('MATLAB model verification passed.\n');
fprintf('dx norm: %.12g\n', norm(dx));
fprintf('final state norm: %.12g\n', norm(z(:, end)));
