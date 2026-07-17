clear; clc;

repo_dir = fileparts(mfilename('fullpath'));
model_dir = fullfile(repo_dir, 'matlab_model');
addpath(model_dir);

param = init_param_zx();
param.ground.enable = true;

dt = 0.001;
t = 0:dt:4.0;
u = zeros(12, 1);

[theta_ground, q_eb0, p_e0, compression0, contact_z_rel] = natural_ground_pose(param);
x0 = [p_e0; zeros(3, 1); q_eb0; zeros(3, 1)];

[~, z] = Runge_Kutta4(@(tt, xx) tandem_zx_dynamics(tt, xx, u, param), t, x0);
xf = z(:, end);
qf = xf(7:10) / norm(xf(7:10));
R_eb_f = quat_to_rotm_local(qf);
euler_f = rotm_to_euler_zyx(R_eb_f);
[f_ground_b, m_ground_b, info] = zx_ground_contact_force( ...
    xf(1:3), xf(4:6), R_eb_f, xf(11:13), param);

normal_sum = sum(info.normal_force);
weight = param.m * param.g;
normal_error = normal_sum - weight;
max_penetration = max(info.penetration);
max_speed_tail = max(vecnorm(diff(z(4:6, max(1, end-500):end), 1, 2)));

fprintf('Ground static equilibrium test passed.\n');
fprintf('theta_ground = %.9f deg\n', theta_ground * param.R2D);
fprintf('initial CG z = %.9g m\n', p_e0(3));
fprintf('initial equal static compression = %.9g m\n', compression0);
fprintf('contact_z_rel spread = %.9g m\n', max(contact_z_rel) - min(contact_z_rel));
fprintf('final position NED = [%.9g %.9g %.9g]\n', xf(1:3));
fprintf('final Euler [roll pitch yaw] deg = [%.9g %.9g %.9g]\n', euler_f * param.R2D);
fprintf('final linear velocity = [%.9g %.9g %.9g]\n', xf(4:6));
fprintf('final angular velocity = [%.9g %.9g %.9g]\n', xf(11:13));
fprintf('penetrations = '); fprintf('%.9g ', info.penetration); fprintf('\n');
fprintf('normal forces = '); fprintf('%.9g ', info.normal_force); fprintf('\n');
fprintf('sum normal = %.9g N, m*g = %.9g N, error = %.9g N\n', normal_sum, weight, normal_error);
fprintf('ground moment body = [%.9g %.9g %.9g] Nm\n', m_ground_b);
fprintf('max penetration = %.9g m\n', max_penetration);
fprintf('tail velocity variation metric = %.9g\n', max_speed_tail);

assert(abs(theta_ground * param.R2D - 26.565051177) < 1e-6, 'Natural pitch angle mismatch.');
assert(max(contact_z_rel) - min(contact_z_rel) < 1e-12, 'Contact points are not coplanar at natural pitch.');
assert(all(isfinite(xf)), 'Final state contains non-finite values.');
assert(all(isfinite([f_ground_b; m_ground_b; info.normal_force(:); info.penetration(:)])), ...
    'Ground contact diagnostic output contains non-finite values.');
assert(abs(normal_error) < 0.5, 'Static total normal force does not balance weight closely enough.');
assert(abs(euler_f(1)) < 1e-3, 'Final roll angle should remain near zero.');
assert(abs(xf(11)) < 1e-3 && abs(xf(12)) < 1e-3, 'Final roll/pitch angular rates should be near zero.');

function [theta, q_eb, p_e, compression, contact_z_rel] = natural_ground_pose(param)
points = param.ground.contact_points_b;
p = polyfit(points(1, :).', points(3, :).', 1);
theta = atan(p(1));
R_eb = pitch_dcm_be(theta);
contact_rel_e = R_eb * points;
contact_z_rel = contact_rel_e(3, :);
N = size(points, 2);
k = param.ground.k;
if ~isscalar(k)
    k_eff = sum(k(:));
else
    k_eff = N * k;
end
compression = param.m * param.g / k_eff;
p_e = [0; 0; param.ground.z - mean(contact_z_rel) + compression];
q_eb = rotm_to_quat_local(R_eb);
end

function R = pitch_dcm_be(theta)
R = [cos(theta), 0, sin(theta); 0, 1, 0; -sin(theta), 0, cos(theta)];
end

function q = rotm_to_quat_local(R)
tr = trace(R);
if tr > 0
    s = sqrt(tr + 1) * 2;
    qw = 0.25 * s;
    qx = (R(3, 2) - R(2, 3)) / s;
    qy = (R(1, 3) - R(3, 1)) / s;
    qz = (R(2, 1) - R(1, 2)) / s;
else
    error('rotm_to_quat_local only supports this test rotation.');
end
q = [qw; qx; qy; qz];
q = q / norm(q);
end

function R = quat_to_rotm_local(q)
qw = q(1); qx = q(2); qy = q(3); qz = q(4);
R = [1 - 2*(qy^2 + qz^2), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw);
     2*(qx*qy + qz*qw), 1 - 2*(qx^2 + qz^2), 2*(qy*qz - qx*qw);
     2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx^2 + qy^2)];
end

function euler = rotm_to_euler_zyx(R)
pitch = asin(-R(3, 1));
roll = atan2(R(3, 2), R(3, 3));
yaw = atan2(R(2, 1), R(1, 1));
euler = [roll; pitch; yaw];
end
