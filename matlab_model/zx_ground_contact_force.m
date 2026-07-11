function [f_ground_b, m_ground_b, info] = zx_ground_contact_force(p_e, v_e, R_eb, omega_b, param)
%ZX_GROUND_CONTACT_FORCE  N-point unilateral spring-damper ground contact.
%
% NED convention: earth z is positive downward. The ground plane is gnd.z,
% and a positive penetration means the contact point is below the ground.
% Contact forces are returned in body axes; moments are about the CG.

gnd = param.ground;
contact_points_b = get_ground_field(gnd, 'contact_points_b', zeros(3, 0));
if size(contact_points_b, 1) ~= 3
    error('zx_ground_contact_force:BadContactPoints', ...
        'param.ground.contact_points_b must be a 3xN matrix.');
end

num_points = size(contact_points_b, 2);
z_ground = get_ground_field(gnd, 'z', 0);
k = expand_ground_param(get_ground_field(gnd, 'k', 700), num_points, 'k');
c = expand_ground_param(get_ground_field(gnd, 'c', 70), num_points, 'c');
mu = expand_ground_param(get_ground_field(gnd, 'mu', 0.55), num_points, 'mu');
xy_damping = expand_ground_param(get_ground_field(gnd, 'xy_damping', 35), num_points, 'xy_damping');
v_eps = get_ground_field(gnd, 'friction_v_eps', 0.05);

R_be = R_eb';
v_cg_b = R_be * v_e(:);
omega_b = omega_b(:);
p_e = p_e(:);

f_ground_b = zeros(3, 1);
m_ground_b = zeros(3, 1);

info.contact_points_b = contact_points_b;
info.contact_pos_e = zeros(3, num_points);
info.contact_vel_b = zeros(3, num_points);
info.contact_vel_e = zeros(3, num_points);
info.penetration = zeros(1, num_points);
info.penetration_rate = zeros(1, num_points);
info.normal_force = zeros(1, num_points);
info.friction_force_e = zeros(3, num_points);
info.force_e = zeros(3, num_points);
info.force_b = zeros(3, num_points);
info.moment_b = zeros(3, num_points);
info.active = false(1, num_points);

for ip = 1:num_points
    r_b = contact_points_b(:, ip);
    contact_pos_e = p_e + R_eb * r_b;
    v_contact_b = v_cg_b + cross(omega_b, r_b);
    v_contact_e = R_eb * v_contact_b;

    penetration = contact_pos_e(3) - z_ground;
    penetration_rate = v_contact_e(3);

    force_e = zeros(3, 1);
    friction_e = zeros(3, 1);
    normal = 0;

    if penetration > 0
        normal = max(0, k(ip) * penetration + c(ip) * penetration_rate);
        if normal > 0
            force_e(3) = -normal;
            friction_e = regularized_friction_force(v_contact_e, normal, mu(ip), xy_damping(ip), v_eps);
            force_e = force_e + friction_e;
        end
    end

    force_b = R_be * force_e;
    moment_b = cross(r_b, force_b);

    f_ground_b = f_ground_b + force_b;
    m_ground_b = m_ground_b + moment_b;

    info.contact_pos_e(:, ip) = contact_pos_e;
    info.contact_vel_b(:, ip) = v_contact_b;
    info.contact_vel_e(:, ip) = v_contact_e;
    info.penetration(ip) = max(penetration, 0);
    info.penetration_rate(ip) = penetration_rate;
    info.normal_force(ip) = normal;
    info.friction_force_e(:, ip) = friction_e;
    info.force_e(:, ip) = force_e;
    info.force_b(:, ip) = force_b;
    info.moment_b(:, ip) = moment_b;
    info.active(ip) = normal > 0;
end
end

function friction_e = regularized_friction_force(v_contact_e, normal, mu, xy_damping, v_eps)
v_t = v_contact_e(1:2);
speed = norm(v_t);
friction_e = zeros(3, 1);
if normal <= 0 || speed <= 0
    return;
end

viscous = xy_damping * speed;
coulomb = mu * normal * tanh(speed / max(v_eps, 1e-9));
magnitude = min(viscous, coulomb);
friction_e(1:2) = -magnitude * v_t / speed;
end

function value = get_ground_field(gnd, name, default_value)
if isfield(gnd, name)
    value = gnd.(name);
else
    value = default_value;
end
end

function value = expand_ground_param(value, num_points, name)
value = value(:).';
if isscalar(value)
    value = repmat(value, 1, num_points);
elseif numel(value) ~= num_points
    error('zx_ground_contact_force:BadGroundParam', ...
        'param.ground.%s must be scalar or length %d.', name, num_points);
end
end
