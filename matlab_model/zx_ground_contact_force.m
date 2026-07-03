function [f_ground_e, info] = zx_ground_contact_force(p_e, v_e, f_non_ground_e, param)
%ZX_GROUND_CONTACT_FORCE  Simple ground contact force for NED dynamics.
%
% NED convention: z is positive downward. The ground plane is ground.z,
% normally 0. The normal reaction points upward, i.e. negative earth-z.
%
% This model is intentionally a center-of-mass contact model. It prevents
% the aircraft from falling through the ground and releases it automatically
% once non-ground forces provide enough upward acceleration for takeoff.

gnd = param.ground;
z_ground = get_ground_field(gnd, 'z', 0);
tol = get_ground_field(gnd, 'tol', 1e-4);
k = get_ground_field(gnd, 'k', 700);
c = get_ground_field(gnd, 'c', 70);
mu = get_ground_field(gnd, 'mu', 0.55);
xy_damping = get_ground_field(gnd, 'xy_damping', 35);

z = p_e(3);
vz = v_e(3);
penetration = max(z - z_ground, 0);
near_ground = z >= z_ground - tol;

f_ground_e = zeros(3, 1);
normal = 0;

if near_ground || penetration > 0
    downward_load = f_non_ground_e(3);
    damping_load = c * max(vz, 0);
    spring_load = k * penetration;

    % If the aircraft is resting on the ground and the non-ground vertical
    % force still points into the ground, the normal force exactly supports
    % that load. If thrust/lift overcomes weight, downward_load <= 0 and the
    % normal force vanishes, allowing natural liftoff.
    normal = max(0, downward_load + damping_load + spring_load);
    f_ground_e(3) = -normal;

    if normal > 0
        f_fric_des = -xy_damping * v_e(1:2);
        f_fric_norm = norm(f_fric_des);
        f_fric_cap = mu * normal;
        if f_fric_norm > f_fric_cap && f_fric_norm > 1e-12
            f_fric_des = f_fric_des * (f_fric_cap / f_fric_norm);
        end
        f_ground_e(1:2) = f_fric_des;
    end
end

if nargout > 1
    info.active = normal > 0;
    info.normal = normal;
    info.penetration = penetration;
    info.near_ground = near_ground;
end
end

function value = get_ground_field(gnd, name, default_value)
if isfield(gnd, name)
    value = gnd.(name);
else
    value = default_value;
end
end
