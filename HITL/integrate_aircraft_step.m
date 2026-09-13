function x_next = integrate_aircraft_step(t, x, u, param, cfg, step_s)
%INTEGRATE_AIRCRAFT_STEP Advance or freeze the HITL plant state.

if nargin < 6 || isempty(step_s)
    step_s = cfg.sample_time;
end
if ~isscalar(step_s) || ~isfinite(step_s) || step_s < 0
    error("integrate_aircraft_step:BadStep", "step_s must be a finite nonnegative scalar.");
end

if cfg.model.force_enable == 1 && step_s > 0
    substep_count = ground_substep_count(x, param, cfg, step_s);
    substep_s = step_s / substep_count;
    x_next = x;

    for substep_index = 1:substep_count
        substep_t = t + (substep_index - 1) * substep_s;
        [~, z] = Runge_Kutta4(@(tt, xx) tandem_zx_dynamics(tt, xx, u, param), ...
            [substep_t substep_t + substep_s], x_next);
        x_next = z(:, end);
        x_next(7:10) = quat_normalize(x_next(7:10));
    end
else
    x_next = x;
end

x_next(7:10) = quat_normalize(x_next(7:10));
end

function count = ground_substep_count(x, param, cfg, step_s)
% Use short integration steps only close to the ground.  This keeps the
% 10x-stiffer contact model stable without slowing the airborne mission.
count = 1;
if ~isfield(param, "ground") || ~isfield(param.ground, "enable") || ...
        ~param.ground.enable || ~isfield(param.ground, "contact_points_b")
    return;
end

max_step_s = 0.0025;
margin_m = 0.10;
if isfield(cfg.model, "ground_max_step_s")
    max_step_s = cfg.model.ground_max_step_s;
end
if isfield(cfg.model, "ground_substep_margin_m")
    margin_m = cfg.model.ground_substep_margin_m;
end
if ~isfinite(max_step_s) || max_step_s <= 0
    return;
end

q_eb = quat_normalize(x(7:10));
R_eb = quat_to_dcm_be(q_eb).';
contact_pos_e = x(1:3) + R_eb * param.ground.contact_points_b;
ground_z = 0;
if isfield(param.ground, "z")
    ground_z = param.ground.z;
end

if max(contact_pos_e(3, :)) >= ground_z - margin_m
    count = max(1, ceil(step_s / max_step_s));
end
end
