function x_next = integrate_aircraft_step(t, x, u, param, cfg, step_s)
%INTEGRATE_AIRCRAFT_STEP Advance or freeze the HITL plant state.

if nargin < 6 || isempty(step_s)
    step_s = cfg.sample_time;
end
if ~isscalar(step_s) || ~isfinite(step_s) || step_s < 0
    error("integrate_aircraft_step:BadStep", "step_s must be a finite nonnegative scalar.");
end

if cfg.model.force_enable == 1 && step_s > 0
    [~, z] = Runge_Kutta4(@(tt, xx) tandem_zx_dynamics(tt, xx, u, param), [t t + step_s], x);
    x_next = z(:, end);
else
    x_next = x;
end

x_next(7:10) = quat_normalize(x_next(7:10));
end
