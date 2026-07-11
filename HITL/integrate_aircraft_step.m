function x_next = integrate_aircraft_step(t, x, u, param, cfg)
%INTEGRATE_AIRCRAFT_STEP Advance or freeze the HITL plant state.

if cfg.model.force_enable == 1
    [~, z] = Runge_Kutta4(@(tt, xx) tandem_zx_dynamics(tt, xx, u, param), [t t + cfg.dt], x);
    x_next = z(:, end);
else
    x_next = x;
end

x_next(7:10) = quat_normalize(x_next(7:10));
end
