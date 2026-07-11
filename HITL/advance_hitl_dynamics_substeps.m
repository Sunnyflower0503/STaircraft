function [x, sim_time, n_substeps] = advance_hitl_dynamics_substeps(x, u, param, cfg, sim_time, do_integrate)
%ADVANCE_HITL_DYNAMICS_SUBSTEPS Advance one HITL main-loop period.

arguments
    x (13, 1) double
    u (12, 1) double
    param struct
    cfg struct
    sim_time (1, 1) double
    do_integrate (1, 1) logical
end

n_substeps = round(cfg.sample_time / cfg.dt);
if n_substeps < 1
    error("advance_hitl_dynamics_substeps:BadStepRatio", ...
        "cfg.sample_time / cfg.dt must be at least 1.");
end

for k = 1:n_substeps
    t0 = sim_time;
    t1 = sim_time + cfg.dt;
    if do_integrate
        [~, z] = Runge_Kutta4(@(tt, xx) tandem_zx_dynamics(tt, xx, u, param), [t0 t1], x);
        x = z(:, end);
        x(7:10) = quat_normalize(x(7:10));
    end
    sim_time = t1;
end
end
