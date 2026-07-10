function state = real_time_pacer(state, sim_time_s, cfg)
%REAL_TIME_PACER Keep simulation time aligned to wall-clock time.

if nargin < 1 || isempty(state)
    state = struct("wall_start", tic, "last_warning_sim_time", -inf);
end

expected_wall_s = sim_time_s / cfg.pace;
elapsed_wall_s = toc(state.wall_start);
lag_s = elapsed_wall_s - expected_wall_s;

if lag_s < 0
    pause(-lag_s);
elseif lag_s > cfg.dt && sim_time_s - state.last_warning_sim_time >= 1
    warning("real_time_pacer:BehindWallClock", ...
        "HITL loop is %.3f s behind wall-clock at sim t=%.2f s.", lag_s, sim_time_s);
    state.last_warning_sim_time = sim_time_s;
end
end
