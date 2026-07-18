function [u_delayed, state] = apply_actuator_transport_delay(time_s, u_commanded, state, cfg)
%APPLY_ACTUATOR_TRANSPORT_DELAY Delay motor and elevon commands independently.
% Motors are u(1:10); elevons are u(11:12).

u_commanded = u_commanded(:);
if numel(u_commanded) ~= 12
    error("apply_actuator_transport_delay:BadInput", "u_commanded must contain 12 elements.");
end

if isempty(state)
    state.time_s = time_s;
    state.u = u_commanded;
else
    if time_s < state.time_s(end)
        error("apply_actuator_transport_delay:NonMonotonicTime", "time_s must be monotonic.");
    end
    if time_s == state.time_s(end)
        state.u(:, end) = u_commanded;
    else
        state.time_s(end + 1) = time_s;
        state.u(:, end + 1) = u_commanded;
    end
end

u_delayed = u_commanded;
u_delayed(1:10) = command_at(time_s - cfg.actuator_delay.motor_s, state, 1:10);
u_delayed(11:12) = command_at(time_s - cfg.actuator_delay.elevon_s, state, 11:12);

oldest_needed_s = time_s - max(cfg.actuator_delay.motor_s, cfg.actuator_delay.elevon_s);
keep_from = find(state.time_s <= oldest_needed_s, 1, "last");
if isempty(keep_from)
    keep_from = 1;
end
state.time_s = state.time_s(keep_from:end);
state.u = state.u(:, keep_from:end);
end

function value = command_at(query_time_s, state, channels)
index = find(state.time_s <= query_time_s, 1, "last");
if isempty(index)
    index = 1;
end
value = state.u(channels, index);
end
