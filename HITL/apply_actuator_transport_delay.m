function [u_delayed, state] = apply_actuator_transport_delay(time_s, u_commanded, state, cfg)
%APPLY_ACTUATOR_TRANSPORT_DELAY Apply transport delay and first-order actuator lag.
% Motors are u(1:10); elevons are u(11:12).

u_commanded = u_commanded(:);
if numel(u_commanded) ~= 12
    error("apply_actuator_transport_delay:BadInput", "u_commanded must contain 12 elements.");
end

if isempty(state)
    state.time_s = time_s;
    state.u = u_commanded;
    state.filter_time_s = time_s;
    state.filtered_u = u_commanded;
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

u_target = u_commanded;
u_target(1:10) = command_at(time_s - cfg.actuator_delay.motor_s, state, 1:10);
u_target(11:12) = command_at(time_s - cfg.actuator_delay.elevon_s, state, 11:12);

dt_s = max(0, time_s - state.filter_time_s);
motor_tau_s = optional_delay_field(cfg.actuator_delay, "motor_tau_s", 0);
elevon_tau_s = optional_delay_field(cfg.actuator_delay, "elevon_tau_s", 0);
state.filtered_u(1:10) = first_order_step(state.filtered_u(1:10), u_target(1:10), dt_s, motor_tau_s);
state.filtered_u(11:12) = first_order_step(state.filtered_u(11:12), u_target(11:12), dt_s, elevon_tau_s);
state.filter_time_s = time_s;
u_delayed = state.filtered_u;

oldest_needed_s = time_s - max(cfg.actuator_delay.motor_s, cfg.actuator_delay.elevon_s);
keep_from = find(state.time_s <= oldest_needed_s, 1, "last");
if isempty(keep_from)
    keep_from = 1;
end
state.time_s = state.time_s(keep_from:end);
state.u = state.u(:, keep_from:end);
end

function value = first_order_step(previous, target, dt_s, tau_s)
if tau_s <= 0 || dt_s <= 0
    if tau_s <= 0
        value = target;
    else
        value = previous;
    end
    return;
end
alpha = 1 - exp(-dt_s / tau_s);
value = previous + alpha .* (target - previous);
end

function value = optional_delay_field(delay_cfg, name, default_value)
if isfield(delay_cfg, name)
    value = delay_cfg.(name);
else
    value = default_value;
end
end

function value = command_at(query_time_s, state, channels)
index = find(state.time_s <= query_time_s, 1, "last");
if isempty(index)
    index = 1;
end
value = state.u(channels, index);
end
