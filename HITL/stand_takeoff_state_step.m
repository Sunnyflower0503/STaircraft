function state = stand_takeoff_state_step(state, main_throttle, active_contact_count, dt, cfg)
%STAND_TAKEOFF_STATE_STEP Update stand-release, liftoff, and landing state.

arguments
    state struct
    main_throttle (1, 1) double
    active_contact_count (1, 1) double
    dt (1, 1) double {mustBeNonnegative}
    cfg struct
end

state = ensure_state_fields(state);
state.just_released = false;
state.just_liftoff_confirmed = false;
state.just_landing_confirmed = false;

switch string(state.phase)
    case "STAND_HOLD"
        if main_throttle > cfg.stand.release_throttle
            state.release_timer_s = state.release_timer_s + dt;
        else
            state.release_timer_s = 0;
        end

        if timer_reached(state.release_timer_s, cfg.stand.release_hold_s) && ~state.stand_released
            state.phase = "FLIGHT";
            state.stand_released = true;
            state.just_released = true;
        end

    case "FLIGHT"
        state.stand_released = true;

    case "LANDED"
        state.stand_released = true;
        return;

    otherwise
        error("stand_takeoff_state_step:BadPhase", "Unknown phase '%s'.", string(state.phase));
end

if state.stand_released && string(state.phase) ~= "LANDED"
    if active_contact_count == 0
        state.no_contact_timer_s = state.no_contact_timer_s + dt;
    else
        state.no_contact_timer_s = 0;
    end

    if ~state.liftoff_confirmed && timer_reached(state.no_contact_timer_s, cfg.landing.liftoff_confirm_s)
        state.liftoff_confirmed = true;
        state.just_liftoff_confirmed = true;
    end

    if state.liftoff_confirmed && active_contact_count >= cfg.landing.min_active_contacts
        state.landing_timer_s = state.landing_timer_s + dt;
    else
        state.landing_timer_s = 0;
    end

    if state.liftoff_confirmed && timer_reached(state.landing_timer_s, cfg.landing.confirm_s)
        state.phase = "LANDED";
        state.just_landing_confirmed = true;
    elseif state.stand_released
        state.phase = "FLIGHT";
    end
end
end

function tf = timer_reached(value, threshold)
tf = value + 10 * eps(max(1, abs(threshold))) >= threshold;
end

function state = ensure_state_fields(state)
defaults = initial_stand_takeoff_state();
names = fieldnames(defaults);
for k = 1:numel(names)
    name = names{k};
    if ~isfield(state, name)
        state.(name) = defaults.(name);
    end
end
end
