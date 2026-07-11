function state = initial_stand_takeoff_state()
%INITIAL_STAND_TAKEOFF_STATE Runtime state for stand takeoff HITL mode.

state = struct();
state.phase = "STAND_HOLD";
state.stand_released = false;
state.liftoff_confirmed = false;
state.release_timer_s = 0;
state.no_contact_timer_s = 0;
state.landing_timer_s = 0;
state.just_released = false;
state.just_liftoff_confirmed = false;
state.just_landing_confirmed = false;
end
