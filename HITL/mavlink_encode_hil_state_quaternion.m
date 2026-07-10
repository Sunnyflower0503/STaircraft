function bytes = mavlink_encode_hil_state_quaternion(payload, cfg)
%MAVLINK_ENCODE_HIL_STATE_QUATERNION Encode HIL_STATE_QUATERNION bytes.

switch string(cfg.mavlink.backend)
    case "stub"
        error("MAVLink encode backend is not configured.");
    otherwise
        error("Unsupported MAVLink encode backend '%s'. Configure pymavlink, mavlink_c, or MATLAB UAV Toolbox integration.", cfg.mavlink.backend);
end

bytes = zeros(0, 1, "uint8"); %#ok<UNRCH>
end
