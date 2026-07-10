function servo_msg = mavlink_decode_servo_output_raw(bytes, cfg)
%MAVLINK_DECODE_SERVO_OUTPUT_RAW Decode latest SERVO_OUTPUT_RAW message.
% The real backend must maintain cross-call parser state for split packets.

persistent parser_state
if isempty(parser_state)
    parser_state = struct("buffer", zeros(0, 1, "uint8"));
end

servo_msg = default_servo_msg(false);
bytes = uint8(bytes(:));

switch string(cfg.mavlink.backend)
    case "stub"
        if isempty(bytes)
            return;
        end
        error("MAVLink decode backend is not configured.");
    otherwise
        error("Unsupported MAVLink decode backend '%s'. Configure pymavlink, mavlink_c, or MATLAB UAV Toolbox integration.", cfg.mavlink.backend);
end
end

function msg = default_servo_msg(is_new)
msg = struct();
msg.is_new = is_new;
msg.timestamp = [];
for k = 1:8
    msg.(sprintf("servo%d_raw", k)) = uint16(0);
end
end
