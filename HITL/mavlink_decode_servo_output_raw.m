function servo_msg = mavlink_decode_servo_output_raw(bytes, cfg)
%MAVLINK_DECODE_SERVO_OUTPUT_RAW Decode latest SERVO_OUTPUT_RAW message.

backend_dir = fullfile(fileparts(mfilename("fullpath")), "mavlink_backend");
if exist(backend_dir, "dir") && ~contains(path, backend_dir)
    addpath(backend_dir);
end

bytes = uint8(bytes(:));
switch string(cfg.mavlink.backend)
    case "stub"
        if isempty(bytes)
            servo_msg = default_servo_msg(false);
            return;
        end
        error("MAVLink decode backend is not configured.");
    case "pymavlink"
        servo_msg = pymavlink_decode_servo_output_raw(bytes, cfg);
    otherwise
        error("Unsupported MAVLink decode backend '%s'. Configure pymavlink or MATLAB UAV Toolbox integration.", cfg.mavlink.backend);
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

