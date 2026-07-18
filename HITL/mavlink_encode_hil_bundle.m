function bytes = mavlink_encode_hil_bundle(sensor_payload, state_payload, cfg)
%MAVLINK_ENCODE_HIL_BUNDLE Encode coherent HIL_SENSOR and HIL state frames.

backend_dir = fullfile(fileparts(mfilename("fullpath")), "mavlink_backend");
if exist(backend_dir, "dir") && ~contains(path, backend_dir)
    addpath(backend_dir);
end

switch string(cfg.mavlink.backend)
    case "stub"
        error("MAVLink encode backend is not configured.");
    case "pymavlink"
        bytes = pymavlink_encode_hil_bundle(sensor_payload, state_payload, cfg);
    otherwise
        error("Unsupported MAVLink encode backend '%s'.", cfg.mavlink.backend);
end

bytes = uint8(bytes(:));
end
