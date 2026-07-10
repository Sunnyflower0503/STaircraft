function bytes = mavlink_encode_hil_state_quaternion(payload, cfg)
%MAVLINK_ENCODE_HIL_STATE_QUATERNION Encode HIL_STATE_QUATERNION bytes.

backend_dir = fullfile(fileparts(mfilename("fullpath")), "mavlink_backend");
if exist(backend_dir, "dir") && ~contains(path, backend_dir)
    addpath(backend_dir);
end

switch string(cfg.mavlink.backend)
    case "stub"
        error("MAVLink encode backend is not configured.");
    case "pymavlink"
        bytes = pymavlink_encode_hil_state_quaternion(payload, cfg);
    otherwise
        error("Unsupported MAVLink encode backend '%s'. Configure pymavlink or MATLAB UAV Toolbox integration.", cfg.mavlink.backend);
end

bytes = uint8(bytes(:));
end

