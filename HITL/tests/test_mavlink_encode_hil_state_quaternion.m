function test_mavlink_encode_hil_state_quaternion()
this_dir = fileparts(mfilename("fullpath"));
root = fileparts(this_dir);
addpath(root); addpath(fullfile(root, "utils")); addpath(fullfile(root, "mavlink_backend"));

cfg = hitl_config();
assert(hitl_contact_bitmask([false false false true true true]) == uint8(56), ...
    "Rear-three contacts must occupy TD_CNTCT bits 3..5.");
assert(hitl_contact_bitmask(true(1, 6)) == uint8(63), ...
    "All six contacts must encode as TD_CNTCT=63.");
payload = struct();
payload.time_usec = uint64(1000000);
payload.attitude_quaternion = single([1 0 0 0]);
payload.rollspeed = single(0);
payload.pitchspeed = single(0);
payload.yawspeed = single(0);
payload.lat = int32(0);
payload.lon = int32(0);
payload.alt = int32(0);
payload.vx = int16(0);
payload.vy = int16(0);
payload.vz = int16(0);
payload.ind_airspeed = uint16(0);
payload.true_airspeed = uint16(0);
payload.xacc = int16(0);
payload.yacc = int16(0);
payload.zacc = int16(0);

try
    bytes = mavlink_encode_hil_state_quaternion(payload, cfg);
catch ME
    if contains(ME.message, "pymavlink") || contains(ME.message, "MAVLink encode backend")
        error("test_mavlink_encode_hil_state_quaternion:BackendUnavailable", ...
            "MAVLink backend is not ready: %s", ME.message);
    end
    rethrow(ME);
end

assert(isa(bytes, "uint8"), "Encoded bytes must be uint8.");
assert(~isempty(bytes), "Encoded bytes must be non-empty.");
assert(bytes(1) == uint8(hex2dec("FD")), "Expected MAVLink v2 frame header 0xFD.");

bytes_with_contact = mavlink_encode_hil_state_quaternion(payload, cfg, uint8(56));
assert(numel(bytes_with_contact) == numel(bytes), ...
    "TD_CNTCT must be present at a fixed frame size for zero and nonzero masks.");
end
