function bytes = pymavlink_encode_hil_state_quaternion(payload, cfg)
%PYMAVLINK_ENCODE_HIL_STATE_QUATERNION Encode HIL_STATE_QUATERNION via pymavlink.

persistent bridge
if isempty(bridge)
    bridge = mavlink_backend_init(cfg);
end

py_payload = py.dict();
py_payload{"time_usec"} = py.int(uint64(payload.time_usec));
py_payload{"attitude_quaternion"} = py.list(num2cell(double(payload.attitude_quaternion(:).')));
py_payload{"rollspeed"} = py.float(double(payload.rollspeed));
py_payload{"pitchspeed"} = py.float(double(payload.pitchspeed));
py_payload{"yawspeed"} = py.float(double(payload.yawspeed));
py_payload{"lat"} = py.int(int32(payload.lat));
py_payload{"lon"} = py.int(int32(payload.lon));
py_payload{"alt"} = py.int(int32(payload.alt));
py_payload{"vx"} = py.int(int16(payload.vx));
py_payload{"vy"} = py.int(int16(payload.vy));
py_payload{"vz"} = py.int(int16(payload.vz));
py_payload{"ind_airspeed"} = py.int(uint16(payload.ind_airspeed));
py_payload{"true_airspeed"} = py.int(uint16(payload.true_airspeed));
py_payload{"xacc"} = py.int(int16(payload.xacc));
py_payload{"yacc"} = py.int(int16(payload.yacc));
py_payload{"zacc"} = py.int(int16(payload.zacc));

py_bytes = bridge.encode_hil_state_quaternion(py_payload);
bytes = uint8(py.array.array("B", py_bytes));
bytes = bytes(:);
end
