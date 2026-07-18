function bytes = pymavlink_encode_hil_sensor(payload, cfg)
%PYMAVLINK_ENCODE_HIL_SENSOR Encode HIL_SENSOR via pymavlink.

persistent bridge
if isempty(bridge)
    bridge = mavlink_backend_init(cfg);
end

py_payload = py.dict();
names = ["xacc", "yacc", "zacc", "xgyro", "ygyro", "zgyro", ...
    "xmag", "ymag", "zmag", "abs_pressure", "diff_pressure", ...
    "pressure_alt", "temperature"];
py_payload{"time_usec"} = py.int(uint64(payload.time_usec));
for name = names
    py_payload{char(name)} = py.float(double(payload.(name)));
end
py_payload{"fields_updated"} = py.int(uint32(payload.fields_updated));
py_payload{"id"} = py.int(uint8(payload.id));

py_bytes = bridge.encode_hil_sensor(py_payload);
bytes = uint8(py.array.array("B", py_bytes));
bytes = bytes(:);
end
