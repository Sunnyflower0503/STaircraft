function bytes = pymavlink_encode_hil_bundle(sensor, state, cfg)
%PYMAVLINK_ENCODE_HIL_BUNDLE Encode sensor and state with one MAVLink sequence.

persistent bridge
if isempty(bridge)
    bridge = mavlink_backend_init(cfg);
end

py_sensor = py.dict();
sensor_float_names = ["xacc", "yacc", "zacc", "xgyro", "ygyro", "zgyro", ...
    "xmag", "ymag", "zmag", "abs_pressure", "diff_pressure", ...
    "pressure_alt", "temperature"];
py_sensor{"time_usec"} = py.int(uint64(sensor.time_usec));
for name = sensor_float_names
    py_sensor{char(name)} = py.float(double(sensor.(name)));
end
py_sensor{"fields_updated"} = py.int(uint32(sensor.fields_updated));
py_sensor{"id"} = py.int(uint8(sensor.id));

py_state = py.dict();
py_state{"time_usec"} = py.int(uint64(state.time_usec));
py_state{"attitude_quaternion"} = py.list(num2cell(double(state.attitude_quaternion(:).')));
state_float_names = ["rollspeed", "pitchspeed", "yawspeed"];
for name = state_float_names
    py_state{char(name)} = py.float(double(state.(name)));
end
state_int_names = ["lat", "lon", "alt", "vx", "vy", "vz", ...
    "ind_airspeed", "true_airspeed", "xacc", "yacc", "zacc"];
for name = state_int_names
    py_state{char(name)} = py.int(state.(name));
end

py_bytes = bridge.encode_hil_sensor_and_state(py_sensor, py_state);
bytes = uint8(py.array.array("B", py_bytes));
bytes = bytes(:);
end
