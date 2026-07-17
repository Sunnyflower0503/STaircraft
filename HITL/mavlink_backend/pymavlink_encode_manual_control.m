function bytes = pymavlink_encode_manual_control(target, x, y, z, r, buttons, cfg)
%PYMAVLINK_ENCODE_MANUAL_CONTROL Encode one MAVLink MANUAL_CONTROL packet.

persistent bridge
if isempty(bridge)
    bridge = mavlink_backend_init(cfg);
end

py_bytes = bridge.encode_manual_control( ...
    py.int(target), py.int(x), py.int(y), py.int(z), py.int(r), py.int(buttons));
bytes = uint8(py.array.array("B", py_bytes));
bytes = bytes(:);
end
