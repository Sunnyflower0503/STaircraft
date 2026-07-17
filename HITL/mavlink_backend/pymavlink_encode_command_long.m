function bytes = pymavlink_encode_command_long(target_system, target_component, command, params, cfg)
%PYMAVLINK_ENCODE_COMMAND_LONG Encode one MAVLink COMMAND_LONG packet.

persistent bridge
if isempty(bridge)
    bridge = mavlink_backend_init(cfg);
end

params = double(params(:));
if numel(params) ~= 7
    error("pymavlink_encode_command_long:BadParams", "params must contain seven values.");
end

py_bytes = bridge.encode_command_long( ...
    py.int(target_system), py.int(target_component), py.int(command), ...
    py.float(params(1)), py.float(params(2)), py.float(params(3)), ...
    py.float(params(4)), py.float(params(5)), py.float(params(6)), py.float(params(7)));
bytes = uint8(py.array.array("B", py_bytes));
bytes = bytes(:);
end
