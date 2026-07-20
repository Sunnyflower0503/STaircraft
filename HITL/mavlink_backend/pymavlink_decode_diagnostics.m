function diagnostics = pymavlink_decode_diagnostics(bytes, cfg)
%PYMAVLINK_DECODE_DIAGNOSTICS Decode latest COMMAND_ACK and STATUSTEXT.

persistent bridge
if isempty(bridge)
    bridge = mavlink_backend_init(cfg);
end

diagnostics = struct( ...
    "command_ack_new", false, ...
    "command", 0, ...
    "ack_result", 0, ...
    "statustext_new", false, ...
    "severity", 0, ...
    "text", "", ...
    "extended_sys_state_new", false, ...
    "vtol_state", 0, ...
    "landed_state", 0, ...
    "heartbeat_new", false, ...
    "base_mode", 0, ...
    "custom_mode", 0, ...
    "armed", false);

bytes = uint8(bytes(:));
if isempty(bytes)
    return;
end

py_values = py.list(num2cell(double(bytes.')));
result = bridge.decode_diagnostics(py_values);
diagnostics.command_ack_new = logical(result{"command_ack_new"});
diagnostics.command = double(result{"command"});
diagnostics.ack_result = double(result{"ack_result"});
diagnostics.statustext_new = logical(result{"statustext_new"});
diagnostics.severity = double(result{"severity"});
diagnostics.text = string(result{"text"});
diagnostics.extended_sys_state_new = logical(result{"extended_sys_state_new"});
diagnostics.vtol_state = double(result{"vtol_state"});
diagnostics.landed_state = double(result{"landed_state"});
diagnostics.heartbeat_new = logical(result{"heartbeat_new"});
diagnostics.base_mode = double(result{"base_mode"});
diagnostics.custom_mode = double(result{"custom_mode"});
diagnostics.armed = logical(result{"armed"});
end
