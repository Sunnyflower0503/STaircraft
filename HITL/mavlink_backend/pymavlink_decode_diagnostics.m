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
    "text", "");

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
end
