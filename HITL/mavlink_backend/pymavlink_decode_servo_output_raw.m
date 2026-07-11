function servo_msg = pymavlink_decode_servo_output_raw(bytes, cfg)
%PYMAVLINK_DECODE_SERVO_OUTPUT_RAW Decode SERVO_OUTPUT_RAW using pymavlink.

persistent bridge
if isempty(bridge)
    bridge = mavlink_backend_init(cfg);
end

servo_msg = default_servo_msg(false);
bytes = uint8(bytes(:));
if isempty(bytes)
    return;
end

py_values = py.list(num2cell(double(bytes.')));
result = bridge.decode_servo_output_raw(py_values);
if isa(result, "py.NoneType")
    return;
end

servo_msg.is_new = true;
servo_msg.timestamp = double(result{"timestamp"});
for k = 1:8
    name = sprintf("servo%d_raw", k);
    servo_msg.(name) = uint16(result{name});
end
end

function msg = default_servo_msg(is_new)
msg = struct();
msg.is_new = is_new;
msg.timestamp = [];
for k = 1:8
    msg.(sprintf("servo%d_raw", k)) = uint16(0);
end
end
