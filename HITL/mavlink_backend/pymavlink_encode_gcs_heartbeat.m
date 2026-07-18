function bytes = pymavlink_encode_gcs_heartbeat(cfg)
%PYMAVLINK_ENCODE_GCS_HEARTBEAT Encode a standard active GCS heartbeat.

persistent bridge
if isempty(bridge)
    bridge = mavlink_backend_init(cfg);
end

py_bytes = bridge.encode_gcs_heartbeat();
bytes = uint8(py.array.array("B", py_bytes));
bytes = bytes(:);
end
