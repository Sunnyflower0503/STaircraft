function serial_write_bytes(ser, bytes)
%SERIAL_WRITE_BYTES Write raw MAVLink bytes without header or terminator.
if isempty(bytes)
    return;
end
write(ser, uint8(bytes(:)), "uint8");
end
