function bytes = serial_read_bytes(ser)
%SERIAL_READ_BYTES Read currently available uint8 bytes without framing.
n = ser.NumBytesAvailable;
if n <= 0
    bytes = zeros(0, 1, "uint8");
    return;
end
bytes = read(ser, n, "uint8");
bytes = uint8(bytes(:));
end
