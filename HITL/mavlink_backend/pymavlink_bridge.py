from pymavlink.dialects.v20 import common as mavlink2


class _MavlinkOutput:
    def __init__(self):
        self.data = bytearray()

    def write(self, value):
        if isinstance(value, int):
            self.data.append(value & 0xFF)
        else:
            self.data.extend(value)

    def clear(self):
        self.data.clear()

    def bytes(self):
        return bytes(self.data)


class MavlinkBridge:
    def __init__(self, sysid=1, compid=1):
        self.output = _MavlinkOutput()
        self.encoder = mavlink2.MAVLink(self.output, srcSystem=int(sysid), srcComponent=int(compid))
        self.encoder.force_mavlink2 = True
        self.decoder = mavlink2.MAVLink(None)
        self.decoder.robust_parsing = True

    def decode_servo_output_raw(self, byte_values):
        latest = None
        for value in byte_values:
            msg = self.decoder.parse_char(bytes([int(value) & 0xFF]))
            if msg is None or msg.get_type() != "SERVO_OUTPUT_RAW":
                continue
            latest = {
                "is_new": True,
                "timestamp": int(getattr(msg, "time_usec", getattr(msg, "time_boot_ms", 0))),
                "servo1_raw": int(msg.servo1_raw),
                "servo2_raw": int(msg.servo2_raw),
                "servo3_raw": int(msg.servo3_raw),
                "servo4_raw": int(msg.servo4_raw),
                "servo5_raw": int(msg.servo5_raw),
                "servo6_raw": int(msg.servo6_raw),
                "servo7_raw": int(msg.servo7_raw),
                "servo8_raw": int(msg.servo8_raw),
            }
        return latest

    def encode_hil_state_quaternion(self, payload):
        q = list(payload["attitude_quaternion"])
        msg = mavlink2.MAVLink_hil_state_quaternion_message(
            int(payload["time_usec"]),
            [float(q[0]), float(q[1]), float(q[2]), float(q[3])],
            float(payload["rollspeed"]),
            float(payload["pitchspeed"]),
            float(payload["yawspeed"]),
            int(payload["lat"]),
            int(payload["lon"]),
            int(payload["alt"]),
            int(payload["vx"]),
            int(payload["vy"]),
            int(payload["vz"]),
            int(payload["ind_airspeed"]),
            int(payload["true_airspeed"]),
            int(payload["xacc"]),
            int(payload["yacc"]),
            int(payload["zacc"]),
        )
        self.output.clear()
        self.encoder.send(msg, force_mavlink1=False)
        return self.output.bytes()
