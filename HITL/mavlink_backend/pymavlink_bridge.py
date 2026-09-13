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
        self.diagnostic_decoder = mavlink2.MAVLink(None)
        self.diagnostic_decoder.robust_parsing = True

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

    def decode_diagnostics(self, byte_values):
        result = {
            "command_ack_new": False,
            "command": 0,
            "ack_result": 0,
            "statustext_new": False,
            "severity": 0,
            "text": "",
            "extended_sys_state_new": False,
            "vtol_state": 0,
            "landed_state": 0,
            "heartbeat_new": False,
            "base_mode": 0,
            "custom_mode": 0,
            "armed": False,
        }
        for value in byte_values:
            msg = self.diagnostic_decoder.parse_char(bytes([int(value) & 0xFF]))
            if msg is None:
                continue
            if msg.get_type() == "COMMAND_ACK":
                result["command_ack_new"] = True
                result["command"] = int(msg.command)
                result["ack_result"] = int(msg.result)
            elif msg.get_type() == "STATUSTEXT":
                result["statustext_new"] = True
                result["severity"] = int(msg.severity)
                text = msg.text
                if isinstance(text, bytes):
                    text = text.decode("utf-8", errors="replace")
                result["text"] = str(text).rstrip("\x00")
            elif msg.get_type() == "EXTENDED_SYS_STATE":
                result["extended_sys_state_new"] = True
                result["vtol_state"] = int(msg.vtol_state)
                result["landed_state"] = int(msg.landed_state)
            elif msg.get_type() == "HEARTBEAT":
                result["heartbeat_new"] = True
                result["base_mode"] = int(msg.base_mode)
                result["custom_mode"] = int(msg.custom_mode)
                result["armed"] = bool(msg.base_mode & mavlink2.MAV_MODE_FLAG_SAFETY_ARMED)
        return result

    def encode_hil_state_quaternion(self, payload, contact_mask=0):
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
        contact_msg = self.encoder.named_value_float_encode(
            int(payload["time_usec"] // 1000),
            b"TD_CNTCT",
            float(int(contact_mask) & 0x3F),
        )
        self.encoder.send(contact_msg, force_mavlink1=False)
        return self.output.bytes()

    def encode_hil_sensor(self, payload):
        msg = mavlink2.MAVLink_hil_sensor_message(
            int(payload["time_usec"]),
            float(payload["xacc"]),
            float(payload["yacc"]),
            float(payload["zacc"]),
            float(payload["xgyro"]),
            float(payload["ygyro"]),
            float(payload["zgyro"]),
            float(payload["xmag"]),
            float(payload["ymag"]),
            float(payload["zmag"]),
            float(payload["abs_pressure"]),
            float(payload["diff_pressure"]),
            float(payload["pressure_alt"]),
            float(payload["temperature"]),
            int(payload["fields_updated"]),
            int(payload.get("id", 0)),
        )
        self.output.clear()
        self.encoder.send(msg, force_mavlink1=False)
        return self.output.bytes()

    def encode_hil_sensor_and_state(self, sensor, state, contact_mask=0):
        sensor_msg = mavlink2.MAVLink_hil_sensor_message(
            int(sensor["time_usec"]),
            float(sensor["xacc"]), float(sensor["yacc"]), float(sensor["zacc"]),
            float(sensor["xgyro"]), float(sensor["ygyro"]), float(sensor["zgyro"]),
            float(sensor["xmag"]), float(sensor["ymag"]), float(sensor["zmag"]),
            float(sensor["abs_pressure"]), float(sensor["diff_pressure"]),
            float(sensor["pressure_alt"]), float(sensor["temperature"]),
            int(sensor["fields_updated"]), int(sensor.get("id", 0)),
        )
        q = list(state["attitude_quaternion"])
        state_msg = mavlink2.MAVLink_hil_state_quaternion_message(
            int(state["time_usec"]),
            [float(q[0]), float(q[1]), float(q[2]), float(q[3])],
            float(state["rollspeed"]), float(state["pitchspeed"]),
            float(state["yawspeed"]), int(state["lat"]), int(state["lon"]),
            int(state["alt"]), int(state["vx"]), int(state["vy"]), int(state["vz"]),
            int(state["ind_airspeed"]), int(state["true_airspeed"]),
            int(state["xacc"]), int(state["yacc"]), int(state["zacc"]),
        )
        self.output.clear()
        self.encoder.send(sensor_msg, force_mavlink1=False)
        self.encoder.send(state_msg, force_mavlink1=False)
        contact_msg = self.encoder.named_value_float_encode(
            int(state["time_usec"] // 1000),
            b"TD_CNTCT",
            float(int(contact_mask) & 0x3F),
        )
        self.encoder.send(contact_msg, force_mavlink1=False)
        return self.output.bytes()

    def encode_manual_control(self, target, x, y, z, r, buttons=0):
        msg = self.encoder.manual_control_encode(
            int(target), int(x), int(y), int(z), int(r), int(buttons)
        )
        self.output.clear()
        self.encoder.send(msg, force_mavlink1=False)
        return self.output.bytes()

    def encode_gcs_heartbeat(self):
        msg = self.encoder.heartbeat_encode(
            mavlink2.MAV_TYPE_GCS,
            mavlink2.MAV_AUTOPILOT_INVALID,
            0,
            0,
            mavlink2.MAV_STATE_ACTIVE,
        )
        self.output.clear()
        self.encoder.send(msg, force_mavlink1=False)
        return self.output.bytes()

    def encode_command_long(self, target_system, target_component, command,
                            param1=0.0, param2=0.0, param3=0.0, param4=0.0,
                            param5=0.0, param6=0.0, param7=0.0):
        msg = self.encoder.command_long_encode(
            int(target_system), int(target_component), int(command), 0,
            float(param1), float(param2), float(param3), float(param4),
            float(param5), float(param6), float(param7),
        )
        self.output.clear()
        self.encoder.send(msg, force_mavlink1=False)
        return self.output.bytes()
