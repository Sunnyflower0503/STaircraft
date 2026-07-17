"""Read a focused PX4 rotor-control parameter snapshot over MAVLink.

This utility is deliberately read-only: it sends PARAM_REQUEST_READ messages
and never sends PARAM_SET, arm/disarm, mode, or actuator commands.
"""

from __future__ import annotations

import argparse
import json
import struct
import time

from pymavlink import mavutil


ROTOR_PARAMETERS = [
    "SYS_AUTOSTART",
    "VT_TYPE",
    "COM_VEHICLE_ID",
    "TD_MC_DIRECT_EN",
    "TD_GND_I_LOCK",
    "TD_GND_THR_REL",
    "TD_TIP_IDLE_PWM",
    "TD_TIP_P_FF",
    "TD_MC_HOV_P",
    "TD_MC_DBG_PITCH",
    "TD_MC_DBG_ROLL",
    "TD_MC_DBG_SPIN",
    "TD_MC_YAW_MAIN",
    "TD_TIP_YAW_REV",
    "MC_ROLL_P",
    "MC_PITCH_P",
    "MC_YAW_P",
    "MC_YAW_WEIGHT",
    "MC_ROLLRATE_MAX",
    "MC_PITCHRATE_MAX",
    "MC_YAWRATE_MAX",
    "MC_ROLLRATE_P",
    "MC_ROLLRATE_I",
    "MC_ROLLRATE_D",
    "MC_ROLLRATE_FF",
    "MC_ROLLRATE_K",
    "MC_PITCHRATE_P",
    "MC_PITCHRATE_I",
    "MC_PITCHRATE_D",
    "MC_PITCHRATE_FF",
    "MC_PITCHRATE_K",
    "MC_YAWRATE_P",
    "MC_YAWRATE_I",
    "MC_YAWRATE_D",
    "MC_YAWRATE_FF",
    "MC_YAWRATE_K",
    "MPC_THR_HOVER",
]


def decode_param_id(value: object) -> str:
    if isinstance(value, bytes):
        return value.split(b"\0", 1)[0].decode("ascii", errors="replace")
    return str(value).split("\0", 1)[0]


def decode_param_value(wire_value: float, param_type: int) -> float | int:
    """Decode PX4 byte-wise MAVLink parameter encoding."""
    raw = struct.pack("<f", float(wire_value))
    formats = {
        mavutil.mavlink.MAV_PARAM_TYPE_UINT8: "<B",
        mavutil.mavlink.MAV_PARAM_TYPE_INT8: "<b",
        mavutil.mavlink.MAV_PARAM_TYPE_UINT16: "<H",
        mavutil.mavlink.MAV_PARAM_TYPE_INT16: "<h",
        mavutil.mavlink.MAV_PARAM_TYPE_UINT32: "<I",
        mavutil.mavlink.MAV_PARAM_TYPE_INT32: "<i",
    }
    if param_type in formats:
        size = struct.calcsize(formats[param_type])
        return int(struct.unpack(formats[param_type], raw[:size])[0])
    return float(wire_value)


def request_parameters(link: mavutil.mavfile, names: list[str]) -> None:
    for name in names:
        link.mav.param_request_read_send(
            link.target_system,
            link.target_component,
            name.encode("ascii"),
            -1,
        )
        time.sleep(0.015)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="COM9")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--heartbeat-timeout", type=float, default=10.0)
    parser.add_argument("--response-timeout", type=float, default=8.0)
    args = parser.parse_args()

    link = mavutil.mavlink_connection(
        args.port,
        baud=args.baud,
        source_system=255,
        source_component=190,
        autoreconnect=False,
    )

    try:
        heartbeat = link.wait_heartbeat(timeout=args.heartbeat_timeout)
        if heartbeat is None:
            raise TimeoutError("No MAVLink HEARTBEAT received")

        heartbeat_data = {
            "source_system": heartbeat.get_srcSystem(),
            "source_component": heartbeat.get_srcComponent(),
            "type": int(heartbeat.type),
            "autopilot": int(heartbeat.autopilot),
            "base_mode": int(heartbeat.base_mode),
            "custom_mode": int(heartbeat.custom_mode),
            "system_status": int(heartbeat.system_status),
            "mavlink_version": int(heartbeat.mavlink_version),
        }

        values: dict[str, dict[str, float | int]] = {}
        missing = set(ROTOR_PARAMETERS)
        deadline = time.monotonic() + args.response_timeout
        retry_at = time.monotonic() + args.response_timeout / 2.0
        request_parameters(link, ROTOR_PARAMETERS)

        while missing and time.monotonic() < deadline:
            msg = link.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.25)
            if msg is not None:
                name = decode_param_id(msg.param_id)
                if name in missing:
                    values[name] = {
                        "value": decode_param_value(float(msg.param_value), int(msg.param_type)),
                        "type": int(msg.param_type),
                    }
                    missing.remove(name)

            if missing and time.monotonic() >= retry_at:
                request_parameters(link, sorted(missing))
                retry_at = float("inf")

        result = {
            "port": args.port,
            "baud": args.baud,
            "heartbeat": heartbeat_data,
            "parameters": {name: values[name] for name in ROTOR_PARAMETERS if name in values},
            "missing": sorted(missing),
        }
        print(json.dumps(result, ensure_ascii=False, indent=2, sort_keys=False))
        return 0 if not missing else 2
    finally:
        link.close()


if __name__ == "__main__":
    raise SystemExit(main())
