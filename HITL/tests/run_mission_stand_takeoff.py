"""Preselect fixed-wing, then run a stand-launch mission over USB MAVLink."""

from __future__ import annotations

import argparse
import math
import struct
import time
from pathlib import Path

from pymavlink import mavutil


def upload_mission(master, items):
    master.mav.mission_clear_all_send(master.target_system, master.target_component)
    clear_ack = master.recv_match(type="MISSION_ACK", blocking=True, timeout=5)
    if clear_ack is None or clear_ack.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
        raise RuntimeError("PX4 did not acknowledge mission clear")
    master.mav.mission_count_send(
        master.target_system,
        master.target_component,
        len(items),
        mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
    )
    sent = set()
    deadline = time.time() + 15
    while time.time() < deadline:
        msg = master.recv_match(
            type=["MISSION_REQUEST", "MISSION_REQUEST_INT", "MISSION_ACK"],
            blocking=True,
            timeout=1,
        )
        if msg is None:
            continue
        if msg.get_type() == "MISSION_ACK":
            if msg.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
                raise RuntimeError(f"Mission rejected with ACK type {msg.type}")
            return
        seq = int(msg.seq)
        if not 0 <= seq < len(items):
            raise RuntimeError(f"PX4 requested invalid mission sequence {seq}")
        item = items[seq]
        master.mav.mission_item_int_send(
            master.target_system,
            master.target_component,
            seq,
            item["frame"],
            item["command"],
            0,
            1,
            item.get("param1", 0),
            item.get("param2", 0),
            item.get("param3", 0),
            item.get("param4", math.nan),
            item.get("x", 0),
            item.get("y", 0),
            item.get("z", 0),
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
        )
        sent.add(seq)
    raise RuntimeError(f"Mission upload timed out; requested sequences={sorted(sent)}")


def wait_command_ack(master, command, timeout=5):
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = master.recv_match(type="COMMAND_ACK", blocking=True, timeout=0.5)
        if msg is not None and int(msg.command) == int(command):
            return int(msg.result)
    raise RuntimeError(f"No COMMAND_ACK for command {command}")


def force_disarm(master):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0,
        21196,
        0,
        0,
        0,
        0,
        0,
    )


def set_int_param(master, name, value):
    raw_float = struct.unpack("<f", struct.pack("<i", int(value)))[0]
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        name.encode(),
        raw_float,
        mavutil.mavlink.MAV_PARAM_TYPE_INT32,
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="COM5")
    parser.add_argument("--duration", type=float, default=31.0)
    parser.add_argument("--takeoff-alt", type=float, default=30.0)
    parser.add_argument("--takeoff-distance", type=float, default=150.0)
    parser.add_argument("--waypoint-distance", type=float, default=300.0)
    args = parser.parse_args()

    runtime_file = Path(__file__).resolve().parents[1] / "runtime_control.txt"
    master = mavutil.mavlink_connection(args.port, baud=115200, source_system=250)
    heartbeat = master.wait_heartbeat(timeout=15)
    if heartbeat is None:
        raise RuntimeError(f"No PX4 heartbeat on {args.port}")

    try:
        set_int_param(master, "COM_RC_IN_MODE", 4)
        time.sleep(0.5)
        global_position = None
        position_warmup_deadline = time.time() + 4.0
        while time.time() < position_warmup_deadline:
            candidate = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=0.5)
            if candidate is not None:
                global_position = candidate
        if global_position is None:
            raise RuntimeError("No GLOBAL_POSITION_INT after the HITL model started")

        latitude = int(global_position.lat)
        longitude = int(global_position.lon)
        current_alt_m = global_position.alt * 1e-3
        takeoff_alt_m = current_alt_m + args.takeoff_alt
        latitude_units_per_metre = 1e7 / 111320.0
        takeoff_latitude = latitude + round(args.takeoff_distance * latitude_units_per_metre)
        waypoint_latitude = latitude + round(args.waypoint_distance * latitude_units_per_metre)

        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_HOME,
            0,
            1,
            0,
            0,
            0,
            0,
            0,
            0,
        )
        set_home_result = wait_command_ack(master, mavutil.mavlink.MAV_CMD_DO_SET_HOME)
        if set_home_result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
            raise RuntimeError(f"PX4 rejected set-current-position home with result {set_home_result}")

        # Match the validated manual sequence: select fixed-wing while still
        # disarmed so the ground-support latch cannot be released by transition
        # thrust. Mission control starts with NAV_TAKEOFF after this preflight step.
        force_disarm(master)
        time.sleep(0.3)
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_DO_VTOL_TRANSITION,
            0,
            mavutil.mavlink.MAV_VTOL_STATE_FW,
            0,
            0,
            0,
            0,
            0,
            0,
        )
        transition_result = wait_command_ack(
            master, mavutil.mavlink.MAV_CMD_DO_VTOL_TRANSITION
        )
        if transition_result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
            raise RuntimeError(f"PX4 rejected disarmed FW selection with result {transition_result}")

        transition_deadline = time.time() + 8
        while time.time() < transition_deadline:
            extended = master.recv_match(type="EXTENDED_SYS_STATE", blocking=True, timeout=0.5)
            if extended is not None and extended.vtol_state == mavutil.mavlink.MAV_VTOL_STATE_FW:
                break
        else:
            raise RuntimeError("PX4 did not reach fixed-wing state before mission start")
        time.sleep(2.0)

        items = [
            {
                "frame": mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                "command": mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                "param1": 10.0,
                "param4": 0.0,
                "x": takeoff_latitude,
                "y": longitude,
                "z": takeoff_alt_m,
            },
            {
                "frame": mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                "command": mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                "param2": 8.0,
                "x": waypoint_latitude,
                "y": longitude,
                "z": takeoff_alt_m,
            },
        ]
        upload_mission(master, items)
        print(
            f"Fixed-wing selected while disarmed; mission uploaded: takeoff {args.takeoff_alt:.1f} m "
            f"at {args.takeoff_distance:.0f} m north -> {args.waypoint_distance:.0f} m north waypoint; "
            f"start alt={current_alt_m:.1f} AMSL"
        )

        current_heartbeat = master.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
        already_in_mission = (
            current_heartbeat is not None
            and mavutil.mode_string_v10(current_heartbeat) == "MISSION"
        )
        mode_result = mavutil.mavlink.MAV_RESULT_ACCEPTED if already_in_mission else None
        if not already_in_mission:
            for _ in range(5):
                master.set_mode("MISSION")
                mode_result = wait_command_ack(master, mavutil.mavlink.MAV_CMD_DO_SET_MODE)
                if mode_result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    break
                current_heartbeat = master.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
                if current_heartbeat is not None and mavutil.mode_string_v10(current_heartbeat) == "MISSION":
                    mode_result = mavutil.mavlink.MAV_RESULT_ACCEPTED
                    break
                time.sleep(1.0)
        if mode_result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
            raise RuntimeError(f"PX4 rejected AUTO.MISSION mode with result {mode_result}")
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,
            21196,
            0,
            0,
            0,
            0,
            0,
        )
        arm_result = wait_command_ack(master, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM)
        if arm_result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
            raise RuntimeError(f"PX4 rejected arming with result {arm_result}")
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_MISSION_START,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        )
        mission_start_result = wait_command_ack(master, mavutil.mavlink.MAV_CMD_MISSION_START)
        if mission_start_result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
            raise RuntimeError(f"PX4 rejected mission start with result {mission_start_result}")

        started = time.monotonic()
        last_print = -1.0
        state = {"mission": -1, "vtol": -1, "landed": -1, "armed": False}
        servo = [math.nan] * 8
        position = [math.nan] * 5
        while time.monotonic() - started < args.duration:
            msg = master.recv_match(
                type=[
                    "HEARTBEAT",
                    "EXTENDED_SYS_STATE",
                    "MISSION_CURRENT",
                    "SERVO_OUTPUT_RAW",
                    "GLOBAL_POSITION_INT",
                    "STATUSTEXT",
                    "COMMAND_ACK",
                ],
                blocking=True,
                timeout=0.2,
            )
            if msg is not None:
                msg_type = msg.get_type()
                if msg_type == "HEARTBEAT":
                    state["armed"] = bool(
                        msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                    )
                elif msg_type == "EXTENDED_SYS_STATE":
                    state["vtol"] = int(msg.vtol_state)
                    state["landed"] = int(msg.landed_state)
                elif msg_type == "MISSION_CURRENT":
                    state["mission"] = int(msg.seq)
                elif msg_type == "SERVO_OUTPUT_RAW":
                    servo = [float(getattr(msg, f"servo{i}_raw")) for i in range(1, 9)]
                elif msg_type == "GLOBAL_POSITION_INT":
                    position = [
                        msg.alt * 1e-3,
                        msg.relative_alt * 1e-3,
                        msg.vx * 1e-2,
                        msg.vy * 1e-2,
                        msg.vz * 1e-2,
                    ]
                elif msg_type == "STATUSTEXT":
                    print(f"STATUSTEXT[{msg.severity}]: {msg.text}")
                elif msg_type == "COMMAND_ACK":
                    print(f"COMMAND_ACK command={msg.command} result={msg.result}")

            elapsed = time.monotonic() - started
            if elapsed - last_print >= 1.0:
                throttle = sum(servo[:4]) / 4 if not math.isnan(servo[0]) else math.nan
                print(
                    f"t={elapsed:5.1f} armed={int(state['armed'])} mission={state['mission']} "
                    f"vtol={state['vtol']} landed={state['landed']} main_pwm={throttle:.0f} "
                    f"amsl={position[0]:.1f} rel_alt={position[1]:+.1f} "
                    f"vel_ned=[{position[2]:+.1f} {position[3]:+.1f} {position[4]:+.1f}]"
                )
                last_print = elapsed
    finally:
        runtime_file.write_text("force_enable=0\n", encoding="utf-8")
        force_disarm(master)
        master.set_mode("LOITER")
        set_int_param(master, "COM_RC_IN_MODE", 0)
        time.sleep(0.5)
        master.close()
        print("Model frozen, PX4 force-disarmed, COM_RC_IN_MODE restored to 0")


if __name__ == "__main__":
    main()
