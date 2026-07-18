"""Preselect fixed-wing, then run a stand-launch mission over USB MAVLink."""

from __future__ import annotations

import argparse
import math
import struct
import time
from pathlib import Path

from pymavlink import mavutil


def send_gcs_heartbeat(master):
    master.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0,
        0,
        mavutil.mavlink.MAV_STATE_ACTIVE,
    )


def quaternion_to_euler_deg(q):
    w, x, y, z = (float(value) for value in q)
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return [math.degrees(roll), math.degrees(pitch), math.degrees(yaw)]


def upload_mission(master, items):
    clear_ack = None
    for _ in range(3):
        master.mav.mission_clear_all_send(master.target_system, master.target_component)
        clear_ack = master.recv_match(type="MISSION_ACK", blocking=True, timeout=1)
        if clear_ack is not None:
            break
    if clear_ack is None:
        # MISSION_COUNT replaces the stored mission as well. Some MAVLink
        # links drop the standalone clear acknowledgement immediately after a
        # reboot, so continue with the transactional upload protocol.
        print("Warning: no MISSION_CLEAR_ALL ACK; replacing mission via MISSION_COUNT")
    elif clear_ack.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
        raise RuntimeError(f"Mission clear rejected with ACK type {clear_ack.type}")
    master.mav.mission_count_send(
        master.target_system,
        master.target_component,
        len(items),
        mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
    )
    sent = set()
    deadline = time.time() + 15
    while time.time() < deadline:
        send_gcs_heartbeat(master)
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
            if len(sent) == len(items):
                return
            # Ignore a delayed MISSION_CLEAR_ALL acknowledgement.
            continue
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
        send_gcs_heartbeat(master)
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


def set_int_param_and_wait(master, name, value, timeout=5):
    set_int_param(master, name, value)
    deadline = time.time() + timeout
    while time.time() < deadline:
        send_gcs_heartbeat(master)
        msg = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
        if msg is not None and msg.param_id.rstrip("\x00") == name:
            return
    raise RuntimeError(f"No PARAM_VALUE confirmation for {name}")


def set_float_param_and_wait(master, name, value, timeout=5):
    deadline = time.time() + timeout
    next_send = 0.0
    last_value = None
    while time.time() < deadline:
        if time.time() >= next_send:
            master.mav.param_set_send(
                master.target_system,
                master.target_component,
                name.encode(),
                float(value),
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
            )
            next_send = time.time() + 0.5
        send_gcs_heartbeat(master)
        msg = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
        if msg is not None and msg.param_id.rstrip("\x00") == name:
            last_value = float(msg.param_value)
            if abs(last_value - float(value)) <= 0.05:
                return
    raise RuntimeError(f"No matching PARAM_VALUE for {name}; last readback={last_value}")


def send_shell_command(master, command):
    data = list((command + "\n").encode())
    flags = (
        mavutil.mavlink.SERIAL_CONTROL_FLAG_EXCLUSIVE
        | mavutil.mavlink.SERIAL_CONTROL_FLAG_RESPOND
    )
    for offset in range(0, len(data), 70):
        chunk = data[offset : offset + 70]
        master.mav.serial_control_send(
            mavutil.mavlink.SERIAL_CONTROL_DEV_SHELL,
            flags,
            0,
            0,
            len(chunk),
            chunk + [0] * (70 - len(chunk)),
        )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="COM5")
    parser.add_argument("--duration", type=float, default=31.0)
    parser.add_argument("--takeoff-alt", type=float, default=30.0)
    parser.add_argument("--takeoff-distance", type=float, default=150.0)
    parser.add_argument("--waypoint-distance", type=float, default=300.0)
    parser.add_argument("--waypoint2-distance", type=float, default=420.0)
    parser.add_argument("--descent-distance", type=float, default=520.0)
    parser.add_argument("--landing-distance", type=float, default=570.0)
    parser.add_argument("--transition-alt", type=float, default=15.0)
    parser.add_argument("--fw-wp-acceptance", type=float, default=30.0)
    parser.add_argument("--approach-airspeed", type=float, default=11.5)
    parser.add_argument("--back-transition-airspeed", type=float, default=13.5)
    parser.add_argument("--back-transition-gate-time", type=float, default=0.5)
    parser.add_argument("--back-transition-throttle", type=float, default=0.40)
    parser.add_argument("--mc-waypoint-acceptance", type=float, default=10.0)
    parser.add_argument(
        "--full-landing",
        action="store_true",
        help="Fly two fixed-wing waypoints, descend, transition to MC, and perform a position landing",
    )
    parser.add_argument(
        "--fw-only",
        action="store_true",
        help="Validate two fixed-wing waypoints and a final straight segment without VTOL transition",
    )
    parser.add_argument(
        "--transition-to-mc-at",
        type=float,
        default=-1.0,
        help="Request a fixed-wing to multicopter transition this many seconds after Mission start",
    )
    args = parser.parse_args()

    runtime_file = Path(__file__).resolve().parents[1] / "runtime_control.txt"
    master = mavutil.mavlink_connection(args.port, baud=115200, source_system=250)
    heartbeat = master.wait_heartbeat(timeout=15)
    if heartbeat is None:
        raise RuntimeError(f"No PX4 heartbeat on {args.port}")
    send_gcs_heartbeat(master)

    try:
        runtime_file.write_text("force_enable=0\n", encoding="utf-8")
        set_int_param_and_wait(master, "COM_RC_IN_MODE", 4)
        set_int_param_and_wait(master, "TD_FW_TKO_EN", 0)
        set_float_param_and_wait(master, "TD_FW_WP_ACC", args.fw_wp_acceptance)
        set_float_param_and_wait(master, "TD_BTR_ARSP", args.back_transition_airspeed)
        set_float_param_and_wait(master, "TD_BTR_GATE_T", args.back_transition_gate_time)
        set_float_param_and_wait(master, "TD_BTR_THR", args.back_transition_throttle)
        time.sleep(0.5)
        global_position = None
        position_warmup_deadline = time.time() + 4.0
        while time.time() < position_warmup_deadline:
            send_gcs_heartbeat(master)
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
        waypoint2_latitude = latitude + round(args.waypoint2_distance * latitude_units_per_metre)
        descent_latitude = latitude + round(args.descent_distance * latitude_units_per_metre)
        landing_latitude = latitude + round(args.landing_distance * latitude_units_per_metre)

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
            send_gcs_heartbeat(master)
            extended = master.recv_match(type="EXTENDED_SYS_STATE", blocking=True, timeout=0.5)
            if extended is not None and extended.vtol_state == mavutil.mavlink.MAV_VTOL_STATE_FW:
                break
        else:
            raise RuntimeError("PX4 did not reach fixed-wing state before mission start")
        for _ in range(4):
            send_gcs_heartbeat(master)
            time.sleep(0.5)

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
        if args.full_landing or args.fw_only:
            transition_alt_m = current_alt_m + (
                args.transition_alt if args.full_landing else args.takeoff_alt
            )
            items.append(
                {
                    "frame": mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                    "command": mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                    "param2": 8.0,
                    "x": waypoint2_latitude,
                    "y": longitude,
                    "z": takeoff_alt_m,
                }
            )
            if args.full_landing:
                items.append(
                    {
                        "frame": mavutil.mavlink.MAV_FRAME_MISSION,
                        "command": mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
                        "param1": 0.0,
                        "param2": args.approach_airspeed,
                        "param3": -1.0,
                    }
                )
            items.append(
                {
                    "frame": mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                    "command": mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                    "param2": 12.0,
                    "x": descent_latitude,
                    "y": longitude,
                    "z": transition_alt_m,
                }
            )
        if args.full_landing:
            items.extend(
                [
                    {
                        "frame": mavutil.mavlink.MAV_FRAME_MISSION,
                        "command": mavutil.mavlink.MAV_CMD_DO_VTOL_TRANSITION,
                        "param1": mavutil.mavlink.MAV_VTOL_STATE_MC,
                    },
                    {
                        "frame": mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                        "command": mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                        "param1": 2.0,
                        "param2": args.mc_waypoint_acceptance,
                        "x": landing_latitude,
                        "y": longitude,
                        "z": transition_alt_m,
                    },
                    {
                        "frame": mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                        "command": mavutil.mavlink.MAV_CMD_NAV_LAND,
                        "x": landing_latitude,
                        "y": longitude,
                        "z": current_alt_m,
                    },
                ]
            )
        upload_mission(master, items)
        if args.full_landing:
            print(
                f"Full mission uploaded: takeoff {args.takeoff_distance:.0f} m -> "
                f"WP1 {args.waypoint_distance:.0f} m -> WP2 {args.waypoint2_distance:.0f} m "
                f"at +{args.takeoff_alt:.1f} m -> descend near {args.descent_distance:.0f} m "
                f"to +{args.transition_alt:.1f} m -> MC transition near the landing point -> land at "
                f"{args.landing_distance:.0f} m north"
            )
        elif args.fw_only:
            print(
                f"Fixed-wing validation mission uploaded: takeoff {args.takeoff_distance:.0f} m -> "
                f"WP1 {args.waypoint_distance:.0f} m -> WP2 {args.waypoint2_distance:.0f} m -> "
                f"final segment {args.descent_distance:.0f} m at +{args.takeoff_alt:.1f} m"
            )
        else:
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
        if mission_start_result in (
            mavutil.mavlink.MAV_RESULT_TEMPORARILY_REJECTED,
            mavutil.mavlink.MAV_RESULT_DENIED,
        ):
            # AUTO.MISSION was selected before arming, so Navigator may already
            # be running item 0 and reject the redundant MISSION_START command.
            # The loop below verifies actual item progression, so accepting this
            # response cannot create a false-positive flight result.
            print(
                f"MISSION_START result {mission_start_result} was redundant; "
                "armed AUTO.MISSION will be verified from mission progress"
            )
        elif mission_start_result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
            raise RuntimeError(f"PX4 rejected mission start with result {mission_start_result}")

        # This is the explicit Mission stand-launch key. Firmware holds idle
        # for two continuous seconds after the 0->1 parameter change.
        launch_switch_enabled_at = time.monotonic()
        set_int_param_and_wait(master, "TD_FW_TKO_EN", 1)

        # Enable plant integration only after PX4 has accepted the mission.
        # The stand-release state machine still holds the aircraft until launch thrust.
        runtime_file.write_text("force_enable=1\n", encoding="utf-8")

        started = time.monotonic()
        last_print = -1.0
        state = {"mission": -1, "vtol": -1, "landed": -1, "armed": False}
        servo = [math.nan] * 8
        position = [math.nan] * 7
        attitude = [math.nan] * 3
        attitude_target = [math.nan] * 3
        navigation = [math.nan] * 3
        airspeed_m_s = math.nan
        shell_requested = False
        tecs_requested = False
        mc_shell_requested = False
        transition_requested = False
        transition_reached = False
        landing_reached = False
        fw_waypoints_reached = False
        fw_waypoints_reached_at = None
        launch_gate_violation = False
        first_launch_output_s = None
        last_gcs_heartbeat = -1.0
        while time.monotonic() - started < args.duration:
            msg = master.recv_match(
                type=[
                    "HEARTBEAT",
                    "EXTENDED_SYS_STATE",
                    "MISSION_CURRENT",
                    "SERVO_OUTPUT_RAW",
                    "GLOBAL_POSITION_INT",
                    "ATTITUDE",
                    "ATTITUDE_TARGET",
                    "NAV_CONTROLLER_OUTPUT",
                    "VFR_HUD",
                    "STATUSTEXT",
                    "COMMAND_ACK",
                    "SERIAL_CONTROL",
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
                        int(msg.lat),
                        int(msg.lon),
                    ]
                elif msg_type == "ATTITUDE":
                    attitude = [math.degrees(msg.roll), math.degrees(msg.pitch), math.degrees(msg.yaw)]
                elif msg_type == "ATTITUDE_TARGET":
                    attitude_target = quaternion_to_euler_deg(msg.q)
                elif msg_type == "NAV_CONTROLLER_OUTPUT":
                    navigation = [float(msg.nav_roll), float(msg.nav_bearing), float(msg.xtrack_error)]
                elif msg_type == "VFR_HUD":
                    airspeed_m_s = float(msg.airspeed)
                elif msg_type == "STATUSTEXT":
                    print(f"STATUSTEXT[{msg.severity}]: {msg.text}")
                elif msg_type == "COMMAND_ACK":
                    print(f"COMMAND_ACK command={msg.command} result={msg.result}")
                elif msg_type == "SERIAL_CONTROL" and msg.count:
                    shell_text = bytes(msg.data[: msg.count]).decode(errors="replace")
                    print(shell_text, end="")

            elapsed = time.monotonic() - started
            if elapsed - last_gcs_heartbeat >= 0.5:
                send_gcs_heartbeat(master)
                last_gcs_heartbeat = elapsed

            if (
                args.transition_to_mc_at >= 0
                and elapsed >= args.transition_to_mc_at
                and not transition_requested
            ):
                master.mav.command_long_send(
                    master.target_system,
                    master.target_component,
                    mavutil.mavlink.MAV_CMD_DO_VTOL_TRANSITION,
                    0,
                    mavutil.mavlink.MAV_VTOL_STATE_MC,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                )
                transition_requested = True
                print(f"Requested FW->MC transition at t={elapsed:.2f}s")

            if (
                (transition_requested or args.full_landing)
                and state["vtol"] == mavutil.mavlink.MAV_VTOL_STATE_MC
            ):
                transition_reached = True

            if (
                args.full_landing
                and transition_reached
                and state["landed"] == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND
            ):
                landing_reached = True

            if args.fw_only and state["mission"] >= 3:
                if fw_waypoints_reached_at is None:
                    fw_waypoints_reached_at = elapsed
                fw_waypoints_reached = elapsed - fw_waypoints_reached_at >= 5.0

            launch_switch_elapsed = time.monotonic() - launch_switch_enabled_at
            main_pwm = sum(servo[:4]) / 4 if not math.isnan(servo[0]) else math.nan
            if not math.isnan(main_pwm) and main_pwm > 1250 and first_launch_output_s is None:
                first_launch_output_s = launch_switch_elapsed
                # PARAM_VALUE and actuator messages traverse different MAVLink
                # queues. Keep 0.2 s transport tolerance around the firmware's
                # exact two-second gate while still detecting an early release.
                launch_gate_violation = first_launch_output_s < 1.8
                print(f"First launch output after switch: {first_launch_output_s:.3f}s")

            if elapsed >= 1.5 and not shell_requested:
                send_shell_command(
                    master,
                    "listener vehicle_status -n 1\n"
                    "listener vehicle_control_mode -n 1\n"
                    "listener position_setpoint_triplet -n 1",
                )
                shell_requested = True

            if elapsed >= 9.0 and not tecs_requested:
                send_shell_command(
                    master,
                    "listener tecs_status -n 1\nlistener airspeed_validated -n 1",
                )
                tecs_requested = True

            if transition_reached and not mc_shell_requested:
                send_shell_command(
                    master,
                    "listener position_setpoint_triplet -n 1\n"
                    "listener trajectory_setpoint -n 1\n"
                    "listener vehicle_local_position -n 1",
                )
                mc_shell_requested = True

            if elapsed - last_print >= 1.0:
                throttle = sum(servo[:4]) / 4 if not math.isnan(servo[0]) else math.nan
                north_to_landing = (
                    (landing_latitude - position[5]) / latitude_units_per_metre
                    if not math.isnan(position[5])
                    else math.nan
                )
                east_to_landing = (
                    (longitude - position[6])
                    / (latitude_units_per_metre * math.cos(math.radians(latitude * 1e-7)))
                    if not math.isnan(position[6])
                    else math.nan
                )
                landing_distance_m = math.hypot(north_to_landing, east_to_landing)
                print(
                    f"t={elapsed:5.1f} armed={int(state['armed'])} mission={state['mission']} "
                    f"vtol={state['vtol']} landed={state['landed']} main_pwm={throttle:.0f} "
                    f"amsl={position[0]:.1f} rel_alt={position[1]:+.1f} "
                    f"land_dist={landing_distance_m:.1f}m "
                    f"airspeed={airspeed_m_s:.1f} "
                    f"vel_ned=[{position[2]:+.1f} {position[3]:+.1f} {position[4]:+.1f}] "
                    f"rpy=[{attitude[0]:+.1f} {attitude[1]:+.1f} {attitude[2]:+.1f}] "
                    f"rpy_sp=[{attitude_target[0]:+.1f} {attitude_target[1]:+.1f} {attitude_target[2]:+.1f}] "
                    f"nav=[roll={navigation[0]:+.1f} bearing={navigation[1]:+.0f} xtrack={navigation[2]:+.1f}]"
                )
                last_print = elapsed

            if landing_reached:
                print(f"Position landing detected at t={elapsed:.2f}s")
                break

            if fw_waypoints_reached:
                print(f"Two fixed-wing waypoints remained controlled for 5 s at t={elapsed:.2f}s")
                break

        if launch_gate_violation:
            raise RuntimeError(
                f"Stand-launch output released too early ({first_launch_output_s:.3f}s after switch)"
            )
        if (args.transition_to_mc_at >= 0 or args.full_landing) and not transition_reached:
            raise RuntimeError("PX4 did not reach multicopter state after FW->MC request")
        if args.full_landing and not landing_reached:
            raise RuntimeError("Full mission did not complete the multicopter position landing")
        if args.fw_only and not fw_waypoints_reached:
            raise RuntimeError("Fixed-wing mission did not complete and stabilize beyond waypoint 2")
        if args.transition_to_mc_at >= 0 or args.full_landing:
            print("FW->MC transition validation passed")
        if args.full_landing:
            print("Two-waypoint descent and position-landing mission passed")
        if args.fw_only:
            print("Two-waypoint fixed-wing tracking validation passed")
    finally:
        runtime_file.write_text("force_enable=0\n", encoding="utf-8")
        force_disarm(master)
        master.set_mode("LOITER")
        set_int_param(master, "TD_FW_TKO_EN", 0)
        set_int_param(master, "COM_RC_IN_MODE", 0)
        time.sleep(0.5)
        master.close()
        print("Model frozen, PX4 force-disarmed, COM_RC_IN_MODE restored to 0")


if __name__ == "__main__":
    main()
