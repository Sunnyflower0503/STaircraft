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
    return None


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
    parser.add_argument("--source-system", type=int, default=255)
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
    parser.add_argument("--mc-speed", type=float, default=1.5)
    parser.add_argument("--cruise-airspeed", type=float, default=13.0)
    parser.add_argument("--pentagon-center", type=float, default=550.0)
    parser.add_argument("--pentagon-radius", type=float, default=220.0)
    parser.add_argument("--pentagon-fw-acceptance", type=float, default=100.0)
    parser.add_argument(
        "--use-current-mission",
        action="store_true",
        help="Run the mission already stored on PX4 without clearing or uploading mission items",
    )
    parser.add_argument(
        "--pentagon-mission",
        action="store_true",
        help="Arm in fixed-wing Stabilized, fly a closed pentagon, back-transition, and hold position",
    )
    parser.add_argument(
        "--quadrilateral-landing",
        action="store_true",
        help="Fly a straight entry and closed quadrilateral, then descend, transition, loiter, and land",
    )
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
    polygon_mission = args.pentagon_mission or args.quadrilateral_landing
    effective_fw_acceptance = (
        args.pentagon_fw_acceptance if polygon_mission else args.fw_wp_acceptance
    )

    runtime_file = Path(__file__).resolve().parents[1] / "runtime_control.txt"
    master = mavutil.mavlink_connection(
        args.port, baud=115200, source_system=args.source_system
    )
    heartbeat = master.wait_heartbeat(timeout=15)
    if heartbeat is None:
        raise RuntimeError(f"No PX4 heartbeat on {args.port}")
    send_gcs_heartbeat(master)

    try:
        runtime_file.write_text("force_enable=0\n", encoding="utf-8")
        set_int_param_and_wait(master, "COM_RC_IN_MODE", 4)
        set_int_param_and_wait(master, "TD_FW_TKO_EN", 0)
        set_float_param_and_wait(master, "TD_FW_WP_ACC", effective_fw_acceptance)
        set_float_param_and_wait(master, "TD_BTR_ARSP", args.back_transition_airspeed)
        set_float_param_and_wait(master, "TD_BTR_GATE_T", args.back_transition_gate_time)
        set_float_param_and_wait(master, "TD_BTR_THR", args.back_transition_throttle)
        time.sleep(0.5)
        global_position = None
        position_warmup_deadline = time.time() + 12.0
        next_position_request = 0.0
        while time.time() < position_warmup_deadline:
            if time.time() >= next_position_request:
                master.mav.command_long_send(
                    master.target_system,
                    master.target_component,
                    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                    0,
                    mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
                    100000,
                    0,
                    0,
                    0,
                    0,
                    0,
                )
                master.mav.request_data_stream_send(
                    master.target_system,
                    master.target_component,
                    mavutil.mavlink.MAV_DATA_STREAM_POSITION,
                    10,
                    1,
                )
                next_position_request = time.time() + 1.0
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
        longitude_units_per_metre = latitude_units_per_metre / math.cos(math.radians(latitude * 1e-7))

        def local_coordinate(north_m, east_m=0.0):
            return (
                latitude + round(north_m * latitude_units_per_metre),
                longitude + round(east_m * longitude_units_per_metre),
            )

        takeoff_latitude = latitude + round(args.takeoff_distance * latitude_units_per_metre)
        waypoint_latitude = latitude + round(args.waypoint_distance * latitude_units_per_metre)
        waypoint2_latitude = latitude + round(args.waypoint2_distance * latitude_units_per_metre)
        descent_latitude = latitude + round(args.descent_distance * latitude_units_per_metre)
        landing_latitude = latitude + round(args.landing_distance * latitude_units_per_metre)
        landing_longitude = longitude

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
        if set_home_result is None:
            print("Warning: no set-home ACK; continuing with the current-position home request")
        elif set_home_result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
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
        if transition_result is not None and transition_result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
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
        transition_item_seq = None
        mc_waypoint_seq = None
        mc_hold_seq = None
        pentagon_points = []
        current_target_available = not args.use_current_mission

        if args.quadrilateral_landing:
            # Straight entry, one closed quadrilateral, then a separate exit
            # and descent path. Repeating vertex 1 is necessary to complete
            # the fourth edge; no other waypoint is reused.
            straight_point = (400.0, 0.0)
            # Enter the rectangle northbound and fly four consistent left
            # turns. This avoids alternating turn directions at adjacent
            # vertices and gives the L1 controller a full straight leg after
            # every corner before the next capture region.
            quadrilateral_points = [
                (700.0, 0.0),
                (700.0, -400.0),
                (350.0, -400.0),
                (350.0, 0.0),
                (700.0, 0.0),
            ]
            exit_point = (1000.0, -100.0)
            # Use a long, shallow fixed-wing descent so the back-transition can
            # start below 10 m AGL without reaching the ground during conversion.
            descent_point = (1600.0, -150.0)
            landing_point = (1680.0, -150.0)
            landing_latitude, landing_longitude = local_coordinate(*landing_point)

            items = [items[0]]
            items.append(
                {
                    "frame": mavutil.mavlink.MAV_FRAME_MISSION,
                    "command": mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
                    "param1": 0.0,
                    "param2": args.cruise_airspeed,
                    "param3": -1.0,
                }
            )
            for north_m, east_m in [straight_point, *quadrilateral_points, exit_point]:
                point_lat, point_lon = local_coordinate(north_m, east_m)
                items.append(
                    {
                        "frame": mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                        "command": mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                        "param2": effective_fw_acceptance,
                        "x": point_lat,
                        "y": point_lon,
                        "z": takeoff_alt_m,
                    }
                )
            items.append(
                {
                    "frame": mavutil.mavlink.MAV_FRAME_MISSION,
                    "command": mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
                    "param1": 0.0,
                    "param2": args.approach_airspeed,
                    "param3": -1.0,
                }
            )
            descent_lat, descent_lon = local_coordinate(*descent_point)
            items.append(
                {
                    "frame": mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                    "command": mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                    "param2": 20.0,
                    "x": descent_lat,
                    "y": descent_lon,
                    "z": current_alt_m + args.transition_alt,
                }
            )
            transition_item_seq = len(items)
            items.extend(
                [
                    {
                        "frame": mavutil.mavlink.MAV_FRAME_MISSION,
                        "command": mavutil.mavlink.MAV_CMD_DO_VTOL_TRANSITION,
                        "param1": mavutil.mavlink.MAV_VTOL_STATE_MC,
                    },
                    {
                        "frame": mavutil.mavlink.MAV_FRAME_MISSION,
                        "command": mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
                        "param1": 1.0,
                        "param2": args.mc_speed,
                        "param3": -1.0,
                    },
                ]
            )
            mc_waypoint_seq = len(items)
            items.append(
                {
                    "frame": mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                    "command": mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                    "param1": 2.0,
                    "param2": args.mc_waypoint_acceptance,
                    "x": landing_latitude,
                    "y": landing_longitude,
                    "z": current_alt_m + args.transition_alt,
                }
            )
            mc_hold_seq = len(items)
            items.extend(
                [
                    {
                        "frame": mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                        "command": mavutil.mavlink.MAV_CMD_NAV_LOITER_TIME,
                        "param1": 5.0,
                        "x": landing_latitude,
                        "y": landing_longitude,
                        "z": current_alt_m + args.transition_alt,
                    },
                    {
                        "frame": mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                        "command": mavutil.mavlink.MAV_CMD_NAV_LAND,
                        "x": landing_latitude,
                        "y": landing_longitude,
                        "z": current_alt_m,
                    },
                ]
            )

        elif args.pentagon_mission:
            items = [items[0]]
            pentagon_angles = [math.pi, 3 * math.pi / 5, math.pi / 5, -math.pi / 5, -3 * math.pi / 5]

            for angle in pentagon_angles:
                north_m = args.pentagon_center + args.pentagon_radius * math.cos(angle)
                east_m = args.pentagon_radius * math.sin(angle)
                pentagon_points.append((north_m, east_m))
                point_lat, point_lon = local_coordinate(north_m, east_m)
                items.append(
                    {
                        "frame": mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                        "command": mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                        "param2": effective_fw_acceptance,
                        "x": point_lat,
                        "y": point_lon,
                        "z": takeoff_alt_m,
                    }
                )

            # Return to vertex 1 to complete all five polygon edges.
            first_lat, first_lon = local_coordinate(*pentagon_points[0])
            items.append(
                {
                    "frame": mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                    "command": mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                    "param2": effective_fw_acceptance,
                    "x": first_lat,
                    "y": first_lon,
                    "z": takeoff_alt_m,
                }
            )
            items.extend(
                [
                    {
                        "frame": mavutil.mavlink.MAV_FRAME_MISSION,
                        "command": mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
                        "param1": 0.0,
                        "param2": args.approach_airspeed,
                        "param3": -1.0,
                    },
                    {
                        "frame": mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                        "command": mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                        "param2": 12.0,
                        "x": descent_latitude,
                        "y": longitude,
                        "z": current_alt_m + args.transition_alt,
                    },
                ]
            )
            transition_item_seq = len(items)
            items.append(
                {
                    "frame": mavutil.mavlink.MAV_FRAME_MISSION,
                    "command": mavutil.mavlink.MAV_CMD_DO_VTOL_TRANSITION,
                    "param1": mavutil.mavlink.MAV_VTOL_STATE_MC,
                }
            )
            items.append(
                {
                    "frame": mavutil.mavlink.MAV_FRAME_MISSION,
                    "command": mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
                    "param1": 1.0,
                    "param2": args.mc_speed,
                    "param3": -1.0,
                }
            )
            mc_waypoint_seq = len(items)
            items.append(
                {
                    "frame": mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                    "command": mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                    "param1": 2.0,
                    "param2": args.mc_waypoint_acceptance,
                    "x": landing_latitude,
                    "y": longitude,
                    "z": current_alt_m + args.transition_alt,
                }
            )
            mc_hold_seq = len(items)
            items.append(
                {
                    "frame": mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                    "command": mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM,
                    "x": landing_latitude,
                    "y": longitude,
                    "z": current_alt_m + args.transition_alt,
                }
            )

        elif args.full_landing or args.fw_only:
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
        if args.use_current_mission:
            master.mav.mission_request_list_send(
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
            )
            mission_count = master.recv_match(type="MISSION_COUNT", blocking=True, timeout=5)
            if mission_count is None:
                raise RuntimeError("No MISSION_COUNT for the mission currently stored on PX4")
            if polygon_mission and int(mission_count.count) != len(items):
                raise RuntimeError(
                    f"Current mission has {mission_count.count} items; expected {len(items)} "
                    "for the pentagon transition sequence"
                )

            if mc_waypoint_seq is not None:
                target_item = None
                target_deadline = time.time() + 8
                while time.time() < target_deadline:
                    master.mav.mission_request_int_send(
                        master.target_system,
                        master.target_component,
                        mc_waypoint_seq,
                        mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
                    )
                    candidate = master.recv_match(
                        type=["MISSION_ITEM_INT", "MISSION_ITEM"],
                        blocking=True,
                        timeout=1,
                    )
                    if candidate is not None and int(candidate.seq) == mc_waypoint_seq:
                        target_item = candidate
                        break
                if target_item is None:
                    print(
                        "Warning: could not read the current MC target; "
                        "final hold will use Mission Loiter progress and groundspeed"
                    )
                else:
                    current_target_available = True
                    if target_item.get_type() == "MISSION_ITEM_INT":
                        landing_latitude = int(target_item.x)
                        landing_longitude = int(target_item.y)
                    else:
                        landing_latitude = round(float(target_item.x) * 1e7)
                        landing_longitude = round(float(target_item.y) * 1e7)

            master.mav.mission_set_current_send(
                master.target_system, master.target_component, 0
            )
            reset_deadline = time.time() + 5
            while time.time() < reset_deadline:
                send_gcs_heartbeat(master)
                current_item = master.recv_match(
                    type="MISSION_CURRENT", blocking=True, timeout=0.5
                )
                if current_item is not None and int(current_item.seq) == 0:
                    break
            else:
                raise RuntimeError("PX4 did not reset the current mission to item 0")
            print(f"Using current PX4 mission unchanged: {mission_count.count} items")
        else:
            upload_mission(master, items)

        if args.quadrilateral_landing and not args.use_current_mission:
            print(
                f"Quadrilateral landing mission uploaded: straight entry at {args.cruise_airspeed:.1f} m/s, "
                f"closed 350 x 400 m route, separate exit, descend to "
                f"+{args.transition_alt:.1f} m, transition, loiter 5 s, and land"
            )
        elif args.pentagon_mission and not args.use_current_mission:
            print(
                f"Pentagon mission uploaded: center={args.pentagon_center:.0f} m north, "
                f"radius={args.pentagon_radius:.0f} m, five closed edges at +{args.takeoff_alt:.1f} m; "
                f"FW acceptance={effective_fw_acceptance:.0f} m; "
                f"descend at {args.descent_distance:.0f} m to +{args.transition_alt:.1f} m, "
                f"transition and hold at {args.landing_distance:.0f} m north at {args.mc_speed:.1f} m/s"
            )
        elif args.full_landing and not args.use_current_mission:
            print(
                f"Full mission uploaded: takeoff {args.takeoff_distance:.0f} m -> "
                f"WP1 {args.waypoint_distance:.0f} m -> WP2 {args.waypoint2_distance:.0f} m "
                f"at +{args.takeoff_alt:.1f} m -> descend near {args.descent_distance:.0f} m "
                f"to +{args.transition_alt:.1f} m -> MC transition near the landing point -> land at "
                f"{args.landing_distance:.0f} m north"
            )
        elif args.fw_only and not args.use_current_mission:
            print(
                f"Fixed-wing validation mission uploaded: takeoff {args.takeoff_distance:.0f} m -> "
                f"WP1 {args.waypoint_distance:.0f} m -> WP2 {args.waypoint2_distance:.0f} m -> "
                f"final segment {args.descent_distance:.0f} m at +{args.takeoff_alt:.1f} m"
            )
        elif not args.use_current_mission:
            print(
                f"Fixed-wing selected while disarmed; mission uploaded: takeoff {args.takeoff_alt:.1f} m "
                f"at {args.takeoff_distance:.0f} m north -> {args.waypoint_distance:.0f} m north waypoint; "
                f"start alt={current_alt_m:.1f} AMSL"
            )

        def select_mode(mode_name):
            mode_result = None
            for _ in range(5):
                master.set_mode(mode_name)
                mode_result = wait_command_ack(master, mavutil.mavlink.MAV_CMD_DO_SET_MODE)
                if mode_result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    return
                current_heartbeat = master.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
                if current_heartbeat is not None and mavutil.mode_string_v10(current_heartbeat) == mode_name:
                    return
                time.sleep(1.0)

            raise RuntimeError(f"PX4 rejected {mode_name} mode with result {mode_result}")

        if polygon_mission:
            # Required operator-equivalent order: arm on the stand in fixed-wing
            # Stabilized, then enter Mission while the launch key is still off.
            select_mode("STABILIZED")

        else:
            select_mode("MISSION")

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
        if arm_result is None:
            arm_deadline = time.time() + 5
            while time.time() < arm_deadline:
                send_gcs_heartbeat(master)
                arm_heartbeat = master.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
                if (
                    arm_heartbeat is not None
                    and arm_heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                ):
                    break
            else:
                raise RuntimeError("No arm ACK and PX4 did not report an armed state")
            print("Warning: no arm ACK; armed state confirmed from HEARTBEAT")
        elif arm_result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
            raise RuntimeError(f"PX4 rejected arming with result {arm_result}")

        if polygon_mission:
            select_mode("MISSION")
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
        if mission_start_result is None:
            print("Warning: no MISSION_START ACK; mission progress will be verified directly")
        elif mission_start_result in (
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
        mc_mission_advanced = False
        landing_reached = False
        tip_protection_reached = False
        pentagon_route_completed = False
        fixed_point_reached = False
        fixed_point_since = None
        fw_waypoints_reached = False
        fw_waypoints_reached_at = None
        launch_gate_violation = False
        first_launch_output_s = None
        last_gcs_heartbeat = -1.0
        last_mission_seq = -1
        mission_progress_at = 0.0
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
            if state["mission"] != last_mission_seq:
                last_mission_seq = state["mission"]
                mission_progress_at = elapsed

            if (
                polygon_mission
                and 1 <= state["mission"] < transition_item_seq
                and elapsed - mission_progress_at > 60.0
            ):
                raise RuntimeError(
                    f"Polygon mission stalled at item {state['mission']} for more than 60 s"
                )
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
                args.quadrilateral_landing
                and not transition_requested
                and transition_item_seq is not None
                and state["mission"] >= transition_item_seq - 1
                and not math.isnan(position[1])
                and position[1] <= 10.0
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
                print(
                    f"Requested altitude-gated FW->MC transition at "
                    f"t={elapsed:.2f}s, rel_alt={position[1]:.1f}m"
                )

            if (
                (transition_requested or args.full_landing or polygon_mission)
                and state["vtol"] == mavutil.mavlink.MAV_VTOL_STATE_MC
            ):
                transition_reached = True

            if (
                args.quadrilateral_landing
                and transition_reached
                and not mc_mission_advanced
                and mc_waypoint_seq is not None
            ):
                master.mav.mission_set_current_send(
                    master.target_system, master.target_component, mc_waypoint_seq
                )
                mc_mission_advanced = True
                print(f"Advanced Mission to MC approach item {mc_waypoint_seq}")

            if (
                (args.full_landing or args.quadrilateral_landing)
                and transition_reached
                and state["landed"] == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND
            ):
                landing_reached = True

            if (
                args.quadrilateral_landing
                and transition_reached
                and not math.isnan(servo[6])
                and 0.5 * (servo[6] + servo[7]) >= 1450.0
            ):
                if not tip_protection_reached:
                    print(
                        f"Rear-contact wingtip protection observed: "
                        f"MAIN7/8=[{servo[6]:.0f}, {servo[7]:.0f}]"
                    )
                tip_protection_reached = True

            target_distance_m = math.nan
            if not math.isnan(position[5]):
                north_to_landing = (landing_latitude - position[5]) / latitude_units_per_metre
                east_to_landing = (landing_longitude - position[6]) / longitude_units_per_metre
                target_distance_m = math.hypot(north_to_landing, east_to_landing)

            if polygon_mission:
                if transition_item_seq is not None and state["mission"] >= transition_item_seq:
                    pentagon_route_completed = True

                horizontal_speed_m_s = math.hypot(position[2], position[3])
                fixed_point_ok = (
                    transition_reached
                    and mc_waypoint_seq is not None
                    and state["mission"] >= mc_waypoint_seq
                    and (
                        target_distance_m <= args.mc_waypoint_acceptance
                        or (
                            args.use_current_mission
                            and not current_target_available
                            and mc_hold_seq is not None
                            and state["mission"] >= mc_hold_seq
                        )
                    )
                    and horizontal_speed_m_s <= 1.0
                )

                if fixed_point_ok:
                    if fixed_point_since is None:
                        fixed_point_since = elapsed
                    fixed_point_reached = elapsed - fixed_point_since >= 5.0

                else:
                    fixed_point_since = None

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
                print(
                    f"t={elapsed:5.1f} armed={int(state['armed'])} mission={state['mission']} "
                    f"vtol={state['vtol']} landed={state['landed']} main_pwm={throttle:.0f} "
                    f"amsl={position[0]:.1f} rel_alt={position[1]:+.1f} "
                    f"target_dist={target_distance_m:.1f}m "
                    f"airspeed={airspeed_m_s:.1f} "
                    f"vel_ned=[{position[2]:+.1f} {position[3]:+.1f} {position[4]:+.1f}] "
                    f"rpy=[{attitude[0]:+.1f} {attitude[1]:+.1f} {attitude[2]:+.1f}] "
                    f"rpy_sp=[{attitude_target[0]:+.1f} {attitude_target[1]:+.1f} {attitude_target[2]:+.1f}] "
                    f"nav=[roll={navigation[0]:+.1f} bearing={navigation[1]:+.0f} xtrack={navigation[2]:+.1f}]"
                )
                last_print = elapsed

            if landing_reached and (not args.quadrilateral_landing or tip_protection_reached):
                print(f"Position landing detected at t={elapsed:.2f}s")
                break

            if args.pentagon_mission and fixed_point_reached:
                print(f"Pentagon mission position hold stable for 5 s at t={elapsed:.2f}s")
                break

            if fw_waypoints_reached:
                print(f"Two fixed-wing waypoints remained controlled for 5 s at t={elapsed:.2f}s")
                break

        if launch_gate_violation:
            raise RuntimeError(
                f"Stand-launch output released too early ({first_launch_output_s:.3f}s after switch)"
            )
        if (args.transition_to_mc_at >= 0 or args.full_landing or polygon_mission) and not transition_reached:
            raise RuntimeError("PX4 did not reach multicopter state after FW->MC request")
        if (args.full_landing or args.quadrilateral_landing) and not landing_reached:
            raise RuntimeError("Full mission did not complete the multicopter position landing")
        if args.quadrilateral_landing and not tip_protection_reached:
            raise RuntimeError("Rear-contact wingtip protection did not reach its commanded PWM")
        if args.fw_only and not fw_waypoints_reached:
            raise RuntimeError("Fixed-wing mission did not complete and stabilize beyond waypoint 2")
        if polygon_mission and not pentagon_route_completed:
            raise RuntimeError("Mission did not complete all fixed-wing polygon edges")
        if args.pentagon_mission and not fixed_point_reached:
            raise RuntimeError("Multicopter did not establish a stable hold at the final target")
        if args.transition_to_mc_at >= 0 or args.full_landing or polygon_mission:
            print("FW->MC transition validation passed")
        if args.full_landing:
            print("Two-waypoint descent and position-landing mission passed")
        if args.quadrilateral_landing:
            print("Vertical landing and rear-contact protection trigger reached")
        if args.fw_only:
            print("Two-waypoint fixed-wing tracking validation passed")
        if args.pentagon_mission:
            print("Stabilized-arm, stand-launch, closed-pentagon, back-transition, and position-hold mission passed")
        if args.quadrilateral_landing:
            print("Straight-entry, closed-quadrilateral, separate-exit, back-transition, and vertical-landing mission passed")
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
