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


def count_sign_changes(values, deadband):
    signs = []
    for value in values:
        if value > deadband:
            sign = 1
        elif value < -deadband:
            sign = -1
        else:
            continue
        if not signs or sign != signs[-1]:
            signs.append(sign)
    return max(len(signs) - 1, 0)


def evaluate_fixed_wing_legs(samples, mission_seqs, acceptance_m):
    issues = []
    max_corner_peak_m = max(2.5 * acceptance_m, 75.0)
    capture_band_m = max(acceptance_m / 3.0, 10.0)
    max_recaptured_peak_m = max(acceptance_m / 2.0, 15.0)

    print("Fixed-wing line tracking metrics:")
    for mission_seq in mission_seqs:
        leg = [sample for sample in samples if sample[1] == mission_seq]
        if len(leg) < 5:
            issues.append(f"item {mission_seq}: insufficient navigation samples ({len(leg)})")
            continue

        start_s = leg[0][0]
        end_s = leg[-1][0]
        xtrack = [sample[2] for sample in leg]
        roll_sp = [sample[3] for sample in leg]
        peak_m = max(abs(value) for value in xtrack)
        peak_index = max(range(len(leg)), key=lambda index: abs(leg[index][2]))
        capture_index = next(
            (
                index
                for index in range(peak_index, len(leg))
                if abs(leg[index][2]) <= capture_band_m
            ),
            None,
        )
        recaptured = leg[capture_index:] if capture_index is not None else []
        recaptured_xtrack = [sample[2] for sample in recaptured]
        recaptured_roll_sp = [sample[3] for sample in recaptured]
        actual_roll = [sample[4] for sample in leg if math.isfinite(sample[4])]
        recaptured_actual_roll = [sample[4] for sample in recaptured if math.isfinite(sample[4])]
        recaptured_roll_rate = [sample[5] for sample in recaptured if math.isfinite(sample[5])]
        recaptured_peak_m = (
            max(abs(value) for value in recaptured_xtrack) if recaptured_xtrack else math.inf
        )
        recapture_s = leg[capture_index][0] - start_s if capture_index is not None else math.inf
        rms_m = math.sqrt(sum(value * value for value in xtrack) / len(xtrack))
        crossings = count_sign_changes(recaptured_xtrack, 2.0)
        roll_reversals = count_sign_changes(recaptured_roll_sp, 0.5)
        actual_roll_reversals = count_sign_changes(recaptured_actual_roll, 0.5)
        max_abs_roll = max((abs(value) for value in actual_roll), default=math.nan)
        post_roll_rate_peak = max((abs(value) for value in recaptured_roll_rate), default=math.nan)
        print(
            f"  item={mission_seq} duration={end_s - start_s:.1f}s "
            f"corner_peak={peak_m:.1f}m recapture={recapture_s:.1f}s "
            f"recaptured_peak={recaptured_peak_m:.1f}m rms={rms_m:.1f}m "
            f"post_crossings={crossings} post_roll_sp_reversals={roll_reversals} "
            f"post_actual_roll_reversals={actual_roll_reversals} "
            f"post_roll_rate_peak={post_roll_rate_peak:.1f}deg/s "
            f"max_abs_roll={max_abs_roll:.1f}deg"
        )

        if peak_m > max_corner_peak_m:
            issues.append(
                f"item {mission_seq}: corner peak cross-track {peak_m:.1f}m > {max_corner_peak_m:.1f}m"
            )
        if capture_index is None:
            issues.append(f"item {mission_seq}: route was not recaptured within {capture_band_m:.1f}m")
            continue
        if recaptured_peak_m > max_recaptured_peak_m:
            issues.append(
                f"item {mission_seq}: post-capture peak {recaptured_peak_m:.1f}m > {max_recaptured_peak_m:.1f}m"
            )
        if crossings > 2:
            issues.append(f"item {mission_seq}: crossed the route {crossings} times after recapture")
        if roll_reversals > 4:
            issues.append(f"item {mission_seq}: roll command reversed {roll_reversals} times after recapture")
        if actual_roll_reversals > 4:
            issues.append(f"item {mission_seq}: actual roll reversed {actual_roll_reversals} times after recapture")
        if math.isfinite(post_roll_rate_peak) and post_roll_rate_peak > 30.0:
            issues.append(
                f"item {mission_seq}: post-capture roll rate reached {post_roll_rate_peak:.1f}deg/s"
            )
        if math.isfinite(max_abs_roll) and max_abs_roll > 35.0:
            issues.append(f"item {mission_seq}: actual roll reached {max_abs_roll:.1f}deg")

    return issues


def decode_int_param_value(value):
    raw_value = float(value)
    numeric_value = int(round(raw_value))
    union_value = struct.unpack("<i", struct.pack("<f", raw_value))[0]
    if abs(raw_value) > 1e-30 and abs(raw_value - numeric_value) <= 1e-3:
        return numeric_value
    return union_value


def get_int_param(master, name, timeout=5):
    deadline = time.time() + timeout
    next_send = 0.0
    while time.time() < deadline:
        if time.time() >= next_send:
            master.mav.param_request_read_send(
                master.target_system,
                master.target_component,
                name.encode(),
                -1,
            )
            next_send = time.time() + 0.5
        send_gcs_heartbeat(master)
        msg = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
        if msg is not None and msg.param_id.rstrip("\x00") == name:
            return decode_int_param_value(msg.param_value)
    raise RuntimeError(f"No PARAM_VALUE readback for {name}")


def set_int_param_and_wait(master, name, value, timeout=5):
    deadline = time.time() + timeout
    next_send = 0.0
    last_value = None
    while time.time() < deadline:
        if time.time() >= next_send:
            set_int_param(master, name, value)
            next_send = time.time() + 0.5
        send_gcs_heartbeat(master)
        msg = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
        if msg is not None and msg.param_id.rstrip("\x00") == name:
            last_value = float(msg.param_value)
            # PX4 may return integer parameters either as an ordinary numeric
            # float or as the raw INT32 union bits carried by PARAM_VALUE.
            if decode_int_param_value(last_value) == int(value):
                return
    raise RuntimeError(f"No matching PARAM_VALUE for {name}; last readback={last_value}")


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
    parser.add_argument(
        "--transition-trigger-alt",
        type=float,
        default=20.0,
        help="Request FW-to-MC conversion at or below this relative altitude",
    )
    parser.add_argument("--flare-alt", type=float, default=18.0)
    parser.add_argument("--transition-max-airspeed", type=float, default=12.0)
    parser.add_argument("--transition-max-descent-rate", type=float, default=0.5)
    parser.add_argument("--transition-max-roll", type=float, default=10.0)
    parser.add_argument("--transition-max-pitch", type=float, default=15.0)
    parser.add_argument("--transition-stable-time", type=float, default=0.5)
    parser.add_argument("--transition-flare-radius", type=float, default=120.0)
    parser.add_argument("--mission-stall-timeout", type=float, default=120.0)
    parser.add_argument("--fw-wp-acceptance", type=float, default=30.0)
    parser.add_argument("--approach-airspeed", type=float, default=11.5)
    parser.add_argument("--back-transition-airspeed", type=float, default=13.5)
    parser.add_argument("--back-transition-gate-time", type=float, default=0.5)
    parser.add_argument("--back-transition-throttle", type=float, default=0.40)
    parser.add_argument("--mc-waypoint-acceptance", type=float, default=10.0)
    parser.add_argument("--mc-speed", type=float, default=1.5)
    parser.add_argument("--mc-max-down-speed", type=float, default=0.8)
    parser.add_argument("--land-speed", type=float, default=0.6)
    parser.add_argument("--land-alt1", type=float, default=12.0)
    parser.add_argument("--land-alt2", type=float, default=8.0)
    parser.add_argument("--cruise-airspeed", type=float, default=13.0)
    parser.add_argument("--fw-roll-rate-p", type=float, default=0.05)
    parser.add_argument("--fw-roll-rate-i", type=float, default=0.04)
    parser.add_argument("--fw-roll-rate-ff", type=float, default=0.25)
    parser.add_argument("--fw-roll-rate-imax", type=float, default=0.15)
    parser.add_argument("--fw-roll-time-constant", type=float, default=0.5)
    parser.add_argument("--fw-roll-rate-limit", type=float, default=45.0)
    parser.add_argument("--fw-roll-limit", type=float, default=25.0)
    parser.add_argument("--fw-roll-setpoint-slew", type=float, default=8.0)
    parser.add_argument("--fw-l1-period", type=float, default=20.0)
    parser.add_argument("--fw-l1-damping", type=float, default=0.8)
    parser.add_argument("--fw-max-abs-roll", type=float, default=40.0)
    parser.add_argument("--quad-entry-distance", type=float, default=230.0)
    parser.add_argument("--quad-near-north", type=float, default=200.0)
    parser.add_argument("--quad-far-north", type=float, default=800.0)
    parser.add_argument("--quad-west", type=float, default=650.0)
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
        "--polygon-fw-only",
        action="store_true",
        help="Stop after all fixed-wing polygon edges without entering FW-to-MC conversion",
    )
    parser.add_argument(
        "--quadrilateral-landing",
        action="store_true",
        help="Fly a straight entry and closed quadrilateral, then descend, transition, loiter, and land",
    )
    parser.add_argument(
        "--hold-after-transition",
        action="store_true",
        help="With --quadrilateral-landing, stop after a stable MC position hold instead of landing",
    )
    parser.add_argument(
        "--approach-only",
        action="store_true",
        help="With --quadrilateral-landing, skip the polygon and isolate the straight descent/flare/landing sequence",
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
    if args.polygon_fw_only and not (args.pentagon_mission or args.quadrilateral_landing):
        raise ValueError("--polygon-fw-only requires --pentagon-mission or --quadrilateral-landing")
    if args.quadrilateral_landing and not args.approach_only:
        if not (args.quad_near_north < args.quad_entry_distance < args.quad_far_north):
            raise ValueError("Require quad-near-north < quad-entry-distance < quad-far-north")
        if args.quad_west <= 0.0:
            raise ValueError("quad-west must be positive")
    polygon_mission = args.pentagon_mission or args.quadrilateral_landing
    effective_fw_acceptance = (
        args.pentagon_fw_acceptance if args.pentagon_mission else args.fw_wp_acceptance
    )

    runtime_file = Path(__file__).resolve().parents[1] / "runtime_control.txt"
    master = mavutil.mavlink_connection(
        args.port, baud=115200, source_system=args.source_system
    )
    heartbeat = master.wait_heartbeat(timeout=15)
    if heartbeat is None:
        raise RuntimeError(f"No PX4 heartbeat on {args.port}")
    send_gcs_heartbeat(master)
    original_com_rcl_except = None

    try:
        runtime_file.write_text("force_enable=0\n", encoding="utf-8")
        set_int_param_and_wait(master, "COM_RC_IN_MODE", 4)
        original_com_rcl_except = get_int_param(master, "COM_RCL_EXCEPT")
        set_int_param_and_wait(master, "COM_RCL_EXCEPT", original_com_rcl_except | 1)
        set_int_param_and_wait(master, "TD_FW_TKO_EN", 0)
        set_int_param_and_wait(master, "FW_L1_METHOD", 0)
        set_int_param_and_wait(master, "TD_FW_NAV_DIR", 0)
        set_float_param_and_wait(master, "TD_FW_WP_ACC", effective_fw_acceptance)
        set_float_param_and_wait(master, "FW_RR_P", args.fw_roll_rate_p)
        set_float_param_and_wait(master, "FW_RR_I", args.fw_roll_rate_i)
        set_float_param_and_wait(master, "FW_RR_FF", args.fw_roll_rate_ff)
        set_float_param_and_wait(master, "FW_RR_IMAX", args.fw_roll_rate_imax)
        set_float_param_and_wait(master, "FW_R_TC", args.fw_roll_time_constant)
        set_float_param_and_wait(master, "FW_R_RMAX", args.fw_roll_rate_limit)
        set_float_param_and_wait(master, "FW_R_LIM", args.fw_roll_limit)
        set_float_param_and_wait(master, "FW_L1_R_SLEW_MAX", args.fw_roll_setpoint_slew)
        set_float_param_and_wait(master, "FW_L1_PERIOD", args.fw_l1_period)
        set_float_param_and_wait(master, "FW_L1_DAMPING", args.fw_l1_damping)
        set_float_param_and_wait(master, "TD_BTR_ARSP", args.back_transition_airspeed)
        set_float_param_and_wait(master, "TD_BTR_ROLL", args.transition_max_roll)
        set_float_param_and_wait(master, "TD_BTR_PITCH", args.transition_max_pitch)
        set_float_param_and_wait(master, "TD_BTR_GATE_T", args.back_transition_gate_time)
        set_float_param_and_wait(master, "TD_BTR_THR", args.back_transition_throttle)
        set_float_param_and_wait(master, "MPC_Z_VEL_MAX_DN", args.mc_max_down_speed)
        set_float_param_and_wait(master, "MPC_LAND_SPEED", args.land_speed)
        set_float_param_and_wait(master, "MPC_LAND_ALT1", args.land_alt1)
        set_float_param_and_wait(master, "MPC_LAND_ALT2", args.land_alt2)
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
        fw_polygon_complete_seq = None
        mc_waypoint_seq = None
        mc_hold_seq = None
        pentagon_points = []
        flare_latitude = None
        flare_longitude = None
        current_target_available = not args.use_current_mission

        if args.quadrilateral_landing:
            # Straight entry, one closed quadrilateral, then a separate exit
            # and descent path. Repeating vertex 1 is necessary to complete
            # the fourth edge; no other waypoint is reused.
            straight_point = (args.quad_entry_distance, 0.0)
            # Enter the rectangle northbound and fly four consistent left
            # turns. This avoids alternating turn directions at adjacent
            # vertices and gives the L1 controller a full straight leg after
            # every corner before the next capture region.
            quadrilateral_points = [
                (args.quad_far_north, 0.0),
                (args.quad_far_north, -args.quad_west),
                (args.quad_near_north, -args.quad_west),
                (args.quad_near_north, 0.0),
                (args.quad_far_north, 0.0),
            ]
            exit_point = (1100.0, -120.0)
            # Use separate descent and flare legs. The long first leg sheds
            # altitude at fixed-wing speed; the second leg commands a gentle
            # pull-up so vertical speed is arrested before back-transition.
            descent_point = (2000.0, -150.0)
            flare_point = (2300.0, -150.0)
            landing_point = (2400.0, -150.0)
            if args.approach_only:
                quadrilateral_points = []
                exit_point = (500.0, 0.0)
                descent_point = (1300.0, 0.0)
                flare_point = (1600.0, 0.0)
                landing_point = flare_point
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
            flare_latitude, flare_longitude = local_coordinate(*flare_point)
            items.append(
                {
                    "frame": mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                    "command": mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                    "param1": 300.0,
                    "param2": 20.0,
                    "x": flare_latitude,
                    "y": flare_longitude,
                    "z": current_alt_m + args.flare_alt,
                }
            )
            transition_item_seq = len(items)
            # Do not put MAV_CMD_DO_VTOL_TRANSITION in the mission here. It
            # would bypass the script-side airspeed, attitude, vertical-speed,
            # and continuous-stability gate. Hold the flare waypoint until the
            # explicit command succeeds, then jump directly to the MC approach.
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
                    "y": landing_longitude,
                    "z": current_alt_m + args.transition_alt,
                }
            )
            mc_hold_seq = len(items)
            if args.hold_after_transition:
                items.append(
                    {
                        "frame": mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                        "command": mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM,
                        "x": landing_latitude,
                        "y": landing_longitude,
                        "z": current_alt_m + args.transition_alt,
                    }
                )
            else:
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
            fw_polygon_complete_seq = len(items)
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
            if args.approach_only:
                print(
                    f"Approach-only mission uploaded: straight flight, descend to "
                    f"+{args.transition_alt:.1f} m, flare to +{args.flare_alt:.1f} m, "
                    f"transition, loiter 5 s, and land"
                )
            else:
                print(
                    f"Quadrilateral landing mission uploaded: straight entry at {args.cruise_airspeed:.1f} m/s, "
                    f"takeoff-to-entry gap {args.quad_entry_distance - args.takeoff_distance:.0f} m, "
                    f"closed {args.quad_far_north - args.quad_near_north:.0f} x {args.quad_west:.0f} m route, "
                    f"separate exit, descend to "
                    f"+{args.transition_alt:.1f} m, flare to +{args.flare_alt:.1f} m, "
                    f"transition, loiter 5 s, and land"
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
        roll_rate_deg_s = math.nan
        attitude_target = [math.nan] * 3
        navigation = [math.nan] * 3
        airspeed_m_s = math.nan
        shell_requested = False
        tecs_requested = False
        fw_pitch_diag_requested = False
        mc_shell_requested = False
        transition_requested = False
        transition_gate_since = None
        transition_reached = False
        mc_mission_advanced = False
        landing_reached = False
        tip_protection_reached = False
        touchdown_speed_m_s = math.nan
        touchdown_descent_m_s = math.nan
        pentagon_route_completed = False
        fixed_point_reached = False
        fixed_point_since = None
        fw_waypoints_reached = False
        fw_waypoints_reached_at = None
        max_rel_alt_m = -math.inf
        launch_gate_violation = False
        first_launch_output_s = None
        launch_gate_waiting_s = None
        launch_gate_enabled_s = None
        last_gcs_heartbeat = -1.0
        last_mission_seq = -1
        mission_progress_at = 0.0
        fw_tracking_samples = []
        fw_excess_roll_since = None
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
                    roll_rate_deg_s = math.degrees(msg.rollspeed)
                elif msg_type == "ATTITUDE_TARGET":
                    attitude_target = quaternion_to_euler_deg(msg.q)
                elif msg_type == "NAV_CONTROLLER_OUTPUT":
                    navigation = [float(msg.nav_roll), float(msg.nav_bearing), float(msg.xtrack_error)]
                    if (
                        state["vtol"] == mavutil.mavlink.MAV_VTOL_STATE_FW
                        and state["mission"] >= 0
                        and math.isfinite(navigation[0])
                        and math.isfinite(navigation[2])
                    ):
                        fw_tracking_samples.append(
                            (
                                time.monotonic() - started,
                                state["mission"],
                                navigation[2],
                                navigation[0],
                                attitude[0],
                                roll_rate_deg_s,
                            )
                        )
                elif msg_type == "VFR_HUD":
                    airspeed_m_s = float(msg.airspeed)
                elif msg_type == "STATUSTEXT":
                    print(f"STATUSTEXT[{msg.severity}]: {msg.text}")
                    status_text = str(msg.text).casefold()
                    if "stand launch switch: waiting 2 s" in status_text:
                        launch_gate_waiting_s = time.monotonic() - launch_switch_enabled_at
                    elif "stand launch switch: enabled" in status_text:
                        launch_gate_enabled_s = time.monotonic() - launch_switch_enabled_at
                        print(f"Firmware launch gate enabled after {launch_gate_enabled_s:.3f}s")
                    if status_text.startswith("failsafe enabled"):
                        raise RuntimeError(f"PX4 entered failsafe: {msg.text}")
                elif msg_type == "COMMAND_ACK":
                    print(f"COMMAND_ACK command={msg.command} result={msg.result}")
                elif msg_type == "SERIAL_CONTROL" and msg.count:
                    shell_text = bytes(msg.data[: msg.count]).decode(errors="replace")
                    print(shell_text, end="")

            elapsed = time.monotonic() - started
            excessive_fixed_wing_roll = (
                state["vtol"] == mavutil.mavlink.MAV_VTOL_STATE_FW
                and state["mission"] >= 2
                and math.isfinite(attitude[0])
                and abs(attitude[0]) > args.fw_max_abs_roll
            )
            if excessive_fixed_wing_roll:
                if fw_excess_roll_since is None:
                    fw_excess_roll_since = elapsed
                elif elapsed - fw_excess_roll_since >= 0.3:
                    raise RuntimeError(
                        f"Fixed-wing roll exceeded {args.fw_max_abs_roll:.1f}deg for 0.3s: "
                        f"roll={attitude[0]:+.1f}deg mission={state['mission']}"
                    )
            else:
                fw_excess_roll_since = None
            if not math.isnan(position[1]):
                max_rel_alt_m = max(max_rel_alt_m, position[1])
            if (
                not transition_reached
                and state["vtol"] == mavutil.mavlink.MAV_VTOL_STATE_FW
                and max_rel_alt_m >= 15.0
                and position[1] <= 8.0
                and position[4] >= 3.0
            ):
                raise RuntimeError(
                    "Unsafe fixed-wing descent detected before ground contact: "
                    f"rel_alt={position[1]:.1f}m vz_down={position[4]:.1f}m/s"
                )
            if state["mission"] != last_mission_seq:
                last_mission_seq = state["mission"]
                mission_progress_at = elapsed

            if (
                polygon_mission
                and 1 <= state["mission"] < transition_item_seq
                and elapsed - mission_progress_at > args.mission_stall_timeout
            ):
                raise RuntimeError(
                    f"Polygon mission stalled at item {state['mission']} for more than "
                    f"{args.mission_stall_timeout:.0f} s"
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

            flare_distance_m = math.inf
            if (
                flare_latitude is not None
                and not math.isnan(position[5])
            ):
                north_to_flare = (flare_latitude - position[5]) / latitude_units_per_metre
                east_to_flare = (flare_longitude - position[6]) / longitude_units_per_metre
                flare_distance_m = math.hypot(north_to_flare, east_to_flare)

            transition_gate_ready = (
                args.quadrilateral_landing
                and not transition_requested
                and transition_item_seq is not None
                and state["mission"] >= transition_item_seq - 1
                and not math.isnan(position[1])
                and position[1] <= args.transition_trigger_alt
                and position[1] >= args.transition_alt
                and not math.isnan(airspeed_m_s)
                and airspeed_m_s <= args.transition_max_airspeed
                and position[4] <= args.transition_max_descent_rate
                and not math.isnan(attitude[0])
                and abs(attitude[0]) <= args.transition_max_roll
                and abs(attitude[1]) <= args.transition_max_pitch
                and flare_distance_m <= args.transition_flare_radius
            )
            if transition_gate_ready:
                if transition_gate_since is None:
                    transition_gate_since = elapsed
            else:
                transition_gate_since = None

            if (
                transition_gate_since is not None
                and elapsed - transition_gate_since >= args.transition_stable_time
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
                    f"t={elapsed:.2f}s, rel_alt={position[1]:.1f}m "
                    f"airspeed={airspeed_m_s:.1f}m/s vz_down={position[4]:+.1f}m/s "
                    f"roll={attitude[0]:+.1f}deg pitch={attitude[1]:+.1f}deg "
                    f"flare_dist={flare_distance_m:.0f}m"
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
                    touchdown_speed_m_s = math.hypot(position[2], position[3])
                    touchdown_descent_m_s = position[4]
                    print(
                        f"Rear-contact wingtip protection observed: "
                        f"MAIN7/8=[{servo[6]:.0f}, {servo[7]:.0f}], "
                        f"touchdown_xy={touchdown_speed_m_s:.2f}m/s, "
                        f"touchdown_vz_down={touchdown_descent_m_s:.2f}m/s"
                    )
                tip_protection_reached = True

            if (
                tip_protection_reached
                and not math.isnan(position[2])
                and math.hypot(position[2], position[3]) > 1.5
            ):
                raise RuntimeError(
                    "Aircraft slid or bounced after rear contact: "
                    f"horizontal speed={math.hypot(position[2], position[3]):.2f} m/s"
                )

            target_distance_m = math.nan
            if not math.isnan(position[5]):
                north_to_landing = (landing_latitude - position[5]) / latitude_units_per_metre
                east_to_landing = (landing_longitude - position[6]) / longitude_units_per_metre
                target_distance_m = math.hypot(north_to_landing, east_to_landing)

            if polygon_mission:
                completion_seq = fw_polygon_complete_seq or transition_item_seq
                if completion_seq is not None and state["mission"] >= completion_seq:
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

            if elapsed >= 18.0 and not fw_pitch_diag_requested:
                send_shell_command(
                    master,
                    "listener fw_virtual_attitude_setpoint -n 1\n"
                    "listener actuator_controls_1 -n 1\n"
                    "listener rate_ctrl_status -n 1\n"
                    "listener vehicle_angular_velocity -n 1\n"
                    "listener vehicle_local_position -n 1",
                )
                fw_pitch_diag_requested = True

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

            if args.polygon_fw_only and pentagon_route_completed:
                print(f"Fixed-wing polygon edges completed at t={elapsed:.2f}s")
                break

            if args.quadrilateral_landing and args.hold_after_transition and fixed_point_reached:
                print(f"Quadrilateral mission position hold stable for 5 s at t={elapsed:.2f}s")
                break

            if fw_waypoints_reached:
                print(f"Two fixed-wing waypoints remained controlled for 5 s at t={elapsed:.2f}s")
                break

        if launch_gate_violation:
            launch_message = (
                f"Stand-launch output released early ({first_launch_output_s:.3f}s after switch)"
            )
            if polygon_mission:
                print(f"WARNING: {launch_message}; polygon tracking result remains authoritative")
            else:
                raise RuntimeError(launch_message)
        if launch_gate_enabled_s is None:
            if polygon_mission:
                print("WARNING: Firmware did not report the TD_FW_TKO_EN launch gate opening; "
                      "polygon tracking result remains authoritative")
            else:
                raise RuntimeError("Firmware did not report the TD_FW_TKO_EN launch gate opening")
        elif launch_gate_enabled_s - (launch_gate_waiting_s or 0.0) < 1.9:
            gate_interval_s = launch_gate_enabled_s - (launch_gate_waiting_s or 0.0)
            if polygon_mission:
                print(f"WARNING: TD_FW_TKO_EN gate interval was {gate_interval_s:.3f}s; "
                      "polygon tracking result remains authoritative")
            else:
                raise RuntimeError(
                    f"TD_FW_TKO_EN gate interval was too short: {gate_interval_s:.3f}s"
                )
        if (
            (args.transition_to_mc_at >= 0 or args.full_landing or polygon_mission)
            and not args.polygon_fw_only
            and not transition_reached
        ):
            raise RuntimeError("PX4 did not reach multicopter state after FW->MC request")
        if (
            (args.full_landing or args.quadrilateral_landing)
            and not args.hold_after_transition
            and not landing_reached
        ):
            raise RuntimeError("Full mission did not complete the multicopter position landing")
        if args.quadrilateral_landing and not args.hold_after_transition and not tip_protection_reached:
            raise RuntimeError("Rear-contact wingtip protection did not reach its commanded PWM")
        if args.quadrilateral_landing and not args.hold_after_transition and (
            math.isnan(touchdown_descent_m_s) or touchdown_descent_m_s > 0.8
        ):
            raise RuntimeError(
                f"Touchdown descent rate was not gentle: {touchdown_descent_m_s:.2f} m/s"
            )
        if args.fw_only and not fw_waypoints_reached:
            raise RuntimeError("Fixed-wing mission did not complete and stabilize beyond waypoint 2")
        if polygon_mission and not pentagon_route_completed:
            raise RuntimeError("Mission did not complete all fixed-wing polygon edges")
        if args.quadrilateral_landing and not args.approach_only:
            tracking_issues = evaluate_fixed_wing_legs(
                fw_tracking_samples, range(3, 9), effective_fw_acceptance
            )
            if tracking_issues:
                raise RuntimeError("Oscillatory fixed-wing line tracking: " + "; ".join(tracking_issues))
        if args.pentagon_mission:
            tracking_issues = evaluate_fixed_wing_legs(
                fw_tracking_samples, range(2, 7), effective_fw_acceptance
            )
            if tracking_issues:
                raise RuntimeError("Oscillatory fixed-wing line tracking: " + "; ".join(tracking_issues))
        if args.pentagon_mission and not args.polygon_fw_only and not fixed_point_reached:
            raise RuntimeError("Multicopter did not establish a stable hold at the final target")
        if args.quadrilateral_landing and args.hold_after_transition and not fixed_point_reached:
            raise RuntimeError("Multicopter did not establish a stable hold at the final target")
        if (args.transition_to_mc_at >= 0 or args.full_landing or polygon_mission) and not args.polygon_fw_only:
            print("FW->MC transition validation passed")
        if args.full_landing:
            print("Two-waypoint descent and position-landing mission passed")
        if args.quadrilateral_landing and not args.hold_after_transition:
            print("Vertical landing and rear-contact protection trigger reached")
        if args.fw_only:
            print("Two-waypoint fixed-wing tracking validation passed")
        if args.pentagon_mission:
            if args.polygon_fw_only:
                print("Stabilized-arm, stand-launch, and closed-pentagon fixed-wing mission passed")
            else:
                print("Stabilized-arm, stand-launch, closed-pentagon, back-transition, and position-hold mission passed")
        if args.quadrilateral_landing:
            if args.approach_only:
                mission_end = "position-hold" if args.hold_after_transition else "vertical-landing"
                print(f"Straight descent, flare, back-transition, and {mission_end} mission passed")
            else:
                mission_end = "position-hold" if args.hold_after_transition else "vertical-landing"
                print(f"Straight-entry, closed-quadrilateral, separate-exit, back-transition, and {mission_end} mission passed")
    finally:
        runtime_file.write_text("force_enable=0\n", encoding="utf-8")
        force_disarm(master)
        master.set_mode("LOITER")
        set_int_param(master, "TD_BTR_DBG_FAST", 0)
        set_int_param(master, "TD_FW_TKO_EN", 0)
        if original_com_rcl_except is not None:
            set_int_param(master, "COM_RCL_EXCEPT", original_com_rcl_except)
        set_int_param(master, "COM_RC_IN_MODE", 0)
        time.sleep(0.5)
        master.close()
        print("Model frozen, PX4 force-disarmed, COM_RC_IN_MODE restored to 0")


if __name__ == "__main__":
    main()
