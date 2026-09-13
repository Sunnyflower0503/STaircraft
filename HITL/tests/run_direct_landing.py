"""Run a no-route, six-contact tandem-tailsitter landing over USB MAVLink.

The MATLAB plant starts eight metres above the ground in a frozen nose-up
multicopter pose.  This controller prepares PX4, arms with the documented HITL
force code, selects AUTO.LAND, and then releases the plant through
runtime_control.txt.  Cleanup always freezes the plant and force-disarms PX4.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import time
from pathlib import Path

from pymavlink import mavutil

from run_mission_stand_takeoff import (
    force_disarm,
    get_int_param,
    send_gcs_heartbeat,
    set_float_param_and_wait,
    set_int_param,
    set_int_param_and_wait,
    wait_command_ack,
)


def get_float_param(master, name, timeout=5.0):
    deadline = time.time() + timeout
    next_send = 0.0
    while time.time() < deadline:
        if time.time() >= next_send:
            master.mav.param_request_read_send(
                master.target_system, master.target_component, name.encode(), -1
            )
            next_send = time.time() + 0.5
        send_gcs_heartbeat(master)
        msg = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
        if msg is not None and msg.param_id.rstrip("\x00") == name:
            return float(msg.param_value)
    raise RuntimeError(f"No PARAM_VALUE readback for {name}")


def request_message_interval(master, message_id, frequency_hz):
    interval_us = int(1_000_000 / frequency_hz)
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        message_id,
        interval_us,
        0,
        0,
        0,
        0,
        0,
    )


def select_mode(master, mode_name, timeout=8.0):
    available = master.mode_mapping() or {}
    candidates = [mode_name]
    if mode_name == "LAND":
        candidates.append("AUTO.LAND")
    selected = next((name for name in candidates if name in available), None)
    if selected is None:
        raise RuntimeError(
            f"Mode {mode_name} unavailable; PX4 reports {sorted(available)}"
        )

    deadline = time.time() + timeout
    next_send = 0.0
    last_mode = "UNKNOWN"
    while time.time() < deadline:
        if time.time() >= next_send:
            master.set_mode(selected)
            next_send = time.time() + 1.0
        send_gcs_heartbeat(master)
        send_manual(master, 500)
        msg = master.recv_match(type=["HEARTBEAT", "COMMAND_ACK"], blocking=True, timeout=0.1)
        if msg is None:
            continue
        if msg.get_type() == "HEARTBEAT":
            last_mode = mavutil.mode_string_v10(msg)
            if last_mode in candidates or (mode_name == "LAND" and "LAND" in last_mode):
                return last_mode
        elif (
            int(msg.command) == int(mavutil.mavlink.MAV_CMD_DO_SET_MODE)
            and int(msg.result) not in (
                mavutil.mavlink.MAV_RESULT_ACCEPTED,
                mavutil.mavlink.MAV_RESULT_IN_PROGRESS,
            )
        ):
            raise RuntimeError(f"PX4 rejected {selected} mode with result {msg.result}")
    raise RuntimeError(f"PX4 did not enter {selected}; last mode was {last_mode}")


def select_multicopter(master, timeout=8.0):
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
    result = wait_command_ack(master, mavutil.mavlink.MAV_CMD_DO_VTOL_TRANSITION)
    if result not in (None, mavutil.mavlink.MAV_RESULT_ACCEPTED):
        raise RuntimeError(f"PX4 rejected multicopter selection with result {result}")

    deadline = time.time() + timeout
    while time.time() < deadline:
        send_gcs_heartbeat(master)
        send_manual(master, 500)
        msg = master.recv_match(type="EXTENDED_SYS_STATE", blocking=True, timeout=0.5)
        if msg is not None and int(msg.vtol_state) == mavutil.mavlink.MAV_VTOL_STATE_MC:
            return
    raise RuntimeError("PX4 did not reach multicopter state")


def send_manual(master, throttle=500):
    master.mav.manual_control_send(
        master.target_system,
        0,
        0,
        int(throttle),
        0,
        0,
    )


def force_arm(master, timeout=8.0):
    deadline = time.time() + timeout
    next_send = 0.0
    last_ack = None
    while time.time() < deadline:
        if time.time() >= next_send:
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
            next_send = time.time() + 1.0
        send_gcs_heartbeat(master)
        send_manual(master, 500)
        msg = master.recv_match(type=["HEARTBEAT", "COMMAND_ACK"], blocking=True, timeout=0.25)
        if msg is None:
            continue
        if msg.get_type() == "HEARTBEAT":
            if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                return
        elif int(msg.command) == int(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM):
            last_ack = int(msg.result)
    raise RuntimeError(f"PX4 did not arm; last ACK={last_ack}")


def wait_hover_outputs(master, timeout=12.0, minimum_pwm=1250.0):
    deadline = time.time() + timeout
    ready_since = None
    main_pwm = [math.nan] * 4
    while time.time() < deadline:
        send_gcs_heartbeat(master)
        send_manual(master, 600)
        msg = master.recv_match(
            type=["HEARTBEAT", "SERVO_OUTPUT_RAW", "STATUSTEXT"],
            blocking=True,
            timeout=0.1,
        )
        if msg is not None:
            if msg.get_type() == "HEARTBEAT" and not (
                msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            ):
                raise RuntimeError("PX4 disarmed during frozen output warmup")
            if msg.get_type() == "SERVO_OUTPUT_RAW":
                main_pwm = [float(getattr(msg, f"servo{i}_raw")) for i in range(1, 5)]
            if msg.get_type() == "STATUSTEXT" and "failsafe" in str(msg.text).lower():
                raise RuntimeError(f"Failsafe during frozen output warmup: {msg.text}")

        ready = all(math.isfinite(value) and value >= minimum_pwm for value in main_pwm)
        if ready:
            if ready_since is None:
                ready_since = time.time()
            elif time.time() - ready_since >= 1.0:
                print(
                    "Frozen output gate passed: MAIN1-4="
                    + "/".join(f"{value:.0f}" for value in main_pwm)
                )
                return
        else:
            ready_since = None
    raise RuntimeError(
        "Frozen output gate failed: MAIN1-4="
        + "/".join("nan" if not math.isfinite(v) else f"{v:.0f}" for v in main_pwm)
    )


def wait_airborne_position(master, min_altitude_m=5.0, timeout=20.0):
    deadline = time.time() + timeout
    good_since = None
    local_z = math.nan
    relative_alt_m = math.nan
    while time.time() < deadline:
        send_gcs_heartbeat(master)
        send_manual(master, 500)
        msg = master.recv_match(
            type=["LOCAL_POSITION_NED", "GLOBAL_POSITION_INT", "STATUSTEXT"],
            blocking=True,
            timeout=0.25,
        )
        if msg is not None:
            if msg.get_type() == "LOCAL_POSITION_NED":
                local_z = float(msg.z)
            elif msg.get_type() == "GLOBAL_POSITION_INT":
                relative_alt_m = float(msg.relative_alt) / 1000.0
            elif msg.get_type() == "STATUSTEXT":
                text = str(msg.text)
                if "failsafe" in text.lower():
                    raise RuntimeError(f"Failsafe before arm: {text}")

        # HIL truth publishes a globally-referenced vehicle_local_position.
        # GLOBAL_POSITION_INT.relative_alt can remain zero until PX4 establishes
        # a home position, so the control-validity gate is the fresh local z.
        valid = local_z <= -min_altitude_m
        if valid:
            if good_since is None:
                good_since = time.time()
            elif time.time() - good_since >= 1.0:
                print(
                    f"Airborne position gate passed: local_z={local_z:.2f}m "
                    f"relative_alt={relative_alt_m:.2f}m (home-relative advisory)"
                )
                return
        else:
            good_since = None
    raise RuntimeError(
        f"Airborne position gate failed: local_z={local_z:.2f}m, "
        f"relative_alt={relative_alt_m:.2f}m"
    )


def write_runtime(runtime_file, enabled):
    runtime_file.write_text(f"force_enable={1 if enabled else 0}\n", encoding="utf-8")


def message_contact_mask(msg):
    if msg.get_type() == "NAMED_VALUE_FLOAT" and msg.name.rstrip("\x00") == "TD_CNTCT":
        return int(round(float(msg.value))) & 0x3F
    if msg.get_type() == "DEBUG_KEY_VALUE" and msg.key.rstrip("\x00") == "TD_CNTCT":
        return int(round(float(msg.value))) & 0x3F
    return None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="COM5")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--duration", type=float, default=80.0)
    parser.add_argument("--land-speed", type=float, default=0.35)
    parser.add_argument("--csv", type=Path, required=True)
    parser.add_argument("--summary", type=Path, required=True)
    parser.add_argument(
        "--runtime-file",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "runtime_control.txt",
    )
    args = parser.parse_args()
    args.csv.parent.mkdir(parents=True, exist_ok=True)
    args.summary.parent.mkdir(parents=True, exist_ok=True)

    write_runtime(args.runtime_file, False)
    master = mavutil.mavlink_connection(args.port, baud=args.baud, source_system=255)
    heartbeat = master.wait_heartbeat(timeout=10)
    if heartbeat is None:
        master.close()
        raise RuntimeError(f"No heartbeat on {args.port}")
    print(
        f"Connected {args.port}: system={master.target_system} "
        f"component={master.target_component}"
    )

    original_int = {}
    applied = {}
    result = {
        "port": args.port,
        "passed": False,
        "failure": "",
        "rear_contact_seen": False,
        "all_contact_seen": False,
        "front_before_rear": False,
        "tip_90_percent_seen": False,
        "on_ground_seen": False,
        "auto_disarm_seen": False,
        "max_descent_rate_mps": None,
        "rear_descent_rate_mps": None,
        "final_contact_mask": 0,
    }

    state = {
        "armed": False,
        "mode": "UNKNOWN",
        "vtol": None,
        "landed": None,
        "rel_alt_m": math.nan,
        "local_z_m": math.nan,
        "vz_mps": math.nan,
        "roll_deg": math.nan,
        "pitch_deg": math.nan,
        "yaw_deg": math.nan,
        "contact_mask": 0,
        "servo": [math.nan] * 8,
    }
    rows = []
    started = None
    last_manual = 0.0
    rear_at = None
    all_at = None
    disarmed_at = None
    max_descent = -math.inf
    failsafe_text = []

    try:
        # Temporary link/control settings are restored in finally.  Landing
        # controller settings are intentionally retained as the tested setup.
        for name in ("COM_RC_IN_MODE", "COM_RCL_EXCEPT", "TD_BTR_DBG_FAST", "TD_FW_TKO_EN", "TD_MC_DIRECT_EN"):
            original_int[name] = get_int_param(master, name)

        intended_int = {
            "TD_TIP_GND_EN": 1,
            "TD_LAND_CTL_EN": 1,
            "TD_BTR_DBG_FAST": 0,
            "TD_FW_TKO_EN": 0,
            "TD_MC_DIRECT_EN": 0,
        }
        intended_float = {
            "TD_TIP_GND_PWM": 1900.0,
            "TD_TIP_GND_T": 0.10,
            "TD_LAND_CNT_T": 0.10,
            "COM_DISARM_LAND": 0.50,
            "MPC_LAND_SPEED": args.land_speed,
            "MPC_Z_VEL_MAX_DN": 0.50,
            "MPC_LAND_ALT1": 8.0,
            "MPC_LAND_ALT2": 4.0,
        }
        for name, value in intended_int.items():
            set_int_param_and_wait(master, name, value)
            applied[name] = value
        for name, value in intended_float.items():
            set_float_param_and_wait(master, name, value)
            applied[name] = value

        set_int_param_and_wait(master, "COM_RC_IN_MODE", 1)
        set_int_param_and_wait(master, "COM_RCL_EXCEPT", original_int["COM_RCL_EXCEPT"] | 1)

        request_message_interval(master, 0, 5)
        request_message_interval(master, 30, 20)
        request_message_interval(master, 32, 10)
        request_message_interval(master, 33, 10)
        request_message_interval(master, 36, 20)
        request_message_interval(master, 186, 20)
        request_message_interval(master, 245, 5)

        wait_airborne_position(master)
        force_disarm(master)
        time.sleep(0.5)
        select_multicopter(master)
        select_mode(master, "ALTCTL")
        force_arm(master)
        print("PX4 armed in multicopter ALTCTL; plant remains frozen")

        # With multicopter position control disabled, PX4's existing takeoff
        # state machine uses skip_takeoff=true and enters FLIGHT immediately.
        # This is the correct entry for an already-airborne HITL initial state.
        select_mode(master, "STABILIZED")
        print("PX4 entered STABILIZED for airborne takeoff-state initialization")
        stabilized_until = time.monotonic() + 1.0
        while time.monotonic() < stabilized_until:
            send_gcs_heartbeat(master)
            send_manual(master, 600)
            master.recv_match(blocking=True, timeout=0.05)
        select_mode(master, "ALTCTL")
        print("PX4 entered ALTCTL after airborne takeoff-state initialization")

        wait_hover_outputs(master)

        land_mode = select_mode(master, "LAND")
        print(f"PX4 entered {land_mode}; releasing direct-landing plant")
        write_runtime(args.runtime_file, True)

        started = time.monotonic()
        next_row = started
        while time.monotonic() - started < args.duration:
            now = time.monotonic()
            elapsed = now - started
            if now - last_manual >= 0.2:
                send_gcs_heartbeat(master)
                send_manual(master, 500)
                last_manual = now

            msg = master.recv_match(
                type=[
                    "HEARTBEAT",
                    "EXTENDED_SYS_STATE",
                    "GLOBAL_POSITION_INT",
                    "LOCAL_POSITION_NED",
                    "ATTITUDE",
                    "SERVO_OUTPUT_RAW",
                    "NAMED_VALUE_FLOAT",
                    "DEBUG_KEY_VALUE",
                    "STATUSTEXT",
                ],
                blocking=True,
                timeout=0.05,
            )
            if msg is not None:
                kind = msg.get_type()
                if kind == "HEARTBEAT":
                    state["armed"] = bool(
                        msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                    )
                    state["mode"] = mavutil.mode_string_v10(msg)
                elif kind == "EXTENDED_SYS_STATE":
                    state["vtol"] = int(msg.vtol_state)
                    state["landed"] = int(msg.landed_state)
                    if state["landed"] == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND:
                        result["on_ground_seen"] = True
                elif kind == "GLOBAL_POSITION_INT":
                    state["rel_alt_m"] = float(msg.relative_alt) / 1000.0
                    state["vz_mps"] = float(msg.vz) / 100.0
                    max_descent = max(max_descent, state["vz_mps"])
                elif kind == "LOCAL_POSITION_NED":
                    state["local_z_m"] = float(msg.z)
                elif kind == "ATTITUDE":
                    state["roll_deg"] = math.degrees(float(msg.roll))
                    state["pitch_deg"] = math.degrees(float(msg.pitch))
                    state["yaw_deg"] = math.degrees(float(msg.yaw))
                elif kind == "SERVO_OUTPUT_RAW":
                    state["servo"] = [float(getattr(msg, f"servo{i}_raw")) for i in range(1, 9)]
                elif kind in ("NAMED_VALUE_FLOAT", "DEBUG_KEY_VALUE"):
                    mask = message_contact_mask(msg)
                    if mask is not None:
                        state["contact_mask"] = mask
                elif kind == "STATUSTEXT":
                    text = str(msg.text)
                    print(f"STATUSTEXT: {text}")
                    lower = text.lower()
                    if "failsafe" in lower and not any(
                        phrase in lower for phrase in ("failsafe disabled", "failsafe mode deactivated")
                    ):
                        failsafe_text.append(text)
                        raise RuntimeError(f"Failsafe during landing: {text}")

            mask = state["contact_mask"]
            rear_complete = (mask & 0x38) == 0x38
            any_front = (mask & 0x07) != 0
            all_complete = mask == 0x3F
            if any_front and rear_at is None:
                result["front_before_rear"] = True
            if rear_complete and rear_at is None:
                rear_at = elapsed
                result["rear_contact_seen"] = True
                result["rear_descent_rate_mps"] = state["vz_mps"]
                print(
                    f"Rear 3/3 seen at {elapsed:.2f}s: vz={state['vz_mps']:.3f}m/s "
                    f"MAIN7/8={state['servo'][6]:.0f}/{state['servo'][7]:.0f}"
                )
            if rear_at is not None and all(
                math.isfinite(value) and 1880 <= value <= 1920
                for value in state["servo"][6:8]
            ):
                result["tip_90_percent_seen"] = True
            if all_complete and all_at is None:
                all_at = elapsed
                result["all_contact_seen"] = True
                print(f"All 6/6 contacts seen at {elapsed:.2f}s")
            if all_at is not None and not state["armed"]:
                if disarmed_at is None:
                    disarmed_at = elapsed
                    result["auto_disarm_seen"] = True
                    print(f"Automatic disarm seen at {elapsed:.2f}s")
                if elapsed - disarmed_at >= 0.5:
                    break

            if now >= next_row:
                rows.append(
                    [
                        elapsed,
                        int(state["armed"]),
                        state["mode"],
                        state["vtol"],
                        state["landed"],
                        state["rel_alt_m"],
                        state["local_z_m"],
                        state["vz_mps"],
                        state["roll_deg"],
                        state["pitch_deg"],
                        state["yaw_deg"],
                        mask,
                        *state["servo"],
                    ]
                )
                next_row = now + 0.1

        result["max_descent_rate_mps"] = max_descent if math.isfinite(max_descent) else None
        result["final_contact_mask"] = state["contact_mask"]
        issues = []
        if not result["rear_contact_seen"]:
            issues.append("rear 3/3 contact was not observed on MAVLink")
        if result["front_before_rear"]:
            issues.append("a front contact was observed before rear 3/3")
        if not result["tip_90_percent_seen"]:
            issues.append("MAIN7/8 did not both hold 1900 PWM after rear contact")
        if not result["all_contact_seen"]:
            issues.append("all 6/6 contact was not observed on MAVLink")
        if not result["on_ground_seen"]:
            issues.append("PX4 never reported MAV_LANDED_STATE_ON_GROUND")
        if not result["auto_disarm_seen"]:
            issues.append("PX4 did not automatically disarm after all contacts")
        if result["rear_descent_rate_mps"] is not None and result["rear_descent_rate_mps"] > 0.8:
            issues.append(
                f"rear-contact descent rate {result['rear_descent_rate_mps']:.2f} m/s exceeded 0.8 m/s"
            )
        if failsafe_text:
            issues.append("failsafe STATUSTEXT: " + " | ".join(failsafe_text))
        result["failure"] = "; ".join(issues)
        result["passed"] = not issues
        if issues:
            raise RuntimeError(result["failure"])
        print("Direct six-contact landing passed")
    except Exception as exc:
        if not result["failure"]:
            result["failure"] = str(exc)
        raise
    finally:
        write_runtime(args.runtime_file, False)
        try:
            force_disarm(master)
            send_manual(master, 0)
            time.sleep(0.5)
            for name, value in original_int.items():
                set_int_param(master, name, value)
            time.sleep(0.5)
        finally:
            master.close()

        with args.csv.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(
                [
                    "elapsed_s",
                    "armed",
                    "mode",
                    "vtol_state",
                    "landed_state",
                    "relative_alt_m",
                    "local_z_m",
                    "vz_down_mps",
                    "roll_deg",
                    "pitch_deg",
                    "yaw_deg",
                    "contact_mask",
                    *[f"main{i}_pwm" for i in range(1, 9)],
                ]
            )
            writer.writerows(rows)
        result["applied_parameters"] = applied
        result["restored_temporary_parameters"] = original_int
        result["csv"] = str(args.csv)
        args.summary.write_text(
            json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8"
        )
        print(f"Plant frozen; PX4 force-disarmed; evidence saved to {args.summary}")


if __name__ == "__main__":
    main()
