"""Download the latest PX4 ULog over a MAVLink serial connection."""

from __future__ import annotations

import argparse
import time
from pathlib import Path

from pymavlink import mavutil


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="COM5")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--log-id", type=int)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    link = mavutil.mavlink_connection(args.port, baud=args.baud)
    link.wait_heartbeat(timeout=10)
    first_id = 0 if args.log_id is None else args.log_id
    last_id = 0xFFFF if args.log_id is None else args.log_id
    link.mav.log_request_list_send(
        link.target_system, link.target_component, first_id, last_id
    )

    entries = {}
    deadline = time.time() + 20
    while time.time() < deadline:
        msg = link.recv_match(type="LOG_ENTRY", blocking=True, timeout=0.5)
        if msg is None:
            continue
        entries[int(msg.id)] = msg
        if args.log_id is not None or len(entries) >= int(msg.num_logs):
            break

    if not entries:
        raise RuntimeError("PX4 returned no LOG_ENTRY messages")

    latest_id = max(entries) if args.log_id is None else args.log_id
    if latest_id not in entries:
        raise RuntimeError(f"PX4 returned no LOG_ENTRY for id {latest_id}")
    expected_size = int(entries[latest_id].size)
    data = bytearray(expected_size)
    received = bytearray(expected_size)
    offset = 0
    last_heartbeat = 0.0
    stalled_attempts = 0

    while offset < expected_size:
        count = min(45_000, expected_size - offset)
        link.mav.log_request_data_send(
            link.target_system, link.target_component, latest_id, offset, count
        )
        chunk_deadline = time.time() + 20
        while time.time() < chunk_deadline:
            if time.time() - last_heartbeat >= 0.5:
                link.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0,
                    0,
                    mavutil.mavlink.MAV_STATE_ACTIVE,
                )
                last_heartbeat = time.time()
            msg = link.recv_match(type="LOG_DATA", blocking=True, timeout=0.5)
            if msg is None or int(msg.id) != latest_id:
                continue
            start = int(msg.ofs)
            payload = bytes(msg.data[: int(msg.count)])
            end = min(start + len(payload), expected_size)
            data[start:end] = payload[: end - start]
            received[start:end] = b"\x01" * (end - start)
            if start <= offset < end:
                while offset < expected_size and received[offset]:
                    offset += 1
            if offset >= expected_size or offset >= start + count:
                stalled_attempts = 0
                break
        else:
            stalled_attempts += 1
            if stalled_attempts >= 8:
                raise RuntimeError(f"Log download stalled at {offset}/{expected_size} bytes")
            print(f"retrying from {offset}/{expected_size} bytes")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_bytes(data)
    link.mav.log_request_end_send(link.target_system, link.target_component)
    link.close()
    print(f"downloaded log id={latest_id} bytes={expected_size} to {args.output}")


if __name__ == "__main__":
    main()
