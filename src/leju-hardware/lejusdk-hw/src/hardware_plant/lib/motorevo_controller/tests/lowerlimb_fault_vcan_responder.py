#!/usr/bin/env python3
"""CAN FD motor responder for lower-limb health tests on a vcan interface.

The responder follows control frames on CAN ID 0x10 so driver initialization is
realistic: FD produces status 0x0000 and FC produces status 0x0001.  After all
configured motors have been enabled and healthy for ``healthy_sec``, it injects
one selected scenario without touching the controller's software operation
state.

Usage:
  lowerlimb_fault_vcan_responder.py IFACE IDS TARGET SCENARIO [HEALTHY_SEC]

Scenarios: healthy, fault, disabled, single_timeout, bus_timeout
"""

from __future__ import annotations

import select
import socket
import struct
import sys
import time

CANFD_FRAME_FMT = "<IBB2x64s"
CANFD_FRAME_SIZE = struct.calcsize(CANFD_FRAME_FMT)
CANFD_BRS = 0x01
CAN_SFF_MASK = 0x7FF

CAN_ID_CONTROL = 0x10
CMD_CLEAR_ERROR = 0xFB
CMD_ENABLE = 0xFC
CMD_DISABLE = 0xFD

STATUS_ENABLED = 0x0001
STATUS_OVER_CURRENT = 0x0004
VALID_SCENARIOS = {"healthy", "fault", "disabled", "single_timeout", "bus_timeout"}


def feedback_payload(status: int) -> bytes:
    payload = bytearray(8)
    payload[0], payload[1] = 0x7F, 0xFF
    payload[2] = 0x7F
    payload[3] = 0xF7
    payload[4] = 0xFF
    payload[5] = 80
    payload[6] = (status >> 8) & 0xFF
    payload[7] = status & 0xFF
    return bytes(payload)


def receive_control_frames(sock: socket.socket, motor_ids: list[int], enabled: dict[int, bool]) -> None:
    while True:
        readable, _, _ = select.select([sock], [], [], 0)
        if not readable:
            return
        raw = sock.recv(CANFD_FRAME_SIZE)
        can_id, length, _flags, payload = struct.unpack(CANFD_FRAME_FMT, raw)
        if (can_id & CAN_SFF_MASK) != CAN_ID_CONTROL or length < 8:
            continue
        for motor_id in motor_ids:
            offset = (motor_id - 1) * 8
            if offset + 8 > length:
                continue
            slot = payload[offset:offset + 8]
            if slot[:7] != b"\xFF" * 7:
                continue
            command = slot[7]
            if command == CMD_ENABLE:
                enabled[motor_id] = True
            elif command == CMD_DISABLE:
                enabled[motor_id] = False
            elif command == CMD_CLEAR_ERROR:
                pass


def main() -> None:
    if len(sys.argv) < 5:
        sys.exit(__doc__)

    iface = sys.argv[1]
    motor_ids = [int(value, 0) for value in sys.argv[2].split(",")]
    target_id = int(sys.argv[3], 0)
    scenario = sys.argv[4]
    healthy_sec = float(sys.argv[5]) if len(sys.argv) > 5 else 0.5
    if scenario not in VALID_SCENARIOS:
        sys.exit(f"unknown scenario: {scenario}")
    if target_id not in motor_ids:
        sys.exit(f"target {target_id} is not in {motor_ids}")

    sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FD_FRAMES, 1)
    sock.bind((iface,))

    enabled = {motor_id: False for motor_id in motor_ids}
    healthy_since = None
    scenario_active = False
    print(
        f"[responder] iface={iface} motors={motor_ids} target={target_id} "
        f"scenario={scenario} healthy_sec={healthy_sec}",
        flush=True,
    )

    while True:
        receive_control_frames(sock, motor_ids, enabled)

        now = time.monotonic()
        all_enabled = all(enabled.values())
        if all_enabled:
            if healthy_since is None:
                healthy_since = now
            if scenario != "healthy" and now - healthy_since >= healthy_sec:
                if not scenario_active:
                    print(f"[responder] activating scenario={scenario}", flush=True)
                scenario_active = True
        else:
            healthy_since = None
            scenario_active = False

        for motor_id in motor_ids:
            if scenario_active and scenario == "bus_timeout":
                continue
            if scenario_active and scenario == "single_timeout" and motor_id == target_id:
                continue

            status = STATUS_ENABLED if enabled[motor_id] else 0
            if scenario_active and motor_id == target_id:
                if scenario == "disabled":
                    status = 0
                elif scenario == "fault":
                    status = STATUS_ENABLED | STATUS_OVER_CURRENT

            data = feedback_payload(status).ljust(64, b"\x00")
            frame = struct.pack(CANFD_FRAME_FMT, motor_id, 8, CANFD_BRS, data)
            sock.send(frame)

        time.sleep(0.005)


if __name__ == "__main__":
    main()
