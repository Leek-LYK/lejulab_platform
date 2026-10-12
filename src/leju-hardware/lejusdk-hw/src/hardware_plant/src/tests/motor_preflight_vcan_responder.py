#!/usr/bin/env python3
"""vcan 上模拟电机参数协议的 responder (配合 motor_preflight_vcan_test.sh)。

模拟 0x67 参数协议:
  读帧 67 <idx> 00 00 00 00 04 76 → 回 <motor_id> <idx> <值 4B LE> 00 76
  写帧 67 <idx> <4B LE> 15 76     → 更新内存值, 不回 ack

用法: motor_preflight_vcan_responder.py <iface> [--ignore-writes] <motor_id:fw_hex> [...]
例:   motor_preflight_vcan_responder.py vcan0 1:0x02030001 2:0x02030001
未在列表中的 motor_id 不响应 (模拟无响应电机)。
--ignore-writes: 所有电机忽略写帧 (模拟参数写不生效, 回读为旧值)。
"""
import socket
import struct
import sys

CAN_FRAME_FMT = "<IB3x8s"
PARAM_ID_BASE = 0x600
FW_IDX = 10


def main() -> None:
    if len(sys.argv) < 3:
        sys.exit(__doc__)
    iface = sys.argv[1]
    args = sys.argv[2:]
    ignore_writes = "--ignore-writes" in args

    # motors[motor_id] = {idx: raw_le_uint32}
    motors: dict[int, dict[int, int]] = {}
    for spec in args:
        if spec.startswith("--"):
            continue
        mid_str, fw_str = spec.split(":")
        motors[int(mid_str, 0)] = {FW_IDX: int(fw_str, 0)}

    s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    s.bind((iface,))
    print(f"[responder] {iface} motors={ {k: hex(v[FW_IDX]) for k, v in motors.items()} }",
          flush=True)

    while True:
        frame, _ = s.recvfrom(16)
        can_id, dlc, data = struct.unpack(CAN_FRAME_FMT, frame)
        can_id &= socket.CAN_EFF_MASK
        motor_id = can_id - PARAM_ID_BASE
        if motor_id not in motors or dlc < 8 or data[0] != 0x67:
            continue
        idx = data[1]
        params = motors[motor_id]
        if data[6] == 0x15:  # 写帧: 更新存储 (固件版本只读, 不允许改)
            if idx != FW_IDX and not ignore_writes:
                params[idx] = struct.unpack("<I", data[2:6])[0]
        elif data[6] == 0x04:  # 读帧: 回响应
            value = params.get(idx, 0)
            payload = bytes([motor_id, idx]) + struct.pack("<I", value) + bytes([0x00, 0x76])
            s.send(struct.pack(CAN_FRAME_FMT, can_id, 8, payload))


if __name__ == "__main__":
    main()
