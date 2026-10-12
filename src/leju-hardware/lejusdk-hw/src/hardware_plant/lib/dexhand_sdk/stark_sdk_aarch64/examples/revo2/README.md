# Revo2 独立测试 demo（经典 CAN）

用于在不依赖上层 ROS / `hardware_plant` 的前提下，直接对 Revo2 灵巧手做端到端验证。

## 文件

| 文件 | 用途 |
|---|---|
| `revo2_can_ctrl.cpp` | 主测：0→1000 循环位置，6 指打通 |
| `Makefile` | 按 `uname -m` 自动链对应架构的 `libbc_stark_sdk.so` |

所有 helper（`stark_common.*`, `can_common.*`）在 `../common/`。头文件（`stark-sdk.h`、`zlgcan/*.h`）在 `../../dist/include/`。

## 前置要求

### aarch64 板端（RK3588 等）

- CAN 接口起来：
  ```bash
  sudo ip link set can0 up type can bitrate 1000000
  ip link show can0   # state UP
  ```
- `glibc ≥ 2.31`, `libudev.so` 存在（见上游 README）。

### x86_64 开发机（用 USB-CANFD 转换器）

- ZLG USBCANFD 系列（200U/100U/100U-mini）：
  - 需要 ZLG 提供的 `libusbcanfd.so`（手动放 `/usr/local/lib/` 或 `LD_LIBRARY_PATH`）。
  - 设置 `STARK_CAN_BACKEND=zlg`。
- 或带 SocketCAN 的 USB-CAN，同 aarch64 用法。

## 编译 + 运行

```bash
# 1. 最简路径：SocketCAN + can0，右手（slave_id=2）
make run_revo2_can_ctrl

# 2. 显式指定 iface
STARK_SOCKETCAN_IFACE=can1 make run_revo2_can_ctrl

# 3. 用 ZLG USB-CANFD
STARK_CAN_BACKEND=zlg make run_revo2_can_ctrl

# 4. 仅编译，不跑
make
./revo2_can_ctrl.exe
```

## Slave ID

源码 `revo2_can_ctrl.cpp:36` 硬编码：

```cpp
uint8_t slave_id = 2;   // 1 = 左手, 2 = 右手
```

测左手把 2 改成 1 再 `make`。

## 验证清单

- [ ] `make` 成功，输出 `[Built] revo2_can_ctrl.exe (arch=...)`
- [ ] 运行后终端打印 `DeviceInfo` / `sku_type` / `firmware_version` 等
- [ ] 六指从 0 到 1000 来回往复，手指可见运动
- [ ] `candump can0` 能看到连续的 CAN 帧

## 常见问题

**`Prebuilt library not found at .../aarch64/libbc_stark_sdk.so`**
→ 你在 aarch64 上但没有 `.so`。仓库已带了，检查 `git status`；若确实缺，跑 `../../download-lib.sh`。

**`cannot open can interface: can0`**
→ `sudo ip link set can0 up type can bitrate 1000000`，再 `ip link show can0` 验证 UP。

**手不动、也没帧**
→ 确认 slave_id 和物理硬件一致；确认 bitrate 和手方配置一致（Revo2 默认 1 Mbps 经典 CAN）。

**`libbc_stark_sdk.so: cannot open shared object`**
→ Makefile 已加 rpath，但如果 `.exe` 被移动位置，用 `LD_LIBRARY_PATH=.../dist/shared/linux/<arch> ./revo2_can_ctrl.exe`。
