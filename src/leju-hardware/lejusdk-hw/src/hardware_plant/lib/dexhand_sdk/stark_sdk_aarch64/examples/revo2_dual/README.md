# revo2_dual — Revo2 双手经典 CAN 测试（DexHandBase 最小实现）

用参考仓库 `kuavo-ros-control` 的架构，单进程同时测左手（bcan2）+ 右手（bcan3）。

## 架构

```
 stark-sdk (v1.1.9)
   ├─ init_logging(INFO)
   ├─ set_can_tx_callback ──┐ 按 slave_id 路由
   ├─ set_can_rx_callback ──┤  0x01 → bcan2, 0x02 → bcan3
   └─ init_device_handler(CAN, 0) × 2（每手一个 DeviceHandler）
                              │
                              ▼
 canbus_sdk (SocketCAN 后端)
   ├─ openCanBus("bcan2", SOCKETCAN, 1Mbps classic)
   ├─ openCanBus("bcan3", SOCKETCAN, 1Mbps classic)
   └─ registerDevice({L:bcan2, R:bcan3}, matcher by CAN ID bits[10:7])
                              │ on_rx_{left,right}()
                              ▼
                        per-hand CanMessageQueue
                              │ stark_rx_cb 按 slave_id 取帧
                              ▼
                         (Stark SDK 解码)
                              │
                              ▼
              Revo2CanHand::setFingerPositions(0~100)
                              │ ×10 缩放
                              ▼
                 stark_set_finger_positions(0~1000)
```

## 文件

| 文件 | 作用 |
|---|---|
| `revo2_can_hand.{h,cpp}` | `DexHandBase` 最小子类。0-100 ↔ 0-1000 缩放，实现 `setFingerPositions`/`getDeviceInfo`/`getFingerStatus`/`setFingerSpeeds`。其余接口 stub |
| `dual_hand_test.cpp` | main：打开 bcan2/bcan3 + 注册回调 + 双手 0-100 sweep |
| `CMakeLists.txt` | 链 `stark_sdk_aarch64` + 仓库已构建的 `libcanbus_sdk.so` |

## 前置要求

1. **已构建过整个 workspace 一次**（生成 `libcanbus_sdk.so`）：
   ```bash
   cd <repo_root>
   # 按项目的标准流程构建（colcon / catkin / cmake — 项目约定的那种）
   ```
   构建脚本会在 `build/…/canbus_sdk/libcanbus_sdk.so` 放好产物。本 demo 的 CMake 会自动找。

2. **bcan2 / bcan3 已 UP**（假设板子开机自动配置，不用手动起）。若需确认：
   ```bash
   ip -details link show bcan2 | grep -E 'state|bitrate'
   ip -details link show bcan3 | grep -E 'state|bitrate'
   # 需要：state UP, bitrate 1000000
   ```

## 编译 + 运行

```bash
cd <sdk_root>/stark_sdk_aarch64/examples/revo2_dual
mkdir build && cd build
cmake ..
make -j

# rpath 已嵌好，直接跑
./revo2_dual_hand_test
```

如果 `cmake` 提示 `libcanbus_sdk.so not found`，说明 workspace 还没构建过。可以手动指路径：
```bash
cmake .. -DCANBUS_SDK_LIB=<repo_root>/build/.../canbus_sdk/libcanbus_sdk.so
```

## 预期行为

- 终端打印：
  ```
  [Revo2CanHand] slave=0x1 bus=bcan2 ready
  [Revo2CanHand] slave=0x2 bus=bcan3 ready
  [left] connected:
  Sku Type: 2       # SKU_TYPE_MEDIUM_LEFT
  Serial Number: ...
  Firmware Version: ...
  [right] connected:
  Sku Type: 1       # SKU_TYPE_MEDIUM_RIGHT
  ...
  [main] starting 0-100 sweep (Ctrl-C to stop)
  ```

- **左右手同步做 0→100→0 开合循环**，大拇指（finger 0/1）幅度打了 40% 折扣，其他 4 指 100%（首次上机保守）
- Ctrl-C 后正常退出

## 关键参数

| 参数 | 值 | 依据 |
|---|---|---|
| 比特率 | 1 Mbps 经典 CAN（`dbitrate=0`）| 参考仓库 `dual_revo2_can_customed.cpp:111-116` |
| slave_id | L=0x01, R=0x02 | 同上第 33-34 行 |
| 上层位置范围 | 0-100（`DexHandBase` 契约）| `dexhand_base.h:181` |
| 底层位置范围 | 0-1000（v1.1.9 统一）| `stark-sdk.h:1505`（Unified range 0~1000）|
| 刷新率 | 50 Hz（20ms）| demo 保守值，生产 200 Hz |

## 验证 Revo2 CAN ID 与手臂电机不冲突

| 设备（bcan2/bcan3）| CAN ID 范围 |
|---|---|
| 手臂电机 RUIWO | `0x01 ~ 0x0A` |
| Revo2 左手（slave=1）| `0x080 ~ 0x0FF` |
| Revo2 右手（slave=2）| `0x100 ~ 0x17F` |

**数字空间零重叠**，互不干扰。

## 不实现但日后会补

- 手势 `runActionSequence` / `setActionSequence`
- `setGripForce` / `setTurboModeEnabled`
- `getFingerStatus` 走推送模式（当前是 poll `stark_get_motor_status`）
- 心跳监控 / 断连重连

## 调参

- **改刷新率**：`dual_hand_test.cpp` 最后的 `std::this_thread::sleep_for(...)`
- **改位置范围**：主循环里的 `p <= 100` 改 `p <= 50`（半幅测）
- **只测左手**：屏蔽右手的 `setFingerPositions` 调用即可（不影响右手 slave 的 handler 创建）
- **加一只手**：在 `dual_hand_test.cpp` 顶上的 `kLhandId`/`kLhandBus` 等常量增加一组，`stark_tx_cb`/`stark_rx_cb` 里加 `else if`
