# stark_sdk_aarch64 — BrainCo Revo2 SDK (v1.1.9+)

新版 BrainCo `bc-stark-sdk` (v1.1.9)，用于接入 **Revo2 灵巧手（经典 CAN）**。

> 目录名以主部署目标命名（aarch64 板子），但实际同时携带 `x86_64` 和 `aarch64` 两份 `.so`。编译/链接时按 `CMAKE_SYSTEM_PROCESSOR` 自动选对应架构。

与老的 `modbus_sdk/` (v0.9.1) **并存**，互不影响。老路径继续服务既有的 `Revo2Dexhand`（Modbus 串口）/ `ProtobufDexhand`（qiangnao）等实现；本目录仅供新的 Revo2 经典 CAN 适配层使用。

> 运行时注意：`modbus_sdk` 与 `stark_sdk_aarch64` 的 `libbc_stark_sdk.so` **符号同名**。同一个可执行文件只能加载一份。新/旧适配层不要在同一个进程里并存。

## 目录布局

```
stark_sdk_aarch64/
├── dist/
│   ├── include/
│   │   ├── stark-sdk.h              # v1.1.9 API，两架构共用
│   │   └── zlgcan/                  # v1.1.9 新增（ZLG USB-CAN/CANFD 头）
│   └── shared/
│       └── linux/
│           ├── x86_64/libbc_stark_sdk.so     # 开发机用
│           └── aarch64/libbc_stark_sdk.so    # RK3588 / 板端用
├── examples/                         # 独立可编译、可运行的 SDK demo
│   ├── common/                       # 官方 helper（signal、CAN 初始化等）
│   └── revo2/
│       ├── revo2_can_ctrl.cpp        # 经典 CAN 主测 demo
│       ├── Makefile                  # 独立 Makefile（自动按 arch 选 .so）
│       └── README.md
├── CMakeLists.txt                    # IMPORTED 目标 `stark_sdk_aarch64`（按 arch 自动选 .so）
├── download-lib.sh                   # 智能下载/更新脚本
├── VERSION
├── .gitignore
└── README.md
```

## 快速验证灵巧手（不依赖 ROS / 其他仓库）

```bash
cd examples/revo2

# 1. 默认走 SocketCAN，can0 已起来（ip link show can0 为 UP）
export STARK_CAN_BACKEND=socketcan
export STARK_SOCKETCAN_IFACE=can0

# 2. 编译 + 运行（自动按 uname -m 选对应架构的 .so）
make run_revo2_can_ctrl
# 默认 slave_id=2（右手），左手修改源码 slave_id=1
```

更多细节见 `examples/revo2/README.md`。

## 作为 CMake 子模块使用（集成到上层项目）

```cmake
add_subdirectory(path/to/stark_sdk_aarch64)

add_executable(your_target your_code.cpp)
target_link_libraries(your_target PRIVATE stark_sdk_aarch64)

# 可选：安装到发行
install(FILES ${STARK_SDK_LIB} DESTINATION lib)
install(DIRECTORY ${STARK_SDK_INCLUDE}/ DESTINATION include/stark_sdk_aarch64)
```

架构由 `CMAKE_SYSTEM_PROCESSOR` 自动决定：
- `aarch64|arm64|ARM64` → 链接 `dist/shared/linux/aarch64/libbc_stark_sdk.so`
- `x86_64|AMD64|amd64`  → 链接 `dist/shared/linux/x86_64/libbc_stark_sdk.so`

消费端 rpath 建议：

```cmake
set_target_properties(your_target PROPERTIES
    BUILD_RPATH "${STARK_SDK_LIB_DIR}"
    INSTALL_RPATH "$ORIGIN/../lib"
)
```

或直接通过 `LD_LIBRARY_PATH` 指向 `stark_sdk_aarch64/dist/shared/linux/<arch>`。

## download-lib.sh 用法

```bash
# 默认：按本机 uname -m 下载（幂等）
./download-lib.sh

# 交叉下载：为另一种架构准备 .so（提交进仓库用）
STARK_TARGET_ARCH=aarch64 ./download-lib.sh
STARK_TARGET_ARCH=x86_64  ./download-lib.sh
STARK_TARGET_ARCH=all     ./download-lib.sh   # 两架构都拉
```

产物落到 `dist/shared/linux/<arch>/libbc_stark_sdk.so`，头文件放在 `dist/include/`（两架构共用）。已经记录在 `VERSION` 的 `(arch, version)` 组合会跳过。

## 版本与来源

- 上游：`https://github.com/BrainCoTech/stark-serialport-example`（`download-lib.sh` 拉 `app.brainco.cn/universal/bc-stark-sdk/libs/v1.1.9`）
- 两架构 `.so` 都已随仓库提交（v1.1.9，2026-04-20）

## 下一步

本目录仅提供 SDK 素材 + CMake 目标 + 独立 demo。**适配层代码（接入 `DexHandBase` 接口）尚未编写**，会在后续放到 `lib/dexhand_sdk/` 相应位置（与现有 `Revo2CanDexhand` 并存，或提交新实现）。
