# S17 上楼梯仿真：三窗口启动说明

跑新的上楼梯模型需要**同时开 3 个终端**，分别启动：

1. MuJoCo 仿真（楼梯场景 + 深度相机）
2. RL 深度行走控制器
3. 虚拟手柄

三者通过 DDS 通信，不依赖 ROS。建议按下面顺序启动；工作目录均为仓库根目录。

## 编译方式

本仓库不是标准 `catkin_make` 工作空间，使用根目录统一 CMakeLists 配置。两种
构建方式不要混用同一组启动命令。

### Catkin 模式（ROS Noetic，输出到 `devel/`）

```bash
cd /root/low/lejulab_platform
source /opt/ros/noetic/setup.zsh

cmake -S . -B build_catkin \
  -DCMAKE_BUILD_TYPE=Release \
  -DFORCE_PURE_CMAKE=OFF \
  -DCATKIN_DEVEL_PREFIX=/root/low/lejulab_platform/devel \
  -DBUILD_VR_CONTROL=OFF \
  -DBUILD_TESTS=OFF \
  -DBUILD_TESTING=OFF
cmake --build build_catkin --target leju-mujoco-sim run_rl_controller -j2
source devel/setup.zsh
```

Catkin 启动文件为 `devel/lib/...`。不要把 Catkin 程序和
`build_cmake/src/...` 或 `build/src/...` 的纯 CMake 程序混用。

### 纯 CMake 模式（不使用 Catkin）

```bash
cd /root/low/lejulab_platform
source /opt/ros/noetic/setup.zsh

cmake -S . -B build_cmake \
  -DFORCE_PURE_CMAKE=ON \
  -DCMAKE_BUILD_TYPE=Release \
  -DENABLE_PINOCCHIO=OFF \
  -DBUILD_MUJOCO_SIM=ON
cmake --build build_cmake --target leju-mujoco-sim run_rl_controller -j2
```

纯 CMake 启动文件为 `build_cmake/src/...`，无需 `source devel/setup.zsh`。

## 两窗口最小启动（Catkin）

如果使用实体手柄或已有 `/rt/joy` 发布器，只需要两个窗口。先编译完成，再分别
执行以下命令。

### 窗口 1：MuJoCo 图形仿真

```bash
cd /root/low/lejulab_platform
source /opt/ros/noetic/setup.zsh
source devel/setup.zsh

ROBOT_VERSION=17 DISPLAY=:1 XDG_RUNTIME_DIR=/tmp \
  devel/lib/leju-mujoco-sim/leju-mujoco-sim \
  --show-depth --dump-depth \
  src/leju-mujoco-sim/model/biped_s17/xml/scene_stair.xml
```

### 窗口 2：RL 控制器

```bash
cd /root/low/lejulab_platform
source /opt/ros/noetic/setup.zsh
source devel/setup.zsh

ROBOT_VERSION=17 \
  devel/lib/leju-rl-controller/run_rl_controller \
  -c src/leju-controllers/leju-rl-controller/config/17/controller_manager_depth_sim.yaml
```

控制器出现“机器人就绪”后，先点击 MuJoCo 的 `Run`，再按手柄 `Start`。没有实体
手柄时，另开第三个窗口运行本 README 后面的虚拟手柄程序。

**环境初始化（终端 2、3 需要）：** Catkin 模式请 source **`devel/setup.zsh`**；纯 CMake 模式无需 source 工作空间 setup。zsh 用 `setup.zsh`，bash 用 `setup.bash`。

```bash
cd /root/low/lejulab_platform
```

---

## 终端 1：MuJoCo 仿真

```bash
ROBOT_VERSION=17 DISPLAY=:1 XDG_RUNTIME_DIR=/tmp devel/lib/leju-mujoco-sim/leju-mujoco-sim --show-depth --dump-depth src/leju-mujoco-sim/model/biped_s17/xml/scene_stair.xml
```

> 若使用 `\` 换行，**反斜杠必须是该行最后一个字符**，后面不能有空格，否则 zsh 会把下一行当成独立命令。

| 参数 / 环境变量 | 含义 |
| --- | --- |
| `ROBOT_VERSION=17` | 机器人型号版本，S17。仿真、控制器、手柄必须一致。 |
| `DISPLAY=:1` | 指定图形显示到 `:1`（常见于容器/远程桌面）。MuJoCo 窗口需要它。 |
| `XDG_RUNTIME_DIR=/tmp` | 运行时目录；部分 GUI/会话组件依赖它，避免权限或路径问题。 |
| `leju-mujoco-sim` | MuJoCo 仿真主程序，发布关节状态/IMU，接收控制指令。 |
| `--show-depth` | 弹出深度预览窗口（`DDS Head Depth`），方便确认头部深度图是否正常。不影响发给控制器的 DDS 数据。 |
| `--dump-depth` | 把深度图周期性写到 `/tmp/leju_depth_camera.pgm`，并在终端打印 min/max/mean，便于调试。 |
| `scene_stair.xml` | 楼梯场景 MJCF 模型路径（位置参数，必须放在最后）。 |

默认会开启 MuJoCo 深度相机，并经 DDS 话题 `rt/depth_camera/frame_meters_36x64` 发布 `36×64` 米制深度（共 2304 个数）。

其他可选参数（本命令未用到）：

| 参数 | 含义 |
| --- | --- |
| `--headless` | 无界面物理循环，不需要 OpenGL/GLFW。 |
| `--run-mujoco-camera` / `--no-mujoco-camera` | 显式打开 / 关闭深度相机（默认打开）。 |
| `--depth-frequency Hz` | 深度发布频率，默认约 30Hz。 |

---

## 终端 2：深度行走控制器

```bash
source /opt/ros/noetic/setup.zsh
source devel/setup.zsh

ROBOT_VERSION=17 devel/lib/leju-rl-controller/run_rl_controller -c src/leju-controllers/leju-rl-controller/config/17/controller_manager_depth_sim.yaml
```

| 参数 / 环境变量 | 含义 |
| --- | --- |
| `ROBOT_VERSION=17` | 与仿真一致，选择 S17 电机/关节布局。 |
| `run_rl_controller` | RL 控制主程序：读状态、跑策略、发关节指令。 |
| `-c` / `--config` | **必填**。控制器管理配置文件路径。 |
| `controller_manager_depth_sim.yaml` | 仿真用深度行走配置；默认控制器为 `depth_walk`（订阅深度图做上楼梯/深度行走策略）。同文件里也配置了 `amp`，可用 CLI 切换。 |

控制器可选参数：

| 参数 | 含义 |
| --- | --- |
| `-u` / `--urdf-path` | URDF 路径（手臂重力补偿等）。一般可省略。 |
| `-t` / `--teleop-config` | 遥操作按键映射 YAML；默认取配置目录下的 `teleop_bindings.yaml`。 |

启动后等到日志提示机器人就绪，再在手柄窗口按 `O` 或 `F` 发送 Start。

---

## 终端 3：虚拟手柄

```bash
export ROBOT_VERSION=17
export DISPLAY=:1
export XDG_RUNTIME_DIR=/tmp

devel/lib/leju-joystick/leju-joystick
```

| 参数 / 环境变量 | 含义 |
| --- | --- |
| `ROBOT_VERSION=17` | 与仿真、控制器一致。 |
| `DISPLAY=:1` | 手柄 GUI 窗口显示到哪个 X 显示器。 |
| `XDG_RUNTIME_DIR=/tmp` | 同仿真，保证 GUI 运行时路径可用。 |
| `leju-joystick` | 虚拟手柄程序，通过 DDS 发布 `/rt/joy`。 |

必须前台运行才能稳定看到窗口。若已有旧进程：

```bash
pkill -x leju-joystick 2>/dev/null || true
```

窗口出现后**先点击窗口获得焦点**，再操作：

| 按键 | 功能 |
| --- | --- |
| `O` / `F` | Start |
| `B` | Back / 停止 |
| `W/S` | 左摇杆前后 |
| `A/D` | 左摇杆左右 |
| `I/K` | 右摇杆上下 |
| `J/L` | 右摇杆左右 |
| `C` | South / A |
| `T` | East / B |
| `R` | North / Y |
| `P` | 左肩键 LB |
| `Space` | 摇杆归零 |

---

## 推荐启动顺序

1. 终端 1：MuJoCo（确认楼梯场景和深度预览正常）
2. 终端 2：`run_rl_controller`（等待就绪）
3. 终端 3：`leju-joystick` → 点击窗口 → `O`/`F` Start

## 常用检查

```bash
ps -ef | grep -E 'leju-mujoco-sim|run_rl_controller|leju-joystick'
ls -l /tmp/leju_depth_camera.pgm   # 开了 --dump-depth 时
```

## 控制器切换（可选）

```bash
ROBOT_VERSION=17 devel/lib/leju-rl-controller/controller_switch_cli depth_walk

ROBOT_VERSION=17 devel/lib/leju-rl-controller/controller_switch_cli amp
```

名称映射：`depth_loco_controller -> depth_walk`，`amp_controller -> amp`。  
若 DDS RPC 提示 `service not available`，用对应 yaml 直接重启控制器即可。
