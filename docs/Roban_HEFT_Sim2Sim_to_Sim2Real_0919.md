# Roban S17 HEFT：Sim2Sim → Sim2Real 完整实验笔记

> 适用项目：`lejulab_platform-heft-dev`  
> 机器人：Leju Roban S17 / `ROBOT_VERSION=17`  
> 控制器：HEFT (`HeftRLController`)  
> 仿真：MuJoCo  
> 真机平台：RK3588 / Ubuntu 22.04 / aarch64  
> 本笔记以本次已经完整跑通的 **Sim2Sim → 真机** 流程为准。

---

## 1. 总体原则

建议长期固定为下面这条工作流：

```text
WSL / 开发机 = 唯一源码 Source of Truth
        │
        ├── build_sim/      → x86_64 MuJoCo Sim2Sim
        │
        └── 打包源码（不带 build 目录）
                 ↓ SCP
Roban RK3588 = 编译 / 运行副本
        │
        └── build_cmake/    → aarch64 原生 Sim2Real
```

核心原则：

1. **所有源码、配置、Policy、Motion 修改都先在 WSL 完成。**
2. **先通过 Sim2Sim，再同步到 Roban。**
3. Roban 上尽量不直接修改源码，只做原生编译和真机验证。
4. Sim2Sim 与 Sim2Real 尽量使用同一份：
   - `controller_manager.yaml`
   - `config_heft.yaml`
   - HEFT ONNX Policy
   - deploy CSV Motion
   - 控制器代码
5. 真机侧仅把后端从 MuJoCo 换成真实硬件/CAN/IMU；RL Controller 本身保持一致。

---

# Part A：Sim2Sim

## 2. 关键目录

项目根目录示例：

```bash
~/projects/lejulab_platform-heft-dev
```

HEFT 相关核心文件：

```text
src/leju-controllers/leju-rl-controller/
├── config/17/
│   ├── controller_manager.yaml
│   ├── teleop_bindings.yaml
│   ├── controllers/
│   │   └── config_heft.yaml
│   ├── policy/
│   │   └── heft/
│   │       └── policy0905.onnx
│   └── motion_data/
│       ├── roban_default_stand_10s_deploy.csv
│       └── roban_key_out_HEFT/
│           └── 001_a_person_crosses_arms_and_stands_still_deploy.csv
│
├── src/
├── include/
└── 3rd_party/
```

机器人模型：

```text
src/leju_assets/models/biped_s17/
```

---

## 3. Controller Manager 配置

文件：

```text
src/leju-controllers/leju-rl-controller/config/17/controller_manager.yaml
```

当前关键逻辑：

```yaml
default_controller: "amp"
```

HEFT 注册：

```yaml
- name: "heft"
  type: "HeftRLController"
  config: "controllers/config_heft.yaml"
  enabled: true

- name: "heft_planner"
  type: "HeftRLController"
  config: "controllers/config_heft.yaml"
  enabled: true

- name: "heft_pico"
  type: "HeftRLController"
  config: "controllers/config_heft.yaml"
  enabled: true
```

三种 HEFT 模式共用 `config_heft.yaml`：

```text
heft          → CSV reference
heft_planner  → Flow Planner
heft_pico     → Pico live reference
```

普通 Motion Tracking 实验主要使用：

```text
heft → CSV
```

---

## 4. HEFT Policy 配置

文件：

```text
src/leju-controllers/leju-rl-controller/config/17/controllers/config_heft.yaml
```

当前示例：

```yaml
HumanoidRobotCfg:
  inference_engine: "onnxruntime"
  loop_dt: 0.001
  policy_path: "../policy/heft/policy0905.onnx"
```

这里的相对路径是从 `config/17/controllers/` 回到 `config/17/`：

```text
../policy/heft/policy0905.onnx
```

最终对应实际文件：

```text
src/leju-controllers/leju-rl-controller/config/17/policy/heft/policy0905.onnx
```

### 更换 HEFT Policy

例如新模型叫：

```text
policy_new.onnx
```

先放到：

```text
config/17/policy/heft/policy_new.onnx
```

再修改：

```yaml
policy_path: "../policy/heft/policy_new.onnx"
```

然后重新启动 Controller 即可。

> ONNX 文件本身属于运行资源，一般不需要因为“只换 Policy”重新编译 C++；但正式实验前仍建议重新确认文件路径和模型 I/O。

### 当前 HEFT Policy 接口约束

当前部署配置要求：

```yaml
expected_obs_dim: 1313
output_name: "action"
```

当前已验证 HEFT Policy：

```text
Input  : [1, 1313]
Output : [1, 21]
```

因此更换 Policy 时至少要确认：

```text
输入维度      = 1313
输出维度      = 21
输出名称      = action
关节顺序      = 与 config_heft.yaml 一致
动作定义      = 与训练时一致
Action Scale  = 与训练时一致
PD / direction / default_pos = 与训练配置一致
```

否则即使 ONNX 能成功加载，也可能发生严重 Sim2Real mismatch。

---

## 5. HEFT Motion 配置

`config_heft.yaml`：

```yaml
motion:
  default_motion: default_stand

  motions:
    - name: default_stand
      file: "../motion_data/roban_default_stand_10s_deploy.csv"

    - name: 001_a_person_crosses_arms_and_stands_still_deploy.csv
      file: "../motion_data/roban_key_out_HEFT/001_a_person_crosses_arms_and_stands_still_deploy.csv"
```

实际对应：

```text
config/17/motion_data/roban_default_stand_10s_deploy.csv

config/17/motion_data/roban_key_out_HEFT/
001_a_person_crosses_arms_and_stands_still_deploy.csv
```

### 当前 HEFT deploy CSV 约定

本项目当前已经验证的 HEFT deploy CSV：

```text
采样率：50 Hz
列数：49
```

基本结构：

```text
root position       3
root quaternion     4   (wxyz)
joint position     21
joint velocity     21
----------------------
total              49
```

因此从训练数据生成新的 Motion 时，需要确认：

- 50 Hz；
- root quaternion 顺序正确；
- 21 个关节的顺序与 HEFT 配置完全一致；
- 位置/速度单位一致；
- 不要直接把训练侧任意 CSV 当 deploy CSV；
- 必须先在 MuJoCo 中观察足底、root、腰部和手臂姿态是否正常。

---

## 6. Sim2Sim Docker 环境

官方项目提供：

```text
docker/build_sim_image.sh
docker/run_sim.sh
```

第一次没有镜像时可构建：

```bash
cd ~/projects/lejulab_platform-heft-dev
./docker/build_sim_image.sh
```

当前仿真镜像：

```text
lejulab-sim-x86:latest
```

进入仿真 Docker：

```bash
./docker/run_sim.sh
```

项目会被挂载到：

```text
/workspace
```

---

## 7. Sim2Sim 编译

进入 Docker 后：

```bash
cd /workspace
export ROBOT_VERSION=17
```

建议建立独立的 `build_sim/`：

```bash
cmake -S . -B build_sim -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_MUJOCO_SIM=ON \
  -DBUILD_RL_CONTROLLER=ON \
  -DBUILD_JOYSTICK=ON \
  -DBUILD_VR_CONTROL=OFF \
  -DBUILD_EXAMPLES=OFF \
  -DBUILD_TESTING=OFF
```

完整编译：

```bash
cmake --build build_sim -j$(nproc)
```

主要产物：

```text
build_sim/src/leju-mujoco-sim/leju-mujoco-sim

build_sim/src/leju-controllers/leju-rl-controller/run_rl_controller

build_sim/src/leju-joystick/leju-joystick
```

---

## 8. Sim2Sim 推荐启动方式

优先使用官方 Launcher：

```bash
cd /workspace
export ROBOT_VERSION=17
export BUILD_DIR="$PWD/build_sim"

bash src/leju_launch/scripts/launch_mujoco_sim.sh --no-shm
```

说明：

```text
--no-shm
```

适合本地开发机没有部署 iceoryx RouDi 的环境。

如果 RouDi 已正确部署，可省略。

---

## 9. Sim2Sim 手动调试方式

当需要分别排查 MuJoCo / Controller / Joystick 时，可以分终端启动。

### Terminal 1：MuJoCo

```bash
cd /workspace
export ROBOT_VERSION=17

SIM=build_sim/src/leju-mujoco-sim/leju-mujoco-sim
$SIM src/leju_assets/models/biped_s17/xml/scene.xml
```

无 GUI：

```bash
$SIM --headless src/leju_assets/models/biped_s17/xml/scene.xml
```

### Terminal 2：RL Controller

```bash
cd /workspace
export ROBOT_VERSION=17

build_sim/src/leju-controllers/leju-rl-controller/run_rl_controller \
  -c src/leju-controllers/leju-rl-controller/config/17/controller_manager.yaml
```

### Terminal 3：Joystick

```bash
cd /workspace
export ROBOT_VERSION=17

build_sim/src/leju-joystick/leju-joystick
```

---

## 10. Sim2Sim 控制逻辑

当前流程中默认 Controller：

```text
AMP
```

运行顺序建议：

```text
启动 MuJoCo + Controller + Joystick
        ↓
确认机器人初始姿态正常
        ↓
START → Running
        ↓
AMP 稳定
        ↓
切换 HEFT
        ↓
default_stand
        ↓
001 cross-arms stand
        ↓
再测试动态 Motion
```

当前已验证按键逻辑在 Sim2Sim 和真机一致。

开发机键盘模拟映射中常用：

```text
C → Gamepad A → HEFT
T → Gamepad B → HEFT Pico
V → Gamepad X → AMP
R → Gamepad Y → HEFT Planner
O/F → START
B → BACK
```

真实按键仍以当前 `teleop_bindings.yaml` 为最终依据。

---

## 11. Sim2Sim 成功判据

迁移真机前至少确认：

- MuJoCo 模型为 S17；
- `ROBOT_VERSION=17`；
- AMP 能正常进入 Running；
- HEFT 能正常切换；
- ONNX 加载无报错；
- ONNX 输入 `[1,1313]`；
- ONNX 输出 `[1,21]`；
- default_stand 稳定；
- 001 静态上肢动作稳定；
- 无明显关节突跳；
- 足底姿态合理；
- root orientation 合理；
- Motion 顺序/速度正确；
- Action Scale / PD / joint_direction 与训练一致；
- 不先用 crawl、jump、jog 等高风险 Motion 判断部署正确性。

**只有 Sim2Sim 完整通过，再进入 Sim2Real。**

---

# Part B：Sim2Real

## 12. 最简单的官方推荐路线

本次最终证明最简单的路线不是交叉编译，也不是 QEMU ARM64 Docker，而是：

```text
WSL 完整源码
      ↓
打包
      ↓ SCP
Roban RK3588
      ↓
官方纯 CMake / scripts 原生编译
      ↓
launch_real.sh
```

优点：

- 真机运行环境和编译环境完全一致；
- 不需要处理交叉编译 sysroot；
- 不需要处理 Docker `/workspace` RUNPATH；
- 官方脚本已针对 RK3588 big.LITTLE 优化；
- Sim2Sim 成功后只需要同步相同源码/config/policy/motion。

---

## 13. WSL 打包

项目原始目录：

```text
~/projects/lejulab_platform-heft-dev
```

不要把 Sim2Sim 编译产物上传到机器人。

推荐排除：

```text
build_sim/
build_arm64/
.git/
```

打包：

```bash
cd ~/projects

tar -czvf lejulab_platform-heft-dev_lyk0919.tar.gz \
  --exclude='lejulab_platform-heft-dev/build_sim' \
  --exclude='lejulab_platform-heft-dev/build_arm64' \
  --exclude='lejulab_platform-heft-dev/.git' \
  lejulab_platform-heft-dev
```

本次实际压缩包约：

```text
654 MB
```

未压缩源码约 1.2 GB 左右。

---

## 14. SCP 到 Roban

机器人地址示例：

```text
192.168.50.212
```

上传：

```bash
scp ~/projects/lejulab_platform-heft-dev_lyk0919.tar.gz \
  lab@192.168.50.212:/home/lab/
```

建议校验：

WSL：

```bash
sha256sum ~/projects/lejulab_platform-heft-dev_lyk0919.tar.gz
```

Roban：

```bash
sha256sum /home/lab/lejulab_platform-heft-dev_lyk0919.tar.gz
```

两边一致再解压。

---

## 15. Roban 使用独立时间线目录

不要直接覆盖官方/其他人的工程。

推荐：

```text
/home/lab/lejulab_platform-heft-dev_lyk0919
```

解压：

```bash
cd /home/lab

mkdir -p lejulab_platform-heft-dev_lyk0919

tar -xzf lejulab_platform-heft-dev_lyk0919.tar.gz \
  -C lejulab_platform-heft-dev_lyk0919 \
  --strip-components=1
```

确认：

```bash
cd ~/lejulab_platform-heft-dev_lyk0919
ls
du -sh .
```

确认完成后可删除压缩包：

```bash
rm /home/lab/lejulab_platform-heft-dev_lyk0919.tar.gz
```

---

## 16. Roban 原生编译：官方脚本方式

Roban：

```bash
cd ~/lejulab_platform-heft-dev_lyk0919
export ROBOT_VERSION=17
```

官方脚本：

```text
scripts/cmake_configure.sh
scripts/cmake_build.sh
scripts/build_env.sh
```

### 16.1 Configure

推荐使用 GCC 11：

```bash
CC=gcc-11 CXX=g++-11 \
scripts/cmake_configure.sh --fast build_cmake \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_RL_CONTROLLER=ON \
  -DBUILD_JOYSTICK=ON \
  -DBUILD_MUJOCO_SIM=OFF \
  -DBUILD_VR_CONTROL=OFF \
  -DBUILD_EXAMPLES=OFF \
  -DBUILD_TESTING=OFF
```

本次成功输出包括：

```text
[configure] ccache enabled
[configure] generator: Ninja
[configure] fast mode: skip tests/examples

GCC 11.4.0
Architecture: aarch64
ONNX Runtime: enabled
OpenCV: found
SDL aarch64: found
CycloneDDS: found
ALSA: found
sox: found
BUILD_VR_CONTROL=OFF
Configuring done
Generating done
```

### 16.2 一键编译

```bash
CC=gcc-11 CXX=g++-11 \
scripts/cmake_build.sh build_cmake
```

这是后续推荐使用的“一键编译”。

`scripts/build_env.sh` 会针对 RK3588：

- 检测 big.LITTLE；
- 必要时绑定大核 4-7；
- Ninja 下自动选择合适并行度；
- 使用 ccache 加速增量编译。

本次真机完整编译：

```text
257/257
```

成功。

编译目录约：

```text
1.4 GB build_cmake/
```

---

## 17. 真机编译产物检查

核心二进制：

```text
build_cmake/src/leju-controllers/leju-rl-controller/run_rl_controller
build_cmake/src/leju-hardware/leju-hardware
build_cmake/src/leju-joystick/leju-joystick
```

检查：

```bash
file build_cmake/src/leju-controllers/leju-rl-controller/run_rl_controller
file build_cmake/src/leju-hardware/leju-hardware
file build_cmake/src/leju-joystick/leju-joystick
```

必须看到：

```text
ELF 64-bit ... ARM aarch64
```

检查动态库：

```bash
ldd build_cmake/src/leju-controllers/leju-rl-controller/run_rl_controller | grep "not found"
ldd build_cmake/src/leju-hardware/leju-hardware | grep "not found"
ldd build_cmake/src/leju-joystick/leju-joystick | grep "not found"
```

正常应无任何 `not found`。

---

## 18. 真机上再次确认 Policy / Motion

正确路径：

```bash
cd ~/lejulab_platform-heft-dev_lyk0919
BASE=src/leju-controllers/leju-rl-controller/config/17
```

检查 Policy：

```bash
ls -lh "$BASE/policy/heft/policy0905.onnx"*
```

检查默认 Motion：

```bash
ls -lh "$BASE/motion_data/roban_default_stand_10s_deploy.csv"
```

检查首个低风险 Motion：

```bash
ls -lh \
  "$BASE/motion_data/roban_key_out_HEFT/001_a_person_crosses_arms_and_stands_still_deploy.csv"
```

本次实际确认：

```text
policy0905.onnx                      19 MB
roban_default_stand_10s_deploy.csv  264 KB
001 cross-arms CSV                  227 KB
```

---

## 19. 避免和机器人旧工程冲突

Roban 上可能存在官方旧工程：

```text
/home/lab/Lejulab_ARM
```

并且可能存在自启动服务：

```text
lejulab_joy_monitor.service
```

启动新工程前检查：

```bash
systemctl status lejulab_joy_monitor.service --no-pager
```

若旧服务为：

```text
active (running)
```

临时停止：

```bash
sudo systemctl stop lejulab_joy_monitor.service
```

不要急着 `disable`，本次实验只需 `stop`。

检查旧进程：

```bash
pgrep -af \
'monitor_lejulab_joy|leju_joy_autostart_manager|leju-hardware|leju-joystick|run_rl_controller|lejusdk_recorder|micphone_to_dds_node|audio_player_node|system_info_node' \
|| echo "No old Leju processes"
```

期望：

```text
No old Leju processes
```

否则不要启动第二套控制链。

> 旧 autostart 与新进程同时存在时，可能出现 DDS / iceoryx 冲突，例如 `POPO__CHUNK_LOCKING_ERROR`。

---

## 20. 真机启动

官方 ARM64/RK3588 推荐入口：

```text
src/leju_launch/scripts/launch_real.sh
```

先进入 root：

```bash
sudo -s
```

然后：

```bash
cd /home/lab/lejulab_platform-heft-dev_lyk0919

export ROBOT_VERSION=17
export BUILD_DIR="$PWD/build_cmake"
export LEJULAB_PROJECT_ROOT="$PWD"
```

这里显式指定：

```text
BUILD_DIR
LEJULAB_PROJECT_ROOT
```

可以避免机器人上存在多个工程时误加载旧目录。

确认：

```bash
echo "$ROBOT_VERSION"
echo "$BUILD_DIR"
echo "$LEJULAB_PROJECT_ROOT"
```

然后启动：

```bash
bash src/leju_launch/scripts/launch_real.sh
```

---

## 21. launch_real.sh 会启动什么

当前流程会启动：

```text
必需：
  leju-hardware
  leju-joystick
  run_rl_controller

附加：
  lejusdk_recorder
  micphone_to_dds_node
  audio_player_node
  system_info_node
```

检查：

```bash
pgrep -af \
'leju-hardware|leju-joystick|run_rl_controller|lejusdk_recorder|micphone_to_dds_node|audio_player_node|system_info_node'
```

所有路径都应来自：

```text
/home/lab/lejulab_platform-heft-dev_lyk0919/
```

不能混入：

```text
/home/lab/Lejulab_ARM/
```

---

## 22. 真机启动日志成功判据

本次成功日志包括：

```text
RUIWO actuator initialized
23 motors enabled
Error: 0
IMU connect success
IMU init success
Hardware interface initialized successfully
Robot cmd subscription successful
Joy subscription successful
Sensor data ready
Moving to default position
Default position reached
Starting ControlLoop
```

最终：

```text
Lifecycle => 机器人就绪
Lifecycle: 机器人就绪，等待`start`启动信号
```

此时系统已经正常启动，但还没有进入 Running。

### CAN 频率

本次观察：

```text
~485.9 Hz
```

目标：

```text
500 Hz
```

属于正常范围。

### Audio retry

如果看到：

```text
mic wake-up retry ...
```

只要 Hardware / IMU / RL 主链正常，这属于可选音频节点问题，不影响运动控制主链。

---

## 23. 真机 Teleop 配置要注意

真机 `run_rl_controller` 实际可能优先使用：

```text
/root/.config/lejuconfig/teleop_bindings.yaml
```

而不是仓库里的：

```text
config/17/teleop_bindings.yaml
```

因此第一次实验建议比较：

```bash
sudo diff -u \
  src/leju-controllers/leju-rl-controller/config/17/teleop_bindings.yaml \
  /root/.config/lejuconfig/teleop_bindings.yaml \
  || true
```

本次最终确认：

```text
Sim2Sim 与 Sim2Real 按键行为完全一致。
```

---

## 24. 真机实验顺序

推荐严格按照风险从低到高：

```text
launch_real.sh
      ↓
等待 Lifecycle = 机器人就绪
      ↓
START
      ↓
AMP Running
      ↓
观察 5~10 秒稳定性
      ↓
切 HEFT
      ↓
default_stand
      ↓
001 cross-arms stand
      ↓
静态 / 原地动作
      ↓
慢速行走
      ↓
动态动作
```

不要第一次上真机就直接运行：

```text
crawl
jump
jog
flip
大幅度动态动作
```

---

# Part C：以后换 Policy / Motion 的标准流程

## 25. 只换 Policy

WSL：

```text
1. 将新 ONNX 放到：
   config/17/policy/heft/

2. 修改：
   config/17/controllers/config_heft.yaml

3. Sim2Sim 验证：
   default_stand → 001 → 其它动作

4. Sim2Sim 通过后重新打包/上传，或只同步对应 ONNX + YAML。

5. Roban 上确认路径。

6. 重新启动 launch_real.sh。
```

若 C++ 未改，一般不需要重新编译；但为避免版本混淆，重要实验建议仍记录 ONNX hash。

推荐：

```bash
sha256sum \
src/leju-controllers/leju-rl-controller/config/17/policy/heft/policy0905.onnx
```

---

## 26. 只换 Motion

WSL：

```text
1. 生成 deploy CSV。
2. 放入 config/17/motion_data/。
3. 在 config_heft.yaml 的 motions 中注册。
4. Sim2Sim 检查 root / 足底 / 关节顺序 / 播放速度。
5. 静态 Motion 先通过。
6. 再迁移到 Roban。
```

只新增 CSV / YAML 时一般也不需要重新编译 C++。

---

## 27. 修改 C++ Controller 后

WSL：

```text
修改源码
→ build_sim 增量编译
→ Sim2Sim
→ 成功
→ 打包 / SCP
```

Roban：

```bash
CC=gcc-11 CXX=g++-11 \
scripts/cmake_build.sh build_cmake
```

Ninja + ccache 会自动增量编译，无需删除 `build_cmake`。

如果 CMakeLists 或编译选项发生变化，再重新执行：

```bash
CC=gcc-11 CXX=g++-11 \
scripts/cmake_configure.sh --fast build_cmake \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_RL_CONTROLLER=ON \
  -DBUILD_JOYSTICK=ON \
  -DBUILD_MUJOCO_SIM=OFF \
  -DBUILD_VR_CONTROL=OFF \
  -DBUILD_EXAMPLES=OFF \
  -DBUILD_TESTING=OFF
```

---

# Part D：常见问题

## 28. `BUILD_VR_CONTROL=ON but Drake was not found`

当前 HEFT 真机不需要 VR：

```bash
-DBUILD_VR_CONTROL=OFF
```

---

## 29. OpenCV 找不到

典型错误：

```text
Could not find OpenCVConfig.cmake
```

Ubuntu：

```bash
sudo apt install libopencv-dev
```

---

## 30. ALSA 找不到

需要：

```bash
sudo apt install libasound2-dev sox
```

真机本次已确认：

```text
ALSA 1.2.6.1
sox ready
```

---

## 31. Policy / Motion 显示 MISSING

首先不要猜路径。

`config_heft.yaml` 在：

```text
config/17/controllers/
```

所以：

```yaml
../policy/heft/xxx.onnx
```

实际是：

```text
config/17/policy/heft/xxx.onnx
```

同理：

```yaml
../motion_data/xxx.csv
```

实际是：

```text
config/17/motion_data/xxx.csv
```

---

## 32. 旧 Joy Monitor 冲突

检查：

```bash
systemctl status lejulab_joy_monitor.service --no-pager
```

实验前可临时：

```bash
sudo systemctl stop lejulab_joy_monitor.service
```

然后确认：

```bash
pgrep -af 'Lejulab_ARM|leju_joy_autostart_manager|monitor_lejulab_joy'
```

新旧控制链不要同时运行。

---

## 33. 真机退出

软件正常退出优先：

```text
BACK
```

或运行终端：

```text
Ctrl+C
```

让 `launch_real.sh` 完成统一 cleanup。

出现明显异常运动时优先现场急停，不等待软件退出。

---

# Part E：最简命令速查

## 34. Sim2Sim

```bash
cd ~/projects/lejulab_platform-heft-dev
./docker/run_sim.sh
```

Docker 内：

```bash
cd /workspace
export ROBOT_VERSION=17

cmake -S . -B build_sim -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_MUJOCO_SIM=ON \
  -DBUILD_RL_CONTROLLER=ON \
  -DBUILD_JOYSTICK=ON \
  -DBUILD_VR_CONTROL=OFF \
  -DBUILD_EXAMPLES=OFF \
  -DBUILD_TESTING=OFF

cmake --build build_sim -j$(nproc)

export BUILD_DIR="$PWD/build_sim"
bash src/leju_launch/scripts/launch_mujoco_sim.sh --no-shm
```

---

## 35. 打包 + SCP

WSL：

```bash
cd ~/projects

tar -czvf lejulab_platform-heft-dev_lyk0919.tar.gz \
  --exclude='lejulab_platform-heft-dev/build_sim' \
  --exclude='lejulab_platform-heft-dev/build_arm64' \
  --exclude='lejulab_platform-heft-dev/.git' \
  lejulab_platform-heft-dev

scp lejulab_platform-heft-dev_lyk0919.tar.gz \
  lab@192.168.50.212:/home/lab/
```

---

## 36. Roban 原生编译

```bash
cd ~/lejulab_platform-heft-dev_lyk0919
export ROBOT_VERSION=17

CC=gcc-11 CXX=g++-11 \
scripts/cmake_configure.sh --fast build_cmake \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_RL_CONTROLLER=ON \
  -DBUILD_JOYSTICK=ON \
  -DBUILD_MUJOCO_SIM=OFF \
  -DBUILD_VR_CONTROL=OFF \
  -DBUILD_EXAMPLES=OFF \
  -DBUILD_TESTING=OFF

CC=gcc-11 CXX=g++-11 \
scripts/cmake_build.sh build_cmake
```

---

## 37. Roban 真机启动

先停旧自启动服务：

```bash
sudo systemctl stop lejulab_joy_monitor.service
```

进入 root：

```bash
sudo -s
```

root 下：

```bash
cd /home/lab/lejulab_platform-heft-dev_lyk0919

export ROBOT_VERSION=17
export BUILD_DIR="$PWD/build_cmake"
export LEJULAB_PROJECT_ROOT="$PWD"

bash src/leju_launch/scripts/launch_real.sh
```

等待：

```text
Lifecycle: 机器人就绪，等待`start`启动信号
```

然后：

```text
START
→ AMP
→ HEFT
→ default_stand
→ 001
→ 其它动作
```

---

# 38. 最终经验总结

本次最终验证表明，对于当前 Roban S17 + HEFT 工程，最稳定、最简单的 Sim2Real 路线是：

```text
同一份源码
同一份 config
同一份 ONNX
同一份 motion CSV
       │
       ├── x86 Docker / MuJoCo → Sim2Sim
       │
       └── SCP source → RK3588 native build → Sim2Real
```

其中真正需要重点防止的不是“编译架构差异”，而是以下几类部署 mismatch：

```text
Policy 版本不一致
Motion 文件不一致
joint order 不一致
Action Scale / PD 不一致
ROBOT_VERSION 不一致
teleop_bindings 不一致
旧工程自启动服务仍在运行
真机加载到了错误目录
```

只要保持 WSL 为唯一源码源头、Sim2Sim 先通过、Roban 只做原生构建和运行，这套流程后续可以稳定复用。
