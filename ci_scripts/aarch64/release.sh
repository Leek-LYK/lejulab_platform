#!/bin/bash
# 合并 package + sync：从 build_aarch64 产物 + git 源码组装发布树，直接推下游仓库
# artifact 只传 installed/（~21M），明文资源从 git checkout 读取，全程不经过 GitLab artifact 存储
set -euo pipefail

DOWNSTREAM_URL="${1:?usage: $0 <downstream-url> <branch>}"
BRANCH="${2:?usage: $0 <downstream-url> <branch>}"

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(realpath "$SCRIPT_DIR/../..")"
cd "$PROJECT_DIR"

STAGING="$PROJECT_DIR/staging"
INSTALLED="$STAGING/installed"

[ -d "$INSTALLED" ] || { echo "[release] ERROR: $INSTALLED missing — build_aarch64 artifact not downloaded?"; exit 1; }

# 以当前提交为发布源，先导出完整仓库；后续只删除明确声明的例外。
# 这样新增的非 CMake 目录（例如 tools、demo、config）也不会被白名单漏掉。
echo "[release 0/9] copying tracked repository contents..."
find "$STAGING" -mindepth 1 -maxdepth 1 ! -name installed -exec rm -rf {} +
git archive --format=tar HEAD | tar -xf - -C "$STAGING"
rm -rf "$STAGING/ci_scripts" "$STAGING/.gitlab-ci.yml" "$STAGING/.catkin_tools"

# ============================================================
# PACKAGE phase — 在 build 产物基础上追加明文资源
# ============================================================

# 1. 清理不需要随车发布的 install 产物
#    开源树保留核心库头和 DDS SDK 公共头——stub 的 INTERFACE_INCLUDE_DIRECTORIES
#    指向 installed/include。其余包的头由源码包提供，删掉避免泄漏内部安装头。
echo "[release 1/9] purging cmake configs / private static libs / non-SDK headers..."
rm -rf "$INSTALLED/lib/cmake"
rm -rf "$INSTALLED/lib/pkgconfig"
rm -rf "$INSTALLED/share"
# 仅保留闭源 SDK 的静态库；其余静态库避免随车发布。
find "$INSTALLED" -name "*.a" -type f \
    ! -name 'liblejusdk-dds-idl.a' \
    ! -name 'liblejusdk-dds-rpc.a' \
    -delete 2>/dev/null || true
# 保留核心库和 DDS SDK 的公共头，删其余安装头（功能包头随源码发布）。
if [ -d "$INSTALLED/include" ]; then
    find "$INSTALLED/include" -mindepth 1 -maxdepth 1 \
        ! -name 'lejusdk-lowlevel' ! -name 'lejusdk-vr' \
        ! -name 'lejusdk-dds-idl' ! -name 'lejusdk-topic-pubsub' \
        ! -name 'dds_rpc' ! -name 'rpc_idl' \
        -exec rm -rf {} + 2>/dev/null || true
fi

# 2. strip 符号（build.sh 已做，此处兜底）
STRIP="$(which aarch64-linux-gnu-strip 2>/dev/null || echo strip)"
echo "[release 2/9] stripping symbols (using $STRIP)..."
while IFS= read -r so; do
    [ -L "$so" ] && continue
    if file "$so" 2>/dev/null | grep -q "ELF"; then
        $STRIP --strip-unneeded "$so" 2>/dev/null || \
            echo "[release] WARN: strip failed on $so"
    fi
done < <(find "$INSTALLED/lib" -name "*.so*" -type f 2>/dev/null || true)
while IFS= read -r exe; do
    if file "$exe" 2>/dev/null | grep -q "ELF.*executable"; then
        $STRIP --strip-all "$exe" 2>/dev/null || \
            echo "[release] WARN: strip failed on $exe"
    fi
done < <(find "$INSTALLED/bin" -type f 2>/dev/null || true)

# 3. 拷 3rd_party 预编译 .so
echo "[release 3/9] copying 3rd_party .so libs..."
shopt -s nullglob
for sof in "$PROJECT_DIR"/src/lejusdk/3rd_party/aarch64/*/lib/*.so*; do
    cp -a "$sof" "$INSTALLED/lib/" 2>/dev/null || true
done
shopt -u nullglob

# 3b. MuJoCo 作为 binary 包发布，纯 CMake 安装不会从源码包进入开源树；
#      显式携带其运行时动态库，避免 installed/bin/leju-mujoco-sim 缺库。
MUJOCO_LIB_DIR="$PROJECT_DIR/src/leju-mujoco-sim/3rd_party/aarch64/mujoco-3.2.0/lib"
for sof in "$MUJOCO_LIB_DIR"/libmujoco.so*; do
    [ -e "$sof" ] || continue
    cp -a "$sof" "$INSTALLED/lib/"
done

# 4. 生成 installed/setup.bash
echo "[release 4/9] generating setup.bash..."
cat > "$INSTALLED/setup.bash" <<'SETUP_EOF'
#!/bin/bash
_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export LD_LIBRARY_PATH="$_DIR/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export PATH="$_DIR/bin${PATH:+:$PATH}"
export LEJULAB_INSTALL_ROOT="$_DIR"
export LEJULAB_PROJECT_ROOT="$(cd "$_DIR/.." && pwd)"
if [ -z "${ROBOT_VERSION:-}" ]; then
    echo "[lejulab] 请 export ROBOT_VERSION=<14|46|52> 后再 launch" >&2
fi
SETUP_EOF
chmod +x "$INSTALLED/setup.bash"

# 5. 拷明文资源（scripts、docs）+ 按清单同步 src 包
echo "[release 5/9] copying plaintext resources..."
if [ -d "$PROJECT_DIR/scripts" ]; then
    cp -r "$PROJECT_DIR/scripts" "$STAGING/"
    find "$STAGING/scripts" -name "__pycache__" -type d -exec rm -rf {} + 2>/dev/null || true
    find "$STAGING/scripts" -name "*.pyc" -type f -delete 2>/dev/null || true
    rm -rf "$STAGING/scripts/logs" 2>/dev/null || true
fi

if [ -d "$PROJECT_DIR/docs" ]; then
    cp -r "$PROJECT_DIR/docs" "$STAGING/"
    rm -rf "$STAGING/docs/plans"
    rm -rf "$STAGING/docs/reports"
    rm -rf "$STAGING/docs/superpowers"
    rm -f  "$STAGING/docs/rk3588-migration-progress.md"
    rm -f  "$STAGING/docs/cross-machine-rl-mujoco-guide.md"
fi

# deploy_joy_autostart.sh 的 sync_resources_to_lejuconfig() 从 <ws>/resources
# 拷贝 action_files/music 到 ~/.config/lejuconfig，供预编译 RL 控制器 /
# leju-audio 播放手势与配乐使用，发布树必须带上，否则遥控器自启动部署时会
# 因 resources/ 缺失而静默跳过同步。
if [ -d "$PROJECT_DIR/resources" ]; then
    cp -r "$PROJECT_DIR/resources" "$STAGING/"
fi

# 5c. 默认发布闭源根 CMakeLists.txt 中的所有 src 包；manifest 仅声明 stub/binary 例外。
MANIFEST="$SCRIPT_DIR/opensource_packages.txt"
OSRC="$SCRIPT_DIR/opensource"
[ -f "$MANIFEST" ] || { echo "[release] ERROR: manifest missing: $MANIFEST"; exit 1; }
source "$SCRIPT_DIR/opensource_manifest_lib.sh"
parse_manifest "$MANIFEST"
parse_opensource_cmake "$PROJECT_DIR/CMakeLists.txt"

# source 包的可执行文件由下游用户自行编译，不与源码重复发布。
# binary 包（hardware / RL / MuJoCo / recorder）的预编译目标保留。
SOURCE_BINARIES=(
    run_dummy_controller
    motor_keyboard_ctrl motor_test_runner tests/test_keyboard_ctrl
    leju-joystick leju_joy_autostart_manager
    hw_state_subscriber joint_state_subscriber imu_data_subscriber
    joint_cmd_publisher joint_cmd_subscriber kuavo_control_example
    vr_01_basic_init vr_02_kuavo_arm vr_03_velocity_cmd vr_04_get_state
    vr_05_head_control vr_06_waist_control vr_07_roban_arm
    vr_08_controller_cli vr_09_play_tact
    micphone_to_dds_node audio_player_node play_audio_cli audio_stream_probe
    lejusdk-csv-recorder
    system_info_node system_info_echo
    quest_udp_to_dds_node quest_vr_control_node quest_vr_abs_control_node
    test_arm_ik_node
)
echo "[release] removing binaries rebuildable from published source..."
for binary in "${SOURCE_BINARIES[@]}"; do
    if [ -e "$INSTALLED/bin/$binary" ]; then
        rm -f "$INSTALLED/bin/$binary"
        echo "[release]   removed source-built binary: $binary"
    fi
done

is_in_list() {
    local needle="$1" item
    shift
    for item in "$@"; do [ "$item" = "$needle" ] && return 0; done
    return 1
}

if ! is_in_list "leju_launch" "${SRC_PKGS[@]}"; then
    SRC_PKGS+=(leju_launch)
fi

# staging 已在发布开始阶段从当前提交完整导出；这里不再清空 src，避免将
# 未被根 CMakeLists 引用的源码、工具或资源误删。
mkdir -p "$STAGING/src"

# 生成开源源码包列表：默认跟随闭源根 CMakeLists，manifest 例外包不进入。
# VR 的 Drake 探测和 add_subdirectory 由开源根 CMakeLists 的专用阶段负责。
SOURCE_CMAKE="$STAGING/source_packages.cmake"
: > "$SOURCE_CMAKE"
echo "# Generated by release.sh from the closed root CMakeLists.txt." >> "$SOURCE_CMAKE"
echo "# Do not edit in the downstream repository." >> "$SOURCE_CMAKE"
for pkg in "${SRC_PKGS[@]}"; do
    if is_in_list "$pkg" "${STUB_PKGS[@]}" "${BINARY_PKGS[@]}"; then
        continue
    fi
    case "$pkg" in
        leju-ik|leju-remote|leju-vr-control)
            continue
            ;;
        leju-joystick)
            {
                echo 'if(BUILD_JOYSTICK)'
                echo "    add_subdirectory(src/$pkg)"
                echo 'endif()'
            } >> "$SOURCE_CMAKE"
            ;;
        lejusdk/examples)
            {
                echo 'if(BUILD_EXAMPLES)'
                echo "    add_subdirectory(src/$pkg)"
                echo 'endif()'
            } >> "$SOURCE_CMAKE"
            ;;
        *)
            echo "add_subdirectory(src/$pkg)" >> "$SOURCE_CMAKE"
            ;;
    esac
done

# 音频、监控和 CSV recorder 以源码发布；它们通过 installed/ 中的 DDS SDK
# IMPORTED targets 编译。CycloneDDS/iceoryx 仅作为预编译构建/运行时依赖提供。
mkdir -p "$STAGING/src/lejusdk/3rd_party"
cp "$PROJECT_DIR/src/lejusdk/3rd_party/arch_select.cmake" "$STAGING/src/lejusdk/3rd_party/"
cp -a "$PROJECT_DIR/src/lejusdk/3rd_party/aarch64" "$STAGING/src/lejusdk/3rd_party/"

# 从完整导出的仓库中删除 manifest 明确声明的 stub 源码；source 包及其他
# 仓库目录保持原样，不再按 SRC_PKGS 白名单复制。
for pkg in "${STUB_PKGS[@]}"; do
    rm -rf "${STAGING:?}/src/$pkg"
    echo "[release]   stub:   $pkg (由 installed/lib+include 提供，不发源码)"
done
for pkg in "${BINARY_PKGS[@]}"; do
    subdirs="${BINARY_SUBDIRS[$pkg]}"
    rm -rf "${STAGING:?}/src/$pkg"
    if [ -z "$subdirs" ]; then
        echo "[release]   binary: $pkg (installed/bin 预编译，不发源码)"
        continue
    fi
    src_pkg="$PROJECT_DIR/src/$pkg"
    IFS=',' read -ra _dirs <<< "$subdirs"
    for sub in "${_dirs[@]}"; do
        sub="$(printf '%s' "$sub" | sed 's/^[[:space:]]*//; s/[[:space:]]*$//')"
        [ -d "$src_pkg/$sub" ] || { echo "[release] ERROR: binary subdir missing: src/$pkg/$sub" >&2; exit 1; }
        mkdir -p "$STAGING/src/$pkg"
        rm -rf "${STAGING:?}/src/$pkg/$sub"
        cp -r "$src_pkg/$sub" "$STAGING/src/$pkg/"
    done
done

# 5d. 部署开源树根文件，并保留例外清单供下游审计。
echo "[release 5d/9] installing opensource root files..."
cp "$OSRC/CMakeLists.txt"        "$STAGING/CMakeLists.txt"
cp "$OSRC/lejusdk-stubs.cmake"   "$STAGING/lejusdk-stubs.cmake"
cp "$MANIFEST"                   "$STAGING/opensource_packages.txt"

# 5.5 脱敏
echo "[release 5.5/9] scrubbing internal IPs (192.168.50.*) -> <YOUR_HOST_IP>..."
find "$STAGING" -type f \
    ! -name "*.png" ! -name "*.jpg" ! -name "*.jpeg" ! -name "*.svg" \
    ! -name "*.onnx" ! -name "*.so*" ! -name "*.a" \
    ! -name "*.urdf" ! -name "*.xacro" ! -name "*.stl" ! -name "*.dae" \
    ! -path "$STAGING/installed/bin/*" \
    -exec grep -lE "192\.168\.50\.[0-9x]+" {} \; 2>/dev/null \
    | while read -r f; do
        sed -i 's/192\.168\.50\.[0-9x]\+/<YOUR_HOST_IP>/g' "$f"
        echo "  scrubbed: ${f#$STAGING/}"
    done

# 6. 生成 VERSION
echo "[release 6/9] writing VERSION..."
UPSTREAM_SHA="$(git rev-parse HEAD 2>/dev/null || echo unknown)"
UPSTREAM_DESCRIBE="$(git describe --tags --always --dirty 2>/dev/null || echo unknown)"
BUILD_ID="$(date -u +%Y%m%d).$(printf '%03d' "${CI_PIPELINE_IID:-0}")"
BUILD_TIME="$(date -u -Iseconds)"
cat > "$STAGING/VERSION" <<EOF
upstream-commit: $UPSTREAM_SHA
upstream-describe: $UPSTREAM_DESCRIBE
build-id: $BUILD_ID
build-time: $BUILD_TIME
EOF

# 7. 生成 README + MANIFEST
echo "[release 7/9] writing README + MANIFEST..."
ls "$INSTALLED/bin" 2>/dev/null | sort > "$INSTALLED/MANIFEST.txt"
if [ -d "$INSTALLED/bin/tests" ]; then
    { echo; echo "# Tests"; ls "$INSTALLED/bin/tests" | sed 's|^|tests/|'; } >> "$INSTALLED/MANIFEST.txt"
fi
cat > "$STAGING/README_DEPLOY.md" <<'README_EOF'
# lejulab-platform aarch64 release

源码 + 预编译核心库和业务产物（stub/binary）混合发布。可开源的控制器以源码形式提供，
RL 控制器因依赖闭源 kuavo_solver 以预编译 binary 提供；
`lejusdk-lowlevel` / `lejusdk-vr` 以预编译 `.so` + 头文件提供（藏实现，对齐上游）。

## 完整 Quest3 VR 链路

方案一发布包含完整源码链路，而不是只有 UDP 桥接层：

```text
Quest3 --UDP--> quest_udp_to_dds_node --DDS--> leju-vr-control
                    /rt/quest/bone_poses, /rt/quest/joysticks
                                      --> leju-ik/Drake --> /rt/arm_trajectory
```

完整 VR 构建要求目标架构的 Drake、Eigen3、yaml-cpp、Protobuf 和 GCC 11。Drake 不随本发布树复制：如果安装在标准位置（例如 `/opt/drake`），配置时会自动发现；安装在其他位置时才需要通过 `DRAKE_PREFIX` 指定：

```bash
# 标准安装位置：无需手动指定 Drake
cmake -B build -S . -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)

# 自定义安装位置：使用目标架构的 Drake
cmake -B build -S . -DCMAKE_BUILD_TYPE=Release \
  -DDRAKE_PREFIX=/path/to/aarch64/drake
cmake --build build -j$(nproc)
```

`DRAKE_PREFIX` 也可以作为环境变量提供。Drake 必须是目标架构版本；编译期自动发现不会替代运行时动态库配置，运行程序时仍需确保 Drake 的 `lib` 在系统动态链接器路径或 `LD_LIBRARY_PATH` 中。方案一应生成 `quest_udp_to_dds_node`、`quest_vr_control_node`、`quest_vr_abs_control_node` 和 `libleju-ik.so`。绝对式节点需要 `src/leju_assets/models/biped_s17/urdf/drake/biped_v3_arm.urdf`；联合启动可使用 `src/leju_launch/launch/vr_teleop.launch`。没有 Drake 时不得将构建标记为完整 VR。

## 依赖
```bash
sudo apt-get install -y g++-11 libacl1-dev libeigen3-dev libyaml-cpp-dev libusb-1.0-0-dev
# 手柄：SDL3（随包在 src/leju-joystick/3rd_party，无需另装）
# RL 控制器：已随包提供预编译运行时，无需编译 leju-rl-controller 源码
```

## 构建

```bash
git clone <this-repo-url>
cd <repo-name>
CC=gcc-11 CXX=g++-11 cmake -B build -S . -DCMAKE_BUILD_TYPE=Release
CC=gcc-11 CXX=g++-11 cmake --build build -j$(nproc)
```

预编译的 cyclonedds 静态库内嵌 GCC 11 LTO，必须用 GCC 11 链接。

## 运行 / 自启动

```bash
export ROBOT_VERSION=46    # 14=Roban2 base, 46=Kuavo 4 UAE, 52=Kuavo 5 base
sudo -E bash src/leju_launch/scripts/launch_real.sh
# 或部署手柄上电自启服务：
sudo bash src/leju-joystick/services/deploy_joy_autostart.sh
```

`sudo -E` 透传环境变量（`LD_LIBRARY_PATH` / `PATH` / `ROBOT_VERSION`）。
`installed/setup.bash` 设好 `LD_LIBRARY_PATH=installed/lib`，预编译 `.so` 的运行期依赖（cyclonedds / iceoryx / onnxruntime / SDL3 / MuJoCo）在其中。

## 目录说明

| 路径 | 内容 |
|---|---|
| `CMakeLists.txt` | 开源树根（include stubs + add_subdirectory 源码包） |
| `lejusdk-stubs.cmake` | 核心库和 DDS SDK 的 IMPORTED stub 定义 |
| `opensource_packages.txt` | stub/binary 例外清单；未声明的根 CMake source 包默认整包发布 |
| `installed/bin/` | 预编译产物（leju-hardware / RL 控制器 / recorder 等） |
| `installed/lib/` | 预编译 .so（lejusdk-lowlevel/vr / kuavo_solver + cyclonedds/iceoryx/onnxruntime/SDL3） |
| `installed/include/` | 核心库头和 DDS SDK 公共头（实现不发布） |
| `src/lejusdk/lejusdk-utils/` | header-only 源码（lowlevel/vr 公开头依赖） |
| `src/leju-controllers/*/` | 可开源控制器源码（dummy / keyboard）；RL 控制器在 installed/bin |
| `src/leju-joystick/` | 手柄源码 + services/ 自启动脚本 + SDL3 3rd_party |
| `src/leju_assets/` `src/leju_launch/` | 资产 / launch / 启动脚本 |
| `src/lejusdk/examples/` | SDK 示例源码 |
| `src/leju-hardware/config/` | 硬件配置（leju-hardware 本体是预编译产物） |
| `scripts/` `docs/` | 工具与文档 |
| `resources/` | 舞蹈配乐 / 手势 tact 文件（部署自启动脚本会同步到 ~/.config/lejuconfig） |
| `VERSION` | 版本信息 |
README_EOF

# 8. sanity 检查
echo "[release 8/9] running sanity checks..."
bash "$SCRIPT_DIR/sanity.sh" "$STAGING"

echo
echo "[release] package done. staging tree:"
du -sh "$STAGING"/* 2>/dev/null || true

# ============================================================
# SYNC phase — rsync 到下游 aarch64 仓库
# ============================================================
echo "[release 9/9] syncing to downstream repo..."

SSH_KEYS=()
for k in /home/gitlab-runner/.ssh/id_ed25519 /home/gitlab-runner/.ssh/id_rsa; do
    [ -f "$k" ] && SSH_KEYS+=("$k")
done
if [ ${#SSH_KEYS[@]} -eq 0 ]; then
    echo "[release] ERROR: no SSH key found in /home/gitlab-runner/.ssh/"
    exit 1
fi
eval "$(ssh-agent -s)" >/dev/null
for k in "${SSH_KEYS[@]}"; do
    ssh-add "$k" 2>&1 | tail -1
done
export GIT_SSH_COMMAND="ssh -o UserKnownHostsFile=/dev/null -o StrictHostKeyChecking=no"

TARGET="/tmp/lejulab-aarch64-downstream-${BRANCH//\//_}"
rm -rf "$TARGET"

echo "[release] cloning downstream branch '$BRANCH' from $DOWNSTREAM_URL..."
if git clone --branch "$BRANCH" --single-branch "$DOWNSTREAM_URL" "$TARGET" 2>/dev/null; then
    echo "[release] branch $BRANCH exists, checked out"
else
    echo "[release] branch $BRANCH does not exist, will create it"
    rm -rf "$TARGET"
    if git clone "$DOWNSTREAM_URL" "$TARGET" 2>/dev/null; then
        cd "$TARGET"
        if git rev-parse --verify HEAD >/dev/null 2>&1; then
            git checkout -b "$BRANCH" 2>/dev/null || git checkout "$BRANCH"
        else
            git checkout -b "$BRANCH"
        fi
        cd - >/dev/null
    else
        echo "[release] remote clone failed, initializing local repo..."
        mkdir -p "$TARGET"
        cd "$TARGET"
        git init
        git checkout -b "$BRANCH"
        git remote add origin "$DOWNSTREAM_URL"
        cd - >/dev/null
    fi
fi

cd "$TARGET"
git config user.email "ci@lejulab"
git config user.name "lejulab-ci"

echo "[release] rsync staging -> $TARGET..."
rsync -a --delete \
  --exclude='.git' \
  --exclude='.gitlab-ci.yml' \
  --exclude='ci_scripts' \
  --exclude='ci_scripts/**' \
  "$STAGING/" "$TARGET/"

git add -A

if git diff --cached --quiet; then
    echo "[release] no changes detected, skipping push"
    exit 0
fi

COMMIT_MSG="sync ${CI_PROJECT_URL:-unknown}/commit/${CI_COMMIT_SHA:-$(date -u -Iseconds)}"
git commit -m "$COMMIT_MSG"

echo "[release] pushing -f to origin $BRANCH..."
git push -f origin "$BRANCH"

echo "[release] done. downstream branch '$BRANCH' updated."
