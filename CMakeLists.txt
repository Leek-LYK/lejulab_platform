cmake_minimum_required(VERSION 3.16)

# 预编译的 cyclonedds/iceoryx 静态库使用 GCC 11 LTO 编译,
# 如果系统默认 GCC 12+, 链接时会报 LTO 版本不匹配。
# 自动检测并优先使用 GCC 11。
if(NOT CMAKE_C_COMPILER AND NOT CMAKE_CXX_COMPILER)
    find_program(_GCC11 gcc-11)
    find_program(_GXX11 g++-11)
    if(_GCC11 AND _GXX11)
        set(CMAKE_C_COMPILER "${_GCC11}" CACHE FILEPATH "C compiler" FORCE)
        set(CMAKE_CXX_COMPILER "${_GXX11}" CACHE FILEPATH "C++ compiler" FORCE)
        message(STATUS "Auto-selected GCC 11: ${_GCC11}, ${_GXX11}")
    endif()
endif()

project(lejulab_platform VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# Standard install dirs
include(GNUInstallDirs)

# Install RPATH: 让 installed/{bin,lib} 下的 ELF 通过 $ORIGIN 互相发现，
# 不依赖系统路径，便于按 §3 目录结构整体搬迁部署。
set(CMAKE_INSTALL_RPATH "$ORIGIN:$ORIGIN/../lib")
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH FALSE)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Options
option(BUILD_TESTS "Build tests" ON)
include(CTest)  # 设置 BUILD_TESTING，供子目录 if(BUILD_TESTING) 使用
option(BUILD_EXAMPLES "Build examples" ON)
option(BUILD_RL_CONTROLLER "Build RL controller (requires OpenVINO or ONNX Runtime)" ON)
option(BUILD_JOYSTICK "Build joystick (requires SDL3)" ON)
option(BUILD_MUJOCO_SIM "Build MuJoCo simulator" ON)
option(BUILD_VR_CONTROL
       "Build complete Quest3 VR chain (leju-ik + leju-remote + leju-vr-control; requires Drake)"
       ON)
option(FORCE_PURE_CMAKE "Force pure CMake build (ignore catkin even if available)" OFF)
if(FORCE_PURE_CMAKE)
    set(CMAKE_DISABLE_FIND_PACKAGE_catkin ON)
    message(STATUS "FORCE_PURE_CMAKE=ON: catkin disabled, using pure CMake layout")
endif()

# 诊断文件日志开关 (默认关闭)。
# 守护 RT/CAN 热路径里的 /tmp/*.log 同步写 (can_tx/can_rx/socketcan_tx_drops/
# *_thread_timing/arm_diag/cmd_receive/sensor_timing)，避免实时线程被 ext4 写阻塞。
# 仅在离线排查时开启: cmake -DLEJU_ENABLE_DIAG_FILELOG=ON
# 根作用域 add_compile_definitions 对其后所有 add_subdirectory 子目标生效。
option(LEJU_ENABLE_DIAG_FILELOG "Enable diagnostic /tmp/*.log writes on RT/CAN hot paths" OFF)
if(LEJU_ENABLE_DIAG_FILELOG)
    add_compile_definitions(LEJU_ENABLE_DIAG_FILELOG=1)
    message(STATUS "LEJU_ENABLE_DIAG_FILELOG=ON: RT/CAN 热路径诊断文件写已启用 (仅排查用)")
endif()

# Drake：显式 DRAKE_PREFIX（CMake cache 或环境变量）优先；否则从当前架构的
# 标准安装前缀自动发现。完整 VR 模式 fail-closed；普通模式允许不安装 Drake。
set(_DRAKE_CONFIG "lib/cmake/drake/drake-config.cmake")
set(_DRAKE_PREFIX_EXPLICIT FALSE)
if(DRAKE_PREFIX)
    set(_DRAKE_PREFIX_EXPLICIT TRUE)
elseif(DEFINED ENV{DRAKE_PREFIX} AND NOT "$ENV{DRAKE_PREFIX}" STREQUAL "")
    set(DRAKE_PREFIX "$ENV{DRAKE_PREFIX}")
    set(_DRAKE_PREFIX_EXPLICIT TRUE)
endif()

if(NOT DRAKE_PREFIX)
    if(CMAKE_CROSSCOMPILING AND CMAKE_SYSROOT)
        # 交叉编译只能使用 sysroot 内的目标架构 Drake，不能误选宿主机版本。
        set(_DRAKE_CANDIDATES "${CMAKE_SYSROOT}/opt/drake")
    elseif(NOT CMAKE_CROSSCOMPILING)
        set(_DRAKE_CANDIDATES /opt/drake /home/test/drake_install_prefix /usr/local /usr)
    endif()
    foreach(_candidate IN LISTS _DRAKE_CANDIDATES)
        if(EXISTS "${_candidate}/${_DRAKE_CONFIG}")
            set(DRAKE_PREFIX "${_candidate}")
            break()
        endif()
    endforeach()
endif()

if(DRAKE_PREFIX AND EXISTS "${DRAKE_PREFIX}/${_DRAKE_CONFIG}")
    set(DRAKE_PREFIX "${DRAKE_PREFIX}" CACHE PATH "Drake install prefix" FORCE)
    list(APPEND CMAKE_PREFIX_PATH "${DRAKE_PREFIX}")
    message(STATUS "Found drake at: ${DRAKE_PREFIX}")
elseif(BUILD_VR_CONTROL)
    if(_DRAKE_PREFIX_EXPLICIT)
        message(FATAL_ERROR
                "BUILD_VR_CONTROL=ON but Drake was not found under DRAKE_PREFIX=${DRAKE_PREFIX}")
    else()
        message(FATAL_ERROR
                "BUILD_VR_CONTROL=ON but Drake was not found in the standard install locations; "
                "set -DDRAKE_PREFIX=<Drake install prefix> for a custom location")
    endif()
elseif(DRAKE_PREFIX)
    message(WARNING "drake not found at ${DRAKE_PREFIX}; VR control is disabled")
else()
    message(STATUS "Drake not found; BUILD_VR_CONTROL=OFF")
endif()
unset(_DRAKE_CANDIDATES)
unset(_DRAKE_CONFIG)

# 全局解析 iceoryx：CycloneDDS config 经 find_dependency(iceoryx_binding_c) 把 iceoryx
# 作为 IMPORTED 目标透传给所有链接 CycloneDDS 的包；IMPORTED 目标是目录作用域的，
# fresh configure 下 tools/joy_trigger 等跨目录消费者看不到该目标(报 not found)。
# 在根作用域统一 find 一次，使其对所有 add_subdirectory 子目录可见。
set(THIRD_PARTY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/lejusdk/3rd_party)
include(${THIRD_PARTY_DIR}/arch_select.cmake)
list(APPEND CMAKE_PREFIX_PATH
    ${THIRD_PARTY_ARCH_DIR}/iceoryx
    ${THIRD_PARTY_ARCH_DIR}/cyclonedds-0.10.2
    ${THIRD_PARTY_ARCH_DIR}/cyclonedds-cxx-0.10.2)
find_package(iceoryx_hoofs QUIET)
find_package(iceoryx_posh QUIET)
find_package(iceoryx_binding_c QUIET)

# Layer 0
add_subdirectory(src/lejusdk/lejusdk-utils)

# Layer 1
add_subdirectory(src/lejusdk/lejusdk-comm-dds/lejusdk-dds-idl)
add_subdirectory(src/lejusdk/lejusdk-comm-dds/lejusdk-topic-pubsub)
add_subdirectory(src/lejusdk/lejusdk-comm-dds/lejusdk-dds-rpc)

# Layer 2
add_subdirectory(src/leju_assets)

# Layer 3
add_subdirectory(src/lejusdk/lejusdk-lowlevel)
add_subdirectory(src/lejusdk/lejusdk-vr)

# Layer 4
add_subdirectory(src/leju-hardware)
if(BUILD_RL_CONTROLLER)
    add_subdirectory(src/leju-controllers/leju-rl-controller)
endif()
add_subdirectory(src/leju-controllers/leju-dummy-controller)
add_subdirectory(src/leju-controllers/leju-keyboard-controller)
if(BUILD_JOYSTICK)
    add_subdirectory(src/leju-joystick)
endif()
if(BUILD_MUJOCO_SIM)
    add_subdirectory(src/leju-mujoco-sim)
endif()

# Layer 5
if(BUILD_EXAMPLES)
    add_subdirectory(src/lejusdk/examples)
endif()
add_subdirectory(src/lejusdk/lejusdk-recorder)
add_subdirectory(src/lejusdk/lejusdk-csv-recorder)

# Layer 5: 麦克风采集 → DDS 节点（不依赖 Drake）
add_subdirectory(src/leju-audio)

# Layer 5: 系统监控节点（/monitor/system_info/*，不依赖 Drake）
add_subdirectory(src/leju-monitor)

# Layer 6: 完整 VR 控制链路（需要 Drake）。
if(BUILD_VR_CONTROL)
    find_package(drake CONFIG REQUIRED)
    add_subdirectory(src/leju-ik)
    add_subdirectory(src/leju-remote)
    add_subdirectory(src/leju-vr-control)
else()
    message(STATUS "BUILD_VR_CONTROL=OFF: skipping leju-ik / leju-remote / leju-vr-control")
endif()

# Tools
add_subdirectory(tools)
