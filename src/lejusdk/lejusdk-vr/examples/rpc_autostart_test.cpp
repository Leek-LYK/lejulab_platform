/**
 * @file rpc_autostart_test.cpp
 * @brief 手柄自启动 RPC 链路端到端验证工具
 *
 * 复刻 leju_joy_autostart_manager 对控制器发起的三个 RPC 调用：
 *   1. getRuntimeState  —— 手柄 START 前探测是否已有外部 runtime
 *   2. startRuntime     —— 第二次 START 触发控制器进入 Running
 *   3. stopRuntime      —— BACK 触发控制器优雅退出
 *
 * 作为独立 requester，对着一个已运行的控制器进程（其 ExternalInterface
 * 注册了上述 replier，由 dds_rpc::Replier 的 worker 线程独立应答，因此即使
 * 控制器主线程仍卡在 waitForDataReady（无 sim）也能往返）验证 RPC 是否打通。
 *
 * 用法：
 *   ROBOT_VERSION=17 ./rpc_autostart_test [--no-start-stop]
 *
 * 退出码：所有发起的 RPC 均往返成功返回 0，否则返回 1。
 */

#include <lejusdk-vr/lejusdk_vr.h>
#include "lejusdk-lowlevel/leju_sdk.h"
#include "lejusdk-utils/robot_version.hpp"

#include <cstring>
#include <iostream>
#include <string>

using namespace leju;

namespace {

const char* hardwareStateName(vr::HardwareState s) {
  switch (s) {
    case vr::HardwareState::kUnknown:      return "kUnknown";
    case vr::HardwareState::kInitializing: return "kInitializing";
    case vr::HardwareState::kReadyOk:      return "kReadyOk";
    case vr::HardwareState::kError:        return "kError";
    case vr::HardwareState::kStopped:      return "kStopped";
    default:                               return "?";
  }
}

const char* lifecycleStateName(vr::LifecycleState s) {
  switch (s) {
    case vr::LifecycleState::kWaitingForReady: return "kWaitingForReady";
    case vr::LifecycleState::kWaitingForStart: return "kWaitingForStart";
    case vr::LifecycleState::kRunning:         return "kRunning";
    case vr::LifecycleState::kExiting:         return "kExiting";
    default:                                   return "?";
  }
}

bool probeRuntimeState(vr::VRBaseAPI& api, const char* tag) {
  vr::RuntimeState state;
  if (!api.getRuntimeState(state, 3000)) {
    std::cerr << "[FAIL] getRuntimeState(" << tag << ") RPC 未往返（超时/无应答）\n";
    return false;
  }
  std::cout << "[ OK ] getRuntimeState(" << tag << "): data_ready="
            << (state.data_ready ? "true" : "false")
            << " hardware=" << hardwareStateName(state.hardware_state)
            << " lifecycle=" << lifecycleStateName(state.lifecycle_state) << "\n";
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  bool do_start_stop = true;
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--no-start-stop") == 0) {
      do_start_stop = false;
    }
  }

  RobotVersion version = RobotVersion::from_env();
  if (!GlobalRobot::init_env(version)) {
    std::cerr << "[FAIL] GlobalRobot::init_env() 失败\n";
    return 1;
  }

  vr::VRBaseAPI api(version);
  if (!api.initialize()) {
    std::cerr << "[FAIL] VRBaseAPI::initialize() 失败\n";
    return 1;
  }
  std::cout << "VRBaseAPI 初始化成功 (robot version " << version.to_string() << ")\n";

  bool all_ok = true;

  // 1. getRuntimeState —— 探测外部 runtime（manager HasExternalRuntime）
  all_ok &= probeRuntimeState(api, "before");

  if (do_start_stop) {
    // 2. startRuntime —— 第二次 START（manager HandleStart）
    std::string msg;
    if (api.startRuntime(&msg, 3000)) {
      std::cout << "[ OK ] startRuntime(): " << msg << "\n";
    } else {
      std::cerr << "[FAIL] startRuntime(): " << msg << "\n";
      all_ok = false;
    }

    probeRuntimeState(api, "after-start");

    // 3. stopRuntime —— BACK（manager HandleBack）
    msg.clear();
    if (api.stopRuntime(&msg, 3000)) {
      std::cout << "[ OK ] stopRuntime(): " << msg << "\n";
    } else {
      std::cerr << "[FAIL] stopRuntime(): " << msg << "\n";
      all_ok = false;
    }

    probeRuntimeState(api, "after-stop");
  }

  api.shutdown();

  if (all_ok) {
    std::cout << "\n=== RPC 自启动链路端到端往返成功 ===\n";
    return 0;
  }
  std::cerr << "\n=== RPC 自启动链路存在未往返调用 ===\n";
  return 1;
}
