/**
 * @file vr_cmd_handler.cpp
 * @brief VR 指令处理示例 - 控制器订阅指令、注册 Handler
 *
 * 本示例演示如何使用 lejusdk-vr SDK 作为服务端（控制器端）：
 * - 订阅关节指令（手臂、头部、腰部）
 * - 订阅速度指令（控制器输入与 VR 输入）
 * - 注册 RPC Handler（设置模式、切换控制器、获取状态）
 */

#include <lejusdk-vr/lejusdk_vr.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <mutex>
#include <thread>

using namespace leju::vr;

// 全局状态
std::atomic<bool> g_running{true};
std::mutex g_state_mutex;

// 模拟的控制器状态
struct SimulatedControllerState {
  std::string current_controller = "amp";
  ControlMode arm_mode = ControlMode::kAuto;
  ControlMode head_mode = ControlMode::kAuto;
  ControlMode waist_mode = ControlMode::kAuto;
  std::vector<std::string> available_controllers = {"amp", "mimic", "vr"};
};
SimulatedControllerState g_controller_state;

void signalHandler(int signum) {
  std::cout << "\nReceived signal " << signum << ", shutting down..." << std::endl;
  g_running = false;
}

int main(int argc, char** argv) {
  std::cout << "=== VR Server Example ===" << std::endl;

  // 设置信号处理
  signal(SIGINT, signalHandler);
  signal(SIGTERM, signalHandler);

  // 1. 创建 API 实例
  KuavoVRAPI vr_api;

  if (!vr_api.initialize()) {
    std::cerr << "Failed to initialize VR API" << std::endl;
    return 1;
  }
  std::cout << "VR API initialized successfully" << std::endl;

  // 2. 订阅手臂关节指令
  std::cout << "\n--- Subscribing to arm joint commands ---" << std::endl;
  vr_api.subscribeArmJointCmd([](const JointTrajectoryPoint& cmd) {
    std::cout << "[Arm] Received joint command with " << cmd.q.size()
              << " joints" << std::endl;
    if (!cmd.q.empty()) {
      std::cout << "  q[0] = " << cmd.q[0] << std::endl;
    }
  });

  // 3. 订阅头部关节指令
  std::cout << "--- Subscribing to head joint commands ---" << std::endl;
  vr_api.subscribeHeadJointCmd([](const JointTrajectoryPoint& cmd) {
    std::cout << "[Head] Received joint command with " << cmd.q.size()
              << " joints" << std::endl;
  });

  // 4. 订阅腰部关节指令
  std::cout << "--- Subscribing to waist joint commands ---" << std::endl;
  vr_api.subscribeWaistJointCmd([](const JointTrajectoryPoint& cmd) {
    std::cout << "[Waist] Received joint command with " << cmd.q.size()
              << " joints" << std::endl;
  });

  // 5. 订阅控制器输入速度指令
  std::cout << "--- Subscribing to resolved velocity commands ---" << std::endl;
  vr_api.subscribeVelocityCmd([](const VelocityCmd& cmd) {
    std::cout << "[Velocity] Received: linear_x=" << cmd.linear_x
              << ", linear_y=" << cmd.linear_y << ", angular_z=" << cmd.angular_z
              << std::endl;
  });

  // 6. 订阅 VR 输入速度指令
  std::cout << "--- Subscribing to VR velocity commands ---" << std::endl;
  vr_api.subscribeVrVelocityCmd([](const VrVelocityCmd& cmd) {
    std::cout << "[VrVelocity] Received: linear_x=" << cmd.linear_x
              << ", linear_y=" << cmd.linear_y << ", angular_z=" << cmd.angular_z
              << std::endl;
  });

  // 7. 注册切换控制器 Handler
  std::cout << "\n--- Registering RPC handlers ---" << std::endl;
  vr_api.registerSwitchControllerHandler(
      [](const std::string& name, std::string& message) -> bool {
        std::lock_guard<std::mutex> lock(g_state_mutex);

        // 检查控制器是否可用
        auto& available = g_controller_state.available_controllers;
        bool found = false;
        for (const auto& ctrl : available) {
          if (ctrl == name) {
            found = true;
            break;
          }
        }

        if (!found) {
          message = "Controller '" + name + "' not available";
          std::cout << "[SwitchController] Failed: " << message << std::endl;
          return false;
        }

        g_controller_state.current_controller = name;
        message = "Switched to controller: " + name;
        std::cout << "[SwitchController] " << message << std::endl;
        return true;
      });

  // 8. 注册设置手臂模式 Handler
  vr_api.registerSetArmModeHandler(
      [](ControlMode mode, std::string& message) -> bool {
        std::lock_guard<std::mutex> lock(g_state_mutex);
        g_controller_state.arm_mode = mode;
        message = "Arm mode set to " + std::to_string(static_cast<int>(mode));
        std::cout << "[SetArmMode] " << message << std::endl;
        return true;
      });

  // 9. 注册设置头部模式 Handler
  vr_api.registerSetHeadModeHandler(
      [](ControlMode mode, std::string& message) -> bool {
        std::lock_guard<std::mutex> lock(g_state_mutex);
        g_controller_state.head_mode = mode;
        message = "Head mode set to " + std::to_string(static_cast<int>(mode));
        std::cout << "[SetHeadMode] " << message << std::endl;
        return true;
      });

  // 10. 注册设置腰部模式 Handler
  vr_api.registerSetWaistModeHandler(
      [](ControlMode mode, std::string& message) -> bool {
        std::lock_guard<std::mutex> lock(g_state_mutex);
        g_controller_state.waist_mode = mode;
        message = "Waist mode set to " + std::to_string(static_cast<int>(mode));
        std::cout << "[SetWaistMode] " << message << std::endl;
        return true;
      });

  // 11. 注册获取状态 Handler
  vr_api.registerGetRuntimeStateHandler([]() -> RuntimeState {
    RuntimeState state;
    state.data_ready = true;
    state.hardware_state = HardwareState::kReadyOk;
    state.lifecycle_state = LifecycleState::kRunning;
    std::cout << "[GetRuntimeState] Returning runtime state" << std::endl;
    return state;
  });
  vr_api.registerGetControllerStateHandler([]() -> ControllerState {
    std::lock_guard<std::mutex> lock(g_state_mutex);
    ControllerState state;
    state.current_controller = g_controller_state.current_controller;
    state.arm_mode = g_controller_state.arm_mode;
    state.head_mode = g_controller_state.head_mode;
    state.waist_mode = g_controller_state.waist_mode;
    state.available_controllers = g_controller_state.available_controllers;
    std::cout << "[GetControllerState] Returning controller state" << std::endl;
    return state;
  });
  vr_api.registerGetMotionStateHandler([]() -> MotionState {
    MotionState state;
    state.supported = true;
    state.current_motion_name = "demo_motion";
    state.available_motion_names = {"demo_motion"};
    std::cout << "[GetMotionState] Returning motion state" << std::endl;
    return state;
  });

  std::cout << "\nServer is running. Press Ctrl+C to stop." << std::endl;

  // 12. 主循环
  while (g_running) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // 13. 关闭
  vr_api.shutdown();
  std::cout << "VR API shutdown complete" << std::endl;

  return 0;
}
