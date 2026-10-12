/**
 * @file vr_cmd_publisher.cpp
 * @brief VR 指令发布示例 - 外部应用发布指令、调用 RPC
 *
 * 本示例演示如何使用 lejusdk-vr SDK 作为客户端（VR 应用端）：
 * - 发布关节指令（手臂、头部、腰部）
 * - 发布速度指令
 * - 调用 RPC 服务（设置模式、切换控制器、获取状态）
 */

#include <lejusdk-vr/lejusdk_vr.h>

#include <chrono>
#include <iostream>
#include <thread>

using namespace leju::vr;

int main(int argc, char** argv) {
  std::cout << "=== VR Client Example ===" << std::endl;

  // 1. 创建 API 实例（默认 Kuavo 机器人）
  KuavoVRAPI vr_api;

  if (!vr_api.initialize()) {
    std::cerr << "Failed to initialize VR API" << std::endl;
    return 1;
  }
  std::cout << "VR API initialized successfully" << std::endl;

  // 2. 获取当前控制器状态
  std::cout << "\n--- Getting controller state ---" << std::endl;
  ControllerState state;
  if (vr_api.getControllerState(state)) {
    std::cout << "Current controller: " << state.current_controller << std::endl;
    std::cout << "Arm mode: " << static_cast<int>(state.arm_mode) << std::endl;
    std::cout << "Available controllers: ";
    for (const auto& ctrl : state.available_controllers) {
      std::cout << ctrl << " ";
    }
    std::cout << std::endl;
  } else {
    std::cerr << "Failed to get controller state" << std::endl;
  }

  // 3. 设置手臂为外部控制模式
  std::cout << "\n--- Setting arm to external mode ---" << std::endl;
  if (!vr_api.setArmMode(ControlMode::kExternal)) {
    std::cerr << "Failed to set arm mode" << std::endl;
    return 1;
  }
  std::cout << "Arm mode set to External successfully" << std::endl;

  // 4. 切换控制器（如果需要）
  std::cout << "\n--- Switching controller ---" << std::endl;
  if (vr_api.switchController("amp")) {
    std::cout << "Switched to 'amp' controller successfully" << std::endl;
  } else {
    std::cerr << "Failed to switch controller" << std::endl;
  }

  // 5. 发布手臂关节指令
  std::cout << "\n--- Publishing arm joint commands ---" << std::endl;
  JointTrajectoryPoint arm_cmd;
  // 关节数量取决于机器人版本
  arm_cmd.q = {0.0, 0.5, -0.3, 0.0, 0.0, 0.0, 0.0,   // 左臂
               0.0, -0.5, 0.3, 0.0, 0.0, 0.0, 0.0};  // 右臂
  arm_cmd.v.resize(arm_cmd.q.size(), 0.0);
  arm_cmd.acc.resize(arm_cmd.q.size(), 0.0);

  for (int i = 0; i < 10; ++i) {
    if (vr_api.publishArmJointCmd(arm_cmd)) {
      std::cout << "Published arm command #" << i + 1 << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // 6. 发布头部关节指令
  std::cout << "\n--- Publishing head joint commands ---" << std::endl;
  JointTrajectoryPoint head_cmd;
  head_cmd.q = {0.1, 0.0};  // pitch, yaw
  head_cmd.v = {0.0, 0.0};
  head_cmd.acc = {0.0, 0.0};

  if (vr_api.publishHeadJointCmd(head_cmd)) {
    std::cout << "Published head command" << std::endl;
  }

  // 7. 发布腰部关节指令
  std::cout << "\n--- Publishing waist joint commands ---" << std::endl;
  JointTrajectoryPoint waist_cmd;
  waist_cmd.q = {0.05};  // 单自由度腰部
  waist_cmd.v = {0.0};
  waist_cmd.acc = {0.0};

  if (vr_api.publishWaistJointCmd(waist_cmd)) {
    std::cout << "Published waist command" << std::endl;
  }

  // 8. 发布 VR 输入速度指令
  std::cout << "\n--- Publishing VR velocity commands ---" << std::endl;
  VrVelocityCmd vel_cmd;
  vel_cmd.linear_x = 0.3;   // 前进 0.3 m/s
  vel_cmd.linear_y = 0.0;
  vel_cmd.angular_z = 0.1;  // 逆时针旋转 0.1 rad/s

  for (int i = 0; i < 5; ++i) {
    if (vr_api.publishVrVelocityCmd(vel_cmd)) {
      std::cout << "Published VR velocity command #" << i + 1 << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // 9. 停止运动
  std::cout << "\n--- Stopping robot ---" << std::endl;
  vel_cmd.linear_x = 0.0;
  vel_cmd.angular_z = 0.0;
  vr_api.publishVrVelocityCmd(vel_cmd);

  // 10. 恢复手臂到自动模式
  std::cout << "\n--- Restoring arm to auto mode ---" << std::endl;
  // vr_api.setArmMode(ControlMode::kAuto);

  // 11. 关闭
  vr_api.shutdown();
  std::cout << "\nVR API shutdown complete" << std::endl;

  return 0;
}
