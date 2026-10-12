/**
 * @file waist_control_example.cpp
 * @brief 腰部控制示例 - 演示如何使用 VR API 控制机器人腰部
 *
 * 功能：
 * - 自动检测机器人类型（从环境变量 ROBOT_VERSION）
 * - 设置腰部为外部控制模式
 * - 循环发送腰部旋转指令
 * - 支持 Ctrl+C 优雅退出
 *
 * 支持机器人：
 * - Kuavo5: 1 DOF (waist_yaw)
 * - Roban2: 1 DOF (waist_yaw_joint)
 * - Kuavo4: 无腰部关节（不支持）
 */

#include <lejusdk-vr/lejusdk_vr.h>
#include <lejusdk-utils/robot_version.hpp>

#include <chrono>
#include <cmath>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <memory>
#include <thread>

using namespace leju::vr;

// 全局变量用于信号处理
volatile bool g_running = true;

void signalHandler(int signum) {
  std::cout << "\nReceived signal " << signum << ", stopping..." << std::endl;
  g_running = false;
}

int main(int argc, char** argv) {
  // 注册信号处理
  signal(SIGINT, signalHandler);
  signal(SIGTERM, signalHandler);

  std::cout << "=== Waist Control Example ===" << std::endl;

  // 1. 从环境变量获取机器人版本
  leju::RobotVersion robot_version;
  try {
    robot_version = leju::RobotVersion::from_env();
  } catch (const std::exception& e) {
    std::cerr << "Failed to get ROBOT_VERSION: " << e.what() << std::endl;
    std::cerr << "Please set ROBOT_VERSION environment variable (e.g., 5.0.0, 1.4.0)" << std::endl;
    return 1;
  }
  std::cout << "Robot version: " << robot_version << std::endl;

  // 2. 检查是否支持腰部控制
  if (IS_KUAVO4_LEGGED(robot_version)) {
    std::cerr << "Kuavo4 does not have waist joint, exiting..." << std::endl;
    return 1;
  }

  // 3. 根据版本创建对应的 VR API
  std::unique_ptr<VRBaseAPI> vr_api;
  if (IS_KUAVO(robot_version)) {
    vr_api = std::make_unique<KuavoVRAPI>(robot_version);
    std::cout << "Using KuavoVRAPI" << std::endl;
  } else if (IS_ROBAN(robot_version)) {
    vr_api = std::make_unique<RobanVRAPI>(robot_version);
    std::cout << "Using RobanVRAPI" << std::endl;
  } else {
    std::cerr << "Unsupported robot version" << std::endl;
    return 1;
  }

  if (!vr_api->initialize()) {
    std::cerr << "Failed to initialize VR API" << std::endl;
    return 1;
  }
  std::cout << "VR API initialized successfully" << std::endl;

  // 4. 设置腰部为外部控制模式
  std::cout << "Setting waist to external control mode..." << std::endl;
  if (!vr_api->setWaistMode(ControlMode::kExternal)) {
    std::cerr << "Failed to set waist mode" << std::endl;
    return 1;
  }
  std::cout << "Waist mode set to External" << std::endl;

  // 5. 准备腰部指令
  // 腰部: 1 DOF (yaw 偏航/旋转) - Kuavo5 和 Roban2 相同
  constexpr int WAIST_DOF = 1;
  JointTrajectoryPoint waist_cmd;
  waist_cmd.q.resize(WAIST_DOF, 0.0);
  waist_cmd.v.resize(WAIST_DOF, 0.0);
  waist_cmd.acc.resize(WAIST_DOF, 0.0);

  // 6. 控制循环参数
  constexpr double FREQUENCY = 100.0;  // 100 Hz
  constexpr double DT = 1.0 / FREQUENCY;
  constexpr double AMPLITUDE = 30.0 * M_PI / 180.0;  // 振幅 +-30 度
  constexpr double PERIOD = 4.0;       // 周期 4 秒

  double t = 0.0;
  int loop_count = 0;

  std::cout << "\nStarting waist control loop at " << FREQUENCY << " Hz" << std::endl;
  std::cout << "Motion: Left-right rotation" << std::endl;
  std::cout << "Amplitude: " << AMPLITUDE << " rad (" << AMPLITUDE * 180.0 / M_PI << " deg)" << std::endl;
  std::cout << "Press Ctrl+C to stop\n" << std::endl;

  auto last_time = std::chrono::steady_clock::now();

  while (g_running) {
    // 计算腰部旋转角度（正弦波运动）
    double phase = 2.0 * M_PI * t / PERIOD;
    waist_cmd.q[0] = AMPLITUDE * std::sin(phase);

    // 计算速度（用于前馈）
    waist_cmd.v[0] = AMPLITUDE * (2.0 * M_PI / PERIOD) * std::cos(phase);

    // 发布指令
    if (!vr_api->publishWaistJointCmd(waist_cmd)) {
      std::cerr << "Failed to publish waist command" << std::endl;
    }

    // 每秒打印一次状态
    if (loop_count % static_cast<int>(FREQUENCY) == 0) {
      std::cout << "t=" << std::fixed << std::setprecision(1) << t << "s"
                << "  waist=" << std::setprecision(3) << waist_cmd.q[0] << " rad"
                << " (" << std::setprecision(1) << waist_cmd.q[0] * 180.0 / M_PI << " deg)"
                << "  vel=" << std::setprecision(3) << waist_cmd.v[0] << " rad/s"
                << std::endl;
    }

    // 更新时间
    t += DT;
    loop_count++;

    // 精确定时
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(now - last_time);
    auto sleep_time = std::chrono::microseconds(static_cast<int>(DT * 1e6)) - elapsed;
    if (sleep_time.count() > 0) {
      std::this_thread::sleep_for(sleep_time);
    }
    last_time = std::chrono::steady_clock::now();
  }

  // 7. 停止并恢复
  std::cout << "\nStopping waist motion..." << std::endl;

  // 恢复自动模式（会自动插值过渡，不需要发送零位指令）
  std::cout << "Restoring waist to auto mode..." << std::endl;
  vr_api->setWaistMode(ControlMode::kAuto);

  vr_api->shutdown();
  std::cout << "Waist control example finished" << std::endl;

  return 0;
}
