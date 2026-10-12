/**
 * @file head_control_example.cpp
 * @brief 头部控制示例 - 演示如何使用 VR API 控制机器人头部
 *
 * 功能：
 * - 自动检测机器人类型（从环境变量 ROBOT_VERSION）
 * - 循环发送头部运动指令（点头 + 摇头）
 * - 头部控制无需设置模式，指令直接透传
 * - 支持 Ctrl+C 优雅退出
 *
 * 支持机器人：Kuavo4, Kuavo5, Roban2（头部都是 2 DOF）
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

  std::cout << "=== Head Control Example ===" << std::endl;

  // 1. 从环境变量获取机器人版本
  leju::RobotVersion robot_version;
  try {
    robot_version = leju::RobotVersion::from_env();
  } catch (const std::exception& e) {
    std::cerr << "Failed to get ROBOT_VERSION: " << e.what() << std::endl;
    std::cerr << "Please set ROBOT_VERSION environment variable (e.g., 5.0.0, 4.0.0, 1.4.0)" << std::endl;
    return 1;
  }
  std::cout << "Robot version: " << robot_version << std::endl;

  // 2. 根据版本创建对应的 VR API
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

  // 3. 头部控制不需要设置模式，直接发送指令即可
  std::cout << "Head control ready (no mode switch needed)" << std::endl;

  // 4. 准备头部指令
  // 头部: 2 DOF (pitch 俯仰, yaw 偏航) - 所有机器人相同
  constexpr int HEAD_DOF = 2;
  JointTrajectoryPoint head_cmd;
  head_cmd.q.resize(HEAD_DOF, 0.0);
  head_cmd.v.resize(HEAD_DOF, 0.0);
  head_cmd.acc.resize(HEAD_DOF, 0.0);

  // 5. 控制循环参数
  constexpr double FREQUENCY = 100.0;  // 100 Hz
  constexpr double DT = 1.0 / FREQUENCY;
  constexpr double PITCH_AMPLITUDE = 25.0 * M_PI / 180.0;  // 俯仰振幅 ±25 度
  constexpr double YAW_AMPLITUDE = 60.0 * M_PI / 180.0;    // 偏航振幅 ±60 度
  constexpr double PITCH_PERIOD = 3.0;     // 俯仰周期 3 秒
  constexpr double YAW_PERIOD = 5.0;       // 偏航周期 5 秒

  double t = 0.0;
  int loop_count = 0;

  std::cout << "\nStarting head control loop at " << FREQUENCY << " Hz" << std::endl;
  std::cout << "Motion: Nodding (pitch) + Shaking (yaw)" << std::endl;
  std::cout << "Press Ctrl+C to stop\n" << std::endl;

  auto last_time = std::chrono::steady_clock::now();

  while (g_running) {
    // 计算俯仰角（点头运动）
    double pitch_phase = 2.0 * M_PI * t / PITCH_PERIOD;
    head_cmd.q[0] = PITCH_AMPLITUDE * std::sin(pitch_phase);

    // 计算偏航角（摇头运动）
    double yaw_phase = 2.0 * M_PI * t / YAW_PERIOD;
    head_cmd.q[1] = YAW_AMPLITUDE * std::sin(yaw_phase);

    // 计算速度（用于前馈）
    head_cmd.v[0] = PITCH_AMPLITUDE * (2.0 * M_PI / PITCH_PERIOD) * std::cos(pitch_phase);
    head_cmd.v[1] = YAW_AMPLITUDE * (2.0 * M_PI / YAW_PERIOD) * std::cos(yaw_phase);

    // 发布指令
    if (!vr_api->publishHeadJointCmd(head_cmd)) {
      std::cerr << "Failed to publish head command" << std::endl;
    }

    // 每秒打印一次状态
    if (loop_count % static_cast<int>(FREQUENCY) == 0) {
      std::cout << "t=" << std::fixed << std::setprecision(1) << t << "s"
                << "  pitch=" << std::setprecision(3) << head_cmd.q[0] << " rad"
                << " (" << std::setprecision(1) << head_cmd.q[0] * 180.0 / M_PI << " deg)"
                << "  yaw=" << std::setprecision(3) << head_cmd.q[1] << " rad"
                << " (" << std::setprecision(1) << head_cmd.q[1] * 180.0 / M_PI << " deg)"
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

  // 6. 停止并恢复
  std::cout << "\nStopping head motion..." << std::endl;

  vr_api->shutdown();
  std::cout << "Head control example finished" << std::endl;

  return 0;
}
