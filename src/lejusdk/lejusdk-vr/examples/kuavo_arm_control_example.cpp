/**
 * @file arm_control_example.cpp
 * @brief 手臂控制示例 - 演示如何使用 VR API 控制机器人手臂
 *
 * 功能：
 * - 设置手臂为外部控制模式
 * - 循环发送正弦波关节指令
 * - 支持 Ctrl+C 优雅退出
 */

#include <lejusdk-vr/lejusdk_vr.h>

#include <chrono>
#include <cmath>
#include <csignal>
#include <iomanip>
#include <iostream>
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

  std::cout << "=== Arm Control Example ===" << std::endl;

  // 1. 创建并初始化 VR API
  KuavoVRAPI vr_api;
  if (!vr_api.initialize()) {
    std::cerr << "Failed to initialize VR API" << std::endl;
    return 1;
  }
  std::cout << "VR API initialized successfully" << std::endl;

  // 2. 设置手臂为外部控制模式
  std::cout << "Setting arm to external control mode..." << std::endl;
  if (!vr_api.setArmMode(ControlMode::kExternal)) {
    std::cerr << "Failed to set arm mode" << std::endl;
    return 1;
  }
  std::cout << "Arm mode set to External" << std::endl;

  // 3. 准备手臂指令
  // Kuavo 手臂: 14 DOF (左臂 7 + 右臂 7)
  constexpr int ARM_DOF = 14;
  JointTrajectoryPoint arm_cmd;
  arm_cmd.q.resize(ARM_DOF, 0.0);
  arm_cmd.v.resize(ARM_DOF, 0.0);
  arm_cmd.acc.resize(ARM_DOF, 0.0);

  // 4. 控制循环参数
  constexpr double FREQUENCY = 100.0;  // 100 Hz
  constexpr double DT = 1.0 / FREQUENCY;
  constexpr double SHOULDER_PITCH_MAX = 60.0 * M_PI / 180.0;  // 肩部前抬最大角度 60 度
  constexpr double SHOULDER_ROLL_MAX = 30.0 * M_PI / 180.0;   // 肩部外展最大角度 30 度
  constexpr double ELBOW_MAX = 90.0 * M_PI / 180.0;  // 手肘最大角度 90 度
  constexpr double PERIOD = 4.0;       // 周期 4 秒

  double t = 0.0;
  int loop_count = 0;

  std::cout << "\nStarting arm control loop at " << FREQUENCY << " Hz" << std::endl;
  std::cout << "Press Ctrl+C to stop\n" << std::endl;

  auto last_time = std::chrono::steady_clock::now();

  while (g_running) {
    // 计算正弦波指令
    double phase = 2.0 * M_PI * t / PERIOD;

    // 手臂关节定义 (7 DOF 版本):
    // [0] zarm_l1 = shoulder_pitch (肩部前屈/后伸)
    // [1] zarm_l2 = shoulder_roll  (肩部外展/内收)
    // [2] zarm_l3 = shoulder_yaw   (肩部旋转)
    // [3] zarm_l4 = elbow          (肘部弯曲)
    // [4] zarm_l5 = wrist_roll
    // [5] zarm_l6 = wrist_pitch
    // [6] zarm_l7 = wrist_yaw
    // 右臂: [7]-[13] 对应 zarm_r1 ~ zarm_r7

    // 左臂运动
    arm_cmd.q[0] = -SHOULDER_PITCH_MAX * 0.5 * (1.0 - std::cos(phase));  // shoulder_pitch: 向前抬起 0~-60度
    arm_cmd.q[1] = SHOULDER_ROLL_MAX * 0.5 * (1.0 - std::cos(phase));    // shoulder_roll: 向外展开
    arm_cmd.q[2] = 0.0;                           // shoulder_yaw: 保持不动
    arm_cmd.q[3] = -ELBOW_MAX * 0.5 * (1.0 - std::cos(phase));  // elbow: 0~-90度弯曲

    // 右臂镜像运动
    arm_cmd.q[7] = -SHOULDER_PITCH_MAX * 0.5 * (1.0 - std::cos(phase));  // shoulder_pitch: 向前抬起 0~-60度
    arm_cmd.q[8] = -SHOULDER_ROLL_MAX * 0.5 * (1.0 - std::cos(phase));   // shoulder_roll: 向外展开
    arm_cmd.q[9] = 0.0;                           // shoulder_yaw: 保持不动
    arm_cmd.q[10] = -ELBOW_MAX * 0.5 * (1.0 - std::cos(phase));  // elbow: 0~-90度弯曲

    // 发布指令
    if (!vr_api.publishArmJointCmd(arm_cmd)) {
      std::cerr << "Failed to publish arm command" << std::endl;
    }

    // 每秒打印一次状态
    if (loop_count % static_cast<int>(FREQUENCY) == 0) {
      std::cout << "t=" << std::fixed << std::setprecision(1) << t << "s"
                << "  shoulder_pitch=" << std::setprecision(1) << arm_cmd.q[0] * 180.0 / M_PI << "deg"
                << "  elbow=" << arm_cmd.q[3] * 180.0 / M_PI << "deg" << std::endl;
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

  // 5. 停止并恢复
  std::cout << "\nStopping arm motion..." << std::endl;

  // 恢复自动模式（会自动插值过渡，不需要发送零位指令）
  std::cout << "Restoring arm to auto mode..." << std::endl;
  vr_api.setArmMode(ControlMode::kAuto);

  vr_api.shutdown();
  std::cout << "Arm control example finished" << std::endl;

  return 0;
}
