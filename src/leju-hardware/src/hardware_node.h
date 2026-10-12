#pragma once

#include <atomic>
#include <thread>
#include "leju_robot_interface.h"
#include "lejusdk-utils/loop.hpp"
#include "lejusdk-lowlevel/robot_api/robot_base.h"

namespace leju {

class HardwareNode : public RobotInterface {
public:
    static constexpr int kSensorPublishPeriodMs = 1;  // 1ms = 1000Hz
    static constexpr int kSensorLoopCpuCore = -1;     // 不绑定CPU核心
    static constexpr int kSensorLoopSchedPriority = 30; // SCHED_FIFO 优先级

    /**
   * @brief 构造函数
   * @param config_dir 配置文件的根目录路径
   */
  explicit HardwareNode(const std::string& config_dir, bool bypass_ankle_solver = false);
  ~HardwareNode();

    /**
     * @brief 初始化硬件节点，建立DDS通讯、发布硬件节点状态就绪
     * @return true/false
     */
    bool init();

    /**
     * @brief 运行硬件节点主线程
     *
     * @note 在 `init()`成功之后，调用该函数持续运行节点
     *
     */
    void run();

    /**
     * @brief 关闭硬件节点并释放相关资源
     *
     */
    void close();

private:
    /**
     * @brief 处理从 DDS 接收到的关节命令, 调用硬件控制
     **/
    void handleJointCommand(const RobotCmdConstPtr& cmd);

    /**
     * @brief 处理从 DDS 接收到的手部命令, 调用硬件控制
     **/
    void handleHandCommand(const HandCmdConstPtr& cmd);

    /**
     * @brief 处理手柄数据: 按下 start (非 back+start 退出组合) 激活头部 runtime_skip,
     * 头部电机从"回零保持"切换为"仅使能不控制"
     **/
    void handleJoyData(const JoyDataConstPtr& joy);

    /**
     * @brief 发布传感器数据的线程函数
     **/
    void sensorThreadFunc();

    /**
     * @brief 正弦测试线程函数 (绕过DDS, 直接调用硬件SDK)
     * 触发方式: touch /tmp/sine_test_start 开始, rm /tmp/sine_test_start 停止
     **/
    void sineTestThreadFunc();

private:
    std::string config_dir_;  // 配置文件根目录路径
    std::unique_ptr<leju::Loop> sensors_loop_;
    size_t num_joints_ = 0;
    std::atomic<bool> sine_test_active_{false};  // 正弦测试激活标志
    std::thread sine_test_thread_;               // 正弦测试线程
    bool bypass_ankle_solver_ = false;           // 踝关节解算绕过标志
    bool lowerlimb_fault_protect_ = false;       // 下肢电机故障保护已触发 (latch, 仅 sensor 线程访问)
    std::atomic<bool> runtime_skip_activated_{false};  // start 按下后置 true (一次性), 激活头部 runtime_skip
    std::atomic<bool> prev_joy_start_{false};    // start 按键上一帧状态 (边沿检测)
};

} // namespace leju
