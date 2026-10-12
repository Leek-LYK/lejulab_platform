#include "motorevo/motor_ctrl.h"
#include "motorevo/motor_def.h"
#include "canbus_sdk/canbus_sdk.h"

#include <iostream>
#include <vector>
#include <map>
#include <thread>
#include <chrono>
#include <cmath>
#include <sstream>
#include <iomanip>
#include <atomic>
#include <csignal>
#include <cstdlib>

// ControlLoop类的实现 - 按照DmHWLoop模式
class ControlLoop {
public:
    explicit ControlLoop(std::shared_ptr<motorevo::RevoMotorControl> motor_control)
        : motor_control_(motor_control)
        , running_(true)
        , loop_thread_([this]() {
            while (running_) {
                this->update();
                std::this_thread::sleep_for(std::chrono::milliseconds(4)); // 250Hz
            }
        }) {
    }

    ~ControlLoop() {
        // 析构时停止线程
        running_ = false;
        if (loop_thread_.joinable()) {
            loop_thread_.join();
        }
    }

    void update() {
        if (motor_control_) {
            motor_control_->write();
        }
    }

    void stop() {
        running_ = false;
    }

private:
    // Timing
    std::thread loop_thread_;
    std::atomic<bool> running_;

    // Motor control interface
    std::shared_ptr<motorevo::RevoMotorControl> motor_control_;

    // 禁止拷贝和赋值
    ControlLoop(const ControlLoop&) = delete;
    ControlLoop& operator=(const ControlLoop&) = delete;
    ControlLoop(ControlLoop&&) = delete;
    ControlLoop& operator=(ControlLoop&&) = delete;
};

// 全局变量用于信号处理
std::atomic<bool> g_running(true);
ControlLoop* g_control_loop = nullptr;

// 信号处理函数
void signalHandler(int signal) {
    std::cout << "\n\033[33m接收到信号 " << signal << "，正在优雅退出...\033[0m" << std::endl;
    g_running = false;
    if (g_control_loop) {
        g_control_loop->stop();
    }
}
int main() {
    // 设置信号处理
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    std::cout << "\033[32m=== RevoMotorControl 控制测试程序 ===\033[0m" << std::endl;
    std::cout << "\033[36m按 CTRL+C 可随时退出程序\033[0m" << std::endl;

    ////////////////////////////////////////////////////////////////////////////
    // 初始化 CANBUS_SDK
    ////////////////////////////////////////////////////////////////////////////
    std::cout << "\033[36m初始化CAN总线控制器...\033[0m" << std::endl;
    auto& canbus_controller = canbus_sdk::CanBusController::getInstance();
    canbus_controller.init();

    // 配置CAN总线比特率
    canbus_sdk::CanBusBitrate bitrate = {
        .nbitrate = 1000,
        .dbitrate = 2000,
        .nsamplepos = 75,
        .dsamplepos = 75
    };

    // 打开CAN总线
    auto result = canbus_controller.openCanBus("can0", canbus_sdk::CanBusModelType::BUSMUST_A, bitrate);
    if (!result.has_value()) {
        std::cout << "\033[31mCAN总线打开失败: " << canbus_sdk::errorToString(result.error()) << "\033[0m" << std::endl;
        return -1;
    }

    canbus_sdk::BusId bus_id = result.value();
    std::cout << "\033[32mCAN总线打开成功, bus_id: " << bus_id << "\033[0m" << std::endl;

    ////////////////////////////////////////////////////////////////////////////
    // 配置电机
    ////////////////////////////////////////////////////////////////////////////
    std::cout << "\033[36m配置电机参数...\033[0m" << std::endl;

    // 电机配置
    std::vector<motorevo::RevoMotorConfig_t> motor_configs;

    // 定义电机ID列表
    std::vector<uint8_t> motor_ids = {0x1, 0x2};

    // 使用for循环生成电机配置
    for (uint8_t id : motor_ids) {
        std::ostringstream name_stream;
        name_stream << "motor_" << static_cast<int>(id);

        motor_configs.push_back({
            .id = static_cast<motorevo::MotorId>(id),
            .name = name_stream.str(),
            .negtive = false,        // 正方向
            .ignore = false,         // 不忽略
            .zero_offset = -0.10f,     // 零点偏移
            .default_params = {
                .vel = 0.0f,         // 速度参数
                .kp_pos = 35.0f,      // 位置比例增益
                .kd_pos = 2.0f,      // 位置微分增益
                .tor = 0.0f,         // 力矩参数
                .kp_vel = 0.0f,      // 速度比例增益
                .kd_vel = 0.0f,      // 速度微分增益
                .ki_vel = 0.0f       // 速度积分增益
            }
        });
    }

    // 创建电机控制器 (传递CAN总线名称，内部注册设备)
    std::cout << "\033[36m创建电机控制器...\033[0m" << std::endl;
    auto motor_control = std::make_shared<motorevo::RevoMotorControl>("can0");

    // 创建控制循环（构造时自动启动250Hz线程）
    std::cout << "\033[36m创建控制循环...\033[0m" << std::endl;
    g_control_loop = new ControlLoop(motor_control);

    // 初始化电机控制器 (内部注册设备和回调)
    if (!motor_control->init(motor_configs, false)) {
        std::cout << "\033[31m电机控制器初始化失败\033[0m" << std::endl;
        return -1;
    }

    std::cout << "\033[32m电机控制器初始化成功\033[0m" << std::endl;

    ////////////////////////////////////////////////////////////////////////////
    // 测试控制功能
    ////////////////////////////////////////////////////////////////////////////

    
    // 初始化目标位置为0
    std::map<motorevo::MotorId, motorevo::RevoMotorCmd_t> initial_targets;
    for (uint8_t id : motor_ids) {
        motorevo::MotorId motor_id = static_cast<motorevo::MotorId>(id);
        initial_targets[motor_id] = {
            .pos = 0.0,
            .vel = 0.0,
            .torque = 0.0,
            .kp = 35.0,
            .kd = 2.0
        };
    }
    motor_control->setTargetPositions(initial_targets);

    std::cout << "\033[32m250Hz控制循环启动成功\033[0m" << std::endl;

    // 正弦波控制测试
    std::cout << "\033[36m开始正弦波控制测试...\033[0m" << std::endl;

    const double amplitude_degrees = 15.0;   // 振幅 (degrees)
    const double amplitude = amplitude_degrees * M_PI / 180.0;  // 转换为弧度
    const double frequency = 0.5;      // 频率 (Hz)
    const double duration = 100.0;       // 测试持续时间 (s)
    const int display_freq = 10;       // 显示频率 (Hz)
    const int display_sleep_ms = 1000 / display_freq;

    auto start_time = std::chrono::steady_clock::now();
    int total_iterations = static_cast<int>(duration * display_freq);

    for (int i = 0; i < total_iterations && g_running; ++i) {
        // 计算当前时间
        auto current_time = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(current_time - start_time).count();

        // 计算正弦波目标位置
        double target_pos = amplitude * std::sin(2 * M_PI * frequency * elapsed);

        // 更新目标位置
        std::map<motorevo::MotorId, motorevo::RevoMotorCmd_t> new_targets;
        for (uint8_t id : motor_ids) {
            motorevo::MotorId motor_id = static_cast<motorevo::MotorId>(id);

            // 为不同电机添加相位差
            double phase_shift = (id == 0x1) ? 0.0 : M_PI / 2; // 电机2相位差90度
            // 基于0的正弦波
            double motor_target_pos = amplitude * std::sin(2 * M_PI * frequency * elapsed + phase_shift);

            new_targets[motor_id] = {
                .pos = motor_target_pos,    // 目标位置 (rad)
                .vel = 0.0,                 // 目标速度 (rad/s)
                .torque = 0.0,              // 目标力矩 (N·m)
                .kp = 35.0,                 // 位置比例增益
                .kd = 2.0                   // 位置微分增益
            };
        }
        motor_control->setTargetPositions(new_targets);

        // 获取位置反馈并显示
        auto positions = motor_control->getPositions();
        std::cout << "\r时间: " << std::fixed << std::setprecision(1) << elapsed << "s";
        for (const auto& [id, pos] : positions) {
            // 计算该电机的目标位置用于显示
            double phase_shift = (id == 0x1) ? 0.0 : M_PI / 2;
            double motor_target_pos = amplitude * std::sin(2 * M_PI * frequency * elapsed + phase_shift);

            std::cout << "  电机" << static_cast<int>(id) << ": 目标=" << std::fixed << std::setprecision(3) << motor_target_pos
                     << ", 实际=" << std::fixed << std::setprecision(3) << pos;
        }
        std::cout << std::flush;

        // 等待下一个显示周期
        std::this_thread::sleep_for(std::chrono::milliseconds(display_sleep_ms));
    }

    // 检查是否因为信号而提前退出
    if (!g_running) {
        std::cout << "\n\033[33m检测到退出信号，正在停止控制循环...\033[0m" << std::endl;
    }

    std::cout << "\n\033[32m正弦波控制测试完成\033[0m" << std::endl;

    // 停止控制循环（手动删除指针，析构时自动停止线程）
    std::cout << "\033[36m停止控制循环...\033[0m" << std::endl;
    delete g_control_loop;
    g_control_loop = nullptr;

    // 优雅退出电机控制器
    std::cout << "\033[36m释放电机控制器...\033[0m" << std::endl;
    motor_control.reset();
    std::cout << "\033[32m电机控制器已释放\033[0m" << std::endl;

    std::this_thread::sleep_for(std::chrono::seconds(2));

    return 0;
}