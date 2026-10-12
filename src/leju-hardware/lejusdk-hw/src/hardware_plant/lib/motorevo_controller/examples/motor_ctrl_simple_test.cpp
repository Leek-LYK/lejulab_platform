#include "motorevo/motor_ctrl.h"
#include "motorevo/motor_def.h"
#include "canbus_sdk/canbus_sdk.h"

#include <iostream>
#include <atomic>
#include <chrono>
#include <thread>
#include <cmath>

#define MOTOR_ID 0x02

motorevo::RevoMotor *motor = nullptr;
canbus_sdk::CanBusController *canbus_controller = nullptr;
std::atomic<bool> feedback_received_flag(true);

void can_receive_callback(canbus_sdk::CanMessageFrame* frame, const canbus_sdk::CallbackContext* context);

int main() {
    ////////////////////////////////////////////////////////////////////////////
    // 初始化 CANBUS_SDK !
    // 初始化CAN总线控制器
    using namespace canbus_sdk;
    canbus_controller = &CanBusController::getInstance();
    canbus_controller->init();

    // 配置CAN总线比特率
    CanBusBitrate bitrate = {
        .nbitrate = 1000,
        .dbitrate = 2000,    
        .nsamplepos = 75,   
        .dsamplepos = 75
    };

    // 打开CAN总线
    auto result = canbus_controller->openCanBus("can0", CanBusModelType::BUSMUST_A, bitrate);
    if (!result.has_value()) {
        printf("\033[31mCAN总线打开失败: %s\033[0m\n", errorToString(result.error()));
        return -1;
    }

    BusId bus_id = result.value();
    printf("\033[32mCAN总线打开成功, bus_id: %d\033[0m\n", bus_id);
    ////////////////////////////////////////////////////////////////////////////
    canbus_sdk::CallbackParams callback_params{
        .msg_callback = can_receive_callback,
        .msg_cb_ctx = nullptr,
        .tef_callback = nullptr,
        .tef_cb_ctx = nullptr
    };
    canbus_controller->registerDevice(canbus_sdk::DeviceInfo(
        "motor0",
        canbus_sdk::DeviceType::MOTOR,
        MOTOR_ID,
        nullptr /* use default */
    ), "can0", callback_params);
    printf("\033[32m设备注册成功: motor0 (ID: %d)\033[0m\n", MOTOR_ID);

    motor = new motorevo::RevoMotor(bus_id, MOTOR_ID, motorevo::MotorMode::TorquePositionMixControlMode);

    // 重置反馈标志
    feedback_received_flag.store(false);

    // 发送进入电机状态命令
    printf("\033[36m发送进入电机状态命令...\033[0m\n");
    motor->enterMotorState();

    // 等待反馈，超时1秒
    printf("\033[36m等待电机反馈 (超时: 1秒)...\033[0m\n");
    auto start_time = std::chrono::steady_clock::now();
    bool feedback_timeout = false;

    while (!feedback_received_flag.load()) {
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start_time);

        if (elapsed.count() >= 1000) {
            printf("\033[31m错误: 电机反馈超时 %ld 毫秒\033[0m\n", elapsed.count());
            feedback_timeout = true;
            break;
        }

        // 短暂休眠避免忙等待
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (feedback_timeout) {
        printf("\033[31m错误: 未收到电机反馈，程序退出...\033[0m\n");
        canbus_controller->closeCanBus("can0");
        delete motor;
        motor = nullptr;
        return -1;
    }

    printf("\033[32m成功: 收到电机反馈\033[0m\n");
    printf("当前电机状态: %d\n", static_cast<int>(motor->getState()));

    // 打印当前电机数据
    printf("\n\033[33m=== 当前电机数据 ===\033[0m\n");
    printf("\033[32m位置: %.4f 弧度\033[0m\n", motor->position());
    printf("\033[32m速度: %.4f 弧度/秒\033[0m\n", motor->velocity());
    printf("\033[32m力矩: %.4f 牛·米\033[0m\n", motor->torque());
    printf("\033[32m温度: %d 摄氏度\033[0m\n", motor->temperature());

    // 使用 sin 函数生成正弦波测试
    printf("\n\033[33m=== 正弦波控制测试 ===\033[0m\n");

    // 记录当前位置作为零点
    double zero_position = motor->position();
    printf("\033[36m设置当前位置 (%.3f 弧度) 为零点\033[0m\n", zero_position);

    // 转换20度为弧度
    double amplitude_deg = 20.0;     // 振幅 20 度
    double amplitude_rad = amplitude_deg * M_PI / 180.0;  // 转换为弧度

    int time_steps = 60;
    printf("\033[36m开始%d秒正弦波控制，振幅±%.1f度...\033[0m\n", time_steps, amplitude_deg);

    auto control_start_time = std::chrono::steady_clock::now();
    const double frequency = 0.5;      // 频率 0.5 Hz
    const int control_duration = time_steps*1000; // 控制时长 time_steps 秒
    const int control_interval = 4000; // 控制间隔 4ms (250Hz)

    while (true) {
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            current_time - control_start_time).count();

        if (elapsed_ms >= control_duration) {
            break;
        }

        // 计算正弦波目标位置（基于零点）
        double time_sec = elapsed_ms / 1000.0;
        double sine_offset = amplitude_rad * sin(2.0 * M_PI * frequency * time_sec);
        double target_position = zero_position + sine_offset;

        // 计算目标速度（正弦波的导数）
        double target_velocity = amplitude_rad * 2.0 * M_PI * frequency * cos(2.0 * M_PI * frequency * time_sec);

        // 发送控制命令 (位置 + 速度控制，力矩为0)
        motor->controlPTM(target_position, target_velocity, 0.0, 35.0, 2.0);

        // 每100ms打印一次数据
        if (elapsed_ms % 100 == 0) {
            printf("\033[34m时间: %.1f秒, 零点: %.3f弧度, 目标: %.3f弧度, 实际: %.3f弧度, 速度: %.3f弧度/秒\033[0m\n",
                   time_sec, zero_position, target_position, motor->position(), motor->velocity());
        }

        // 等待下一个控制周期
        std::this_thread::sleep_for(std::chrono::microseconds(control_interval));
    }

    printf("\033[32m正弦波控制测试完成.\033[0m\n");

    motor->enterRestState();
    printf("\033[32m电机进入休眠状态.\033[0m\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // 等待电机休眠（实际肯定不会这样sleep太不优雅）

    canbus_controller->closeCanBus("can0");
    printf("\033[32mCAN总线已关闭.\033[0m\n");

    // 释放内存
    delete motor;
    motor = nullptr;
    printf("\033[32m程序执行完成，资源已释放.\033[0m\n");
    return 0;
}


void can_receive_callback(canbus_sdk::CanMessageFrame* frame, const canbus_sdk::CallbackContext* context)
{
#if DEBUG_PRINT_CAN_DATA
  printf("\033[32mCAN FD Received: ID: 0x%X, DLC: %d, Data: ", frame->id.SID, frame->ctrl.rx.dlc);
  for (int i = 0; i < frame->ctrl.rx.dlc; ++i)
  {
    printf("%02x ", frame->payload[i]);
  }
  printf("\033[0m\n");
#endif

    uint8_t payload[8];
    memccpy(payload, frame->payload, 0, 8);
    motorevo::FeedbackFrame feedback(payload);
    motor->receiveFeedback(feedback);
  
    feedback_received_flag.store(true);
    
    freeCanMessageFrame(frame);
}