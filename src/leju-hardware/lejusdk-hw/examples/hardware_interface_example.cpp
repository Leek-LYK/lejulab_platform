#include "lejusdk_hw/lejusdk_hw.h"
#include <iostream>
#include <chrono>
#include <thread>

int main() {
    std::cout << "=== Leju Hardware Interface Example ===" << std::endl;

    // 获取硬件接口单例
    leju::hw::HardwareInterface& hw = leju::hw::HardwareInterface::GetInstance();

    // 初始化硬件接口
    std::cout << "Initializing hardware interface..." << std::endl;
    const std::string config_dir = "/home/leju/config";
    if (hw.Initialize(config_dir) == leju::hw::HwErrorType::Success) {
        std::cout << "Hardware interface initialized successfully!" << std::endl;
    } else {
        std::cerr << "Failed to initialize hardware interface!" << std::endl;
        return -1;
    }

    std::vector<double> ready_inital_pos(hw.GetMotorNumber(), 0);
    ready_inital_pos[12] = 0.2;
    hw.JointMoveTo(ready_inital_pos, 60.0);  // speed=60deg/s

    // 运行一段时间
    std::cout << "Hardware interface is running..." << std::endl;
    std::cout << "Press Ctrl+C to stop" << std::endl;

    // 模拟运行
    for (int i = 0; i < 5; ++i) {
        std::cout << "Running... " << (5 - i) << " seconds remaining" << std::endl;
        leju::hw::JointData_t joint_state;
        if(hw.GetJointState(joint_state) == leju::hw::HwErrorType::Success) {
            std::cout << joint_state;
        }
        leju::hw::ImuData_t imu_data;
        if(hw.GetImuData(imu_data) == leju::hw::HwErrorType::Success) {
            std::cout << imu_data;
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // 关闭硬件接口
    std::cout << "Shutting down hardware interface..." << std::endl;
    hw.Shutdown();

    std::cout << "Example completed successfully!" << std::endl;
    return 0;
}