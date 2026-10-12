#include "motorevo/motorevo_actuator.h"
#include "canbus_sdk/canbus_sdk.h"
#include <iostream>
#include <set>
#include <string>

int main(int argc, char* argv[]) {
    try {
        std::cout << "\033[36m===============================================\033[0m" << std::endl;
        std::cout << "\033[36m      MotorevoActuator 简单示例程序\033[0m" << std::endl;
        std::cout << "\033[36m===============================================\033[0m" << std::endl;

        std::string config_file_path;
        if (argc > 1) {
            config_file_path = argv[1];
            std::cout << "\033[33m使用指定的配置文件: " << config_file_path << "\033[0m" << std::endl;
        } else {
            config_file_path = "../../canbus_sdk/roban2_canbus_device_cofig.yaml";
            std::cout << "\033[33m使用默认配置文件: " << config_file_path << "\033[0m" << std::endl;
        }
        // 创建 MotorevoActuator 实例
        // 不进行零点校准(cali_buses 为空), 控制频率 250Hz
        motorevo::MotorevoActuator actuator(
            config_file_path,
            std::set<std::string>{},  // 空集 => 不校准任何总线
            250
        );

        std::cout << "\033[32mMotorevoActuator 实例创建成功\033[0m" << std::endl;

        // 初始化执行器
        std::cout << "\033[36m正在初始化执行器...\033[0m" << std::endl;
        if (actuator.init() != 0) {
            std::cout << "\033[31m初始化失败，程序退出\033[0m" << std::endl;
            return -1;
        }
        std::cout << "\033[32m执行器初始化成功\033[0m" << std::endl;

        // 获取电机状态信息
        std::cout << "\033[33m=== 电机状态信息 ===\033[0m" << std::endl;
        auto positions = actuator.getPositions();
        auto velocities = actuator.getVelocities();
        auto torques = actuator.getTorques();

        std::cout << "发现 " << positions.size() << " 个电机" << std::endl;

        for (size_t i = 0; i < positions.size(); ++i) {
            std::cout << "电机 " << i << ": ";
            std::cout << "位置=" << positions[i] << " rad";
            if (i < velocities.size()) {
                std::cout << ", 速度=" << velocities[i] << " rad/s";
            }
            if (i < torques.size()) {
                std::cout << ", 力矩=" << torques[i] << " N·m";
            }
            std::cout << std::endl;
        }

        // 关闭执行器
        std::cout << "\033[36m正在关闭执行器...\033[0m" << std::endl;
        actuator.reset();
        std::cout << "\033[32m执行器已关闭\033[0m" << std::endl;

        std::cout << "\n\033[32m===============================================\033[0m" << std::endl;
        std::cout << "\033[32m      程序执行完成\033[0m" << std::endl;
        std::cout << "\033[32m===============================================\033[0m" << std::endl;

        // 关闭CAN总线
        canbus_sdk::CanBusController::getInstance().closeAllCanBuses();
        std::cout << "\033[32mCAN总线已关闭\033[0m" << std::endl;


        return 0;

    } catch (const std::exception& e) {
        std::cout << "\033[31m程序异常: " << e.what() << "\033[0m" << std::endl;
        return -1;
    }
}