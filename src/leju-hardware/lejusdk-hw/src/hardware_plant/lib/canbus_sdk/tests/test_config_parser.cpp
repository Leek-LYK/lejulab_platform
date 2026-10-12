#include "canbus_sdk/config_parser.h"
#include <iostream>
#include <string>

using namespace canbus_sdk;

int main() {
    ConfigParser parser;

    std::cout << "=== 测试 ConfigParser ===" << std::endl;

    // 测试配置文件路径
    std::string config_path = "../canbus_device_cofig.yaml";

    try {
        // 解析配置文件
        std::cout << "正在解析配置文件: " << config_path << std::endl;
        bool result = parser.parseFromFile(config_path);

        if (result) {
            std::cout << "✓ 配置文件解析成功!" << std::endl;

            // 打印配置信息
            parser.printConfig();

            // 测试获取CAN总线配置
            std::cout << "\n=== 测试获取CAN总线配置 ===" << std::endl;
            auto canbus_configs = parser.getCanBusConfigs();
            std::cout << "找到 " << canbus_configs.size() << " 个CAN总线配置:" << std::endl;

            for (const auto& canbus : canbus_configs) {
                std::cout << "  - " << canbus.name << " (类型: "
                          << (canbus.type == CanBusModelType::BUSMUST_A ? "BUSMUST_A" : "BUSMUST_B") << ")" << std::endl;

                // 测试获取设备列表
                auto devices = parser.getDevices(canbus.name);
                std::cout << "    设备数量: " << devices.size() << std::endl;

                // 测试获取特定类型设备
                auto motors = parser.getDevices(canbus.name, DeviceType::MOTOR);
                auto hands = parser.getDevices(canbus.name, DeviceType::REVO2_HAND);

                std::cout << "    电机数量: " << motors.size() << std::endl;
                std::cout << "    灵巧手数量: " << hands.size() << std::endl;
            }

        } else {
            std::cout << "✗ 配置文件解析失败!" << std::endl;
            return 1;
        }

    } catch (const std::exception& e) {
        std::cout << "✗ 解析过程中发生异常: " << e.what() << std::endl;
        return 1;
    }

    std::cout << "\n=== 测试完成 ===" << std::endl;
    return 0;
}