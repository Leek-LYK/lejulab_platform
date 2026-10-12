#include "canbus_sdk/config_parser.h"
#include <iostream>

using namespace canbus_sdk;

int main() {
    try {
        // 创建配置解析器
        ConfigParser parser;

        std::cout << "从默认配置文件路径读取配置..." << std::endl;

        // 获取默认配置文件路径
        std::string default_config_path;
        try {
            default_config_path = ConfigParser::getDefaultConfigFilePath();
            std::cout << "默认配置文件路径: " << default_config_path << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "获取默认配置文件路径失败: " << e.what() << std::endl;
            return 1;
        }

        // 从默认配置文件解析
        if (!parser.parseFromFile(default_config_path)) {
            std::cerr << "解析默认配置文件失败" << std::endl;
            return 1;
        }

        std::cout << "配置解析成功，写入当前目录的temp.yaml..." << std::endl;

        // 写入当前目录的temp.yaml
        std::string output_file = "./temp.yaml";
        if (parser.writeToFile(output_file)) {
            std::cout << "配置文件已成功写入: " << output_file << std::endl;

            // 打印配置内容概览
            std::cout << "\n=== 配置概览 ===" << std::endl;
            const auto& canbus_configs = parser.getCanBusConfigs();
            std::cout << "CAN总线数量: " << canbus_configs.size() << std::endl;

            for (const auto& canbus : canbus_configs) {
                auto devices = parser.getDevices(canbus.name);
                std::cout << "  " << canbus.name << ": " << devices.size() << " 个设备" << std::endl;
            }

            std::cout << "\n详细配置内容请查看 temp.yaml 文件" << std::endl;
            return 0;
        } else {
            std::cerr << "写入配置文件失败" << std::endl;
            return 1;
        }

    } catch (const std::exception& e) {
        std::cerr << "异常: " << e.what() << std::endl;
        return 1;
    }
}