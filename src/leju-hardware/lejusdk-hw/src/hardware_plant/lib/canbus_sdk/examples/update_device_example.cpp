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

        std::cout << "配置解析成功！" << std::endl;

        // 测试更新设备 ignore 状态
        std::cout << "\n=== 测试更新设备 ignore 状态 ===" << std::endl;

        // 禁用某个设备
        std::string test_device = "Rhand_revo2_hand";
        bool result = parser.updateDeviceIgnore(test_device, true);
        if (result) {
            std::cout << "成功设置设备 " << test_device << " 的 ignore 状态为 true" << std::endl;
        } else {
            std::cout << "设置设备 " << test_device << " 的 ignore 状态失败（设备不存在）" << std::endl;
        }

        // 启用某个设备
        test_device = "Lhand_revo2_hand";
        result = parser.updateDeviceIgnore(test_device, false);
        if (result) {
            std::cout << "成功设置设备 " << test_device << " 的 ignore 状态为 false" << std::endl;
        } else {
            std::cout << "设置设备 " << test_device << " 的 ignore 状态失败（设备不存在）" << std::endl;
        }

        // 测试设置电机 negtive 字段
        std::cout << "\n=== 测试设置电机 negtive 字段 ===" << std::endl;

        // 设置某个电机的 negtive 状态
        std::string test_motor = "Head_pitch";
        result = parser.setMotorNegative(test_motor, false);
        if (result) {
            std::cout << "成功设置电机 " << test_motor << " 的 negtive 状态为 false" << std::endl;
        } else {
            std::cout << "设置电机 " << test_motor << " 的 negtive 状态失败（设备不存在或不是电机）" << std::endl;
        }

        // 测试对非电机设备设置 negtive
        test_motor = "Lhand_revo2_hand";
        result = parser.setMotorNegative(test_motor, true);
        if (result) {
            std::cout << "成功设置电机 " << test_motor << " 的 negtive 状态为 true" << std::endl;
        } else {
            std::cout << "设置设备 " << test_motor << " 的 negtive 状态失败（设备不存在或不是电机）" << std::endl;
        }

        // 写入更新后的配置
        std::string output_file = "./updated_config.yaml";
        if (parser.writeToFile(output_file)) {
            std::cout << "\n更新后的配置已成功写入: " << output_file << std::endl;
        } else {
            std::cerr << "写入更新后的配置文件失败" << std::endl;
            return 1;
        }

        return 0;

    } catch (const std::exception& e) {
        std::cerr << "异常: " << e.what() << std::endl;
        return 1;
    }
}