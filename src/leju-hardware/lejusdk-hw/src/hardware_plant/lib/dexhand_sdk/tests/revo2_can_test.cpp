#include "revo2_hand_can_customed.h"
#include "canbus_sdk/canbus_sdk.h"
#include "canbus_sdk/config_parser.h"
#include <iostream>
#include <thread>
#include <memory>
#include <algorithm>

using namespace dexhand;
using namespace canbus_sdk;

class Revo2CanTest {
private:
    std::vector<CanBusConfig> canbus_configs_;
    std::vector<DeviceConfig> device_configs_;
    std::vector<std::unique_ptr<DexHandBase>> hands_;
    bool is_initialized_;

public:
    Revo2CanTest() : is_initialized_(false) {}

    // 初始化CAN总线
    bool init_can_buses() {
        printf("初始化CAN总线...\n");

        // 解析配置文件
        canbus_sdk::ConfigParser parser;
        auto config_file = canbus_sdk::ConfigParser::getDefaultConfigFilePath();
        if (!parser.parseFromFile(config_file)) {
            printf("\033[31m[Revo2CanTest] ERROR: 解析配置文件失败: %s\033[0m\n", config_file.c_str());
            return false;
        }

        // 获取CAN总线配置
        canbus_configs_ = parser.getCanBusConfigs();
        if (canbus_configs_.empty()) {
            printf("\033[31m[Revo2CanTest] ERROR: 未找到CAN总线配置\033[0m\n");
            return false;
        }

        // 初始化CAN控制器
        CanBusController::getInstance().init();

        // 遍历所有CAN总线配置，先检查是否有非忽略设备
        device_configs_.clear();
        for (const auto& canbus_config : canbus_configs_) {
            // 先检查该CAN总线上是否有非忽略的REVO2设备
            auto devices = parser.getDevices(canbus_config.name, DeviceType::REVO2_HAND);
            bool has_valid_device = std::any_of(devices.begin(), devices.end(),
                [](const auto& device) { return !device.ignore; });

            if (!has_valid_device) {
                printf("\033[33m[Revo2CanTest] WARN: CAN总线 %s 上没有有效的REVO2设备，跳过\033[0m\n", canbus_config.name.c_str());
                continue; // 跳过没有设备的CAN总线
            }

            // 有有效设备，才打开CAN总线
            auto result = CanBusController::getInstance().openCanBus(
                canbus_config.name, canbus_config.type, canbus_config.bitrate);
            if (!result.has_value()) {
                printf("\033[33m[Revo2CanTest] WARN: 打开CAN总线 %s 失败: %s，跳过该总线上的设备\033[0m\n",
                       canbus_config.name.c_str(), errorToString(result.error()));
                continue; // 跳过这个失败的CAN总线
            }

            printf("\033[32m[Revo2CanTest] INFO: 成功打开CAN总线: %s\033[0m\n", canbus_config.name.c_str());

            // 保存该CAN总线上的所有非忽略设备
            for (const auto& device : devices) {
                if (device.ignore) {
                    printf("\033[33m[Revo2CanTest] WARN: 设备 %s 被忽略，跳过\033[0m\n", device.name.c_str());
                    continue;
                }
                device_configs_.push_back(device);
                printf("\033[36m[Revo2CanTest] INFO: 在总线 %s 上找到设备: %s\033[0m\n",
                       canbus_config.name.c_str(), device.name.c_str());
            }
        }

        return true;
    }

    // 连接REVO2设备
    bool connect_revo2_devices() {
        printf("\033[36m[Revo2CanTest] INFO: 连接REVO2设备...\033[0m\n");

        if (device_configs_.empty()) {
            printf("\033[31m[Revo2CanTest] ERROR: 未找到REVO2设备 - 请先调用init_can_buses()!\033[0m\n");
            return false;
        }

        // 显示找到的设备
        for (const auto& device_config : device_configs_) {
            printf("\033[36m[Revo2CanTest] INFO: 找到设备: %s\033[0m\n", device_config.name.c_str());
        }

        // 连接所有设备
        int connected_count = 0;
        hands_.clear();
        for (const auto& device_config : device_configs_) {
            auto hand = Revo2CanDexhand::Connect(device_config);
            if (hand) {
                hands_.push_back(std::move(hand));
                printf("\033[32m[Revo2CanTest] INFO: 设备 %s 连接成功\033[0m\n", device_config.name.c_str());
                connected_count++;
            } else {
                printf("\033[31m[Revo2CanTest] ERROR: 设备 %s 连接失败\033[0m\n", device_config.name.c_str());
            }
        }

        printf("\033[36m[Revo2CanTest] INFO: 连接成功 %d/%d 个设备\033[0m\n", connected_count, static_cast<int>(device_configs_.size()));
        is_initialized_ = connected_count > 0;
        return is_initialized_;
    }

    // 显示设备信息
    void show_device_info() {
        printf("\033[36m\n=== 设备信息 ===\033[0m\n");

        for (size_t i = 0; i < hands_.size(); ++i) {
            if (hands_[i]) {
                DeviceInfo_t dev_info;
                if (hands_[i]->getDeviceInfo(dev_info)) {
                    std::cout << "\033[36m[Revo2CanTest] INFO: Device " << dev_info << "\033[0m" << std::endl;
                }
            }
        }

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // 设置抓力等级
    void setup_grip_force() {
        printf("\033[36m\n=== 设置抓力等级 ===\033[0m\n");

        for (size_t i = 0; i < hands_.size(); ++i) {
            if (hands_[i]) {
                auto force_level = hands_[i]->getGripForce();
                std::cout << "\033[36m[Revo2CanTest] INFO: Device " << i << " force_level: " << static_cast<int>(force_level) << "\033[0m" << std::endl;
                hands_[i]->setGripForce(dexhand::GripForce::FORCE_LEVEL_NORMAL);
            }
        }
    }

    // 启动状态监控线程
    std::thread start_status_monitor(bool& running) {
        return std::thread([&]() {
            while(running) {
                for (size_t i = 0; i < hands_.size(); ++i) {
                    dexhand::FingerStatus status;
                    if (hands_[i] && hands_[i]->getFingerStatus(status)) {
                        std::cout << "\033[36m[Revo2CanTest] STATUS: Device " << i << " status: " << status << "\033[0m" << std::endl;
                    }
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        });
    }

    // 执行波浪运动测试
    void wave_motion_test() {
        printf("\033[36m\n=== 波浪运动测试 ===\033[0m\n");

        uint16_t positions_list[][6] = {
            {10, 10, 10, 10, 10, 10},
            {20, 10, 90, 10, 10, 10},
            {30, 10, 90, 90, 10, 10},
            {40, 10, 90, 90, 90, 10},
            {50, 10, 90, 90, 90, 90},
        };
        const int num_steps = sizeof(positions_list) / sizeof(positions_list[0]);
        const int delay_ms = 1000;

        printf("\033[36m[Revo2CanTest] INFO: 开始位置控制测试 - 波浪运动模式\033[0m\n");

        for(int j = 0; j < 10; ++j) {
            printf("\033[36m[Revo2CanTest] INFO: 第 %d 轮波浪运动\033[0m\n", j + 1);

            // 正向波浪
            for (int i = 0; i < num_steps; ++i) {
                UnsignedFingerArray positions;
                for (int k = 0; k < 6; ++k) {
                    positions[k] = positions_list[i][k];
                }

                // 控制所有设备
                for (size_t dev_idx = 0; dev_idx < hands_.size(); ++dev_idx) {
                    if (hands_[dev_idx]) {
                        hands_[dev_idx]->setFingerPositions(positions);
                        dexhand::FingerStatus status;
                        if(hands_[dev_idx]->getFingerStatus(status)) {
                            printf("\033[36m[Revo2CanTest] STATUS: Device %zu 状态: pos=[%d,%d,%d,%d,%d,%d], cur=[%d,%d,%d,%d,%d,%d]\033[0m\n",
                                dev_idx,
                                status.positions[0], status.positions[1], status.positions[2],
                                status.positions[3], status.positions[4], status.positions[5],
                                status.currents[0], status.currents[1], status.currents[2],
                                status.currents[3], status.currents[4], status.currents[5]);
                        }                        
                    }
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
            }

            // 反向波浪
            for (int i = num_steps - 1; i >= 0; --i) {
                UnsignedFingerArray positions;
                for (int k = 0; k < 6; ++k) {
                    positions[k] = positions_list[i][k];
                }

                // 控制所有设备
                for (auto& hand : hands_) {
                    if (hand) {
                        hand->setFingerPositions(positions);
                    }
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
            }
        }

        printf("\033[32m[Revo2CanTest] INFO: 波浪运动测试完成\033[0m\n");
    }

    // 执行速度控制测试
    void speed_control_test() {
        printf("\033[36m\n=== 速度控制测试 ===\033[0m\n");

        printf("\033[36m[Revo2CanTest] INFO: 测试正向速度...\033[0m\n");
        for (auto& hand : hands_) {
            if (hand) {
                hand->setFingerSpeeds({20, 20, 20, 20, 20, 20});
            }
        }
        std::this_thread::sleep_for(std::chrono::seconds(3));

        printf("\033[36m[Revo2CanTest] INFO: 测试反向速度...\033[0m\n");
        for (auto& hand : hands_) {
            if (hand) {
                hand->setFingerSpeeds({-20, -20, -20, -20, -20, -20});
            }
        }
        std::this_thread::sleep_for(std::chrono::seconds(3));

        printf("\033[32m[Revo2CanTest] INFO: 速度控制测试完成\033[0m\n");
    }

    // 恢复初始状态
    void reset_to_initial_state() {
        printf("\033[36m\n=== 恢复初始状态 ===\033[0m\n");

        UnsignedFingerArray positions = {0, 0, 0, 0, 0, 0};
        for (auto& hand : hands_) {
            if (hand) {
                hand->setFingerPositions(positions);
            }
        }
        printf("\033[32m[Revo2CanTest] INFO: 所有设备已恢复到张开状态\033[0m\n");
    }

    // 运行完整测试
    void run_test() {
        printf("\033[35m=== CAN Revo2灵巧手测试程序 ===\033[0m\n");

        // 初始化CAN总线
        if (!init_can_buses()) {
            printf("\033[31m[Revo2CanTest] ERROR: CAN总线初始化失败，测试终止\033[0m\n");
            return;
        }

        // 连接REVO2设备
        if (!connect_revo2_devices()) {
            printf("\033[31m[Revo2CanTest] ERROR: REVO2设备连接失败，测试终止\033[0m\n");
            return;
        }

        // 显示设备信息
        show_device_info();

        // 设置抓力等级
        setup_grip_force();

        // 启动状态监控线程
        bool running = true;
        auto status_thread = start_status_monitor(running);

        // 执行测试
        wave_motion_test();
        speed_control_test();
        reset_to_initial_state();

        // 停止监控线程
        running = false;
        status_thread.join();

        printf("\033[35m\n=== 测试完成 ===\033[0m\n");
    }
};

#ifndef REVO2_CAN_TEST_NO_MAIN
int main() {
    printf("\033[35m[Revo2CanTest] INFO: 启动CAN Revo2灵巧手测试程序\033[0m\n");
    Revo2CanTest test;
    test.run_test();
    return 0;
}
#endif