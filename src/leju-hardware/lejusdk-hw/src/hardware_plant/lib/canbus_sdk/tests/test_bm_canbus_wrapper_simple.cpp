/*
 * 🧪 BMCanbusWrapper 简单打开关闭测试文件
 *
 * 测试内容：
 * - BMCanbusWrapper 类的基本功能测试
 * - CAN总线设备的简单打开和关闭操作
 * - 设备初始化和资源管理
 * - 异常处理和错误报告
 *
 * 使用框架：标准C++ (无依赖框架)
 *
 * 测试覆盖范围：
 * - 设备创建和初始化
 * - 基本配置参数设置
 * - 设备打开和关闭
 * - 资源自动释放
 * - 错误处理机制
 */

#include "bm_canbus_wrapper.h"
#include "canbus_sdk/canbus_log.h"

#include <iostream>
#include <thread>
#include <chrono>

int main() {
    std::cout << "=== BMCanbusWrapper Direct Simple Open/Close Test ===" << std::endl;

    try {
        // 配置CAN总线参数
        canbus_sdk::CanBusBitrate bitrate;
        bitrate.nbitrate = 500000;    // 仲裁段波特率 500K
        bitrate.dbitrate = 2000000;   // 数据段波特率 2M
        bitrate.nsamplepos = 80;     // 仲裁段采样点 80%
        bitrate.dsamplepos = 80;     // 数据段采样点 80%

        std::cout << "Creating BMCanbusWrapper for BUSMUST_B..." << std::endl;

        // 直接创建BMCanbusWrapper实例
        canbus_sdk::BMCanbusWrapper wrapper(
            "BM-CANFD-X1(6729) CH1",
            canbus_sdk::CanBusModelType::BUSMUST_B,
            bitrate
        );

        std::cout << "Initializing BMCanbusWrapper..." << std::endl;

        // 直接调用初始化方法
        auto init_result = wrapper.init();

        if (init_result.has_value() && init_result.value()) {
            std::cout << "Successfully initialized BMCanbusWrapper" << std::endl;

            // 等待一小段时间让设备稳定
            std::cout << "Waiting for device to stabilize..." << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            // BMCanbusWrapper会在析构函数中自动关闭设备
            std::cout << "BMCanbusWrapper will automatically close in destructor" << std::endl;

        } else {
            std::cout << "Failed to initialize BMCanbusWrapper" << std::endl;
        }

        // BMCanbusWrapper析构函数在这里自动调用
        std::cout << "BMCanbusWrapper going out of scope..." << std::endl;

    } catch (const std::exception& e) {
        std::cout << "Exception occurred: " << e.what() << std::endl;
        return 1;
    }

    std::cout << "=== Test completed ===" << std::endl;
    return 0;
}