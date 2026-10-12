#include <iostream>
#include <thread>
#include <chrono>
#include "lejusdk-lowlevel/leju_sdk.h"

using namespace leju;

/**
 * @brief 使用示例1：基本的机器人初始化和使用
 */
void exampleBasicUsage() {
    std::cout << "=== Example 1: Basic Usage ===" << std::endl;

    try {
        // 从环境变量自动读取机器人版本
        auto robot_version = RobotVersion::from_env();
        std::cout << "Detected robot version from ROBOT_VERSION: " 
                  << robot_version.version_name() << std::endl;

        // 初始化机器人
        if (GlobalRobot::init_env(robot_version)) {
            std::cout << "Robot initialized successfully" << std::endl;

            // 获取机器人实例
            auto& robot = GlobalRobot::getInstance();

            // 打印机器人信息
            std::cout << "Motor Count: " << static_cast<int>(robot.getMotorNumber()) << std::endl;

            // 获取电机名称
            auto motor_names = robot.getMotorNames();
            std::cout << "Motor Names: ";
            for (size_t i = 0; i < motor_names.size(); ++i) {
                std::cout << motor_names[i];
                if (i < motor_names.size() - 1) std::cout << ", ";
            }
            std::cout << std::endl;

        } else {
            std::cerr << "Failed to initialize robot" << std::endl;
        }
    } catch (const std::invalid_argument& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        std::cerr << "Please set ROBOT_VERSION environment variable." << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Failed to initialize robot: " << e.what() << std::endl;
    }
}

/**
 * @brief 使用示例2：数据订阅
 */
void exampleDataSubscription() {
    std::cout << "\n=== Example 2: Data Subscription ===" << std::endl;

    auto& robot = GlobalRobot::getInstance();

    // 订阅IMU数据
    robot.subscribeImuData([](const ImuDataConstPtr& imu_data) {
        std::cout << "[IMU] Received IMU data:" << std::endl;
        // 使用重载的<<运算符输出完整IMU数据
        std::cout << *imu_data << std::endl;
    });

    // 订阅机器人状态
    robot.subscribeRobotState([](const RobotStateConstPtr& robot_state) {
        std::cout << "[State] Received robot state:" << std::endl;
        // 使用重载的<<运算符输出完整机器人状态
        std::cout << *robot_state << std::endl;
    });

    std::cout << "Subscribed to IMU and robot state data" << std::endl;

    // 模拟运行一段时间
    std::cout << "Waiting for data (1 seconds)..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));
}

/**
 * @brief 使用示例3：控制指令发布
 */
void exampleCommandPublishing() {
    std::cout << "\n=== Example 3: Command Publishing ===" << std::endl;

    auto& robot = GlobalRobot::getInstance();

    // 获取机器人电机数量
    size_t motor_count = robot.getMotorNumber();
    std::cout << "Creating command for " << motor_count << " motors" << std::endl;

    // 创建控制指令，指定电机数量
    RobotCmd cmd(motor_count);

    // 设置一些示例控制数据
    for (size_t i = 0; i < motor_count; ++i) {
        cmd.q[i] = 0.1 * i;        // 位置指令
        cmd.v[i] = 0.0;            // 速度指令
        cmd.tau[i] = 0.0;          // 力矩指令
        cmd.kp[i] = 10.0;          // 位置增益
        cmd.kd[i] = 1.0;           // 速度增益
        cmd.modes[i] = 1;          // 控制模式 (CSP - 位置控制)
    }

    // 设置时间戳
    cmd.timestamp = 0.0;

    // 验证指令数据
    if (!cmd.isValid()) {
        std::cout << "Invalid command data!" << std::endl;
        return;
    }

    // 发布指令
    for (int i = 0; i < 5; i++) {
        bool success = robot.publishRobotCmd(cmd);
        if (success) {
            std::cout << "Command published successfully" << std::endl;
        } else {
            std::cout << "Failed to publish command" << std::endl;
        }
    }
}


/**
 * @brief 使用示例5：错误处理
 */
void exampleErrorHandling() {
    std::cout << "\n=== Example 5: Error Handling ===" << std::endl;

    std::cout << "Note: GlobalRobot is already initialized from previous examples." << std::endl;
    std::cout << "In a fresh program, the following would throw exceptions:" << std::endl;

    // 演示重复初始化错误（这个会真的抛出异常）
    try {
        // 尝试重复初始化（即使使用相同的版本也会失败）
        auto robot_version = RobotVersion::from_env();
        GlobalRobot::init_env(robot_version);
        std::cout << "This should not print" << std::endl;
    } catch (const std::runtime_error& e) {
        std::cout << "✓ Expected error caught: " << e.what() << std::endl;
    }

    // 正常情况 - 已经初始化的获取实例
    try {
        auto& robot = GlobalRobot::getInstance();
        std::cout << "✓ Successfully retrieved robot instance (already initialized)" << std::endl;
    } catch (const std::exception& e) {
        std::cout << "Unexpected error: " << e.what() << std::endl;
    }

    std::cout << "Error handling example completed" << std::endl;
}

int main() {
    std::cout << "=== Leju SDK Usage Examples ===" << std::endl;

    try {
        // 运行所有示例
        exampleBasicUsage();
        exampleDataSubscription();
        exampleCommandPublishing();
        exampleErrorHandling();

        std::cout << "\n✅ All examples completed successfully!" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "❌ Example failed with exception: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cerr << "❌ Example failed with unknown exception" << std::endl;
        return 1;
    }

    return 0;
}