#include "lejusdk-lowlevel/leju_sdk.h"
#include "sdk_log.h"
#include "lejusdk-lowlevel/robot_api/roban_humanoid.h"
#include "lejusdk-lowlevel/robot_api/kuavo_humanoid.h"
#include "communication.h"
#include <stdexcept>

namespace leju {

// GlobalRobot 静态成员定义
std::unique_ptr<RobotBaseAPI> GlobalRobot::instance_ = nullptr;
std::mutex GlobalRobot::mutex_;

// 初始化机器人运行环境实现
bool GlobalRobot::init_env(const RobotVersion& version) {
    // 获取锁
    std::lock_guard<std::mutex> lock(mutex_);

    // 检查是否已初始化
    if (instance_) {
        std::string error_msg = "Robot already initialized";
        LJSDK_LOG_FAILURE("Robot already initialized");
        throw std::runtime_error(error_msg);
    }

    LJSDK_LOG_I("Initializing robot environment: version=%s", version.version_name().c_str());

    try {
        // 初始化通讯模块
        auto& communication = Communication::getInstance();
        if (!communication.initialize()) {
            LJSDK_LOG_FAILURE("Failed to initialize communication module");
            return false;
        }
        LJSDK_LOG_SUCCESS("Communication module initialized successfully");

        instance_ = createRobot(version);

        LJSDK_LOG_SUCCESS("Robot environment initialized successfully: v%s", version.version_name().c_str());
        return true;
    } catch (const std::exception& e) {
        LJSDK_LOG_FAILURE("Failed to initialize robot environment: %s", e.what());
        // 清理状态
        instance_.reset();
        return false;
    }
}

// 获取已初始化的机器人实现
RobotBaseAPI& GlobalRobot::getInstance() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!instance_) {
        LJSDK_LOG_FAILURE("Robot not initialized. Call init_env() first.");
        throw std::runtime_error("Robot not initialized. Call init_env() first.");
    }
    return *instance_;
}

// 工厂方法实现
std::unique_ptr<RobotBaseAPI> GlobalRobot::createRobot(const RobotVersion& version) {

    if(IS_KUAVO_LEGGED(version)) {
        return std::unique_ptr<KuavoHumanoid>(new KuavoHumanoid(version));
    }
    else if(IS_ROBAN_LEGGED(version)) {
        return std::unique_ptr<RobanHumanoid>(new RobanHumanoid(version));
    }
    // else if(IS_KUAVO_WHEELED(version)) {
    // }

    std::string version_str = version.to_string();
    LJSDK_LOG_FAILURE("Unsupported robot version: %s", version_str.c_str());
    throw std::runtime_error("GlobalRobot::createRobot Unsupported robot version: " + version_str);
}

// 状态查询方法实现
bool GlobalRobot::is_initialized() {
    std::lock_guard<std::mutex> lock(mutex_);
    return instance_ != nullptr;
}
} // namespace leju