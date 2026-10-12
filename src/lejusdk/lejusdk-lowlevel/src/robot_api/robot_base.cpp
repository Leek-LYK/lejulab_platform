#include "lejusdk-lowlevel/robot_api/robot_base.h"
#include "communication.h"
#include "sdk_log.h"
#include "internal/common/callback_wrapper.hpp"
#include <algorithm>  // for std::transform
#include <cctype>    // for ::tolower
#include <map>       // for std::map

namespace leju {

// 构造函数实现
RobotBaseAPI::RobotBaseAPI(const RobotVersion& version) : robot_version_(version) {
    // 初始化所有回调向量
    imu_data_callbacks_ = new internal::CallbackVector<ImuDataCallback>();
    robot_state_callbacks_ = new internal::CallbackVector<RobotStateCallback>();
    joy_data_callbacks_ = new internal::CallbackVector<JoyDataCallback>();
    robot_cmd_callbacks_ = new internal::CallbackVector<RobotCmdCallback>();
    hand_cmd_callbacks_ = new internal::CallbackVector<HandCmdCallback>();
    hand_state_callbacks_ = new internal::CallbackVector<HandStateCallback>();
    hw_state_callbacks_ = new internal::CallbackVector<HwStateCallback>();
    stop_robot_callbacks_ = new internal::CallbackVector<StringDataCallback>();
    reload_teleop_config_callbacks_ = new internal::CallbackVector<StringDataCallback>();
    reload_controller_config_callbacks_ = new internal::CallbackVector<StringDataCallback>();
}

RobotBaseAPI::~RobotBaseAPI() {
    #define CLEANUP_CALLBACKS(callback_vector) \
        if (callback_vector) { \
            callback_vector->clear(); \
            delete callback_vector; \
            callback_vector = nullptr; \
        }

    // 清理所有回调向量
    CLEANUP_CALLBACKS(imu_data_callbacks_);
    CLEANUP_CALLBACKS(robot_state_callbacks_);
    CLEANUP_CALLBACKS(joy_data_callbacks_);
    CLEANUP_CALLBACKS(robot_cmd_callbacks_);
    CLEANUP_CALLBACKS(hand_cmd_callbacks_);
    CLEANUP_CALLBACKS(hand_state_callbacks_);
    CLEANUP_CALLBACKS(hw_state_callbacks_);
    CLEANUP_CALLBACKS(stop_robot_callbacks_);
    CLEANUP_CALLBACKS(reload_teleop_config_callbacks_);
    CLEANUP_CALLBACKS(reload_controller_config_callbacks_);

    #undef CLEANUP_CALLBACKS
}

void RobotBaseAPI::subscribeImuData(const ImuDataCallback& callback) {
    LJSDK_LOG_I("Adding IMU data callback...");

    bool should_subscribe = false;

    // 使用CallbackVector的线程安全方法
    {
        should_subscribe = (imu_data_callbacks_->size() == 0);
        imu_data_callbacks_->add(callback);
    }

    if (should_subscribe) {
        try {
            auto& communication = Communication::getInstance();
            bool result = communication.subscribeImuData([this](const ImuDataConstPtr& data) {
                // 使用CallbackVector调用所有回调
                imu_data_callbacks_->call_all(data);
            });

            if (result) {
                LJSDK_LOG_SUCCESS("IMU data subscription successful");
            } else {
                LJSDK_LOG_FAILURE("Failed to subscribe to IMU data");
                imu_data_callbacks_->clear();  // 清空回调
            }
        } catch (const std::exception& e) {
            LJSDK_LOG_FAILURE("Exception during IMU subscription: %s", e.what());
            imu_data_callbacks_->clear();  // 清空回调
        }
    } else {
        size_t total_callbacks = imu_data_callbacks_->size();
        LJSDK_LOG_I("IMU data already subscribed, callback added (total: %zu)", total_callbacks);
    }
}

void RobotBaseAPI::subscribeRobotState(const RobotStateCallback& callback) {
    LJSDK_LOG_I("Adding robot state callback...");

    bool should_subscribe = false;

    // 使用CallbackVector的线程安全方法
    {
        should_subscribe = (robot_state_callbacks_->size() == 0);
        robot_state_callbacks_->add(callback);
    }

    if (should_subscribe) {
        try {
            auto& communication = Communication::getInstance();
            bool result = communication.subscribeJointState([this](const RobotStateConstPtr& state) {
                // 使用CallbackVector调用所有回调
                robot_state_callbacks_->call_all(state);
            });

            if (result) {
                LJSDK_LOG_SUCCESS("Robot state subscription successful");
            } else {
                LJSDK_LOG_FAILURE("Failed to subscribe to robot state");
                robot_state_callbacks_->clear();  // 清空回调
            }
        } catch (const std::exception& e) {
            LJSDK_LOG_FAILURE("Exception during robot state subscription: %s", e.what());
            robot_state_callbacks_->clear();  // 清空回调
        }
    } else {
        size_t total_callbacks = robot_state_callbacks_->size();
        LJSDK_LOG_I("Robot state already subscribed, callback added (total: %zu)", total_callbacks);
    }
}

void RobotBaseAPI::subscribeJoyData(const JoyDataCallback& callback) {
    LJSDK_LOG_I("Adding joy data callback...");

    bool should_subscribe = false;

    // 使用CallbackVector的线程安全方法
    {
        should_subscribe = (joy_data_callbacks_->size() == 0);
        joy_data_callbacks_->add(callback);
    }

    if (should_subscribe) {
        try {
            auto& communication = Communication::getInstance();
            bool result = communication.subscribeJoyData([this](const JoyDataConstPtr& data) {
                // 使用CallbackVector调用所有回调
                joy_data_callbacks_->call_all(data);
            });

            if (result) {
                LJSDK_LOG_SUCCESS("Joy data subscription successful");
            } else {
                LJSDK_LOG_FAILURE("Failed to subscribe to joy data");
                joy_data_callbacks_->clear();  // 清空回调
            }
        } catch (const std::exception& e) {
            LJSDK_LOG_FAILURE("Exception during joy data subscription: %s", e.what());
            joy_data_callbacks_->clear();  // 清空回调
        }
    } else {
        size_t total_callbacks = joy_data_callbacks_->size();
        LJSDK_LOG_I("Joy data already subscribed, callback added (total: %zu)", total_callbacks);
    }
}

bool RobotBaseAPI::publishRobotCmd(const RobotCmd& cmd) {
    // 验证指令数据
    if (!cmd.isValid()) {
        LJSDK_LOG_FAILURE("Invalid robot command data!");
        return false;
    }

    try {
        auto& communication = Communication::getInstance();
        bool result = communication.publishRobotCmd(cmd);

        // if (result) {
        //     LJSDK_LOG_SUCCESS("Robot command published successfully");
        // } else {
        //     LJSDK_LOG_FAILURE("Failed to publish robot command");
        // }

        return result;
    } catch (const std::exception& e) {
        LJSDK_LOG_FAILURE("Exception during robot command publishing: %s", e.what());
        return false;
    }
}


bool RobotBaseAPI::publishHandCmd(const HandCmd& cmd) {
    if (!cmd.isValid()) {
        LJSDK_LOG_FAILURE("Invalid hand command data!");
        return false;
    }

    try {
        auto& communication = Communication::getInstance();
        return communication.publishHandCmd(cmd);
    } catch (const std::exception& e) {
        LJSDK_LOG_FAILURE("Exception during hand command publishing: %s", e.what());
        return false;
    }
}

bool RobotBaseAPI::publishHandState(const HandState& state) {
    if (!state.isValid()) {
        LJSDK_LOG_FAILURE("Invalid hand state data!");
        return false;
    }

    try {
        return Communication::getInstance().publishHandState(state);
    } catch (const std::exception& e) {
        LJSDK_LOG_FAILURE("Exception during hand state publishing: %s", e.what());
        return false;
    }
}

bool RobotBaseAPI::publishStopRobot()
{
    try {
        auto& communication = Communication::getInstance();
        bool result = communication.publishStopRobot();
        return result;
    } catch (const std::exception& e) {
        LJSDK_LOG_FAILURE("Exception during robot stopping: %s", e.what());
        return false;
    }
}

bool RobotBaseAPI::publishPostureHeightCmd(double height_m) {
    try {
        auto& communication = Communication::getInstance();
        return communication.publishPostureHeightCmd(height_m);
    } catch (const std::exception& e) {
        LJSDK_LOG_FAILURE("Exception during posture height command publishing: %s", e.what());
        return false;
    }
}

bool RobotBaseAPI::publishHardwareState(HardwareState hw_state) {
    try {
        auto& communication = Communication::getInstance();
        bool result = communication.publishHardwareState(hw_state);
        return result;
    } catch (const std::exception& e) {
        LJSDK_LOG_FAILURE("Exception during publish hardware state: %s", e.what());
        return false;
    }
}

bool RobotBaseAPI::publishImuData(const ImuData& imu_data) {
    try {
        auto& communication = Communication::getInstance();
        bool result = communication.publishImuData(imu_data);

        return result;
    } catch (const std::exception& e) {
        LJSDK_LOG_FAILURE("Exception during imu data publishing: %s", e.what());
        return false;
    }
}

bool RobotBaseAPI::publishRobotState(const RobotState& state) {
    if (!state.isValid()) {
        LJSDK_LOG_FAILURE("Invalid robot state data!");
        return false;
    }

    try {
        auto& communication = Communication::getInstance();
        bool result = communication.publishJointState(state);

        return result;
    } catch (const std::exception& e) {
        LJSDK_LOG_FAILURE("Exception during robot state publishing: %s", e.what());
        return false;
    }
}

bool RobotBaseAPI::publishMotorState(const RobotState& motor_state) {
    if (!motor_state.isValid()) {
        LJSDK_LOG_FAILURE("Invalid motor state data!");
        return false;
    }

    try {
        auto& communication = Communication::getInstance();
        bool result = communication.publishMotorState(motor_state);

        return result;
    } catch (const std::exception& e) {
        LJSDK_LOG_FAILURE("Exception during motor state publishing: %s", e.what());
        return false;
    }
}

bool RobotBaseAPI::publishMotorCmd(const RobotCmd& motor_cmd) {
    if (!motor_cmd.isValid()) {
        LJSDK_LOG_FAILURE("Invalid motor cmd data!");
        return false;
    }

    try {
        auto& communication = Communication::getInstance();
        bool result = communication.publishMotorCmd(motor_cmd);

        return result;
    } catch (const std::exception& e) {
        LJSDK_LOG_FAILURE("Exception during motor cmd publishing: %s", e.what());
        return false;
    }
}

bool RobotBaseAPI::publishJoyData(const JoyData& joy) {
    try {
        auto& communication = Communication::getInstance();
        bool result = communication.publishJoyData(joy);

        return result;
    } catch (const std::exception& e) {
        LJSDK_LOG_FAILURE("Exception during joy data publishing: %s", e.what());
        return false;
    }
}

void RobotBaseAPI::subscribeHardwareState(const HwStateCallback& callback) {
    LJSDK_LOG_I("Adding hardware state callback...");

    bool should_subscribe = false;

    // 使用CallbackVector的线程安全方法
    {
        should_subscribe = (hw_state_callbacks_->size() == 0);
        hw_state_callbacks_->add(callback);
    }

    if (should_subscribe) {
        try {
            auto& communication = Communication::getInstance();
            bool result = communication.subscribeHardwareState([this](const StringDataConstPtr& hw_state_data) {
                // 使用CallbackVector调用所有回调
                hw_state_callbacks_->call_all(hw_state_data);
            });

            if (result) {
                LJSDK_LOG_SUCCESS("Hardware state subscription successful");
            } else {
                LJSDK_LOG_FAILURE("Failed to subscribe to hardware state");
                hw_state_callbacks_->clear();  // 清空回调
            }
        } catch (const std::exception& e) {
            LJSDK_LOG_FAILURE("Exception during hardware state subscription: %s", e.what());
            hw_state_callbacks_->clear();  // 清空回调
        }
    } else {
        size_t total_callbacks = hw_state_callbacks_->size();
        LJSDK_LOG_I("Hardware state already subscribed, callback added (total: %zu)", total_callbacks);
    }
}

void RobotBaseAPI::subscribeRobotCmd(const RobotCmdCallback& callback) {
    LJSDK_LOG_I("Adding robot cmd callback...");

    bool should_subscribe = false;

    // 使用CallbackVector的线程安全方法
    {
        should_subscribe = (robot_cmd_callbacks_->size() == 0);
        robot_cmd_callbacks_->add(callback);
    }

    if (should_subscribe) {
        try {
            auto& communication = Communication::getInstance();
            bool result = communication.subscribeRobotCmd([this](const RobotCmdConstPtr& cmd) {
                // 使用CallbackVector调用所有回调
                robot_cmd_callbacks_->call_all(cmd);
            });

            if (result) {
                LJSDK_LOG_SUCCESS("Robot cmd subscription successful");
            } else {
                LJSDK_LOG_FAILURE("Failed to subscribe to robot cmd");
                robot_cmd_callbacks_->clear();  // 清空回调
            }
        } catch (const std::exception& e) {
            LJSDK_LOG_FAILURE("Exception during robot cmd subscription: %s", e.what());
            robot_cmd_callbacks_->clear();  // 清空回调
        }
    } else {
        size_t total_callbacks = robot_cmd_callbacks_->size();
        LJSDK_LOG_I("Robot cmd already subscribed, callback added (total: %zu)", total_callbacks);
    }
}


void RobotBaseAPI::subscribeHandCmd(const HandCmdCallback& callback) {
    LJSDK_LOG_I("Adding hand cmd callback...");

    bool should_subscribe = false;
    {
        should_subscribe = (hand_cmd_callbacks_->size() == 0);
        hand_cmd_callbacks_->add(callback);
    }

    if (should_subscribe) {
        try {
            auto& communication = Communication::getInstance();
            bool result = communication.subscribeHandCmd([this](const HandCmdConstPtr& cmd) {
                hand_cmd_callbacks_->call_all(cmd);
            });

            if (result) {
                LJSDK_LOG_SUCCESS("Hand cmd subscription successful");
            } else {
                LJSDK_LOG_FAILURE("Failed to subscribe to hand cmd");
                hand_cmd_callbacks_->clear();
            }
        } catch (const std::exception& e) {
            LJSDK_LOG_FAILURE("Exception during hand cmd subscription: %s", e.what());
            hand_cmd_callbacks_->clear();
        }
    } else {
        LJSDK_LOG_I("Hand cmd already subscribed, callback added");
    }
}

void RobotBaseAPI::subscribeHandState(const HandStateCallback& callback) {
    LJSDK_LOG_I("Adding hand state callback...");

    const bool should_subscribe = hand_state_callbacks_->size() == 0;
    hand_state_callbacks_->add(callback);

    if (should_subscribe) {
        try {
            auto& communication = Communication::getInstance();
            const bool result = communication.subscribeHandState(
                [this](const HandStateConstPtr& state) {
                    hand_state_callbacks_->call_all(state);
                });

            if (result) {
                LJSDK_LOG_SUCCESS("Hand state subscription successful");
            } else {
                LJSDK_LOG_FAILURE("Failed to subscribe to hand state");
                hand_state_callbacks_->clear();
            }
        } catch (const std::exception& e) {
            LJSDK_LOG_FAILURE("Exception during hand state subscription: %s", e.what());
            hand_state_callbacks_->clear();
        }
    } else {
        LJSDK_LOG_I("Hand state already subscribed, callback added");
    }
}

void RobotBaseAPI::subscribeStopRobot(const StringDataCallback& callback) {
    LJSDK_LOG_I("Adding stop robot callback...");

    bool should_subscribe = false;

    {
        should_subscribe = (stop_robot_callbacks_->size() == 0);
        stop_robot_callbacks_->add(callback);
    }

    if (should_subscribe) {
        try {
            auto& communication = Communication::getInstance();
            bool result = communication.subscribeStopRobot([this](const StringDataConstPtr& data) {
                stop_robot_callbacks_->call_all(data);
            });

            if (result) {
                LJSDK_LOG_SUCCESS("Stop robot subscription successful");
            } else {
                LJSDK_LOG_FAILURE("Failed to subscribe to stop robot");
                stop_robot_callbacks_->clear();
            }
        } catch (const std::exception& e) {
            LJSDK_LOG_FAILURE("Exception during stop robot subscription: %s", e.what());
            stop_robot_callbacks_->clear();
        }
    } else {
        LJSDK_LOG_I("Stop robot already subscribed, callback added");
    }
}

void RobotBaseAPI::subscribeReloadTeleopConfig(const StringDataCallback& callback) {
    LJSDK_LOG_I("Adding reload teleop config callback...");

    bool should_subscribe = false;

    {
        should_subscribe = (reload_teleop_config_callbacks_->size() == 0);
        reload_teleop_config_callbacks_->add(callback);
    }

    if (should_subscribe) {
        try {
            auto& communication = Communication::getInstance();
            bool result = communication.subscribeReloadTeleopConfig([this](const StringDataConstPtr& data) {
                reload_teleop_config_callbacks_->call_all(data);
            });

            if (result) {
                LJSDK_LOG_SUCCESS("Reload teleop config subscription successful");
            } else {
                LJSDK_LOG_FAILURE("Failed to subscribe to reload teleop config");
                reload_teleop_config_callbacks_->clear();
            }
        } catch (const std::exception& e) {
            LJSDK_LOG_FAILURE("Exception during reload teleop config subscription: %s", e.what());
            reload_teleop_config_callbacks_->clear();
        }
    } else {
        LJSDK_LOG_I("Reload teleop config already subscribed, callback added");
    }
}

void RobotBaseAPI::subscribeReloadControllerConfig(const StringDataCallback& callback) {
    LJSDK_LOG_I("Adding reload controller config callback...");

    bool should_subscribe = false;

    {
        should_subscribe = (reload_controller_config_callbacks_->size() == 0);
        reload_controller_config_callbacks_->add(callback);
    }

    if (should_subscribe) {
        try {
            auto& communication = Communication::getInstance();
            bool result = communication.subscribeReloadControllerConfig([this](const StringDataConstPtr& data) {
                reload_controller_config_callbacks_->call_all(data);
            });

            if (result) {
                LJSDK_LOG_SUCCESS("Reload controller config subscription successful");
            } else {
                LJSDK_LOG_FAILURE("Failed to subscribe to reload controller config");
                reload_controller_config_callbacks_->clear();
            }
        } catch (const std::exception& e) {
            LJSDK_LOG_FAILURE("Exception during reload controller config subscription: %s", e.what());
            reload_controller_config_callbacks_->clear();
        }
    } else {
        LJSDK_LOG_I("Reload controller config already subscribed, callback added");
    }
}

bool RobotBaseAPI::publishReloadTeleopConfig(const std::string& config_path) {
    try {
        auto& communication = Communication::getInstance();
        return communication.publishReloadTeleopConfig(config_path);
    } catch (const std::exception& e) {
        LJSDK_LOG_FAILURE("Exception during reload teleop config publishing: %s", e.what());
        return false;
    }
}

void RobotBaseAPI::shutdown() {
    this->publishStopRobot();
    Communication::getInstance().shutdown();
}

HardwareState String2HwState(const std::string& hw_state_str) {
    // Convert string to lowercase for case-insensitive comparison
    std::string hw_state_str_lower = hw_state_str;
    std::transform(hw_state_str_lower.begin(), hw_state_str_lower.end(),
                   hw_state_str_lower.begin(), ::tolower);

    // Map of string values to HardwareState enum values
    static const std::map<std::string, HardwareState> hw_state_map = {
        {"unknown", HardwareState::UNKNOWN},
        {"initializing", HardwareState::INITIALIZING},
        {"ready_ok", HardwareState::READY_OK},
        {"error", HardwareState::ERROR},
        {"stopped", HardwareState::STOPPED},
    };

    // Find the state in the map
    auto it = hw_state_map.find(hw_state_str_lower);
    if (it != hw_state_map.end()) {
        return it->second;
    } else {
        // Return UNKNOWN for invalid strings
        LJSDK_LOG_WARNING("Unknown hardware state string: %s", hw_state_str.c_str());
        return HardwareState::UNKNOWN;
    }
}

const char* HardwareState2String(HardwareState hw_state)
{
    switch (hw_state)
    {
    case HardwareState::UNKNOWN:
        return "unknown";
    case HardwareState::INITIALIZING:
        return "initializing";
    case HardwareState::READY_OK:
        return "ready_ok";
    case HardwareState::ERROR:
        return "error";
    case HardwareState::STOPPED:
        return "stopped";    
    default:
        return "unknown";
    }
}

} // namespace leju
