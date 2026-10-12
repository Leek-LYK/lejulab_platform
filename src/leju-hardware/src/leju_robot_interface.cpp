#include "leju_robot_interface.h"
#include "hw_log.h"
#include "lejusdk-lowlevel/leju_sdk.h"
#include "lejusdk-utils/robot_version.hpp"

#include <iostream>

namespace leju {

RobotInterface::RobotInterface()
    : hardware_state_(HardwareState::UNKNOWN)
    , stop_received_(false)
    , robot_api_(nullptr) {
}

RobotInterface::~RobotInterface() {
    if(hw_state_loop_) {
        hw_state_loop_->shutdown();
        hw_state_loop_ = nullptr;
    }
}

bool RobotInterface::initialize() {
    // Basic initialization - can be overridden by derived classes
    stop_received_.store(false);

    auto robot_version = leju::RobotVersion::from_env();  // if fail, will throw exception. enn, it's ok.
    if(!leju::GlobalRobot::init_env(robot_version)) {
        HWLOG_FAILURE("Failed to initialize robot");
    }

    robot_api_ = &GlobalRobot::getInstance();

    // 订阅停止命令
    robot_api_->subscribeStopRobot([this](const StringDataConstPtr& msg) {
        this->onStopCommandReceived(msg);
    });

    hw_state_loop_ =  std::make_unique<leju::Loop>(
        "hardware_state_loop",
        [this]() {
            this->publishHardwareState();
        },
        1000  // 1000ms = 1Hz
    );
    hw_state_loop_->start();
    return true;
}

bool RobotInterface::publishJointState(const RobotState& joint_state) {
    return robot_api_->publishRobotState(joint_state);
}

bool RobotInterface::publishMotorState(const RobotState& motor_state) {
    return robot_api_->publishMotorState(motor_state);
}

bool RobotInterface::publishMotorCmd(const RobotCmd& motor_cmd) {
    return robot_api_->publishMotorCmd(motor_cmd);
}

bool RobotInterface::publishHandState(const HandState& hand_state) {
    return robot_api_->publishHandState(hand_state);
}

bool RobotInterface::publishImuData(const ImuData& imu_data) {
    return robot_api_->publishImuData(imu_data);
}

bool RobotInterface::publishHardwareState() {
    return robot_api_->publishHardwareState(this->hardware_state_.load());
}

bool RobotInterface::publishStopRobot() {
    return robot_api_->publishStopRobot();
}

bool RobotInterface::isStopReceived() const {
    return stop_received_.load();
}

void RobotInterface::setStopReceived() {
    stop_received_.store(true);
}

void RobotInterface::onStopCommandReceived(const StringDataConstPtr& msg) {
    HWLOG_I("Stop command received!");
    this->setStopReceived();
}

void RobotInterface::subscribeRobotCmd(const RobotCmdCallback& callback) {
    robot_api_->subscribeRobotCmd(callback);
}

void RobotInterface::subscribeHandCmd(const HandCmdCallback& callback) {
    robot_api_->subscribeHandCmd(callback);
}

void RobotInterface::subscribeJoyData(const JoyDataCallback& callback) {
    robot_api_->subscribeJoyData(callback);
}

} // namespace leju
