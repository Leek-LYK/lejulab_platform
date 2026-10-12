/**
 * @file vr_base.cpp
 * @brief VR 控制 API 基类实现
 */

#include "lejusdk-vr/vr_api/vr_base.h"

#include "communication/vr_communication.h"
#include "communication/vr_dds_communicator.h"

namespace leju {
namespace vr {

VRBaseAPI::VRBaseAPI(const RobotVersion& version) : robot_version_(version) {}

VRBaseAPI::~VRBaseAPI() = default;

bool VRBaseAPI::initialize() {
  // 初始化通信层（如已初始化则直接返回 true）
  return VRCommunication::getInstance().initialize();
}

void VRBaseAPI::shutdown() {
  VRCommunication::getInstance().shutdown();
}

bool VRBaseAPI::isInitialized() const {
  // 返回通信层的实际初始化状态
  return VRCommunication::getInstance().isInitialized();
}

bool VRBaseAPI::publishArmJointCmd(const JointTrajectoryPoint& cmd) {
  return VRCommunication::getInstance().publishArmJointCmd(cmd);
}

bool VRBaseAPI::publishHeadJointCmd(const JointTrajectoryPoint& cmd) {
  return VRCommunication::getInstance().publishHeadJointCmd(cmd);
}

bool VRBaseAPI::publishWaistJointCmd(const JointTrajectoryPoint& cmd) {
  return VRCommunication::getInstance().publishWaistJointCmd(cmd);
}

bool VRBaseAPI::publishVelocityCmd(const VelocityCmd& cmd) {
  return VRCommunication::getInstance().publishVelocityCmd(cmd);
}

bool VRBaseAPI::publishVrVelocityCmd(const VrVelocityCmd& cmd) {
  return VRCommunication::getInstance().publishVrVelocityCmd(cmd);
}

bool VRBaseAPI::publishQuestBonePoses(const QuestBonePosesData& data) {
  return VRCommunication::getInstance().publishQuestBonePoses(data);
}

bool VRBaseAPI::publishQuestJoystickData(const QuestJoystickData& data) {
  return VRCommunication::getInstance().publishQuestJoystickData(data);
}

bool VRBaseAPI::publishQuestConnected(const std::string& data) {
  return VRCommunication::getInstance().publishQuestConnected(data);
}

void VRBaseAPI::subscribeArmJointCmd(ArmTrajectoryCallback callback) {
  VRCommunication::getInstance().subscribeArmJointCmd(callback);
}

void VRBaseAPI::subscribeHeadJointCmd(HeadTrajectoryCallback callback) {
  VRCommunication::getInstance().subscribeHeadJointCmd(callback);
}

void VRBaseAPI::subscribeWaistJointCmd(WaistTrajectoryCallback callback) {
  VRCommunication::getInstance().subscribeWaistJointCmd(callback);
}

bool VRBaseAPI::switchController(const std::string& controller_name,
                                  std::string* message,
                                  int timeout_ms) {
  auto* dds_comm = dynamic_cast<VRDdsCommunicator*>(&VRCommunication::getInstance());
  if (dds_comm) {
    return dds_comm->switchController(controller_name, message, timeout_ms);
  }
  if (message) {
    *message = "Communication interface does not support message output";
  }
  return VRCommunication::getInstance().switchController(controller_name, timeout_ms);
}

bool VRBaseAPI::setArmMode(ControlMode mode, int timeout_ms) {
  return VRCommunication::getInstance().setArmMode(mode, timeout_ms);
}

bool VRBaseAPI::setHeadMode(ControlMode mode, int timeout_ms) {
  return VRCommunication::getInstance().setHeadMode(mode, timeout_ms);
}

bool VRBaseAPI::setWaistMode(ControlMode mode, int timeout_ms) {
  return VRCommunication::getInstance().setWaistMode(mode, timeout_ms);
}

bool VRBaseAPI::getControllerState(ControllerState& state, int timeout_ms) {
  return VRCommunication::getInstance().getControllerState(state, timeout_ms);
}

bool VRBaseAPI::getRuntimeState(RuntimeState& state, int timeout_ms) {
  return VRCommunication::getInstance().getRuntimeState(state, timeout_ms);
}

bool VRBaseAPI::getMotionState(MotionState& state, int timeout_ms) {
  return VRCommunication::getInstance().getMotionState(state, timeout_ms);
}

bool VRBaseAPI::startRuntime(std::string* message, int timeout_ms) {
  return VRCommunication::getInstance().startRuntime(message, timeout_ms);
}

bool VRBaseAPI::stopRuntime(std::string* message, int timeout_ms) {
  return VRCommunication::getInstance().stopRuntime(message, timeout_ms);
}

bool VRBaseAPI::startMotion(const std::string& name,
                            std::string* message,
                            int timeout_ms) {
  return VRCommunication::getInstance().startMotion(name, message, timeout_ms);
}

bool VRBaseAPI::playTact(const std::string& name,
                         std::string* message,
                         int timeout_ms) {
  return VRCommunication::getInstance().playTact(name, message, timeout_ms);
}

void VRBaseAPI::subscribeVelocityCmd(VelocityCmdCallback callback) {
  VRCommunication::getInstance().subscribeVelocityCmd(callback);
}

void VRBaseAPI::subscribeVrVelocityCmd(VrVelocityCmdCallback callback) {
  VRCommunication::getInstance().subscribeVrVelocityCmd(callback);
}

void VRBaseAPI::subscribeQuestBonePoses(QuestBonePosesCallback callback) {
  VRCommunication::getInstance().subscribeQuestBonePoses(callback);
}

void VRBaseAPI::subscribeQuestJoystickData(QuestJoystickDataCallback callback) {
  VRCommunication::getInstance().subscribeQuestJoystickData(callback);
}

void VRBaseAPI::subscribeTactCommand(TactCmdCallback callback) {
  VRCommunication::getInstance().subscribeTactCommand(callback);
}

void VRBaseAPI::registerSwitchControllerHandler(SwitchControllerHandler handler) {
  VRCommunication::getInstance().registerSwitchControllerHandler(handler);
}

void VRBaseAPI::registerSetArmModeHandler(SetModeHandler handler) {
  VRCommunication::getInstance().registerSetArmModeHandler(handler);
}

void VRBaseAPI::registerSetHeadModeHandler(SetModeHandler handler) {
  VRCommunication::getInstance().registerSetHeadModeHandler(handler);
}

void VRBaseAPI::registerSetWaistModeHandler(SetModeHandler handler) {
  VRCommunication::getInstance().registerSetWaistModeHandler(handler);
}

void VRBaseAPI::registerGetRuntimeStateHandler(GetRuntimeStateHandler handler) {
  VRCommunication::getInstance().registerGetRuntimeStateHandler(handler);
}

void VRBaseAPI::registerGetControllerStateHandler(GetControllerStateHandler handler) {
  VRCommunication::getInstance().registerGetControllerStateHandler(handler);
}

void VRBaseAPI::registerGetMotionStateHandler(GetMotionStateHandler handler) {
  VRCommunication::getInstance().registerGetMotionStateHandler(handler);
}

void VRBaseAPI::registerStartRuntimeHandler(TriggerHandler handler) {
  VRCommunication::getInstance().registerStartRuntimeHandler(handler);
}

void VRBaseAPI::registerStopRuntimeHandler(TriggerHandler handler) {
  VRCommunication::getInstance().registerStopRuntimeHandler(handler);
}

void VRBaseAPI::registerStartMotionHandler(SetStringHandler handler) {
  VRCommunication::getInstance().registerStartMotionHandler(handler);
}

void VRBaseAPI::registerPlayTactHandler(SetStringHandler handler) {
  VRCommunication::getInstance().registerPlayTactHandler(handler);
}

}  // namespace vr
}  // namespace leju
