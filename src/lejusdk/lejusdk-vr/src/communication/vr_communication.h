/**
 * @file vr_communication.h
 * @brief VR 通信接口定义（内部使用）
 */

#ifndef LEJUSDK_VR_COMMUNICATION_VR_COMMUNICATION_H_
#define LEJUSDK_VR_COMMUNICATION_VR_COMMUNICATION_H_

#include "lejusdk-vr/data_types.h"

namespace leju {
namespace vr {

/**
 * @brief VR 通信接口
 *
 * 定义 VR 控制的通信接口，支持 Topic 发布/订阅和 RPC 服务调用。
 */
class IVRCommunication {
 public:
  virtual ~IVRCommunication() = default;

  // Topic 发布
  virtual bool publishArmJointCmd(const JointTrajectoryPoint& cmd) = 0;
  virtual bool publishHeadJointCmd(const JointTrajectoryPoint& cmd) = 0;
  virtual bool publishWaistJointCmd(const JointTrajectoryPoint& cmd) = 0;
  virtual bool publishVelocityCmd(const VelocityCmd& cmd) = 0;
  virtual bool publishVrVelocityCmd(const VrVelocityCmd& cmd) = 0;
  virtual bool publishQuestBonePoses(const QuestBonePosesData& data) = 0;
  virtual bool publishQuestJoystickData(const QuestJoystickData& data) = 0;

  /**
   * @brief 发布 Quest 连接状态心跳
   * @param data 状态字符串（"connected"）
   * @return true 成功，false 失败
   */
  virtual bool publishQuestConnected(const std::string& data) = 0;

  // Topic 订阅
  virtual bool subscribeArmJointCmd(const ArmTrajectoryCallback& callback) = 0;
  virtual bool subscribeHeadJointCmd(const HeadTrajectoryCallback& callback) = 0;
  virtual bool subscribeWaistJointCmd(const WaistTrajectoryCallback& callback) = 0;
  virtual bool subscribeVelocityCmd(const VelocityCmdCallback& callback) = 0;
  virtual bool subscribeVrVelocityCmd(const VrVelocityCmdCallback& callback) = 0;
  virtual bool subscribeQuestBonePoses(const QuestBonePosesCallback& callback) = 0;
  virtual bool subscribeQuestJoystickData(const QuestJoystickDataCallback& callback) = 0;
  virtual bool subscribeTactCommand(const TactCmdCallback& callback) = 0;

  // RPC 客户端（带超时）
  virtual bool switchController(const std::string& controller_name,
                                int timeout_ms = 3000) = 0;
  virtual bool setArmMode(ControlMode mode, int timeout_ms = 3000) = 0;
  virtual bool setHeadMode(ControlMode mode, int timeout_ms = 3000) = 0;
  virtual bool setWaistMode(ControlMode mode, int timeout_ms = 3000) = 0;
  virtual bool getControllerState(ControllerState& state,
                                  int timeout_ms = 3000) = 0;
  virtual bool getRuntimeState(RuntimeState& state, int timeout_ms = 3000) = 0;
  virtual bool getMotionState(MotionState& state, int timeout_ms = 3000) = 0;
  virtual bool startRuntime(std::string* message = nullptr, int timeout_ms = 3000) = 0;
  virtual bool stopRuntime(std::string* message = nullptr, int timeout_ms = 3000) = 0;
  virtual bool startMotion(const std::string& name, std::string* message = nullptr, int timeout_ms = 3000) = 0;
  virtual bool playTact(const std::string& name, std::string* message = nullptr, int timeout_ms = 3000) = 0;

  // RPC 服务端（注册 Handler）
  virtual void registerSwitchControllerHandler(SwitchControllerHandler handler) = 0;
  virtual void registerSetArmModeHandler(SetModeHandler handler) = 0;
  virtual void registerSetHeadModeHandler(SetModeHandler handler) = 0;
  virtual void registerSetWaistModeHandler(SetModeHandler handler) = 0;
  virtual void registerGetRuntimeStateHandler(GetRuntimeStateHandler handler) = 0;
  virtual void registerGetControllerStateHandler(GetControllerStateHandler handler) = 0;
  virtual void registerGetMotionStateHandler(GetMotionStateHandler handler) = 0;
  virtual void registerStartRuntimeHandler(TriggerHandler handler) = 0;
  virtual void registerStopRuntimeHandler(TriggerHandler handler) = 0;
  virtual void registerStartMotionHandler(SetStringHandler handler) = 0;
  virtual void registerPlayTactHandler(SetStringHandler handler) = 0;

  virtual void shutdown() = 0;

  /**
   * @brief 初始化通信层
   * @return true 成功，false 失败
   */
  virtual bool initialize() = 0;

  /**
   * @brief 检查通信层是否已初始化
   * @return true 已初始化，false 未初始化
   */
  virtual bool isInitialized() const = 0;
};

/**
 * @brief VR 通信单例
 */
class VRCommunication {
 public:
  static IVRCommunication& getInstance();

 private:
  VRCommunication() = default;
  ~VRCommunication() = default;
  VRCommunication(const VRCommunication&) = delete;
  VRCommunication& operator=(const VRCommunication&) = delete;
};

}  // namespace vr
}  // namespace leju

#endif  // LEJUSDK_VR_COMMUNICATION_VR_COMMUNICATION_H_
