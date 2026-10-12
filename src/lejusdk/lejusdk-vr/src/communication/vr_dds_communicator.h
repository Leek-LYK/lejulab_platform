/**
 * @file vr_dds_communicator.h
 * @brief VR DDS 通信实现（内部使用）
 */

#ifndef LEJUSDK_VR_COMMUNICATION_VR_DDS_COMMUNICATOR_H_
#define LEJUSDK_VR_COMMUNICATION_VR_DDS_COMMUNICATOR_H_

#include "vr_communication.h"

#include <atomic>
#include <memory>

#include <dds/dds.hpp>
#include <dds_rpc/replier.hpp>
#include <dds_rpc/requester.hpp>
#include <lejusdk-dds-idl/JointTrajectory.hpp>
#include <lejusdk-dds-idl/QuestBonePoses.hpp>
#include <lejusdk-dds-idl/QuestJoysticks.hpp>
#include <lejusdk-dds-idl/VelocityCmd.hpp>
#include <lejusdk-dds-idl/StringData.hpp>
#include <lejusdk-topic-pubsub/topic_publisher.hpp>
#include <lejusdk-topic-pubsub/topic_subscriber.hpp>
#include <lejusdk-utils/callback_wrapper.hpp>
#include <rpc_idl/ControllerService.hpp>
#include <rpc_idl/StdSrvs.hpp>

namespace leju {
namespace vr {

/**
 * @brief RPC 服务名称定义
 *
 * 这些服务由 rl-controller 提供，用于控制器管理、模式切换和原始状态查询。
 */
namespace rpc_services {

/// @brief 切换控制器服务
/// @details 请求类型: SwitchControllerRequest { controller_name: string }
///          响应类型: SwitchControllerReply { success: bool, message: string }
constexpr const char* kSwitchController = "ControllerService/SwitchController";

/// @brief 获取控制器状态服务
/// @details 请求类型: GetControllerStateRequest {}
///          响应类型: GetControllerStateReply { current_controller, arm_mode, head_mode, waist_mode, controller_transitioning, available_controllers }
constexpr const char* kGetControllerState = "ControllerService/GetControllerState";

/// @brief 获取 runtime 状态服务
/// @details 请求类型: GetRuntimeStateRequest {}
///          响应类型: GetRuntimeStateReply { data_ready, hardware_state, lifecycle_state }
constexpr const char* kGetRuntimeState = "ControllerService/GetRuntimeState";

/// @brief 获取 motion 状态服务
/// @details 请求类型: GetMotionStateRequest {}
///          响应类型: GetMotionStateReply { supported, motion_playing, current_motion_name, available_motion_names }
constexpr const char* kGetMotionState = "ControllerService/GetMotionState";

/// @brief 触发 runtime 进入运行态服务
/// @details 请求类型: TriggerRequest {}
///          响应类型: TriggerReply { success: bool, message: string }
constexpr const char* kStartRuntime = "ControllerService/StartRuntime";

/// @brief 触发 runtime 退出服务
/// @details 请求类型: TriggerRequest {}
///          响应类型: TriggerReply { success: bool, message: string }
constexpr const char* kStopRuntime = "ControllerService/StopRuntime";

/// @brief 触发 motion 播放服务
/// @details 请求类型: StartMotionRequest { name: string }
///          响应类型: TriggerReply { success: bool, message: string }
constexpr const char* kStartMotion = "ControllerService/StartMotion";

/// @brief 设置手臂控制模式服务
/// @details 请求类型: SetInt32Request { data: int32 }  // 0=KeepPose, 1=Auto, 2=External
///          响应类型: SetInt32Reply { success: bool }
constexpr const char* kSetArmMode = "ControllerService/SetArmMode";

/// @brief 设置头部控制模式服务
/// @details 请求类型: SetInt32Request { data: int32 }  // 0=KeepPose, 1=Auto, 2=External
///          响应类型: SetInt32Reply { success: bool }
constexpr const char* kSetHeadMode = "ControllerService/SetHeadMode";

/// @brief 设置腰部控制模式服务
/// @details 请求类型: SetInt32Request { data: int32 }  // 0=KeepPose, 1=Auto, 2=External
///          响应类型: SetInt32Reply { success: bool }
constexpr const char* kSetWaistMode = "ControllerService/SetWaistMode";

/// @brief 播放 tact 动作服务（独立于 StartMotion 的 CSV motion）
/// @details 请求类型: SetStringRequest { data: string }  // tact 动作名
///          响应类型: SetStringReply { success: bool, message: string }
constexpr const char* kPlayTact = "TactPlayerService/PlayTact";

}  // namespace rpc_services

/**
 * @brief 基于 DDS/RPC 的 VR 通信实现
 *
 * 负责：
 * - VR 相关 topic 的发布与订阅
 * - 控制器 RPC 的客户端调用
 * - rl-controller 侧 RPC 服务端注册
 *
 */
class VRDdsCommunicator : public IVRCommunication {
 public:
  VRDdsCommunicator();
  ~VRDdsCommunicator() override;

  // Topic 发布
  bool publishArmJointCmd(const JointTrajectoryPoint& cmd) override;
  bool publishHeadJointCmd(const JointTrajectoryPoint& cmd) override;
  bool publishWaistJointCmd(const JointTrajectoryPoint& cmd) override;
  bool publishVelocityCmd(const VelocityCmd& cmd) override;
  bool publishVrVelocityCmd(const VrVelocityCmd& cmd) override;
  bool publishQuestBonePoses(const QuestBonePosesData& data) override;
  bool publishQuestJoystickData(const QuestJoystickData& data) override;
  bool publishQuestConnected(const std::string& data) override;

  // Topic 订阅
  bool subscribeArmJointCmd(const ArmTrajectoryCallback& callback) override;
  bool subscribeHeadJointCmd(const HeadTrajectoryCallback& callback) override;
  bool subscribeWaistJointCmd(const WaistTrajectoryCallback& callback) override;
  bool subscribeVelocityCmd(const VelocityCmdCallback& callback) override;
  bool subscribeVrVelocityCmd(const VrVelocityCmdCallback& callback) override;
  bool subscribeQuestBonePoses(const QuestBonePosesCallback& callback) override;
  bool subscribeQuestJoystickData(const QuestJoystickDataCallback& callback) override;
  bool subscribeTactCommand(const TactCmdCallback& callback) override;

  // RPC 客户端
  bool switchController(const std::string& controller_name,
                        int timeout_ms = 3000) override;
  bool switchController(const std::string& controller_name,
                        std::string* message,
                        int timeout_ms = 3000);
  bool setArmMode(ControlMode mode, int timeout_ms = 3000) override;
  bool setHeadMode(ControlMode mode, int timeout_ms = 3000) override;
  bool setWaistMode(ControlMode mode, int timeout_ms = 3000) override;
  bool getControllerState(ControllerState& state,
                          int timeout_ms = 3000) override;
  bool getRuntimeState(RuntimeState& state,
                       int timeout_ms = 3000) override;
  bool getMotionState(MotionState& state,
                      int timeout_ms = 3000) override;
  bool startRuntime(std::string* message = nullptr,
                    int timeout_ms = 3000) override;
  bool stopRuntime(std::string* message = nullptr,
                   int timeout_ms = 3000) override;
  bool startMotion(const std::string& name,
                   std::string* message = nullptr,
                   int timeout_ms = 3000) override;
  bool playTact(const std::string& name,
                std::string* message = nullptr,
                int timeout_ms = 3000) override;

  // RPC 服务端
  void registerSwitchControllerHandler(SwitchControllerHandler handler) override;
  void registerSetArmModeHandler(SetModeHandler handler) override;
  void registerSetHeadModeHandler(SetModeHandler handler) override;
  void registerSetWaistModeHandler(SetModeHandler handler) override;
  void registerGetRuntimeStateHandler(GetRuntimeStateHandler handler) override;
  void registerGetControllerStateHandler(GetControllerStateHandler handler) override;
  void registerGetMotionStateHandler(GetMotionStateHandler handler) override;
  void registerStartRuntimeHandler(TriggerHandler handler) override;
  void registerStopRuntimeHandler(TriggerHandler handler) override;
  void registerStartMotionHandler(SetStringHandler handler) override;
  void registerPlayTactHandler(SetStringHandler handler) override;

  void shutdown() override;

  bool initialize() override;

  bool isInitialized() const override { return initialized_.load(); }

 private:

  static constexpr int kDefaultDomainId = 0;

  std::atomic<bool> initialized_{false};
  dds::domain::DomainParticipant participant_{dds::core::null};

  // Topic Publishers
  std::unique_ptr<dds_common::TopicPublisher<leju::msgs::JointTrajectoryPoint>>
      arm_traj_pub_;
  std::unique_ptr<dds_common::TopicPublisher<leju::msgs::JointTrajectoryPoint>>
      head_traj_pub_;
  std::unique_ptr<dds_common::TopicPublisher<leju::msgs::JointTrajectoryPoint>>
      waist_traj_pub_;
  std::unique_ptr<dds_common::TopicPublisher<leju::msgs::VelocityCmd>>
      vel_cmd_pub_;
  std::unique_ptr<dds_common::TopicPublisher<leju::msgs::VelocityCmd>>
      vr_vel_cmd_pub_;
  std::unique_ptr<dds_common::TopicPublisher<leju::msgs::QuestBonePoses>>
      quest_bone_poses_pub_;
  std::unique_ptr<dds_common::TopicPublisher<leju::msgs::QuestJoysticks>>
      quest_joysticks_pub_;
  std::unique_ptr<dds_common::TopicPublisher<leju::msgs::StringData>>
      quest_connected_pub_;

  // Topic Subscribers
  std::unique_ptr<dds_common::TopicSubscriber<leju::msgs::JointTrajectoryPoint>>
      arm_traj_sub_;
  std::unique_ptr<dds_common::TopicSubscriber<leju::msgs::JointTrajectoryPoint>>
      head_traj_sub_;
  std::unique_ptr<dds_common::TopicSubscriber<leju::msgs::JointTrajectoryPoint>>
      waist_traj_sub_;
  std::unique_ptr<dds_common::TopicSubscriber<leju::msgs::VelocityCmd>>
      vel_cmd_sub_;
  std::unique_ptr<dds_common::TopicSubscriber<leju::msgs::VelocityCmd>>
      vr_vel_cmd_sub_;
  std::unique_ptr<dds_common::TopicSubscriber<leju::msgs::QuestBonePoses>>
      quest_bone_poses_sub_;
  std::unique_ptr<dds_common::TopicSubscriber<leju::msgs::QuestJoysticks>>
      quest_joysticks_sub_;
  std::unique_ptr<dds_common::TopicSubscriber<leju::msgs::StringData>>
      tact_cmd_sub_;

  // Topic Callbacks (线程安全的回调包装器)
  internal::CallbackWrapper<ArmTrajectoryCallback> arm_callback_;
  internal::CallbackWrapper<HeadTrajectoryCallback> head_callback_;
  internal::CallbackWrapper<WaistTrajectoryCallback> waist_callback_;
  internal::CallbackWrapper<VelocityCmdCallback> vel_cmd_callback_;
  internal::CallbackWrapper<VrVelocityCmdCallback> vr_vel_cmd_callback_;
  internal::CallbackWrapper<QuestBonePosesCallback> quest_bone_poses_callback_;
  internal::CallbackWrapper<QuestJoystickDataCallback> quest_joysticks_callback_;
  internal::CallbackWrapper<TactCmdCallback> tact_cmd_callback_;

  // RPC Requesters
  std::unique_ptr<dds_rpc::Requester<leju::srv::SwitchControllerRequest,
                                     leju::srv::SwitchControllerReply>>
      switch_controller_requester_;
  std::unique_ptr<dds_rpc::Requester<leju::srv::GetControllerStateRequest,
                                     leju::srv::GetControllerStateReply>>
      get_controller_state_requester_;
  std::unique_ptr<dds_rpc::Requester<leju::srv::GetRuntimeStateRequest,
                                     leju::srv::GetRuntimeStateReply>>
      get_runtime_state_requester_;
  std::unique_ptr<dds_rpc::Requester<leju::srv::GetMotionStateRequest,
                                     leju::srv::GetMotionStateReply>>
      get_motion_state_requester_;
  std::unique_ptr<dds_rpc::Requester<std_srvs::TriggerRequest,
                                     std_srvs::TriggerReply>>
      start_runtime_requester_;
  std::unique_ptr<dds_rpc::Requester<std_srvs::TriggerRequest,
                                     std_srvs::TriggerReply>>
      stop_runtime_requester_;
  std::unique_ptr<dds_rpc::Requester<std_srvs::SetStringRequest,
                                     std_srvs::SetStringReply>>
      start_motion_requester_;
  std::unique_ptr<dds_rpc::Requester<std_srvs::SetStringRequest,
                                     std_srvs::SetStringReply>>
      play_tact_requester_;
  std::unique_ptr<dds_rpc::Requester<std_srvs::SetInt32Request,
                                     std_srvs::SetInt32Reply>>
      set_arm_mode_requester_;
  std::unique_ptr<dds_rpc::Requester<std_srvs::SetInt32Request,
                                     std_srvs::SetInt32Reply>>
      set_head_mode_requester_;
  std::unique_ptr<dds_rpc::Requester<std_srvs::SetInt32Request,
                                     std_srvs::SetInt32Reply>>
      set_waist_mode_requester_;

  // RPC Handlers（服务端）
  SwitchControllerHandler switch_controller_handler_;
  SetModeHandler set_arm_mode_handler_;
  SetModeHandler set_head_mode_handler_;
  SetModeHandler set_waist_mode_handler_;
  GetRuntimeStateHandler get_runtime_state_handler_;
  GetControllerStateHandler get_controller_state_handler_;
  GetMotionStateHandler get_motion_state_handler_;
  TriggerHandler start_runtime_handler_;
  TriggerHandler stop_runtime_handler_;
  SetStringHandler start_motion_handler_;
  SetStringHandler play_tact_handler_;

  // RPC Repliers（服务端）
  std::unique_ptr<dds_rpc::Replier<leju::srv::SwitchControllerRequest,
                                   leju::srv::SwitchControllerReply>>
      switch_controller_replier_;
  std::unique_ptr<dds_rpc::Replier<leju::srv::GetControllerStateRequest,
                                   leju::srv::GetControllerStateReply>>
      get_controller_state_replier_;
  std::unique_ptr<dds_rpc::Replier<leju::srv::GetRuntimeStateRequest,
                                   leju::srv::GetRuntimeStateReply>>
      get_runtime_state_replier_;
  std::unique_ptr<dds_rpc::Replier<leju::srv::GetMotionStateRequest,
                                   leju::srv::GetMotionStateReply>>
      get_motion_state_replier_;
  std::unique_ptr<dds_rpc::Replier<std_srvs::TriggerRequest,
                                   std_srvs::TriggerReply>>
      start_runtime_replier_;
  std::unique_ptr<dds_rpc::Replier<std_srvs::TriggerRequest,
                                   std_srvs::TriggerReply>>
      stop_runtime_replier_;
  std::unique_ptr<dds_rpc::Replier<std_srvs::SetStringRequest,
                                   std_srvs::SetStringReply>>
      start_motion_replier_;
  std::unique_ptr<dds_rpc::Replier<std_srvs::SetStringRequest,
                                   std_srvs::SetStringReply>>
      play_tact_replier_;
  std::unique_ptr<dds_rpc::Replier<std_srvs::SetInt32Request,
                                   std_srvs::SetInt32Reply>>
      set_arm_mode_replier_;
  std::unique_ptr<dds_rpc::Replier<std_srvs::SetInt32Request,
                                   std_srvs::SetInt32Reply>>
      set_head_mode_replier_;
  std::unique_ptr<dds_rpc::Replier<std_srvs::SetInt32Request,
                                   std_srvs::SetInt32Reply>>
      set_waist_mode_replier_;
};

}  // namespace vr
}  // namespace leju

#endif  // LEJUSDK_VR_COMMUNICATION_VR_DDS_COMMUNICATOR_H_
