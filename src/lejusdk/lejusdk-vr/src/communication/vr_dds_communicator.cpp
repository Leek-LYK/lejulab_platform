/**
 * @file vr_dds_communicator.cpp
 * @brief VR DDS 通信实现
 */

#include "vr_dds_communicator.h"
#include "vr_msg_converter.hpp"

#include <algorithm>
#include <chrono>
#include <iostream>
#include <string>

#include <lejusdk-topic-pubsub/topic_names.h>
#include <lejusdk-dds-idl/StringData.hpp>

namespace leju {
namespace vr {

namespace {

// 服务发现超时（仅在服务未 matched 时等待）
constexpr int kServiceDiscoveryTimeoutMs = 1000;

/**
* @brief 通用 RPC 调用模板
*
* @tparam Requester Requester 类型
* @tparam Callable 调用函数类型
* @param requester Requester 智能指针引用
* @param func_name 函数名（用于日志）
* @param timeout_ms 超时时间
* @param call_fn 实际执行请求的函数 (requester, timeout_ms) -> bool
* @return 调用是否成功
*/
template <typename Requester, typename Callable>
bool callRpcService(
    std::unique_ptr<Requester>& requester,
    const char* func_name,
    int timeout_ms,
    Callable&& call_fn) {
  if (!requester) {
    std::cerr << "[VRCommunication] " << func_name
              << ": requester not initialized" << std::endl;
    return false;
  }

  try {
    // 确保服务已 matched（如果已 matched 会立即返回）
    int discovery_timeout = std::min(kServiceDiscoveryTimeoutMs, timeout_ms);
    if (!requester->wait_for_service(std::chrono::milliseconds(discovery_timeout))) {
      std::cerr << "[VRCommunication] " << func_name
                << ": service not available" << std::endl;
      return false;
    }

    return call_fn(*requester, timeout_ms);
  } catch (const dds_rpc::TimeoutException&) {
    std::cerr << "[VRCommunication] " << func_name
              << " failed: request timeout (timeout=" << timeout_ms << "ms)"
              << std::endl;
    return false;
  } catch (const std::exception& e) {
    std::cerr << "[VRCommunication] " << func_name
              << " failed: " << e.what() << std::endl;
    return false;
  }
}

}  // namespace

VRDdsCommunicator::VRDdsCommunicator() = default;

VRDdsCommunicator::~VRDdsCommunicator() {
  // 清理所有资源
  initialized_ = false;

  // 清理 Subscribers
  arm_traj_sub_.reset();
  head_traj_sub_.reset();
  waist_traj_sub_.reset();
  vel_cmd_sub_.reset();
  vr_vel_cmd_sub_.reset();
  quest_bone_poses_sub_.reset();
  quest_joysticks_sub_.reset();
  tact_cmd_sub_.reset();

  // 清理 Publishers
  arm_traj_pub_.reset();
  head_traj_pub_.reset();
  waist_traj_pub_.reset();
  vel_cmd_pub_.reset();
  vr_vel_cmd_pub_.reset();
  quest_bone_poses_pub_.reset();
  quest_joysticks_pub_.reset();
  quest_connected_pub_.reset();

  // 清理 RPC Requesters
  switch_controller_requester_.reset();
  get_controller_state_requester_.reset();
  get_runtime_state_requester_.reset();
  get_motion_state_requester_.reset();
  start_runtime_requester_.reset();
  stop_runtime_requester_.reset();
  start_motion_requester_.reset();
  play_tact_requester_.reset();
  set_arm_mode_requester_.reset();
  set_head_mode_requester_.reset();
  set_waist_mode_requester_.reset();

  // 清理 RPC Repliers
  switch_controller_replier_.reset();
  get_controller_state_replier_.reset();
  get_runtime_state_replier_.reset();
  get_motion_state_replier_.reset();
  start_runtime_replier_.reset();
  stop_runtime_replier_.reset();
  start_motion_replier_.reset();
  play_tact_replier_.reset();
  set_arm_mode_replier_.reset();
  set_head_mode_replier_.reset();
  set_waist_mode_replier_.reset();

  // 在析构函数中销毁 DDS Participant
  if (participant_ != dds::core::null) {
    participant_ = dds::core::null;
  }

  std::cout << "[VRCommunication] Shutdown complete" << std::endl;
}

bool VRDdsCommunicator::initialize() {
  if (initialized_) {
    return true;
  }

  try {
    // 如果 participant 不存在，创建新的
    // 如果已存在（shutdown 后重新初始化），复用现有的
    if (participant_ == dds::core::null) {
      participant_ = dds::domain::DomainParticipant(kDefaultDomainId);
    }

    dds_entity_t participant_handle = participant_->get_ddsc_entity();

    // 创建 Topic Publishers
    arm_traj_pub_ =
        std::make_unique<dds_common::TopicPublisher<leju::msgs::JointTrajectoryPoint>>(
            participant_, dds_topics::kArmTrajectory);
    head_traj_pub_ =
        std::make_unique<dds_common::TopicPublisher<leju::msgs::JointTrajectoryPoint>>(
            participant_, dds_topics::kHeadTrajectory);
    waist_traj_pub_ =
        std::make_unique<dds_common::TopicPublisher<leju::msgs::JointTrajectoryPoint>>(
            participant_, dds_topics::kWaistTrajectory);
    vel_cmd_pub_ =
        std::make_unique<dds_common::TopicPublisher<leju::msgs::VelocityCmd>>(
            participant_, dds_topics::kCmdVel);
    vr_vel_cmd_pub_ =
        std::make_unique<dds_common::TopicPublisher<leju::msgs::VelocityCmd>>(
            participant_, dds_topics::kVrCmdVel);
    quest_bone_poses_pub_ =
        std::make_unique<dds_common::TopicPublisher<leju::msgs::QuestBonePoses>>(
            participant_, dds_topics::kQuestBonePoses);
    quest_joysticks_pub_ =
        std::make_unique<dds_common::TopicPublisher<leju::msgs::QuestJoysticks>>(
            participant_, dds_topics::kQuestJoysticks);
    quest_connected_pub_ =
        std::make_unique<dds_common::TopicPublisher<leju::msgs::StringData>>(
            participant_, dds_topics::kQuestConnected);

    // 创建 RPC Requesters（提前初始化，避免首次调用延迟）
    // std::cerr << "[VRCommunication] Creating RPC requesters..." << std::endl;

    switch_controller_requester_ = std::make_unique<
        dds_rpc::Requester<leju::srv::SwitchControllerRequest, leju::srv::SwitchControllerReply>>(
        participant_handle, rpc_services::kSwitchController);
    get_controller_state_requester_ = std::make_unique<
        dds_rpc::Requester<leju::srv::GetControllerStateRequest, leju::srv::GetControllerStateReply>>(
        participant_handle, rpc_services::kGetControllerState);
    get_runtime_state_requester_ = std::make_unique<
        dds_rpc::Requester<leju::srv::GetRuntimeStateRequest, leju::srv::GetRuntimeStateReply>>(
        participant_handle, rpc_services::kGetRuntimeState);
    get_motion_state_requester_ = std::make_unique<
        dds_rpc::Requester<leju::srv::GetMotionStateRequest, leju::srv::GetMotionStateReply>>(
        participant_handle, rpc_services::kGetMotionState);
    start_runtime_requester_ = std::make_unique<
        dds_rpc::Requester<std_srvs::TriggerRequest, std_srvs::TriggerReply>>(
        participant_handle, rpc_services::kStartRuntime);
    stop_runtime_requester_ = std::make_unique<
        dds_rpc::Requester<std_srvs::TriggerRequest, std_srvs::TriggerReply>>(
        participant_handle, rpc_services::kStopRuntime);
    start_motion_requester_ = std::make_unique<
        dds_rpc::Requester<std_srvs::SetStringRequest, std_srvs::SetStringReply>>(
        participant_handle, rpc_services::kStartMotion);
    play_tact_requester_ = std::make_unique<
        dds_rpc::Requester<std_srvs::SetStringRequest, std_srvs::SetStringReply>>(
        participant_handle, rpc_services::kPlayTact);
    set_arm_mode_requester_ = std::make_unique<
        dds_rpc::Requester<std_srvs::SetInt32Request, std_srvs::SetInt32Reply>>(
        participant_handle, rpc_services::kSetArmMode);
    set_head_mode_requester_ = std::make_unique<
        dds_rpc::Requester<std_srvs::SetInt32Request, std_srvs::SetInt32Reply>>(
        participant_handle, rpc_services::kSetHeadMode);
    set_waist_mode_requester_ = std::make_unique<
        dds_rpc::Requester<std_srvs::SetInt32Request, std_srvs::SetInt32Reply>>(
        participant_handle, rpc_services::kSetWaistMode);

    initialized_ = true;
    std::cout << "[VRCommunication] Initialized successfully" << std::endl;
    return true;
  } catch (const std::exception& e) {
    std::cerr << "[VRCommunication] Failed to initialize: " << e.what()
              << std::endl;
    // 注意：不调用 shutdown()，避免破坏 participant
    initialized_ = false;
    return false;
  }
}

void VRDdsCommunicator::shutdown() {
  if (!initialized_) {
    return;  // 已经 shutdown，无需重复操作
  }

  initialized_ = false;

  // 清理 Subscribers
  arm_traj_sub_.reset();
  head_traj_sub_.reset();
  waist_traj_sub_.reset();
  vel_cmd_sub_.reset();
  vr_vel_cmd_sub_.reset();
  quest_bone_poses_sub_.reset();
  quest_joysticks_sub_.reset();
  tact_cmd_sub_.reset();

  // 清理 Publishers
  arm_traj_pub_.reset();
  head_traj_pub_.reset();
  waist_traj_pub_.reset();
  vel_cmd_pub_.reset();
  vr_vel_cmd_pub_.reset();
  quest_bone_poses_pub_.reset();
  quest_joysticks_pub_.reset();
  quest_connected_pub_.reset();

  // 清理 RPC Requesters
  switch_controller_requester_.reset();
  get_controller_state_requester_.reset();
  get_runtime_state_requester_.reset();
  get_motion_state_requester_.reset();
  start_runtime_requester_.reset();
  stop_runtime_requester_.reset();
  start_motion_requester_.reset();
  play_tact_requester_.reset();
  set_arm_mode_requester_.reset();
  set_head_mode_requester_.reset();
  set_waist_mode_requester_.reset();

  // 清理 RPC Repliers
  switch_controller_replier_.reset();
  get_controller_state_replier_.reset();
  get_runtime_state_replier_.reset();
  get_motion_state_replier_.reset();
  start_runtime_replier_.reset();
  stop_runtime_replier_.reset();
  start_motion_replier_.reset();
  play_tact_replier_.reset();
  set_arm_mode_replier_.reset();
  set_head_mode_replier_.reset();
  set_waist_mode_replier_.reset();

  // 注意：不销毁 DDS Participant
  // CycloneDDS 在同一进程中销毁并重建 Participant 可能导致问题
  // Participant 仅在单例析构时（程序退出）销毁

  std::cout << "[VRCommunication] Shutdown complete" << std::endl;
}

bool VRDdsCommunicator::publishArmJointCmd(const JointTrajectoryPoint& cmd) {
  if (!initialized_ || !arm_traj_pub_) return false;
  return arm_traj_pub_->publish(toMsg(cmd));
}

bool VRDdsCommunicator::publishHeadJointCmd(const JointTrajectoryPoint& cmd) {
  if (!initialized_ || !head_traj_pub_) return false;
  return head_traj_pub_->publish(toMsg(cmd));
}

bool VRDdsCommunicator::publishWaistJointCmd(const JointTrajectoryPoint& cmd) {
  if (!initialized_ || !waist_traj_pub_) return false;
  return waist_traj_pub_->publish(toMsg(cmd));
}

bool VRDdsCommunicator::publishVelocityCmd(const VelocityCmd& cmd) {
  if (!initialized_ || !vel_cmd_pub_) return false;
  return vel_cmd_pub_->publish(toMsg(cmd));
}

bool VRDdsCommunicator::publishVrVelocityCmd(const VrVelocityCmd& cmd) {
  if (!initialized_ || !vr_vel_cmd_pub_) return false;
  return vr_vel_cmd_pub_->publish(toMsg(cmd));
}

bool VRDdsCommunicator::publishQuestBonePoses(const QuestBonePosesData& data) {
  if (!initialized_ || !quest_bone_poses_pub_) return false;
  return quest_bone_poses_pub_->publish(toMsg(data));
}

bool VRDdsCommunicator::publishQuestJoystickData(const QuestJoystickData& data) {
  if (!initialized_ || !quest_joysticks_pub_) return false;
  return quest_joysticks_pub_->publish(toMsg(data));
}

bool VRDdsCommunicator::publishQuestConnected(const std::string& data) {
  if (!initialized_ || !quest_connected_pub_) {
    // 心跳线程每秒调用一次，失败日志需节流，否则刷屏
    static auto last_warn = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::seconds>(now - last_warn).count() >= 10) {
      std::cerr << "[VRCommunication] publishQuestConnected: not initialized or pub missing"
                << std::endl;
      last_warn = now;
    }
    return false;
  }
  auto now = std::chrono::system_clock::now();
  auto sec = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();
  auto nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count() % 1000000000;
  leju::msgs::StringData msg(static_cast<int32_t>(sec), static_cast<uint32_t>(nsec), data);
  return quest_connected_pub_->publish(msg);
}

bool VRDdsCommunicator::subscribeArmJointCmd(const ArmTrajectoryCallback& callback) {
  if (!initialized_) return false;

  // 只在首次时创建 Subscriber
  if (!arm_traj_sub_) {
    arm_traj_sub_ =
        std::make_unique<dds_common::TopicSubscriber<leju::msgs::JointTrajectoryPoint>>(
            participant_, dds_topics::kArmTrajectory,
            [this](const leju::msgs::JointTrajectoryPoint& msg) {
              arm_callback_.call_if(fromMsg(msg));
            });
  }

  // 更新回调（线程安全）
  arm_callback_.set(callback);
  return true;
}

bool VRDdsCommunicator::subscribeHeadJointCmd(const HeadTrajectoryCallback& callback) {
  if (!initialized_) return false;

  if (!head_traj_sub_) {
    head_traj_sub_ =
        std::make_unique<dds_common::TopicSubscriber<leju::msgs::JointTrajectoryPoint>>(
            participant_, dds_topics::kHeadTrajectory,
            [this](const leju::msgs::JointTrajectoryPoint& msg) {
              head_callback_.call_if(fromMsg(msg));
            });
  }

  head_callback_.set(callback);
  return true;
}

bool VRDdsCommunicator::subscribeWaistJointCmd(const WaistTrajectoryCallback& callback) {
  if (!initialized_) return false;

  if (!waist_traj_sub_) {
    waist_traj_sub_ =
        std::make_unique<dds_common::TopicSubscriber<leju::msgs::JointTrajectoryPoint>>(
            participant_, dds_topics::kWaistTrajectory,
            [this](const leju::msgs::JointTrajectoryPoint& msg) {
              waist_callback_.call_if(fromMsg(msg));
            });
  }

  waist_callback_.set(callback);
  return true;
}

bool VRDdsCommunicator::switchController(const std::string& controller_name,
                                          int timeout_ms) {
  return switchController(controller_name, nullptr, timeout_ms);
}

bool VRDdsCommunicator::switchController(const std::string& controller_name,
                                          std::string* message,
                                          int timeout_ms) {
  if (!initialized_) {
    if (message) {
      *message = "Communicator not initialized";
    }
    return false;
  }
  return callRpcService(
      switch_controller_requester_,
      "switchController",
      timeout_ms,
      [&](auto& requester, int timeout) {
        leju::srv::SwitchControllerRequest req;
        req.controller_name(controller_name);
        auto reply = requester.send_request(
            req, std::chrono::milliseconds(timeout));
        if (message) {
          *message = reply.message();
        }
        return reply.success();
      });
}

bool VRDdsCommunicator::setArmMode(ControlMode mode, int timeout_ms) {
  if (!initialized_) return false;
  return callRpcService(
      set_arm_mode_requester_,
      "setArmMode",
      timeout_ms,
      [mode](auto& requester, int timeout) {
        std_srvs::SetInt32Request req;
        req.data(static_cast<int>(mode));
        auto reply = requester.send_request(
            req, std::chrono::milliseconds(timeout));
        return reply.success();
      });
}

bool VRDdsCommunicator::setHeadMode(ControlMode mode, int timeout_ms) {
  if (!initialized_) return false;
  return callRpcService(
      set_head_mode_requester_,
      "setHeadMode",
      timeout_ms,
      [mode](auto& requester, int timeout) {
        std_srvs::SetInt32Request req;
        req.data(static_cast<int>(mode));
        auto reply = requester.send_request(
            req, std::chrono::milliseconds(timeout));
        return reply.success();
      });
}

bool VRDdsCommunicator::setWaistMode(ControlMode mode, int timeout_ms) {
  if (!initialized_) return false;
  return callRpcService(
      set_waist_mode_requester_,
      "setWaistMode",
      timeout_ms,
      [mode](auto& requester, int timeout) {
        std_srvs::SetInt32Request req;
        req.data(static_cast<int>(mode));
        auto reply = requester.send_request(
            req, std::chrono::milliseconds(timeout));
        return reply.success();
      });
}

bool VRDdsCommunicator::getControllerState(ControllerState& state,
                                            int timeout_ms) {
  if (!initialized_) return false;
  return callRpcService(
      get_controller_state_requester_,
      "getControllerState",
      timeout_ms,
      [&state](auto& requester, int timeout) {
        leju::srv::GetControllerStateRequest req;
        auto reply = requester.send_request(
            req, std::chrono::milliseconds(timeout));
        state.current_controller = reply.current_controller();
        state.arm_mode = static_cast<ControlMode>(reply.arm_mode());
        state.head_mode = static_cast<ControlMode>(reply.head_mode());
        state.waist_mode = static_cast<ControlMode>(reply.waist_mode());
        state.controller_transitioning = reply.controller_transitioning();
        state.available_controllers = reply.available_controllers();
        return true;
      });
}

bool VRDdsCommunicator::getRuntimeState(RuntimeState& state,
                                          int timeout_ms) {
  if (!initialized_) return false;
  return callRpcService(
      get_runtime_state_requester_,
      "getRuntimeState",
      timeout_ms,
      [&state](auto& requester, int timeout) {
        leju::srv::GetRuntimeStateRequest req;
        auto reply = requester.send_request(
            req, std::chrono::milliseconds(timeout));
        state.data_ready = reply.data_ready();
        state.hardware_state = static_cast<HardwareState>(reply.hardware_state());
        state.lifecycle_state = static_cast<LifecycleState>(reply.lifecycle_state());
        return true;
      });
}

bool VRDdsCommunicator::getMotionState(MotionState& state,
                                        int timeout_ms) {
  if (!initialized_) return false;
  return callRpcService(
      get_motion_state_requester_,
      "getMotionState",
      timeout_ms,
      [&state](auto& requester, int timeout) {
        leju::srv::GetMotionStateRequest req;
        auto reply = requester.send_request(
            req, std::chrono::milliseconds(timeout));
        state.supported = reply.supported();
        state.motion_playing = reply.motion_playing();
        state.current_motion_name = reply.current_motion_name();
        state.available_motion_names = reply.available_motion_names();
        return true;
      });
}

bool VRDdsCommunicator::startRuntime(std::string* message,
                                      int timeout_ms) {
  if (!initialized_) {
    if (message) {
      *message = "Communicator not initialized";
    }
    return false;
  }
  return callRpcService(
      start_runtime_requester_,
      "startRuntime",
      timeout_ms,
      [&](auto& requester, int timeout) {
        std_srvs::TriggerRequest req;
        auto reply = requester.send_request(
            req, std::chrono::milliseconds(timeout));
        if (message) {
          *message = reply.message();
        }
        return reply.success();
      });
}

bool VRDdsCommunicator::stopRuntime(std::string* message,
                                      int timeout_ms) {
  if (!initialized_) {
    if (message) {
      *message = "Communicator not initialized";
    }
    return false;
  }
  return callRpcService(
      stop_runtime_requester_,
      "stopRuntime",
      timeout_ms,
      [&](auto& requester, int timeout) {
        std_srvs::TriggerRequest req;
        auto reply = requester.send_request(
            req, std::chrono::milliseconds(timeout));
        if (message) {
          *message = reply.message();
        }
        return reply.success();
      });
}

bool VRDdsCommunicator::startMotion(const std::string& name,
                                    std::string* message,
                                    int timeout_ms) {
  if (!initialized_) {
    if (message) {
      *message = "Communicator not initialized";
    }
    return false;
  }
  return callRpcService(
      start_motion_requester_,
      "startMotion",
      timeout_ms,
      [&](auto& requester, int timeout) {
        std_srvs::SetStringRequest req;
        req.data(name);
        auto reply = requester.send_request(
            req, std::chrono::milliseconds(timeout));
        if (message) {
          *message = reply.message();
        }
        return reply.success();
      });
}

bool VRDdsCommunicator::playTact(const std::string& name,
                                 std::string* message,
                                 int timeout_ms) {
  if (!initialized_) {
    if (message) {
      *message = "Communicator not initialized";
    }
    return false;
  }
  return callRpcService(
      play_tact_requester_,
      "playTact",
      timeout_ms,
      [&](auto& requester, int timeout) {
        std_srvs::SetStringRequest req;
        req.data(name);
        auto reply = requester.send_request(
            req, std::chrono::milliseconds(timeout));
        if (message) {
          *message = reply.message();
        }
        return reply.success();
      });
}

bool VRDdsCommunicator::subscribeVelocityCmd(const VelocityCmdCallback& callback) {
  if (!initialized_) return false;

  // 只在首次时创建 Subscriber
  if (!vel_cmd_sub_) {
    vel_cmd_sub_ =
        std::make_unique<dds_common::TopicSubscriber<leju::msgs::VelocityCmd>>(
            participant_, dds_topics::kCmdVel,
            [this](const leju::msgs::VelocityCmd& msg) {
              vel_cmd_callback_.call_if(fromMsg(msg));
            });
  }

  // 更新回调（线程安全）
  vel_cmd_callback_.set(callback);
  return true;
}

bool VRDdsCommunicator::subscribeVrVelocityCmd(
    const VrVelocityCmdCallback& callback) {
  if (!initialized_) return false;

  if (!vr_vel_cmd_sub_) {
    vr_vel_cmd_sub_ =
        std::make_unique<dds_common::TopicSubscriber<leju::msgs::VelocityCmd>>(
            participant_, dds_topics::kVrCmdVel,
            [this](const leju::msgs::VelocityCmd& msg) {
              vr_vel_cmd_callback_.call_if(fromVrMsg(msg));
            });
  }

  vr_vel_cmd_callback_.set(callback);
  return true;
}

bool VRDdsCommunicator::subscribeQuestBonePoses(
    const QuestBonePosesCallback& callback) {
  if (!initialized_) return false;

  if (!quest_bone_poses_sub_) {
    quest_bone_poses_sub_ =
        std::make_unique<dds_common::TopicSubscriber<leju::msgs::QuestBonePoses>>(
            participant_, dds_topics::kQuestBonePoses,
            [this](const leju::msgs::QuestBonePoses& msg) {
              quest_bone_poses_callback_.call_if(fromMsg(msg));
            });
  }

  quest_bone_poses_callback_.set(callback);
  return true;
}

bool VRDdsCommunicator::subscribeQuestJoystickData(
    const QuestJoystickDataCallback& callback) {
  if (!initialized_) return false;

  if (!quest_joysticks_sub_) {
    quest_joysticks_sub_ =
        std::make_unique<dds_common::TopicSubscriber<leju::msgs::QuestJoysticks>>(
            participant_, dds_topics::kQuestJoysticks,
            [this](const leju::msgs::QuestJoysticks& msg) {
              quest_joysticks_callback_.call_if(fromMsg(msg));
            });
  }

  quest_joysticks_callback_.set(callback);
  return true;
}

bool VRDdsCommunicator::subscribeTactCommand(const TactCmdCallback& callback) {
  if (!initialized_) return false;

  if (!tact_cmd_sub_) {
    tact_cmd_sub_ = std::make_unique<dds_common::TopicSubscriber<leju::msgs::StringData>>(
        participant_, dds_topics::kTactCommand,
        [this](const leju::msgs::StringData& msg) {
          tact_cmd_callback_.call_if(msg.data());
        });
  }

  tact_cmd_callback_.set(callback);
  return true;
}

void VRDdsCommunicator::registerSwitchControllerHandler(SwitchControllerHandler handler) {
  if (!initialized_) return;
  switch_controller_handler_ = handler;

  // 如果 Replier 已存在，只需更新 handler 即可
  if (switch_controller_replier_) return;

  dds_entity_t participant_handle = participant_->get_ddsc_entity();
  switch_controller_replier_ = std::make_unique<
      dds_rpc::Replier<leju::srv::SwitchControllerRequest,
                      leju::srv::SwitchControllerReply>>(
      participant_handle, rpc_services::kSwitchController);

  switch_controller_replier_->set_handler(
      [this](const leju::srv::SwitchControllerRequest& req) {
        leju::srv::SwitchControllerReply reply;
        if (switch_controller_handler_) {
          std::string message;
          bool success = switch_controller_handler_(req.controller_name(), message);
          reply.success(success);
          reply.message(message);
          reply.current_controller(req.controller_name());
        } else {
          reply.success(false);
          reply.message("No handler registered");
        }
        return reply;
      });

  switch_controller_replier_->start();
}

void VRDdsCommunicator::registerSetArmModeHandler(SetModeHandler handler) {
  if (!initialized_) return;
  set_arm_mode_handler_ = handler;

  if (set_arm_mode_replier_) return;

  dds_entity_t participant_handle = participant_->get_ddsc_entity();
  set_arm_mode_replier_ = std::make_unique<
      dds_rpc::Replier<std_srvs::SetInt32Request, std_srvs::SetInt32Reply>>(
      participant_handle, rpc_services::kSetArmMode);

  set_arm_mode_replier_->set_handler(
      [this](const std_srvs::SetInt32Request& req) {
        std_srvs::SetInt32Reply reply;
        if (set_arm_mode_handler_) {
          std::string message;
          bool success = set_arm_mode_handler_(static_cast<ControlMode>(req.data()), message);
          reply.success(success);
          reply.message(message);
        } else {
          reply.success(false);
          reply.message("No handler registered");
        }
        return reply;
      });

  set_arm_mode_replier_->start();
}

void VRDdsCommunicator::registerSetHeadModeHandler(SetModeHandler handler) {
  if (!initialized_) return;
  set_head_mode_handler_ = handler;

  if (set_head_mode_replier_) return;

  dds_entity_t participant_handle = participant_->get_ddsc_entity();
  set_head_mode_replier_ = std::make_unique<
      dds_rpc::Replier<std_srvs::SetInt32Request, std_srvs::SetInt32Reply>>(
      participant_handle, rpc_services::kSetHeadMode);

  set_head_mode_replier_->set_handler(
      [this](const std_srvs::SetInt32Request& req) {
        std_srvs::SetInt32Reply reply;
        if (set_head_mode_handler_) {
          std::string message;
          bool success = set_head_mode_handler_(static_cast<ControlMode>(req.data()), message);
          reply.success(success);
          reply.message(message);
        } else {
          reply.success(false);
          reply.message("No handler registered");
        }
        return reply;
      });

  set_head_mode_replier_->start();
}

void VRDdsCommunicator::registerSetWaistModeHandler(SetModeHandler handler) {
  if (!initialized_) return;
  set_waist_mode_handler_ = handler;

  if (set_waist_mode_replier_) return;

  dds_entity_t participant_handle = participant_->get_ddsc_entity();
  set_waist_mode_replier_ = std::make_unique<
      dds_rpc::Replier<std_srvs::SetInt32Request, std_srvs::SetInt32Reply>>(
      participant_handle, rpc_services::kSetWaistMode);

  set_waist_mode_replier_->set_handler(
      [this](const std_srvs::SetInt32Request& req) {
        std_srvs::SetInt32Reply reply;
        if (set_waist_mode_handler_) {
          std::string message;
          bool success = set_waist_mode_handler_(static_cast<ControlMode>(req.data()), message);
          reply.success(success);
          reply.message(message);
        } else {
          reply.success(false);
          reply.message("No handler registered");
        }
        return reply;
      });

  set_waist_mode_replier_->start();
}

void VRDdsCommunicator::registerGetRuntimeStateHandler(GetRuntimeStateHandler handler) {
  if (!initialized_) return;
  get_runtime_state_handler_ = handler;
  if (get_runtime_state_replier_) return;

  dds_entity_t participant_handle = participant_->get_ddsc_entity();
  get_runtime_state_replier_ = std::make_unique<
      dds_rpc::Replier<leju::srv::GetRuntimeStateRequest,
                       leju::srv::GetRuntimeStateReply>>(
      participant_handle, rpc_services::kGetRuntimeState);

  get_runtime_state_replier_->set_handler(
      [this](const leju::srv::GetRuntimeStateRequest& /* req */) {
        leju::srv::GetRuntimeStateReply reply;
        if (!get_runtime_state_handler_) {
          return reply;
        }
        RuntimeState state = get_runtime_state_handler_();
        reply.data_ready(state.data_ready);
        reply.hardware_state(static_cast<int>(state.hardware_state));
        reply.lifecycle_state(static_cast<int>(state.lifecycle_state));
        return reply;
      });

  get_runtime_state_replier_->start();
}

void VRDdsCommunicator::registerGetControllerStateHandler(GetControllerStateHandler handler) {
  if (!initialized_) return;
  get_controller_state_handler_ = handler;
  if (get_controller_state_replier_) return;

  dds_entity_t participant_handle = participant_->get_ddsc_entity();
  get_controller_state_replier_ = std::make_unique<
      dds_rpc::Replier<leju::srv::GetControllerStateRequest,
                       leju::srv::GetControllerStateReply>>(
      participant_handle, rpc_services::kGetControllerState);

  get_controller_state_replier_->set_handler(
      [this](const leju::srv::GetControllerStateRequest& /* req */) {
        leju::srv::GetControllerStateReply reply;
        if (!get_controller_state_handler_) {
          return reply;
        }
        ControllerState state = get_controller_state_handler_();
        reply.current_controller(state.current_controller);
        reply.arm_mode(static_cast<int>(state.arm_mode));
        reply.head_mode(static_cast<int>(state.head_mode));
        reply.waist_mode(static_cast<int>(state.waist_mode));
        reply.controller_transitioning(state.controller_transitioning);
        reply.available_controllers(state.available_controllers);
        return reply;
      });

  get_controller_state_replier_->start();
}

void VRDdsCommunicator::registerGetMotionStateHandler(GetMotionStateHandler handler) {
  if (!initialized_) return;
  get_motion_state_handler_ = handler;
  if (get_motion_state_replier_) return;

  dds_entity_t participant_handle = participant_->get_ddsc_entity();
  get_motion_state_replier_ = std::make_unique<
      dds_rpc::Replier<leju::srv::GetMotionStateRequest,
                       leju::srv::GetMotionStateReply>>(
      participant_handle, rpc_services::kGetMotionState);

  get_motion_state_replier_->set_handler(
      [this](const leju::srv::GetMotionStateRequest& /* req */) {
        leju::srv::GetMotionStateReply reply;
        if (!get_motion_state_handler_) {
          return reply;
        }
        MotionState state = get_motion_state_handler_();
        reply.supported(state.supported);
        reply.motion_playing(state.motion_playing);
        reply.current_motion_name(state.current_motion_name);
        reply.available_motion_names(state.available_motion_names);
        return reply;
      });

  get_motion_state_replier_->start();
}

void VRDdsCommunicator::registerStartRuntimeHandler(TriggerHandler handler) {
  if (!initialized_) return;
  start_runtime_handler_ = handler;
  if (start_runtime_replier_) return;

  dds_entity_t participant_handle = participant_->get_ddsc_entity();
  start_runtime_replier_ = std::make_unique<
      dds_rpc::Replier<std_srvs::TriggerRequest, std_srvs::TriggerReply>>(
      participant_handle, rpc_services::kStartRuntime);
  start_runtime_replier_->set_handler(
      [this](const std_srvs::TriggerRequest& /* req */) {
        std_srvs::TriggerReply reply;
        if (!start_runtime_handler_) {
          reply.success(false);
          reply.message("No handler registered");
          return reply;
        }
        std::string message;
        bool success = start_runtime_handler_(message);
        reply.success(success);
        reply.message(message);
        return reply;
      });
  start_runtime_replier_->start();
}

void VRDdsCommunicator::registerStopRuntimeHandler(TriggerHandler handler) {
  if (!initialized_) return;
  stop_runtime_handler_ = handler;
  if (stop_runtime_replier_) return;

  dds_entity_t participant_handle = participant_->get_ddsc_entity();
  stop_runtime_replier_ = std::make_unique<
      dds_rpc::Replier<std_srvs::TriggerRequest, std_srvs::TriggerReply>>(
      participant_handle, rpc_services::kStopRuntime);
  stop_runtime_replier_->set_handler(
      [this](const std_srvs::TriggerRequest& /* req */) {
        std_srvs::TriggerReply reply;
        if (!stop_runtime_handler_) {
          reply.success(false);
          reply.message("No handler registered");
          return reply;
        }
        std::string message;
        bool success = stop_runtime_handler_(message);
        reply.success(success);
        reply.message(message);
        return reply;
      });
  stop_runtime_replier_->start();
}

void VRDdsCommunicator::registerStartMotionHandler(SetStringHandler handler) {
  if (!initialized_) return;
  start_motion_handler_ = handler;
  if (start_motion_replier_) return;

  dds_entity_t participant_handle = participant_->get_ddsc_entity();
  start_motion_replier_ = std::make_unique<
      dds_rpc::Replier<std_srvs::SetStringRequest, std_srvs::SetStringReply>>(
      participant_handle, rpc_services::kStartMotion);
  start_motion_replier_->set_handler(
      [this](const std_srvs::SetStringRequest& req) {
        std_srvs::SetStringReply reply;
        if (!start_motion_handler_) {
          reply.success(false);
          reply.message("No handler registered");
          return reply;
        }
        std::string message;
        bool success = start_motion_handler_(req.data(), message);
        reply.success(success);
        reply.message(message);
        return reply;
      });
  start_motion_replier_->start();
}

void VRDdsCommunicator::registerPlayTactHandler(SetStringHandler handler) {
  if (!initialized_) return;
  play_tact_handler_ = handler;
  if (play_tact_replier_) return;

  dds_entity_t participant_handle = participant_->get_ddsc_entity();
  play_tact_replier_ = std::make_unique<
      dds_rpc::Replier<std_srvs::SetStringRequest, std_srvs::SetStringReply>>(
      participant_handle, rpc_services::kPlayTact);
  play_tact_replier_->set_handler(
      [this](const std_srvs::SetStringRequest& req) {
        std_srvs::SetStringReply reply;
        if (!play_tact_handler_) {
          reply.success(false);
          reply.message("No handler registered");
          return reply;
        }
        std::string message;
        bool success = play_tact_handler_(req.data(), message);
        reply.success(success);
        reply.message(message);
        return reply;
      });
  play_tact_replier_->start();
}

}  // namespace vr
}  // namespace leju
