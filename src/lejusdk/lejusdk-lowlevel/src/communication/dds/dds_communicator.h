#ifndef LEJUSDK_COMMUNICATION_DDS_COMMUNICATOR_H_
#define LEJUSDK_COMMUNICATION_DDS_COMMUNICATOR_H_

#include <memory>
#include <atomic>

#include "dds/dds.hpp"
#include "lejusdk-lowlevel/robot_api/robot_base.h"
#include "communication.h"
#include "dds_data_converter.h"
#include "lejusdk-topic-pubsub/topic_publisher.hpp"
#include "lejusdk-topic-pubsub/topic_subscriber.hpp"

#include "lejusdk-dds-idl/ImuData.hpp"
#include "lejusdk-dds-idl/JointState.hpp"
#include "lejusdk-dds-idl/JointCmd.hpp"
#include "lejusdk-dds-idl/Joy.hpp"
#include "lejusdk-dds-idl/StringData.hpp"
#include "lejusdk-dds-idl/Float64Types.hpp"
#include "lejusdk-dds-idl/HandState.hpp"

namespace leju {

/**
 * @brief DDS通信管理器
 *
 * 负责通过DDS进行数据订阅和发布：
 * 1. 订阅关节状态数据并转发回调
 * 2. 订阅IMU数据并转发回调
 * 3. 发布机器人控制指令
 */
class DdsCommunicator : public ICommunication {
public:
  DdsCommunicator();

  ~DdsCommunicator() override;

  bool initialize() override;
  
  void shutdown() override;

  bool subscribeJointState(const RobotStateCallback& callback) override;

  bool subscribeImuData(const ImuDataCallback& callback) override;

  bool subscribeJoyData(const JoyDataCallback& callback) override;

  bool subscribeHardwareState(const HwStateCallback& callback) override;

  bool publishRobotCmd(const RobotCmd& cmd) override;
  bool publishHandCmd(const HandCmd& cmd) override;
  bool publishHandState(const HandState& state) override;

  bool publishStopRobot() override;

  bool publishPostureHeightCmd(double height_m) override;

  bool publishHardwareState(HardwareState hw_state) override;

  bool publishJointState(const RobotState& state) override;
  bool publishMotorState(const RobotState& motor_state) override;
  bool publishMotorCmd(const RobotCmd& motor_cmd) override;
  bool publishImuData(const ImuData& imu_data) override;
  bool publishJoyData(const JoyData& joy_data) override;
  bool subscribeRobotCmd(const RobotCmdCallback& callback) override;
  bool subscribeHandCmd(const HandCmdCallback& callback) override;
  bool subscribeHandState(const HandStateCallback& callback) override;
  bool subscribeStopRobot(const StringDataCallback& callback) override;
  bool subscribeReloadTeleopConfig(const StringDataCallback& callback) override;
  bool publishReloadTeleopConfig(const std::string& config_path) override;
  bool subscribeReloadControllerConfig(const StringDataCallback& callback) override;
  bool publishReloadControllerConfig(const std::string& config_path) override;

private:
  /**
   * @brief 处理接收到的关节状态数据
   * @param data DDS关节状态数据
   */
  void onJointStateDataReceived(const leju::msgs::JointState& data);

  /**
   * @brief 处理接收到的IMU数据
   * @param data DDS IMU数据
   */
  void onImuDataReceived(const leju::msgs::ImuData& data);

  /**
   * @brief 处理接收到的手柄数据
   * @param data DDS手柄数据
   */
  void onJoyDataReceived(const leju::msgs::Joy& data);

  void onHardwareStateReceived(const leju::msgs::StringData& data);

  void onRobotCmdReceived(const leju::msgs::JointCmd& data);
  void onHandCmdReceived(const leju::msgs::Float64Array& data);
  void onHandStateReceived(const leju::msgs::HandState& data);

  void onStopRobotReceived(const leju::msgs::StringData& data);

  void onReloadTeleopConfigReceived(const leju::msgs::StringData& data);
  void onReloadControllerConfigReceived(const leju::msgs::StringData& data);

private:
  dds::domain::DomainParticipant participant_;  // DDS participant
  std::unique_ptr<leju::dds_common::TopicSubscriber<leju::msgs::JointState>>   joint_state_subscriber_;
  std::unique_ptr<leju::dds_common::TopicSubscriber<leju::msgs::ImuData>>      imu_data_subscriber_;
  std::unique_ptr<leju::dds_common::TopicSubscriber<leju::msgs::Joy>>          joy_data_subscriber_;
  std::unique_ptr<leju::dds_common::TopicSubscriber<leju::msgs::StringData>>   hardware_state_subscriber_;
  std::unique_ptr<leju::dds_common::TopicSubscriber<leju::msgs::JointCmd>> joint_cmd_subscriber_;
  std::unique_ptr<leju::dds_common::TopicSubscriber<leju::msgs::Float64Array>> hand_cmd_subscriber_;
  std::unique_ptr<leju::dds_common::TopicSubscriber<leju::msgs::HandState>> hand_state_subscriber_;
  std::unique_ptr<leju::dds_common::TopicSubscriber<leju::msgs::StringData>> stop_robot_subscriber_;
  std::unique_ptr<leju::dds_common::TopicSubscriber<leju::msgs::StringData>> reload_teleop_config_subscriber_;
  std::unique_ptr<leju::dds_common::TopicSubscriber<leju::msgs::StringData>> reload_controller_config_subscriber_;

  leju::dds_common::TopicPublisher<leju::msgs::JointCmd>      joint_cmd_publisher_;
  leju::dds_common::TopicPublisher<leju::msgs::Float64Array>  hand_cmd_publisher_;
  leju::dds_common::TopicPublisher<leju::msgs::HandState>     hand_state_publisher_;
  leju::dds_common::TopicPublisher<leju::msgs::Float64>       posture_height_cmd_publisher_;  ///< /rt/posture_height_cmd
  leju::dds_common::TopicPublisher<leju::msgs::StringData>    stop_robot_publisher_;
  leju::dds_common::TopicPublisher<leju::msgs::StringData>    reload_teleop_config_publisher_;
  leju::dds_common::TopicPublisher<leju::msgs::StringData>    reload_controller_config_publisher_;
  leju::dds_common::TopicPublisher<leju::msgs::StringData>    hw_state_publisher_;

  leju::dds_common::TopicPublisher<leju::msgs::JointState> joint_state_publisher_;
  leju::dds_common::TopicPublisher<leju::msgs::ImuData> imu_data_publisher_;
  leju::dds_common::TopicPublisher<leju::msgs::Joy> joy_publisher_;
  leju::dds_common::TopicPublisher<leju::msgs::JointState> motor_state_publisher_;
  leju::dds_common::TopicPublisher<leju::msgs::JointCmd>   motor_cmd_publisher_;
  //////////////////////////////////////////////
  std::atomic<bool> initialized_;
  std::atomic<bool> hw_ready_;  // 硬件就绪标志
};

} // namespace leju

#endif // LEJUSDK_COMMUNICATION_DDS_COMMUNICATOR_H_
