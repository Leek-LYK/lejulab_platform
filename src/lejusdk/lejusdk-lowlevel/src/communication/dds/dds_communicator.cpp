#include "dds_communicator.h"
#include "sdk_log.h"
#include "dds_data_converter.h"
#include "lejusdk-topic-pubsub/topic_names.h"
#include "lejusdk-utils/time_utils.hpp"

#include <functional>
#include <chrono>
#include <iostream>

namespace leju {

namespace {
// 传感器数据使用 BEST_EFFORT QoS，避免 write 阻塞等待 ACK
dds::pub::qos::DataWriterQos makeBestEffortQos() {
    dds::pub::qos::DataWriterQos qos;
    qos << dds::core::policy::Reliability::BestEffort();
    return qos;
}
} // anonymous namespace

namespace detail {

// Callback类型定义
using DdsJointStateCallback = std::function<void(const std::shared_ptr<leju::msgs::JointState>&)>;
using DdsImuDataCallback = std::function<void(const std::shared_ptr<leju::msgs::ImuData>&)>;

} // namespace detail

///////////////////////////////////////////////////////////////////////////////
// DdsCommunicator Implementation
///////////////////////////////////////////////////////////////////////////////

DdsCommunicator::DdsCommunicator()
  : participant_(org::eclipse::cyclonedds::domain::default_id())
  , joint_state_subscriber_(nullptr)
  , imu_data_subscriber_(nullptr)
  , joy_data_subscriber_(nullptr)
  , hardware_state_subscriber_(nullptr)
  , joint_cmd_publisher_(participant_, leju::dds_topics::kJointCmd)
  , hand_cmd_publisher_(participant_, leju::dds_topics::kHandCmd)
  , hand_state_publisher_(participant_, leju::dds_topics::kHandState, makeBestEffortQos())
  , posture_height_cmd_publisher_(participant_, leju::dds_topics::kPostureHeightCmd)
  , stop_robot_publisher_(participant_, leju::dds_topics::kHardwareStop)
  , reload_teleop_config_publisher_(participant_, leju::dds_topics::kReloadTeleopConfig)
  , reload_controller_config_publisher_(participant_, leju::dds_topics::kReloadControllerConfig)
  , joint_state_publisher_(participant_, leju::dds_topics::kJointState, makeBestEffortQos())
  , imu_data_publisher_(participant_, leju::dds_topics::kImuState, makeBestEffortQos())
  , joy_publisher_(participant_, leju::dds_topics::kJoy)
  , motor_state_publisher_(participant_, leju::dds_topics::kMotorState, makeBestEffortQos())
  , motor_cmd_publisher_(participant_, leju::dds_topics::kMotorCmd,   makeBestEffortQos())
  , hw_state_publisher_(participant_, leju::dds_topics::kHardwareState)
  , joint_cmd_subscriber_(nullptr)
  , hand_cmd_subscriber_(nullptr)
  , hand_state_subscriber_(nullptr)
  , stop_robot_subscriber_(nullptr)
  , reload_teleop_config_subscriber_(nullptr)
  , initialized_(false)
  , hw_ready_(false) {

  if (!joint_cmd_publisher_.is_valid()) {
    throw std::runtime_error("Failed to initialize joint command publisher");
  }
}

bool DdsCommunicator::initialize() {
    // 检查是否已经初始化，支持幂等调用
    if (initialized_.load()) {
        LJSDK_LOG_W("DdsCommunicator already initialized, skipping initialization");
        return true;
    }

    initialized_.store(true);
    LJSDK_LOG_I("DDS communicator initialized successfully");
    return true;
}

DdsCommunicator::~DdsCommunicator() {
    shutdown();
}

void DdsCommunicator::shutdown() {
    // 检查是否已初始化，避免重复清理
    if (!initialized_.load()) {
      return;
    }

    // 先清理所有的 callbacks
    joint_state_callback_.clear();
    imu_data_callback_.clear();
    joy_data_callback_.clear();
    hardware_state_callback_.clear();
    hand_cmd_callback_.clear();
    hand_state_callback_.clear();
    stop_robot_callback_.clear();
    reload_teleop_config_callback_.clear();

    joint_state_subscriber_.reset();
    imu_data_subscriber_.reset();
    joy_data_subscriber_.reset();
    hardware_state_subscriber_.reset();
    joint_cmd_subscriber_.reset();
    hand_cmd_subscriber_.reset();
    hand_state_subscriber_.reset();
    stop_robot_subscriber_.reset();
    reload_teleop_config_subscriber_.reset();

    initialized_.store(false);
    LJSDK_LOG_I("DDS communicator shutdown completed");
}

bool DdsCommunicator::subscribeJointState(const RobotStateCallback& callback) {
  if (!initialized_.load()) {
    LJSDK_LOG_E("DDS communicator not initialized");
    return false;
  }

  try {
    if (!joint_state_subscriber_) {
      joint_state_subscriber_ = std::make_unique<leju::dds_common::TopicSubscriber<leju::msgs::JointState>>(
          participant_, leju::dds_topics::kJointState,
          [this](const leju::msgs::JointState& data) {
              this->onJointStateDataReceived(data);
          });

      if (!joint_state_subscriber_->is_valid()) {
        LJSDK_LOG_E("Failed to create joint state subscriber");
        joint_state_subscriber_.reset();
        return false;
      }
    }

    joint_state_callback_.set(callback);
    LJSDK_LOG_I("Joint state subscription setup completed");
    return true;
  } catch (const dds::core::Exception& e) {
    LJSDK_LOG_FAILURE("Failed to subscribe to joint state: %s", e.what());
    return false;
  }
}

bool DdsCommunicator::subscribeImuData(const ImuDataCallback& callback) {
  if (!initialized_.load()) {
    LJSDK_LOG_E("DDS communicator not initialized");
    return false;
  }

  try {
    if (!imu_data_subscriber_) {
      imu_data_subscriber_ = std::make_unique<leju::dds_common::TopicSubscriber<leju::msgs::ImuData>>(
          participant_, leju::dds_topics::kImuState,
          [this](const leju::msgs::ImuData& data) {
              this->onImuDataReceived(data);
          });

      if (!imu_data_subscriber_->is_valid()) {
        LJSDK_LOG_E("Failed to create IMU data subscriber");
        imu_data_subscriber_.reset();
        return false;
      }
    }

    imu_data_callback_.set(callback);
    LJSDK_LOG_I("IMU data subscription setup completed");
    return true;
  } catch (const dds::core::Exception& e) {
    LJSDK_LOG_FAILURE("Failed to subscribe to IMU data: %s", e.what());
    return false;
  }
}

bool DdsCommunicator::subscribeJoyData(const JoyDataCallback& callback) {
  if (!initialized_.load()) {
    LJSDK_LOG_E("DDS communicator not initialized");
    return false;
  }

  try {
    if (!joy_data_subscriber_) {
      joy_data_subscriber_ = std::make_unique<leju::dds_common::TopicSubscriber<leju::msgs::Joy>>(
          participant_, leju::dds_topics::kJoy,
          [this](const leju::msgs::Joy& data) {
              this->onJoyDataReceived(data);
          });

      if (!joy_data_subscriber_->is_valid()) {
        LJSDK_LOG_E("Failed to create joy data subscriber");
        joy_data_subscriber_.reset();
        return false;
      }
    }

    joy_data_callback_.set(callback);
    LJSDK_LOG_I("Joy data subscription setup completed");
    return true;
  } catch (const dds::core::Exception& e) {
    LJSDK_LOG_FAILURE("Failed to subscribe to joy data: %s", e.what());
    return false;
  }
}

bool DdsCommunicator::subscribeHardwareState(const HwStateCallback& callback) {
  if (!initialized_.load()) {
    LJSDK_LOG_E("DDS communicator not initialized");
    return false;
  }

  try {
    if (!hardware_state_subscriber_) {
      hardware_state_subscriber_ = std::make_unique<leju::dds_common::TopicSubscriber<leju::msgs::StringData>>(
          participant_, leju::dds_topics::kHardwareState,
          [this](const leju::msgs::StringData& data) {
              this->onHardwareStateReceived(data);
          });

      if (!hardware_state_subscriber_->is_valid()) {
        LJSDK_LOG_E("Failed to create hardware state subscriber");
        hardware_state_subscriber_.reset();
        return false;
      }
    }

    hardware_state_callback_.set(callback);
    LJSDK_LOG_I("Hardware state subscription setup completed");
    return true;
  } catch (const dds::core::Exception& e) {
    LJSDK_LOG_FAILURE("Failed to subscribe to hardware state: %s", e.what());
    return false;
  }
}

bool DdsCommunicator::publishRobotCmd(const RobotCmd& cmd) {
  if (!initialized_.load()) {
    LJSDK_LOG_WARNING("DDS communicator not initialized");
    return false;
  }

  try {
    auto dds_cmd = DdsDataConverter<leju::msgs::JointCmd, RobotCmd>::toDds(cmd);
    return joint_cmd_publisher_.publish_async(dds_cmd);
  } catch (const dds::core::Exception& e) {
    LJSDK_LOG_FAILURE("Failed to publish robot command: %s", e.what());
    return false;
  } catch (const std::exception& e) {
    LJSDK_LOG_FAILURE("Unexpected error publishing robot command: %s", e.what());
    return false;
  }
}


bool DdsCommunicator::publishHandCmd(const HandCmd& cmd) {
  if (!initialized_.load()) {
    LJSDK_LOG_WARNING("DDS communicator not initialized");
    return false;
  }

  try {
    auto dds_cmd = DdsDataConverter<leju::msgs::Float64Array, HandCmd>::toDds(cmd);
    return hand_cmd_publisher_.publish(dds_cmd);
  } catch (const dds::core::Exception& e) {
    LJSDK_LOG_FAILURE("Failed to publish hand command: %s", e.what());
    return false;
  }
}

bool DdsCommunicator::publishHandState(const HandState& state) {
  if (!initialized_.load()) {
    LJSDK_LOG_WARNING("DDS communicator not initialized");
    return false;
  }
  if (!state.isValid()) {
    LJSDK_LOG_WARNING("Invalid hand state data");
    return false;
  }

  try {
    auto dds_state = DdsDataConverter<leju::msgs::HandState, HandState>::toDds(state);
    return hand_state_publisher_.publish_async(dds_state);
  } catch (const dds::core::Exception& e) {
    LJSDK_LOG_FAILURE("Failed to publish hand state: %s", e.what());
    return false;
  } catch (const std::exception& e) {
    LJSDK_LOG_FAILURE("Unexpected error publishing hand state: %s", e.what());
    return false;
  }
}

bool DdsCommunicator::publishStopRobot() {
  if (!initialized_.load()) {
    LJSDK_LOG_WARNING("DDS communicator not initialized");
    return false;
  }

  try {
    auto dds_cmd = leju::msgs::StringData();
    return stop_robot_publisher_.publish(dds_cmd);
  } catch (const dds::core::Exception& e) {
    LJSDK_LOG_FAILURE("Failed to publish stop robot: %s", e.what());
    return false;
  }
}

bool DdsCommunicator::publishPostureHeightCmd(double height_m) {
  if (!initialized_.load()) {
    LJSDK_LOG_WARNING("DDS communicator not initialized");
    return false;
  }

  try {
    leju::msgs::Float64 dds_cmd;
    dds_cmd.data(height_m);
    return posture_height_cmd_publisher_.publish(dds_cmd);
  } catch (const dds::core::Exception& e) {
    LJSDK_LOG_FAILURE("Failed to publish posture height command: %s", e.what());
    return false;
  }
}

bool DdsCommunicator::publishHardwareState(HardwareState hw_state) {
  if (!initialized_.load()) {
    LJSDK_LOG_WARNING("DDS communicator not initialized");
    return false;
  }

  try {
    auto state_str = leju::HardwareState2String(hw_state);
    auto dds_cmd = leju::msgs::StringData();
    int32_t sec; uint32_t nsec;
    leju::common::GetUnixTimestampSecNsec(sec, nsec);
    dds_cmd.header_sec(sec);
    dds_cmd.header_nanosec(nsec);
    dds_cmd.data(state_str);
    return hw_state_publisher_.publish(dds_cmd);
  } catch (const dds::core::Exception& e) {
    LJSDK_LOG_FAILURE("Failed to publish hardware state: %s", e.what());
    return false;
  }
}


void DdsCommunicator::onJointStateDataReceived(const leju::msgs::JointState& data) {
  try {
    auto robot_state = DdsDataConverter<leju::msgs::JointState, RobotState>::fromDds(data);
    auto robot_state_ptr = std::make_shared<RobotState>(std::move(robot_state));

    if (!joint_state_callback_.call_if(robot_state_ptr)) {
      // No callback set, just return silently
      return;
    }

  } catch (const std::exception& e) {
    LJSDK_LOG_E("Error processing joint state data: %s", e.what());
  }
}

void DdsCommunicator::onImuDataReceived(const leju::msgs::ImuData& data) {
    try {
      auto imu_data_converted = DdsDataConverter<leju::msgs::ImuData, ImuData>::fromDds(data);
      auto imu_data_ptr = std::make_shared<ImuData>(std::move(imu_data_converted));

      if (!imu_data_callback_.call_if(imu_data_ptr)) {
        // No callback set, just return silently
        return;
      }

    } catch (const std::exception& e) {
      LJSDK_LOG_E("Error processing IMU data: %s", e.what());
    }
}

void DdsCommunicator::onJoyDataReceived(const leju::msgs::Joy& data) {
    try {
      auto joy_data_converted = DdsDataConverter<leju::msgs::Joy, JoyData>::fromDds(data);

      if (!joy_data_converted.has_value()) {
        LJSDK_LOG_WARNING("Invalid joy data: axes or buttons size mismatch");
        return;
      }

      auto joy_data_ptr = std::make_shared<JoyData>(std::move(joy_data_converted.value()));

      if (!joy_data_callback_.call_if(joy_data_ptr)) {
        // No callback set, just return silently
        return;
      }

    } catch (const std::exception& e) {
      LJSDK_LOG_E("Error processing joy data: %s", e.what());
    }
}

bool DdsCommunicator::publishJointState(const RobotState& state) {
  if (!initialized_.load()) {
    LJSDK_LOG_WARNING("DDS communicator not initialized");
    return false;
  }

  try {
    using Clock = std::chrono::steady_clock;
    auto dds_state = DdsDataConverter<leju::msgs::JointState, RobotState>::toDds(state);
    auto t1 = Clock::now();
    bool ret = joint_state_publisher_.publish_async(dds_state);
    auto t2 = Clock::now();
    auto write_us = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
    if (write_us > 200) {
      constexpr int print_interval = 2;  // 每 print_interval 秒打印一次
      static auto last_print_time = Clock::now() - std::chrono::seconds(print_interval);
      auto now = Clock::now();
      if (std::chrono::duration_cast<std::chrono::seconds>(now - last_print_time).count() >= print_interval) {
        std::cout << "\033[33m[DDS PubJoint] write=" << write_us << "us\033[0m" << std::endl;
        last_print_time = now;
      }
    }
    return ret;
  } catch (const dds::core::Exception& e) {
    LJSDK_LOG_FAILURE("Failed to publish robot state: %s", e.what());
    return false;
  } catch (const std::exception& e) {
    LJSDK_LOG_FAILURE("Unexpected error publishing robot state: %s", e.what());
    return false;
  }
}

bool DdsCommunicator::publishMotorState(const RobotState& motor_state) {
  if (!initialized_.load()) {
    LJSDK_LOG_WARNING("DDS communicator not initialized");
    return false;
  }

  try {
    auto dds_state = DdsDataConverter<leju::msgs::JointState, RobotState>::toDds(motor_state);
    return motor_state_publisher_.publish(dds_state);
  } catch (const dds::core::Exception& e) {
    LJSDK_LOG_FAILURE("Failed to publish motor state: %s", e.what());
    return false;
  }
}

bool DdsCommunicator::publishMotorCmd(const RobotCmd& motor_cmd) {
  if (!initialized_.load()) {
    LJSDK_LOG_WARNING("DDS communicator not initialized");
    return false;
  }

  try {
    auto dds_cmd = DdsDataConverter<leju::msgs::JointCmd, RobotCmd>::toDds(motor_cmd);
    return motor_cmd_publisher_.publish(dds_cmd);
  } catch (const dds::core::Exception& e) {
    LJSDK_LOG_FAILURE("Failed to publish motor cmd: %s", e.what());
    return false;
  }
}

bool DdsCommunicator::publishImuData(const ImuData& imu_data) {
  if (!initialized_.load()) {
    LJSDK_LOG_WARNING("DDS communicator not initialized");
    return false;
  }

  try {
    using Clock = std::chrono::steady_clock;
    auto dds_imu_data = DdsDataConverter<leju::msgs::ImuData, ImuData>::toDds(imu_data);
    auto t1 = Clock::now();
    bool ret = imu_data_publisher_.publish_async(dds_imu_data);
    auto t2 = Clock::now();
    auto write_us = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
    if (write_us > 200) {
      constexpr int print_interval = 2;  // 每 print_interval 秒打印一次
      static auto last_print_time = Clock::now() - std::chrono::seconds(print_interval);
      auto now = Clock::now();
      if (std::chrono::duration_cast<std::chrono::seconds>(now - last_print_time).count() >= print_interval) {
        std::cout << "\033[33m[DDS PubImu] write=" << write_us << "us\033[0m" << std::endl;
        last_print_time = now;
      }
    }
    return ret;
  } catch (const dds::core::Exception& e) {
    LJSDK_LOG_FAILURE("Failed to publish imu data: %s", e.what());
    return false;
  } catch (const std::exception& e) {
    LJSDK_LOG_FAILURE("Unexpected error publishing imu data: %s", e.what());
    return false;
  }
}

bool DdsCommunicator::publishJoyData(const JoyData& joy_data) {
  if (!initialized_.load()) {
    LJSDK_LOG_WARNING("DDS communicator not initialized");
    return false;
  }

  try {
    auto dds_joy_data = DdsDataConverter<leju::msgs::Joy, JoyData>::toDds(joy_data);
    return joy_publisher_.publish(dds_joy_data);
  } catch (const dds::core::Exception& e) {
    LJSDK_LOG_FAILURE("Failed to publish joy data: %s", e.what());
    return false;
  }
}

bool DdsCommunicator::subscribeRobotCmd(const RobotCmdCallback& callback) {
  if (!initialized_.load()) {
    LJSDK_LOG_E("DDS communicator not initialized");
    return false;
  }

  try {
    if (!joint_cmd_subscriber_) {
      joint_cmd_subscriber_ = std::make_unique<leju::dds_common::TopicSubscriber<leju::msgs::JointCmd>>(
          participant_, leju::dds_topics::kJointCmd,
          [this](const leju::msgs::JointCmd& data) {
              this->onRobotCmdReceived(data);
          });
      
      if (!joint_cmd_subscriber_->is_valid()) {
        LJSDK_LOG_E("Failed to create joint cmd subscriber");
        joint_cmd_subscriber_.reset();
        return false;
      }
    }

    robot_cmd_callback_.set(callback);
    LJSDK_LOG_I("Joint cmd subscription setup completed");
    return true;
  } catch (const dds::core::Exception& e) {
    LJSDK_LOG_FAILURE("Failed to subscribe to robot cmd: %s", e.what());
    return false;
  }
}

void DdsCommunicator::onRobotCmdReceived(const leju::msgs::JointCmd& data) {
  

  try {
    auto robot_cmd = DdsDataConverter<leju::msgs::JointCmd, RobotCmd>::fromDds(data);
    auto robot_cmd_ptr = std::make_shared<RobotCmd>(std::move(robot_cmd));
   
    if (!robot_cmd_callback_.call_if(robot_cmd_ptr)) {
      // No callback set, just return silently
      return;
    }

  } catch (const std::exception& e) {
    LJSDK_LOG_E("Error processing robot cmd data: %s", e.what());
  }
}


bool DdsCommunicator::subscribeHandCmd(const HandCmdCallback& callback) {
  if (!initialized_.load()) {
    LJSDK_LOG_E("DDS communicator not initialized");
    return false;
  }

  try {
    if (!hand_cmd_subscriber_) {
      hand_cmd_subscriber_ = std::make_unique<leju::dds_common::TopicSubscriber<leju::msgs::Float64Array>>(
          participant_, leju::dds_topics::kHandCmd,
          [this](const leju::msgs::Float64Array& data) {
              this->onHandCmdReceived(data);
          });

      if (!hand_cmd_subscriber_->is_valid()) {
        LJSDK_LOG_E("Failed to create hand cmd subscriber");
        hand_cmd_subscriber_.reset();
        return false;
      }
    }

    hand_cmd_callback_.set(callback);
    LJSDK_LOG_I("Hand cmd subscription setup completed");
    return true;
  } catch (const dds::core::Exception& e) {
    LJSDK_LOG_FAILURE("Failed to subscribe to hand cmd: %s", e.what());
    return false;
  }
}

bool DdsCommunicator::subscribeHandState(const HandStateCallback& callback) {
  if (!initialized_.load()) {
    LJSDK_LOG_E("DDS communicator not initialized");
    return false;
  }

  try {
    if (!hand_state_subscriber_) {
      hand_state_subscriber_ =
          std::make_unique<leju::dds_common::TopicSubscriber<leju::msgs::HandState>>(
              participant_, leju::dds_topics::kHandState,
              [this](const leju::msgs::HandState& data) {
                this->onHandStateReceived(data);
              });

      if (!hand_state_subscriber_->is_valid()) {
        LJSDK_LOG_E("Failed to create hand state subscriber");
        hand_state_subscriber_.reset();
        return false;
      }
    }

    hand_state_callback_.set(callback);
    LJSDK_LOG_I("Hand state subscription setup completed");
    return true;
  } catch (const dds::core::Exception& e) {
    LJSDK_LOG_FAILURE("Failed to subscribe to hand state: %s", e.what());
    return false;
  }
}

void DdsCommunicator::onHandCmdReceived(const leju::msgs::Float64Array& data) {
  try {
    auto cmd = DdsDataConverter<leju::msgs::Float64Array, HandCmd>::fromDds(data);
    auto cmd_ptr = std::make_shared<HandCmd>(std::move(cmd));
    if (!hand_cmd_callback_.call_if(cmd_ptr)) {
      LJSDK_LOG_W("No hand cmd callback registered");
    }
  } catch (const std::exception& e) {
    LJSDK_LOG_E("Error processing hand cmd: %s", e.what());
  }
}

void DdsCommunicator::onHandStateReceived(const leju::msgs::HandState& data) {
  try {
    auto state = DdsDataConverter<leju::msgs::HandState, HandState>::fromDds(data);
    if (!state.isValid()) {
      LJSDK_LOG_W("Invalid hand state received: expected 12 channels");
      return;
    }
    auto state_ptr = std::make_shared<HandState>(std::move(state));
    hand_state_callback_.call_if(state_ptr);
  } catch (const std::exception& e) {
    LJSDK_LOG_E("Error processing hand state: %s", e.what());
  }
}

bool DdsCommunicator::subscribeStopRobot(const StringDataCallback& callback) {
  if (!initialized_.load()) {
    LJSDK_LOG_E("DDS communicator not initialized");
    return false;
  }

  try {
    if (!stop_robot_subscriber_) {
      stop_robot_subscriber_ = std::make_unique<leju::dds_common::TopicSubscriber<leju::msgs::StringData>>(
          participant_, leju::dds_topics::kHardwareStop,
          [this](const leju::msgs::StringData& data) {
              this->onStopRobotReceived(data);
          });

      if (!stop_robot_subscriber_->is_valid()) {
        LJSDK_LOG_E("Failed to create stop robot subscriber");
        stop_robot_subscriber_.reset();
        return false;
      }
    }

    stop_robot_callback_.set(callback);
    LJSDK_LOG_I("Stop robot subscription setup completed");
    return true;
  } catch (const dds::core::Exception& e) {
    LJSDK_LOG_FAILURE("Failed to subscribe to stop robot: %s", e.what());
    return false;
  }
}

void DdsCommunicator::onStopRobotReceived(const leju::msgs::StringData& data) {
    try {
        auto stop_data = DdsDataConverter<leju::msgs::StringData, StringData>::fromDds(data);
        auto stop_data_ptr = std::make_shared<StringData>(std::move(stop_data));

        if (!stop_robot_callback_.call_if(stop_data_ptr)) {
            return;
        }
    } catch (const std::exception& e) {
        LJSDK_LOG_E("Error processing stop robot data: %s", e.what());
    }
}

bool DdsCommunicator::subscribeReloadTeleopConfig(const StringDataCallback& callback) {
  if (!initialized_.load()) {
    LJSDK_LOG_E("DDS communicator not initialized");
    return false;
  }

  try {
    if (!reload_teleop_config_subscriber_) {
      reload_teleop_config_subscriber_ = std::make_unique<leju::dds_common::TopicSubscriber<leju::msgs::StringData>>(
          participant_, leju::dds_topics::kReloadTeleopConfig,
          [this](const leju::msgs::StringData& data) {
              this->onReloadTeleopConfigReceived(data);
          });

      if (!reload_teleop_config_subscriber_->is_valid()) {
        LJSDK_LOG_E("Failed to create reload teleop config subscriber");
        reload_teleop_config_subscriber_.reset();
        return false;
      }
    }

    reload_teleop_config_callback_.set(callback);
    LJSDK_LOG_I("Reload teleop config subscription setup completed");
    return true;
  } catch (const dds::core::Exception& e) {
    LJSDK_LOG_FAILURE("Failed to subscribe to reload teleop config: %s", e.what());
    return false;
  }
}

bool DdsCommunicator::publishReloadTeleopConfig(const std::string& config_path) {
  if (!initialized_.load()) {
    LJSDK_LOG_WARNING("DDS communicator not initialized");
    return false;
  }

  try {
    auto dds_cmd = leju::msgs::StringData();
    int32_t sec; uint32_t nsec;
    leju::common::GetUnixTimestampSecNsec(sec, nsec);
    dds_cmd.header_sec(sec);
    dds_cmd.header_nanosec(nsec);
    dds_cmd.data(config_path);
    return reload_teleop_config_publisher_.publish(dds_cmd);
  } catch (const dds::core::Exception& e) {
    LJSDK_LOG_FAILURE("Failed to publish reload teleop config: %s", e.what());
    return false;
  }
}

void DdsCommunicator::onReloadTeleopConfigReceived(const leju::msgs::StringData& data) {
    try {
        auto reload_data = DdsDataConverter<leju::msgs::StringData, StringData>::fromDds(data);
        auto reload_data_ptr = std::make_shared<StringData>(std::move(reload_data));

        if (!reload_teleop_config_callback_.call_if(reload_data_ptr)) {
            return;
        }
    } catch (const std::exception& e) {
        LJSDK_LOG_E("Error processing reload teleop config data: %s", e.what());
    }
}

void DdsCommunicator::onReloadControllerConfigReceived(const leju::msgs::StringData& data) {
    try {
        auto reload_data = DdsDataConverter<leju::msgs::StringData, StringData>::fromDds(data);
        auto reload_data_ptr = std::make_shared<StringData>(std::move(reload_data));

        if (!reload_controller_config_callback_.call_if(reload_data_ptr)) {
            return;
        }
    } catch (const std::exception& e) {
        LJSDK_LOG_E("Error processing reload controller config data: %s", e.what());
    }
}

bool DdsCommunicator::subscribeReloadControllerConfig(const StringDataCallback& callback) {
  if (!initialized_.load()) {
    LJSDK_LOG_E("DDS communicator not initialized");
    return false;
  }

  try {
    if (!reload_controller_config_subscriber_) {
      reload_controller_config_subscriber_ = std::make_unique<leju::dds_common::TopicSubscriber<leju::msgs::StringData>>(
          participant_, leju::dds_topics::kReloadControllerConfig,
          [this](const leju::msgs::StringData& data) {
              this->onReloadControllerConfigReceived(data);
          });

      if (!reload_controller_config_subscriber_->is_valid()) {
        LJSDK_LOG_E("Failed to create reload controller config subscriber");
        reload_controller_config_subscriber_.reset();
        return false;
      }
    }

    reload_controller_config_callback_.set(callback);
    LJSDK_LOG_I("Reload controller config subscription setup completed");
    return true;
  } catch (const dds::core::Exception& e) {
    LJSDK_LOG_FAILURE("Failed to subscribe to reload controller config: %s", e.what());
    return false;
  }
}

bool DdsCommunicator::publishReloadControllerConfig(const std::string& config_path) {
  if (!initialized_.load()) {
    LJSDK_LOG_WARNING("DDS communicator not initialized");
    return false;
  }

  try {
    auto dds_cmd = leju::msgs::StringData();
    int32_t sec; uint32_t nsec;
    leju::common::GetUnixTimestampSecNsec(sec, nsec);
    dds_cmd.header_sec(sec);
    dds_cmd.header_nanosec(nsec);
    dds_cmd.data(config_path);
    return reload_controller_config_publisher_.publish(dds_cmd);
  } catch (const dds::core::Exception& e) {
    LJSDK_LOG_FAILURE("Failed to publish reload controller config: %s", e.what());
    return false;
  }
}

void DdsCommunicator::onHardwareStateReceived(const leju::msgs::StringData& data) {
    try {
        // Convert DDS StringData to our StringData format using DdsDataConverter
        auto hw_state = DdsDataConverter<leju::msgs::StringData, StringData>::fromDds(data);
        auto hw_state_ptr = std::make_shared<StringData>(std::move(hw_state));

        if (!hardware_state_callback_.call_if(hw_state_ptr)) {
            // No callback set, just return silently
            return;
        }

    } catch (const std::exception& e) {
        LJSDK_LOG_E("Error processing hardware state data: %s", e.what());
    }
}

} // namespace leju
