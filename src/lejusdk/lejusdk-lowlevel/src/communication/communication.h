#ifndef LEJUSDK_COMMUNICATION_COMMUNICATION_H_
#define LEJUSDK_COMMUNICATION_COMMUNICATION_H_

#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include "lejusdk-lowlevel/data_types.h"
#include "lejusdk-lowlevel/robot_api/robot_base.h"
#include "internal/common/callback_wrapper.hpp"

namespace leju {

/**
 * @brief Communication interface
 *
 * Defines the common interface for robot communication, supporting data subscription
 * and command publishing functionality.
 * Specific communication implementations (like DDS, TCP, UDP, etc.) should implement
 * this interface.
 */
class ICommunication {
public:
  /**
   * @brief Virtual destructor
   */
  virtual ~ICommunication() = default;

  /**
   * @brief Initialize communication module
   *
   * Establishes communication connection and sets up necessary resources.
   * This function should be idempotent.
   *
   * @return Whether initialization was successful
   */
  virtual bool initialize() = 0;

  /**
   * @brief Shutdown communication module
   *
   * Clean up communication resources and disconnect.
   */
  virtual void shutdown() = 0;

  /**
   * @brief Subscribe to joint state data
   *
   * Set up callback function to receive robot joint state data.
   *
   * @param callback Callback function for joint state data
   * @return Whether subscription was successful
   */
  virtual bool subscribeJointState(const RobotStateCallback& callback) = 0;

  /**
   * @brief Subscribe to IMU data
   *
   * Set up callback function to receive robot IMU sensor data.
   *
   * @param callback Callback function for IMU data
   * @return Whether subscription was successful
   */
  virtual bool subscribeImuData(const ImuDataCallback& callback) = 0;

  /**
   * @brief Subscribe to joystick data
   *
   * Set up callback function to receive joystick controller data.
   *
   * @param callback Callback function for joystick data
   * @return Whether subscription was successful
   */
  virtual bool subscribeJoyData(const JoyDataCallback& callback) = 0;

  /**
   * @brief Publish robot control command
   *
   * Send control command to the robot.
   *
   * @param cmd Robot control command
   * @return Whether publishing was successful
   */
  virtual bool publishRobotCmd(const RobotCmd& cmd) = 0;
  virtual bool publishHandCmd(const HandCmd& cmd) = 0;
  virtual bool publishHandState(const HandState& state) = 0;

  /**
   * @brief Publish stop robot command
   *
   * Send stop robot command to the robot.
   *
   * @return Whether publishing was successful
   */
  virtual bool publishStopRobot() = 0;

  /**
   * @brief Publish posture height command
   *
   * Send target standing height to the posture controller.
   * The controller will set cmd_stance=1 and adjust body height accordingly.
   *
   * @param height_m Target standing height in meters (e.g. 0.77 for full stand, 0.56 for deep squat)
   * @return Whether publishing was successful
   */
  virtual bool publishPostureHeightCmd(double height_m) = 0;

  /**
   * @brief Publish hardware state
   * 
   * @param hw_state 
   * @return true 
   * @return false 
   */
  virtual bool publishHardwareState(HardwareState hw_state) = 0;

  virtual bool publishJointState(const RobotState& state) = 0;
  virtual bool publishImuData(const ImuData& imu_data) = 0;
  virtual bool publishMotorState(const RobotState& motor_state) = 0;
  virtual bool publishMotorCmd(const RobotCmd& motor_cmd) = 0;
  virtual bool publishJoyData(const JoyData& joy_data) = 0;
  virtual bool subscribeHardwareState(const HwStateCallback& callback) = 0;
  virtual bool subscribeRobotCmd(const RobotCmdCallback& callback) = 0;
  virtual bool subscribeHandCmd(const HandCmdCallback& callback) = 0;
  virtual bool subscribeHandState(const HandStateCallback& callback) = 0;
  virtual bool subscribeStopRobot(const StringDataCallback& callback) = 0;

  /// @brief 订阅遥控器绑定配置热重载信号
  virtual bool subscribeReloadTeleopConfig(const StringDataCallback& callback) = 0;

  /// @brief 发布遥控器绑定配置热重载信号 (config_path 可选, 空表示重读已加载路径)
  virtual bool publishReloadTeleopConfig(const std::string& config_path) = 0;

  /// @brief 订阅控制器列表配置热重载信号
  virtual bool subscribeReloadControllerConfig(const StringDataCallback& callback) = 0;

  /// @brief 发布控制器列表配置热重载信号 (config_path 可选, 空表示重读已加载路径)
  virtual bool publishReloadControllerConfig(const std::string& config_path) = 0;

protected:
  // Use CallbackWrapper from internal::common namespace
  internal::CallbackWrapper<RobotStateCallback> joint_state_callback_;
  internal::CallbackWrapper<ImuDataCallback> imu_data_callback_;
  internal::CallbackWrapper<JoyDataCallback> joy_data_callback_;
  internal::CallbackWrapper<HwStateCallback> hardware_state_callback_;
  internal::CallbackWrapper<RobotCmdCallback> robot_cmd_callback_;
  internal::CallbackWrapper<HandCmdCallback> hand_cmd_callback_;
  internal::CallbackWrapper<HandStateCallback> hand_state_callback_;
  internal::CallbackWrapper<StringDataCallback> stop_robot_callback_;
  internal::CallbackWrapper<StringDataCallback> reload_teleop_config_callback_;
  internal::CallbackWrapper<StringDataCallback> reload_controller_config_callback_;
};

/**
 * @brief Communication singleton
 *
 * Provides a global communication instance that can be used across the system.
 * Manages the lifecycle of the communication module.
 */
class Communication {
public:
  /**
   * @brief Get the communication singleton instance
   *
   * @return ICommunication& Reference to the communication interface
   */
  static ICommunication& getInstance();

private:
  Communication() = default;
  ~Communication() = default;

  // Disable copy and assignment
  Communication(const Communication&) = delete;
  Communication& operator=(const Communication&) = delete;
};

} // namespace leju

#endif // LEJUSDK_COMMUNICATION_COMMUNICATION_H_
