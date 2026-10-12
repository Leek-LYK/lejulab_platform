/**
 * @file vr_msg_converter.hpp
 * @brief SDK 类型与 DDS 消息类型转换
 */

#ifndef LEJUSDK_VR_COMMUNICATION_VR_MSG_CONVERTER_HPP_
#define LEJUSDK_VR_COMMUNICATION_VR_MSG_CONVERTER_HPP_

#include <chrono>
#include <cmath>
#include <utility>

#include <lejusdk-dds-idl/JointTrajectory.hpp>
#include <lejusdk-dds-idl/QuestBonePoses.hpp>
#include <lejusdk-dds-idl/QuestJoysticks.hpp>
#include <lejusdk-dds-idl/VelocityCmd.hpp>

#include "lejusdk-vr/data_types.h"

namespace leju {
namespace vr {

namespace detail {

inline double ddsTimestampToDouble(int32_t sec, uint32_t nsec) noexcept {
  return static_cast<double>(sec) + static_cast<double>(nsec) * 1e-9;
}

inline std::pair<int32_t, uint32_t> doubleToDdsTimestamp(
    double timestamp_sec) noexcept {
  int32_t sec = static_cast<int32_t>(std::floor(timestamp_sec));
  double fractional = timestamp_sec - static_cast<double>(sec);
  uint32_t nsec = static_cast<uint32_t>(fractional * 1e9 + 0.5);

  if (nsec >= 1000000000U) {
    nsec = 999999999U;
  }

  return {sec, nsec};
}

}  // namespace detail

/// @brief SDK JointTrajectoryPoint -> DDS 消息
inline leju::msgs::JointTrajectoryPoint toMsg(const JointTrajectoryPoint& cmd) {
  leju::msgs::JointTrajectoryPoint msg;
  auto now = std::chrono::system_clock::now();
  auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                now.time_since_epoch())
                .count();
  msg.header_sec(static_cast<int32_t>(ns / 1000000000));
  msg.header_nanosec(static_cast<uint32_t>(ns % 1000000000));
  msg.q(cmd.q);
  msg.v(cmd.v);
  msg.acc(cmd.acc);
  return msg;
}

/// @brief SDK VelocityCmd -> DDS 消息
inline leju::msgs::VelocityCmd toMsg(const VelocityCmd& cmd) {
  leju::msgs::VelocityCmd msg;
  auto [sec, nsec] = detail::doubleToDdsTimestamp(cmd.timestamp);
  msg.linear_x(cmd.linear_x);
  msg.linear_y(cmd.linear_y);
  msg.angular_z(cmd.angular_z);
  msg.header_sec(sec);
  msg.header_nanosec(nsec);
  return msg;
}

/// @brief SDK VrVelocityCmd -> DDS 消息
inline leju::msgs::VelocityCmd toMsg(const VrVelocityCmd& cmd) {
  leju::msgs::VelocityCmd msg;
  auto [sec, nsec] = detail::doubleToDdsTimestamp(cmd.timestamp);
  msg.linear_x(cmd.linear_x);
  msg.linear_y(cmd.linear_y);
  msg.angular_z(cmd.angular_z);
  msg.header_sec(sec);
  msg.header_nanosec(nsec);
  return msg;
}

/// @brief SDK Pose -> DDS 消息
inline leju::msgs::Pose toMsg(const Pose& pose) {
  leju::msgs::Pose msg;
  msg.x(pose.x);
  msg.y(pose.y);
  msg.z(pose.z);
  msg.qx(pose.qx);
  msg.qy(pose.qy);
  msg.qz(pose.qz);
  msg.qw(pose.qw);
  return msg;
}

/// @brief SDK QuestBonePosesData -> DDS 消息
inline leju::msgs::QuestBonePoses toMsg(const QuestBonePosesData& data) {
  leju::msgs::QuestBonePoses msg;
  std::vector<leju::msgs::Pose> poses;
  poses.reserve(data.poses.size());
  for (const auto& pose : data.poses) {
    poses.push_back(toMsg(pose));
  }
  msg.header_sec(data.header_sec);
  msg.header_nanosec(data.header_nanosec);
  msg.timestamp_ms(data.timestamp_ms);
  msg.is_high_confidence(data.is_high_confidence);
  msg.is_hand_tracking(data.is_hand_tracking);
  msg.poses(std::move(poses));
  return msg;
}

/// @brief SDK QuestJoystickData -> DDS 消息
inline leju::msgs::QuestJoysticks toMsg(const QuestJoystickData& data) {
  leju::msgs::QuestJoysticks msg;
  msg.header_sec(data.header_sec);
  msg.header_nanosec(data.header_nanosec);
  msg.left_x(data.left_x);
  msg.left_y(data.left_y);
  msg.left_trigger(data.left_trigger);
  msg.left_grip(data.left_grip);
  msg.left_first_button_pressed(data.left_first_button_pressed);
  msg.left_second_button_pressed(data.left_second_button_pressed);
  msg.left_first_button_touched(data.left_first_button_touched);
  msg.left_second_button_touched(data.left_second_button_touched);
  msg.right_x(data.right_x);
  msg.right_y(data.right_y);
  msg.right_trigger(data.right_trigger);
  msg.right_grip(data.right_grip);
  msg.right_first_button_pressed(data.right_first_button_pressed);
  msg.right_second_button_pressed(data.right_second_button_pressed);
  msg.right_first_button_touched(data.right_first_button_touched);
  msg.right_second_button_touched(data.right_second_button_touched);
  return msg;
}

/// @brief DDS 消息 -> SDK JointTrajectoryPoint
inline JointTrajectoryPoint fromMsg(const leju::msgs::JointTrajectoryPoint& msg) {
  JointTrajectoryPoint cmd;
  cmd.q = msg.q();
  cmd.v = msg.v();
  cmd.acc = msg.acc();
  return cmd;
}

/// @brief DDS 消息 -> SDK VelocityCmd
inline VelocityCmd fromMsg(const leju::msgs::VelocityCmd& msg) {
  VelocityCmd cmd;
  cmd.linear_x = msg.linear_x();
  cmd.linear_y = msg.linear_y();
  cmd.angular_z = msg.angular_z();
  cmd.timestamp = detail::ddsTimestampToDouble(msg.header_sec(), msg.header_nanosec());
  return cmd;
}

/// @brief DDS 消息 -> SDK VrVelocityCmd
inline VrVelocityCmd fromVrMsg(const leju::msgs::VelocityCmd& msg) {
  VrVelocityCmd cmd;
  cmd.linear_x = msg.linear_x();
  cmd.linear_y = msg.linear_y();
  cmd.angular_z = msg.angular_z();
  cmd.timestamp = detail::ddsTimestampToDouble(msg.header_sec(), msg.header_nanosec());
  return cmd;
}

/// @brief DDS 消息 -> SDK Pose
inline Pose fromMsg(const leju::msgs::Pose& msg) {
  Pose pose;
  pose.x = msg.x();
  pose.y = msg.y();
  pose.z = msg.z();
  pose.qx = msg.qx();
  pose.qy = msg.qy();
  pose.qz = msg.qz();
  pose.qw = msg.qw();
  return pose;
}

/// @brief DDS 消息 -> SDK QuestBonePosesData
inline QuestBonePosesData fromMsg(const leju::msgs::QuestBonePoses& msg) {
  QuestBonePosesData data;
  data.header_sec = msg.header_sec();
  data.header_nanosec = msg.header_nanosec();
  data.timestamp_ms = msg.timestamp_ms();
  data.is_high_confidence = msg.is_high_confidence();
  data.is_hand_tracking = msg.is_hand_tracking();
  data.poses.reserve(msg.poses().size());
  for (const auto& pose : msg.poses()) {
    data.poses.push_back(fromMsg(pose));
  }
  return data;
}

/// @brief DDS 消息 -> SDK QuestJoystickData
inline QuestJoystickData fromMsg(const leju::msgs::QuestJoysticks& msg) {
  QuestJoystickData data;
  data.header_sec = msg.header_sec();
  data.header_nanosec = msg.header_nanosec();
  data.left_x = msg.left_x();
  data.left_y = msg.left_y();
  data.left_trigger = msg.left_trigger();
  data.left_grip = msg.left_grip();
  data.left_first_button_pressed = msg.left_first_button_pressed();
  data.left_second_button_pressed = msg.left_second_button_pressed();
  data.left_first_button_touched = msg.left_first_button_touched();
  data.left_second_button_touched = msg.left_second_button_touched();
  data.right_x = msg.right_x();
  data.right_y = msg.right_y();
  data.right_trigger = msg.right_trigger();
  data.right_grip = msg.right_grip();
  data.right_first_button_pressed = msg.right_first_button_pressed();
  data.right_second_button_pressed = msg.right_second_button_pressed();
  data.right_first_button_touched = msg.right_first_button_touched();
  data.right_second_button_touched = msg.right_second_button_touched();
  return data;
}

}  // namespace vr
}  // namespace leju

#endif  // LEJUSDK_VR_COMMUNICATION_VR_MSG_CONVERTER_HPP_
