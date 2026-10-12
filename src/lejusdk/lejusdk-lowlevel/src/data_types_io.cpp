#include "lejusdk-lowlevel/data_types.h"
#include <iostream>
#include <sstream>
#include <iomanip>

namespace leju {

std::ostream& operator<<(std::ostream& os, const MotorControlMode& mode) {
  switch (mode) {
    case MotorControlMode::CST: os << "CST"; break;
    case MotorControlMode::CSP: os << "CSP"; break;
    case MotorControlMode::CSV: os << "CSV"; break;
    default: os << "UNKNOWN"; break;
  }
  return os;
}

// 辅助函数：输出数组
template<typename T>
void printArray(std::ostream& os, const std::vector<T>& arr, const std::string& indent = "  ") {
  if (arr.empty()) {
    os << "[]";
    return;
  }

  os << "[";
  for (size_t i = 0; i < arr.size(); ++i) {
    if (i > 0) {
      os << ", ";
    }
    os << std::fixed << std::setprecision(8) << arr[i];
  }
  os << "]";
}

std::ostream& operator<<(std::ostream& os, const RobotState& state) {
  os << "--------------------\n";
  os << "RobotState:\n";
  os << "  timestamp: " << std::fixed << std::setprecision(6) << state.timestamp << "\n";

  os << "  q: ";
  printArray(os, state.q, "  ");
  os << "\n";

  os << "  v: ";
  printArray(os, state.v, "  ");
  os << "\n";

  os << "  tau: ";
  printArray(os, state.tau, "  ");

  return os;
}

std::ostream& operator<<(std::ostream& os, const RobotCmd& cmd) {
  os << "--------------------\n";
  os << "RobotCmd:\n";
  os << "  timestamp: " << std::fixed << std::setprecision(6) << cmd.timestamp << "\n";

  os << "  q: ";
  printArray(os, cmd.q, "  ");
  os << "\n";

  os << "  v: ";
  printArray(os, cmd.v, "  ");
  os << "\n";

  os << "  tau: ";
  printArray(os, cmd.tau, "  ");
  os << "\n";

  os << "  kp: ";
  printArray(os, cmd.kp, "  ");
  os << "\n";

  os << "  kd: ";
  printArray(os, cmd.kd, "  ");
  os << "\n";

  os << "  modes: ";
  if (cmd.modes.empty()) {
    os << "[]";
  } else {
    os << "[";
    for (size_t i = 0; i < cmd.modes.size(); ++i) {
      if (i > 0) {
        os << ", ";
      }
      os << static_cast<MotorControlMode>(cmd.modes[i]);
    }
    os << "]";
  }

  return os;
}

std::ostream& operator<<(std::ostream& os, const ImuData& imu) {
  os << "--------------------\n";
  os << "ImuData:\n";
  os << "  timestamp: " << std::fixed << std::setprecision(6) << imu.timestamp << "\n";
  os << "  gyro: ["
     << std::fixed << std::setprecision(8)
     << imu.gyro[0] << ", " << imu.gyro[1] << ", " << imu.gyro[2] << "]\n";
  os << "  acc: ["
     << std::fixed << std::setprecision(8)
     << imu.acc[0] << ", " << imu.acc[1] << ", " << imu.acc[2] << "]\n";
  os << "  free_acc: ["
     << std::fixed << std::setprecision(8)
     << imu.free_acc[0] << ", " << imu.free_acc[1] << ", " << imu.free_acc[2] << "]\n";
  os << "  quat: ["
     << std::fixed << std::setprecision(8)
     << imu.quat[0] << ", " << imu.quat[1] << ", " << imu.quat[2] << ", " << imu.quat[3] << "]";

  return os;
}

std::ostream& operator<<(std::ostream& os, const JoyData& joy) {
  os << "--------------------\n";
  os << "JoyData:\n";
  os << "  timestamp: " << std::fixed << std::setprecision(6) << joy.timestamp << "\n";

  os << "  axes: ["
     << "left_x=" << std::fixed << std::setprecision(4) << joy.axes.left_x << ", "
     << "left_y=" << joy.axes.left_y << ", "
     << "right_x=" << joy.axes.right_x << ", "
     << "right_y=" << joy.axes.right_y << ", "
     << "left_trigger=" << joy.axes.left_trigger << ", "
     << "right_trigger=" << joy.axes.right_trigger
     << "]\n";

  os << "  buttons: ["
     << "south=" << joy.buttons.south << ", "
     << "east=" << joy.buttons.east << ", "
     << "west=" << joy.buttons.west << ", "
     << "north=" << joy.buttons.north << ", "
     << "back=" << joy.buttons.back << ", "
     << "guide=" << joy.buttons.guide << ", "
     << "start=" << joy.buttons.start << ", "
     << "left_stick=" << joy.buttons.left_stick << ", "
     << "right_stick=" << joy.buttons.right_stick << ", "
     << "left_shoulder=" << joy.buttons.left_shoulder << ", "
     << "right_shoulder=" << joy.buttons.right_shoulder << ", "
     << "dpad_up=" << joy.buttons.dpad_up << ", "
     << "dpad_down=" << joy.buttons.dpad_down << ", "
     << "dpad_left=" << joy.buttons.dpad_left << ", "
     << "dpad_right=" << joy.buttons.dpad_right << ", "
     << "misc1=" << joy.buttons.misc1
     << "]";

  return os;
}

} // namespace leju