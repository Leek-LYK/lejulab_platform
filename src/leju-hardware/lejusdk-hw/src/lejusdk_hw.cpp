#include "lejusdk_hw/lejusdk_hw.h"
#include <algorithm>
#include <iostream>

namespace leju {
namespace hw {

// ImuType to string conversion function
std::string ImuType2String(ImuType type) {
    switch (type) {
        case ImuType::None:
            return "None";
        case ImuType::Xsens:
            return "Xsens";
        case ImuType::Hipnuc:
            return "Hipnuc";
        default:
            return "Unknown";
    }
}

// String to ImuType conversion function
ImuType String2ImuType(const std::string& type_str) {
    std::string lower_str = type_str;
    std::transform(lower_str.begin(), lower_str.end(), lower_str.begin(), ::tolower);

    if (lower_str == "none") {
        return ImuType::None;
    } else if (lower_str == "xsens") {
        return ImuType::Xsens;
    } else if (lower_str == "hipnuc") {
        return ImuType::Hipnuc;
    }

    return ImuType::None;
}

} // namespace hw
} // namespace leju

// Output stream operator for JointData_t
std::ostream& operator<<(std::ostream& os, const leju::hw::JointData_t& joint_data) {
    os << "JointData_t:\n";

    // Position (q)
    os << "  q: [";
    for (size_t i = 0; i < joint_data.q.size(); ++i) {
        os << joint_data.q[i];
        if (i < joint_data.q.size() - 1) os << ", ";
    }
    os << "]\n";

    // Velocity (v)
    os << "  v: [";
    for (size_t i = 0; i < joint_data.v.size(); ++i) {
        os << joint_data.v[i];
        if (i < joint_data.v.size() - 1) os << ", ";
    }
    os << "]\n";

    // Acceleration (vd)
    os << "  vd: [";
    for (size_t i = 0; i < joint_data.vd.size(); ++i) {
        os << joint_data.vd[i];
        if (i < joint_data.vd.size() - 1) os << ", ";
    }
    os << "]\n";

    // Torque (tau)
    os << "  tau: [";
    for (size_t i = 0; i < joint_data.tau.size(); ++i) {
        os << joint_data.tau[i];
        if (i < joint_data.tau.size() - 1) os << ", ";
    }
    os << "]";

    return os;
}

// Output stream operator for ImuData_t
std::ostream& operator<<(std::ostream& os, const leju::hw::ImuData_t& imu_data) {
    os << "ImuData_t:\n";

    // Gyroscope
    os << "  gyro:\n";
    os << "    x: " << imu_data.gyro[0] << "\n";
    os << "    y: " << imu_data.gyro[1] << "\n";
    os << "    z: " << imu_data.gyro[2] << "\n";

    // Accelerometer
    os << "  acc:\n";
    os << "    x: " << imu_data.acc[0] << "\n";
    os << "    y: " << imu_data.acc[1] << "\n";
    os << "    z: " << imu_data.acc[2] << "\n";

    // Free acceleration
    os << "  free_acc:\n";
    os << "    x: " << imu_data.free_acc[0] << "\n";
    os << "    y: " << imu_data.free_acc[1] << "\n";
    os << "    z: " << imu_data.free_acc[2] << "\n";

    // Quaternion (output in x, y, z, w order as ROS format)
    os << "  quat:\n";
    os << "    x: " << imu_data.quat[1] << "\n";
    os << "    y: " << imu_data.quat[2] << "\n";
    os << "    z: " << imu_data.quat[3] << "\n";
    os << "    w: " << imu_data.quat[0];

    return os;
}

// Output stream operator for JointCommand_t
std::ostream& operator<<(std::ostream& os, const leju::hw::JointCommand_t& joint_command) {
    os << "JointCommand_t:\n";

    // Target position
    os << "  q: [";
    for (size_t i = 0; i < joint_command.q.size(); ++i) {
        os << joint_command.q[i];
        if (i < joint_command.q.size() - 1) os << ", ";
    }
    os << "]\n";

    // Target velocity
    os << "  v: [";
    for (size_t i = 0; i < joint_command.v.size(); ++i) {
        os << joint_command.v[i];
        if (i < joint_command.v.size() - 1) os << ", ";
    }
    os << "]\n";

    // Target torque
    os << "  tau: [";
    for (size_t i = 0; i < joint_command.tau.size(); ++i) {
        os << joint_command.tau[i];
        if (i < joint_command.tau.size() - 1) os << ", ";
    }
    os << "]\n";

    // Position gain
    os << "  kp: [";
    for (size_t i = 0; i < joint_command.kp.size(); ++i) {
        os << joint_command.kp[i];
        if (i < joint_command.kp.size() - 1) os << ", ";
    }
    os << "]\n";

    // Velocity gain
    os << "  kd: [";
    for (size_t i = 0; i < joint_command.kd.size(); ++i) {
        os << joint_command.kd[i];
        if (i < joint_command.kd.size() - 1) os << ", ";
    }
    os << "]\n";

    // Max torque
    os << "  tau_max: [";
    for (size_t i = 0; i < joint_command.tau_max.size(); ++i) {
        os << joint_command.tau_max[i];
        if (i < joint_command.tau_max.size() - 1) os << ", ";
    }
    os << "]\n";

    // Torque ratio
    os << "  tau_ratio: [";
    for (size_t i = 0; i < joint_command.tau_ratio.size(); ++i) {
        os << joint_command.tau_ratio[i];
        if (i < joint_command.tau_ratio.size() - 1) os << ", ";
    }
    os << "]\n";

    // Control modes
    os << "  modes: [";
    for (size_t i = 0; i < joint_command.modes.size(); ++i) {
        os << static_cast<int>(joint_command.modes[i]);
        if (i < joint_command.modes.size() - 1) os << ", ";
    }
    os << "]";

    return os;
}