#include "utils/path_utils.h"
#include <cstdlib>
#include <pwd.h>
#include <unistd.h>

namespace leju {
namespace hw {



namespace path {

std::string config_dir() {
    // Get current user information
    const char* sudo_user = getenv("SUDO_USER");
    passwd* pw = nullptr;

    if (sudo_user != nullptr) {
        pw = getpwnam(sudo_user);
    } else {
        pw = getpwuid(getuid());
    }

    if (pw != nullptr) {
        return std::string(pw->pw_dir) + "/.config/lejuconfig";
    }

    // WARN!!! 
    return "/home/lab/.config/lejuconfig";
}

std::string imu_type_path() {
    // Get configuration root directory
    std::string config_root = config_dir();
    if (config_root.empty()) {
        return ""; // Return empty if config directory not found
    }

    // Construct path to IMU type configuration file
    std::string imu_type_path = config_root + "/ImuType.ini";

    return imu_type_path;
}

std::string ecmaster_type_path() {
    // Get configuration root directory
    std::string config_root = config_dir();
    if (config_root.empty()) {
        return ""; // Return empty if config directory not found
    }

    // Construct path to EC master type configuration file
    std::string ecmaster_type_path = config_root + "/EcMasterType.ini";

    return ecmaster_type_path;
}

std::string offset_path() {
    // Get configuration root directory
    std::string config_root = config_dir();
    if (config_root.empty()) {
        return ""; // Return empty if config directory not found
    }

    // Construct path to motor offset configuration file
    std::string offset_path = config_root + "/offset.yaml";

    return offset_path;
}

std::string motor_revo_offset_path() {
    // Get configuration root directory
    std::string config_root = config_dir();
    if (config_root.empty()) {
        return ""; // Return empty if config directory not found
    }

    // Construct path to motor revo offset configuration file
    std::string motor_revo_offset_path = config_root + "/arms_zero.yaml";

    return motor_revo_offset_path;
}

std::string canbus_wiring_type_path() {
    // Get configuration root directory
    std::string config_root = config_dir();
    if (config_root.empty()) {
        return ""; // Return empty if config directory not found
    }

    // Construct path to CAN bus wiring type configuration file
    std::string canbus_wiring_type_path = config_root + "/CanbusWiringType.ini";

    return canbus_wiring_type_path;
}

std::string canbus_config_path() {
    // Get configuration root directory
    std::string config_root = config_dir();
    if (config_root.empty()) {
        return ""; // Return empty if config directory not found
    }

    // Construct path to CAN bus configuration file
    std::string canbus_config_path = config_root + "/canbus_device_cofig.yaml";

    return canbus_config_path;
}

} // namespace path
} // namespace hw
} // namespace leju