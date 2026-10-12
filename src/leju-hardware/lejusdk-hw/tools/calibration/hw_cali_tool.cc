#include <iostream>
#include <gflags/gflags.h>
#include "calibration_base.h"
#include "humanoid_cali.h"

// ANSI color codes
const char* ANSI_RESET = "\033[0m";
const char* ANSI_BOLD = "\033[1m";
const char* ANSI_BLACK = "\033[30m";
const char* ANSI_RED = "\033[31m";
const char* ANSI_GREEN = "\033[32m";
const char* ANSI_YELLOW = "\033[33m";
const char* ANSI_BLUE = "\033[34m";
const char* ANSI_MAGENTA = "\033[35m";
const char* ANSI_CYAN = "\033[36m";
const char* ANSI_WHITE = "\033[37m";

/*
 * --config_dir    Configuration directory path
 * --cali_mode     Calibration mode:
 *                  - 1: Tooling      卡工装校准
 *                  - 2: Manual       手动校准，从当前位置 +- 圈数进行校准
 *                  - 3: JointLimited 基于关节限位校准
 * --cali_type     Calibration type:
 *                  - 1: FullBody     全身关节校准
 *                  - 2: UpperBody    上身校准
 *                  - 3: LowerBody    下身校准
 *                  - 4: Single       单电机校准 (运行中交互选择电机序号)
 */

DEFINE_string(config_dir, "", "Configuration directory path");
DEFINE_int32(cali_mode, 0, "Calibration mode: 1-Tooling, 2-Manual, 3-JointLimited");
DEFINE_int32(cali_type, 0, "Calibration type: 1-FullBody, 2-UpperBody, 3-LowerBody, 4-Single");

const char* GetCalibrationModeName(leju::hw::CaliMode mode) {
    switch (mode) {
        case leju::hw::CaliMode::Tooling: return "Tooling (卡工装校准)";
        case leju::hw::CaliMode::Manual: return "Manual (手动校准)";
        case leju::hw::CaliMode::JointLimited: return "JointLimited (基于关节限位校准)";
        default: return "Unknown";
    }
}

const char* GetCalibrationTypeName(leju::hw::CaliType type) {
    switch (type) {
        case leju::hw::CaliType::FullBody: return "FullBody (全身关节校准)";
        case leju::hw::CaliType::UpperBody: return "UpperBody (上身校准)";
        case leju::hw::CaliType::LowerBody: return "LowerBody (下身校准)";
        case leju::hw::CaliType::Single: return "Single (单电机校准)";
        default: return "Unknown";
    }
}

void PrintUsage() {
    std::cout << std::endl;
    std::cout << ANSI_GREEN << "Usage: hw_cali_tool --config_dir <dir> --cali_mode <1|2|3> --cali_type <1|2|3>" << ANSI_RESET << std::endl;
    std::cout << std::endl;
    std::cout << ANSI_GREEN << "Arguments:" << std::endl;
    std::cout << "  --config_dir <path>   : Configuration directory path (required)" << std::endl;
    std::cout << "  --cali_mode <mode>    : Calibration mode (required):" << std::endl;
    std::cout << "                           1: Tooling - 工装校准, 将当前位置记做零点" << std::endl;
    std::cout << "                           2: Manual  - 手动校准, 从当前位置 +- 圈数进行校准" << std::endl;
    std::cout << "                           3: JointLimited - 基于关节限位校准" << std::endl;
    std::cout << "  --cali_type <type>    : Calibration type (required):" << std::endl;
    std::cout << "                           1: FullBody  - 全身关节校准" << std::endl;
    std::cout << "                           2: UpperBody - 上身校准" << std::endl;
    std::cout << "                           3: LowerBody - 下身校准" << std::endl;
    std::cout << "                           4: Single    - 单电机校准 (运行中交互选择电机序号)" << std::endl;
    std::cout << ANSI_RESET << std::endl;
    std::cout << "Example:" << std::endl;
    std::cout << "  " << ANSI_YELLOW << "hw_cali_tool --config_dir /path/to/config --cali_mode 1 --cali_type 2" << ANSI_RESET << std::endl;
}

int main(int argc, char* argv[]) {

    std::cout << ANSI_CYAN 
        << "-----------------------------------------------------\n"
        << "- Hardware Calibration Tool\n" 
        << "-----------------------------------------------------\n"
        << ANSI_RESET << std::endl;


    // Check for help flag before parsing
    for (int i = 1; i < argc; i++) {
        if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
            PrintUsage();
            return 0;
        }
    }

    // Parse command line flags, but suppress help output
    gflags::SetUsageMessage("Use --help for more information.");
    google::ParseCommandLineFlags(&argc, &argv, false);  // Don't exit on help

      // Validate config directory
    if (FLAGS_config_dir.empty()) {
        std::cout << ANSI_RED << "错误: --config_dir 是必需参数" << ANSI_RESET << std::endl;
        PrintUsage();
        return 1;
    }

    // Validate calibration mode and type using enums
    auto mode = static_cast<leju::hw::CaliMode>(FLAGS_cali_mode);
    auto type = static_cast<leju::hw::CaliType>(FLAGS_cali_type);

    if (mode == leju::hw::CaliMode::None) {
        std::cout << ANSI_RED << "错误: 无效的校准模式或未输入 --cali_mode 参数" << ANSI_RESET << std::endl;
        PrintUsage();
        return 1;
    }

    if (type == leju::hw::CaliType::None) {
        std::cout << ANSI_RED << "错误: 无效的校准类型或未输入 --cali_type 参数" << ANSI_RESET << std::endl;
        PrintUsage();
        return 1;
    }

    std::cout << "硬件校准工具已启动" << std::endl;
    std::cout << "配置目录: " << FLAGS_config_dir << std::endl;
    std::cout << "校准模式: " << FLAGS_cali_mode << " - " << GetCalibrationModeName(mode) << std::endl;
    std::cout << "校准类型: " << FLAGS_cali_type << " - " << GetCalibrationTypeName(type) << std::endl;

    // Humanoid Only
    using namespace leju::hw;
    HumanoidCali humanoid_cali = HumanoidCali(FLAGS_config_dir, mode, type);
    if (humanoid_cali.init()) {
        humanoid_cali.cali();
    }

    return 0;
}