#include "humanoid_cali.h"
#include "hardware_plant.h"

#include <termios.h>
#include <unistd.h>
#include <cstdio>
#include <csignal>
#include <iomanip>
#include <cmath>
#include <iostream>
#include <limits>

namespace leju {
namespace hw {

// SIGINT 标志: Ctrl+C 触发时设为 1, 交互循环检测到后安全退出
static volatile sig_atomic_t g_cali_sigint_received = 0;

static void cali_sigint_handler(int) {
    g_cali_sigint_received = 1;
}

// 非阻塞键盘检测
static bool kbhit_nb() {
    struct timeval tv = {0, 0};
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds);
    return select(STDIN_FILENO + 1, &fds, nullptr, nullptr, &tv) > 0;
}

HumanoidCali::HumanoidCali(const std::string& config_dir, CaliMode mode, CaliType type)
    : CalibrationBase(config_dir, mode, type) {
}

HumanoidCali::~HumanoidCali() {
    initialized_ = false;

    if(hardware_plant_) {
        hardware_plant_->HWPlantDeInit();
        delete hardware_plant_;
        hardware_plant_ = nullptr;
    }
}

bool HumanoidCali::init() {
    HighlyDynamic::HardwareParam hardware_param;
    // 标定模式: 根据 cali_type 决定标定哪些部位
    // 对于全 RUIWO 机型 (CAN+CANFD), cali_leg/cali_arm 控制 RuiwoActuator 的标定范围
    // Single(单电机校准): 不做整条总线的批量清零, 全部电机按现有零点文件加载,
    // 交互阶段只对操作者选中的那一个电机调用 calibrateSingleMotor()
    hardware_param.cali_leg = (CaliType::FullBody == type_ || CaliType::LowerBody == type_);
    hardware_param.cali_arm = (CaliType::FullBody == type_ || CaliType::UpperBody == type_);
    // Single 模式: 标定过程中机器人不可能站立承重, 所以启动阶段直接不整体使能任何电机
    // (而不是使能后再锁定当前位置) —— 全部电机保持失能静止, 交互阶段只对操作者选中的
    // 那一个电机单独执行 清多圈->使能->读位置->记零点->失能, 不牵连其它电机
    hardware_param.cali_single_motor_mode = (CaliType::Single == type_);
    hardware_param.cali     = false;
    hardware_param.teach_pendant_ = false;
    hardware_param.only_half_up_body = false;
    hardware_param.robot_version = RobotVersion::from_env();
    hardware_param.kuavo_assets_path = config_dir_;

    printf("Hardware version: %s\n", hardware_param.robot_version.to_string().c_str());

    const double dt = 0.001;
    hardware_plant_ = new HighlyDynamic::HardwarePlant(dt, hardware_param);

    if (hardware_plant_->HWPlantInit() != 0) {
        return false;
    }

    initialized_ = true;
    return true;
}

bool HumanoidCali::cali() {
    if(!initialized_) {
        std::cerr << "HumanoidCali::cali() called before initialized" << std::endl;
        return false;
    }

    switch (mode_)
    {
        case CaliMode::Tooling:         return toolingCali();
        case CaliMode::Manual:          return manualCali();
        case CaliMode::JointLimited:    return jointLimitedCali();
        default:
            std::cerr << "HumanoidCali::cali() called with unsupported mode" << std::endl;
            return false;
    }
}

bool HumanoidCali::toolingCali() {
    bool has_ruiwo = (hardware_plant_->getRuiwoActuator() != nullptr);
    bool has_ec = (hardware_plant_->getCountECMasters() > 0);

    // EC_MASTER 电机校准 (仅在有 EC 电机时)
    if (has_ec && (CaliType::FullBody == type_ || CaliType::LowerBody == type_)) {
        hardware_plant_->setCurrentPositionAsOffset();
    }

    // RUIWO 电机校准 (CAN + CANFD 全部走这条路径)
    if (has_ruiwo) {
        return toolingCaliCanFd(hardware_plant_->getRuiwoActuator());
    }

    if (!has_ec && !has_ruiwo) {
        std::cout << "\033[31m未检测到任何电机\033[0m\n";
        return false;
    }
    return true;
}

bool HumanoidCali::toolingCaliCanFd(RuiwoActuatorBase* ruiwo) {
    auto names = ruiwo->getMotorRefNames();
    int motor_count = static_cast<int>(names.size());
    if (motor_count == 0) {
        std::cout << "\033[31mRUIWO 电机数量为 0\033[0m\n";
        return false;
    }

    auto printPositions = [&]() {
        auto zero_points = ruiwo->getMotorZeroPoints();
        std::cout << "\n\033[36m=== CANFD/RUIWO 电机零点标定 ===\033[0m\n";
        std::cout << std::left << std::setw(6) << "序号"
                  << std::setw(20) << "名称"
                  << std::right << std::setw(12) << "零点偏移(rad)"
                  << std::setw(12) << "零点偏移(°)" << "\n";
        std::cout << std::string(50, '-') << "\n";
        for (int i = 0; i < motor_count; ++i) {
            double offset_rad = (i < static_cast<int>(zero_points.size())) ? zero_points[i] : 0.0;
            double offset_deg = offset_rad * 180.0 / M_PI;
            std::cout << std::left << std::setw(6) << i
                      << std::setw(20) << (i < static_cast<int>(names.size()) ? names[i] : "?")
                      << std::right << std::fixed << std::setprecision(4)
                      << std::setw(12) << offset_rad
                      << std::setw(12) << offset_deg << "\n";
        }
    };

    printPositions();

    if (CaliType::Single == type_) {
        std::cout << "\n\033[33m[单电机标定模式] 所有电机保持失能(无力矩)状态，不会整体使能，"
                     "可以随时徒手把关节摆到目标零位。\n"
                     "                  对摆好的关节执行 's' 时会自动完成: "
                     "清多圈编码器 → 单独使能该电机 → 读取当前位置记为零点 → 重新失能，"
                     "全程不影响同总线其它电机。\033[0m\n"
                     "\033[33m                  注意: 请确保标定期间机器人不处于站立承重状态"
                     "（腿部总线清零/使能时会让同总线其它关节短暂处于失能态）。\033[0m\n";
    }

    const bool is_single_mode = (CaliType::Single == type_);

    std::cout << "\n\033[32m"
              << "将所有关节卡到工装零位后，按以下键操作:\n";
    if (!is_single_mode) {
        std::cout << "  'c' — 保存当前位置为零点 (全部电机)\n";
    }
    if (is_single_mode) {
        std::cout << "  's' — 单独标定一个电机 (只把选中的电机置零, 不影响其它电机)\n";
    }
    std::cout << "  'p' — 打印当前零点偏移\n"
              << "  'q' — 退出 (不保存)\n"
              << "  Ctrl+C — 安全退出 (同 'q', 不保存)\n"
              << "\033[0m" << std::endl;

    // 安装 SIGINT handler: Ctrl+C 设置标志位, 由交互循环检测后安全退出, 不直接杀进程
    g_cali_sigint_received = 0;
    auto old_sigint_handler = signal(SIGINT, cali_sigint_handler);

    while (!g_cali_sigint_received) {
        if (kbhit_nb()) {
            char ch = getchar();
            if (ch == 'c') {
                if (is_single_mode) {
                    std::cout << "\033[31m当前是单电机标定模式 (--cali_type 4)，"
                                  "'c' 全身保存在此模式下不生效 (未整体使能/无有效位置数据)，"
                                  "请使用 's' 单独标定。\033[0m\n";
                    continue;
                }
                std::cout << "\033[33m正在执行完整标定流程(清零编码器 → 使能 → 记录零点 → 保存)...\033[0m\n";
                if (ruiwo->calibrateAllAndSave()) {
                    std::cout << "\033[32m零点标定并保存成功\033[0m\n";
                    printPositions();
                    return true;
                } else {
                    std::cout << "\033[31m零点标定失败!\033[0m\n";
                }
            } else if (ch == 's') {
                if (!is_single_mode) {
                    std::cout << "\033[31m当前非单电机标定模式 (--cali_type "
                              << static_cast<int>(type_)
                              << ")，其它电机可能已整体使能/承重，'s' 会向同总线其它电机"
                                 "广播失能帧，存在安全风险，已禁止。请使用 "
                                 "--cali_mode 1 --cali_type 4 重新启动后再单独标定。\033[0m\n";
                    continue;
                }
                std::cout << "\n请输入要单独标定的电机序号 (0~" << (motor_count - 1) << ", 非法输入取消): ";
                int idx = -1;
                if (!(std::cin >> idx)) {
                    std::cin.clear();
                    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                    std::cout << "\033[31m输入无效，已取消\033[0m\n";
                    continue;
                }
                if (idx < 0 || idx >= motor_count) {
                    std::cout << "\033[31m序号超出范围 [0, " << (motor_count - 1) << "]，已取消\033[0m\n";
                    continue;
                }
                std::cout << "将 [" << idx << "] " << names[idx]
                          << " 当前位置标定为零点？确认请输入 y: ";
                char confirm = 0;
                std::cin >> confirm;
                if (confirm != 'y' && confirm != 'Y') {
                    std::cout << "已取消\n";
                    continue;
                }
                if (ruiwo->calibrateSingleMotor(idx)) {
                    std::cout << "\033[32m电机 [" << idx << "] " << names[idx] << " 零点已标定并保存\033[0m\n";
                } else {
                    std::cout << "\033[31m电机 [" << idx << "] " << names[idx] << " 标定失败\033[0m\n";
                }
                printPositions();
            } else if (ch == 'p') {
                printPositions();
            } else if (ch == 'q') {
                std::cout << "退出标定 (未保存)\n";
                return false;
            }
        }
        usleep(100000);  // 100ms
    }

    // 恢复原有 SIGINT handler
    signal(SIGINT, old_sigint_handler);

    if (g_cali_sigint_received) {
        std::cin.clear();  // 清理 stdin 状态（信号可能打断了 cin >> 操作）
        std::cout << "\n退出标定 (Ctrl+C, 未保存)\n";
    }
    return false;
}

bool HumanoidCali::jointLimitedCali() {
    std::cout << "Not implemented \n";
    // TODO(ohh): IMPLEMENT
    return false;
}

bool HumanoidCali::manualCali() {
    std::cout << "Not implemented \n";
    // TODO(ohh): IMPLEMENT
    return false;
}

} // namespace leju
} // namespace hw