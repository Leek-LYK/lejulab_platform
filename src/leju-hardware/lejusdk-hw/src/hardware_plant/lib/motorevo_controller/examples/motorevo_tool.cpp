#include <iostream>
#include <string>
#include <functional>
#include <map>
#include <set>
#include <vector>
#include <memory>
#include <chrono>
#include <thread>
#include <cmath>
#include <fstream>
#include <algorithm>
#include <cctype>
#include "motorevo/motor_ctrl.h"
#include "motorevo/motor_def.h"
#include "motorevo/motorevo_actuator.h"
#include "canbus_sdk/canbus_sdk.h"
#include "canbus_sdk/config_parser.h"

using namespace motorevo;
// 上下文结构体，传递给各个功能函数
struct EntryContext {
    const char* program_name;
    int argc;
    char** argv;

    EntryContext(const char* prog, int ac, char** av)
        : program_name(prog), argc(ac), argv(av) {}
};

//////////////////// Entry Functions ///////////////////////////
void show_help(const char* program_name) {
    std::cout << "使用方法: " << program_name << " [选项]" << std::endl;
    std::cout << "选项:" << std::endl;
    std::cout << "  --negative  电机方向辨识" << std::endl;
    std::cout << "  --cali      电机校准" << std::endl;
    std::cout << "  --help     显示此帮助信息" << std::endl;
    std::cout << std::endl;
    std::cout << "示例:" << std::endl;
    std::cout << "  " << program_name << " --negative  # 设置电机方向" << std::endl;
    std::cout << "  " << program_name << " --cali      # 电机校准" << std::endl;
}
void entry_negative_setting(const EntryContext& context);
void entry_calibration(const EntryContext& context);
void entry_help(const EntryContext& context) {
    show_help(context.program_name);
}
////////////////////////////////////////////////////////////////
//                       Motor Control Functions                      //
////////////////////////////////////////////////////////////////

struct MotorCtrlContext {
    bool  feedback_received;          // 是否收到反馈
    RevoMotor* motor;                 // 电机实例指针
};
std::map<uint32_t, MotorCtrlContext> motors; // 存储所有电机对象，以设备ID为键

void can_receive_callback(canbus_sdk::CanMessageFrame* frame, const canbus_sdk::CallbackContext* context)
{
#if DEBUG_PRINT_CAN_DATA
  printf("\033[32mCAN FD Received: ID: 0x%X, DLC: %d, Data: ", frame->id.SID, frame->ctrl.rx.dlc);
  for (int i = 0; i < frame->ctrl.rx.dlc; ++i)
  {
    printf("%02x ", frame->payload[i]);
  }
  printf("\033[0m\n");
#endif

    uint8_t payload[8];
    memccpy(payload, frame->payload, 0, 8);
    motorevo::FeedbackFrame feedback(payload);

    motorevo::MotorId motor_id = feedback.motor_id();
    auto data_it = motors.find(motor_id);
    if (data_it == motors.end()) {
        printf("Received CAN message for unknown motor ID: 0x%02X\n", motor_id);
        return;
    }
    data_it->second.motor->receiveFeedback(feedback);
    data_it->second.feedback_received = true;
      
    freeCanMessageFrame(frame);
}

void entry_negative_setting(const EntryContext& context) {
    std::cout << "\033[33m==> 电机方向辨识功能 <==\033[0m" << std::endl;

    ////////////////////////////////////////////////////////////////////////////
    //                      Configuration Parser Setup                       //
    ////////////////////////////////////////////////////////////////////////////
    canbus_sdk::ConfigParser parser;

    try {
        // 获取默认配置文件路径 解析配置文件
        std::string config_path = canbus_sdk::ConfigParser::getDefaultConfigFilePath();
        if (!parser.parseFromFile(config_path)) {
            std::cerr << "错误: 无法解析配置文件" << std::endl;
            return;
        }
        // 获取所有 CAN 总线配置
        const auto& canbus_configs = parser.getCanBusConfigs();
        bool found_motors = false;
        // 遍历每个 CAN 总线
        std::cout << "\n===========================================================\n";
        std::cout << "Canbus 配置:\n";
        for (const auto& canbus : canbus_configs) {
            // 获取该 CAN 总线上的所有电机设备
            auto motor_devices = parser.getDevices(canbus.name, canbus_sdk::DeviceType::MOTOR);

            if (motor_devices.empty()) {
                continue;
            }

            // 使用列表格式显示电机 ID
            std::cout << canbus.name << ": [";
            for (size_t i = 0; i < motor_devices.size(); ++i) {
                std::cout << "0x" << std::hex << static_cast<uint32_t>(motor_devices[i].device_id) << std::dec;
                if (i < motor_devices.size() - 1) {
                    std::cout << ", ";
                }
            }
            std::cout << "]" << std::endl;
            found_motors = true;
        }
        std::cout << "===========================================================\n\n";

        if (!found_motors) {
            std::cout << "未找到任何电机设备" << std::endl;
            return;
        }

    } catch (const std::exception& e) {
        std::cerr << "异常: " << e.what() << std::endl;
        return;
    }

    ////////////////////////////////////////////////////////////////////////////
    // 初始化 CANBUS_SDK
    ////////////////////////////////////////////////////////////////////////////
    using namespace canbus_sdk;
    canbus_sdk::CanBusController *canbus_controller = &CanBusController::getInstance();
    canbus_controller->init();

    // 遍历每个CAN总线配置，创建对应的控制器
    const auto& canbus_configs = parser.getCanBusConfigs();
    for (const auto& canbus : canbus_configs) {
        // 打开CAN总线
        auto result = canbus_controller->openCanBus(canbus.name, canbus.type, canbus.bitrate);
        if (!result.has_value()) {
            printf("\033[31mCAN总线打开失败: %s\033[0m\n", errorToString(result.error()));
            return;
        }
    }

    ////////////////////////////////////////////////////////////////////////////
    // 注册设备
    ////////////////////////////////////////////////////////////////////////////
    canbus_sdk::CallbackParams callback_params {
        .msg_callback = can_receive_callback,
        .msg_cb_ctx = nullptr,
        .tef_callback = nullptr,
        .tef_cb_ctx = nullptr
    };

    // 遍历每个CAN总线配置，注册所有电机设备
    for (const auto& canbus : canbus_configs) {
        // 获取该CAN总线上的所有电机设备
        auto motor_devices = parser.getDevices(canbus.name, canbus_sdk::DeviceType::MOTOR);
        for (const auto& device : motor_devices) {
            // 注册设备到CAN总线控制器
            auto result = canbus_controller->registerDevice(canbus_sdk::DeviceInfo(
                device.name,
                canbus_sdk::DeviceType::MOTOR,
                device.device_id,
                nullptr /* use default */
            ), canbus.name, callback_params);

            if (result.has_value()) {
                std::cout << "\033[32m设备注册成功: " << device.name << " (ID: 0x"
                         << std::hex << static_cast<uint32_t>(device.device_id) << std::dec << ")\033[0m" << std::endl;

                auto bus_id = canbus_controller->getBusIdByName(canbus.name);
                if(bus_id.has_value()) {
                    // 创建电机对象并添加到全局motors map
                    motors[device.device_id] = MotorCtrlContext{
                        .feedback_received = false,
                        .motor = new motorevo::RevoMotor(bus_id.value(), device.device_id, motorevo::MotorMode::TorquePositionMixControlMode)
                    };
                    std::cout << "\033[32m电机对象创建成功: " << device.name << " (ID: 0x"
                         << std::hex << static_cast<uint32_t>(device.device_id) << std::dec << ")\033[0m" << std::endl;
                }

            } else {
                std::cout << "\033[31m设备注册失败: " << device.name << " (ID: 0x"
                         << std::hex << static_cast<uint32_t>(device.device_id) << std::dec << "), 错误: "
                         << canbus_sdk::errorToString(result.error()) << "\033[0m" << std::endl;
            }
        }
    }

    // 检查是否有可用的电机
    if (motors.empty()) {
        std::cout << "\033[31m错误: 没有已注册的电机设备\033[0m" << std::endl;
        return;
    }

    // 显示可用的电机列表
    std::cout << "\033[36m可用的电机设备:\033[0m" << std::endl;
    for (const auto& pair : motors) {
        std::cout << "  ID: " << pair.first << " (0x" << std::hex << pair.first << std::dec << ")" << std::endl;
    }

    std::cout << "\033[36m请输入要测试的电机ID (十进制)，按 Ctrl+C 结束:\033[0m" << std::endl;

    std::string input;
    while (std::cin >> input) {
        uint32_t motor_id = 0;

        // 解析输入的电机ID（只支持十进制）
        try {
            motor_id = std::stoul(input);
        } catch (const std::exception& e) {
            std::cout << "\033[31m错误: 无效的电机ID格式，请输入十进制数字\033[0m" << std::endl;
            std::cout << "\033[36m请重新输入电机ID:\033[0m" << std::endl;
            continue;
        }

        // 检查电机是否存在
        auto motor_it = motors.find(motor_id);
        if (motor_it == motors.end()) {
            std::cout << "\033[31m错误: 电机ID " << motor_id << " (0x" << std::hex << motor_id << std::dec << ") 不存在\033[0m" << std::endl;
            std::cout << "\033[36m请重新输入电机ID:\033[0m" << std::endl;
            continue;
        }

        std::cout << "\033[32m已选择电机ID: " << motor_id << " (0x" << std::hex << motor_id << std::dec << ")\033[0m" << std::endl;

        // 获取电机对象
        auto& motor_context = motor_it->second;
        auto& motor = motor_context.motor;

        // 重置反馈标志
        motor_context.feedback_received = false;

        // 使能电机
        std::cout << "\033[36m正在使能电机...\033[0m" << std::endl;
        if (!motor->enterMotorState()) {
            std::cout << "\033[31m错误: 电机使能命令发送失败\033[0m" << std::endl;
            continue;
        }

        // 等待反馈
        auto start_time = std::chrono::steady_clock::now();
        bool feedback_timeout = false;

        while (!motor_context.feedback_received) {
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - start_time);

            if (elapsed.count() >= 1000) {
                std::cout << "\033[31m错误: 电机反馈超时\033[0m" << std::endl;
                feedback_timeout = true;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        if (feedback_timeout) {
            std::cout << "\033[31m电机使能失败，请检查连接\033[0m" << std::endl;
            std::cout << "\033[36m请输入下一个电机ID:\033[0m" << std::endl;
            continue;
        }

        std::cout << "\033[32m电机使能成功\033[0m" << std::endl;

        // 记录当前位置
        double start_position = motor->position();
        std::cout << "\033[36m当前位置: " << start_position << " 弧度\033[0m" << std::endl;

        // 转换10度为弧度
        double target_angle_deg = 15.0;
        double target_angle_rad = target_angle_deg * M_PI / 180.0;
        double target_position = start_position + target_angle_rad;

        std::cout << "\033[36m开始运动: 正向转10度...\033[0m" << std::endl;

        // 正向转10度，频率1Hz，持续1秒
        auto motion_start = std::chrono::steady_clock::now();
        const double frequency = 1.0; // 1Hz
        const int duration_ms = 1000; // 1秒
        const int control_interval_ms = 100; // 100ms间隔

        while (true) {
            auto current_time = std::chrono::steady_clock::now();
            auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                current_time - motion_start).count();

            if (elapsed_ms >= duration_ms) {
                break;
            }

            // 计算目标位置（简单的从起点到终点的线性插值）
            double progress = static_cast<double>(elapsed_ms) / duration_ms;
            double current_target = start_position + (target_position - start_position) * progress;

            // 发送控制命令
            motor->controlPTM(current_target, 0.0, 0.0, 20.0, 1.0);

            std::this_thread::sleep_for(std::chrono::milliseconds(control_interval_ms));
        }

        ///////////////////////////////////////////////////////////////////
        // 询问用户电机方向，用户需要输入Y/N/K
        std::cout << "\033[36m电机运动测试完成，请观察电机转动方向。\033[0m" << std::endl;
        std::cout << "\033[33m电机方向设置？(Y:反转, N:正转, K:跳过): \033[0m" << std::endl;

        std::string user_input;
        bool need_update_config = false;

        // 循环等待用户输入Y/N/K
        while (std::cin >> user_input) {
            // 转换为大写进行比较
            std::transform(user_input.begin(), user_input.end(), user_input.begin(), ::toupper);

            if (user_input == "Y" || user_input == "YES") {
                need_update_config = true;
                break;
            } else if (user_input == "N" || user_input == "NO") {
                need_update_config = true;
                break;
            } else if (user_input == "K") {
                need_update_config = false;
                break;
            } else {
                std::cout << "\033[31m无效输入，请输入 Y (反转)、N (正转) 或 K (跳过): \033[0m" << std::endl;
            }
        }

        // 如果用户选择Y或N，更新配置文件
        if (need_update_config) {
            std::cout << "\033[33m正在更新电机配置...\033[0m" << std::endl;

            // 获取设备名称（从配置文件中查找）
            std::string device_name = "unknown";
            auto canbus_it = std::find_if(canbus_configs.begin(), canbus_configs.end(),
                [&motor_id, &device_name, &parser](const auto& canbus) {
                    auto motor_devices = parser.getDevices(canbus.name, canbus_sdk::DeviceType::MOTOR);
                    auto device_it = std::find_if(motor_devices.begin(), motor_devices.end(),
                        [motor_id](const auto& device) {
                            return device.device_id == motor_id;
                        });
                    if (device_it != motor_devices.end()) {
                        device_name = device_it->name;
                        return true;
                    }
                    return false;
                });
            if (canbus_it == canbus_configs.end()) {
                device_name = "unknown";
            }

            // 根据用户输入设置电机方向
            bool is_negative = (user_input == "Y" || user_input == "YES");
            bool update_success = parser.setMotorNegative(device_name, is_negative);
            if (update_success) {
                // 备份原配置文件
                std::string original_config = canbus_sdk::ConfigParser::getDefaultConfigFilePath();
                std::string backup_config = original_config + ".backup";

                // 复制原文件为备份
                std::ifstream src(original_config, std::ios::binary);
                std::ofstream dst(backup_config, std::ios::binary);
                if (src && dst) {
                    dst << src.rdbuf();
                    std::cout << "\033[32m配置文件已备份至: " << backup_config << "\033[0m" << std::endl;
                }
                src.close();
                dst.close();

                // 写回更新后的配置
                bool write_success = parser.writeToFile(original_config);
                if (write_success) {
                    std::string direction_str = is_negative ? "反向" : "正向";
                    std::cout << "\033[32m配置更新成功！电机 " << device_name << " (ID: " << motor_id << ") 方向已设置为" << direction_str << "\033[0m" << std::endl;
                } else {
                    std::cout << "\033[31m错误: 配置文件写入失败\033[0m" << std::endl;
                }
            } else {
                std::cout << "\033[31m错误: 找不到设备 " << device_name << " 或配置更新失败\033[0m" << std::endl;
            }
        } else {
            std::cout << "\033[32m跳过配置更新\033[0m" << std::endl;
        }

        ///////////////////////////////////////////////////////////////////
        std::cout << "\033[36m转回到起始位置...\033[0m" << std::endl;

        // 转回到起始位置
        motion_start = std::chrono::steady_clock::now();
        while (true) {
            auto current_time = std::chrono::steady_clock::now();
            auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                current_time - motion_start).count();

            if (elapsed_ms >= duration_ms) {
                break;
            }

            // 计算目标位置（从终点回到起点）
            double progress = static_cast<double>(elapsed_ms) / duration_ms;
            double current_target = target_position + (start_position - target_position) * progress;

            // 发送控制命令
            motor->controlPTM(current_target, 0.0, 0.0, 20.0, 1.0);

            std::this_thread::sleep_for(std::chrono::milliseconds(control_interval_ms));
        }

        std::cout << "\033[32m运动完成\033[0m" << std::endl;

        // 失能电机
        std::cout << "\033[36m正在失能电机...\033[0m" << std::endl;
        motor->enterRestState();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        std::cout << "\033[32m电机失能完成\033[0m" << std::endl;
        std::cout << "\033[36m请输入下一个电机ID (或按 Ctrl+C 结束):\033[0m" << std::endl;
    }

    // 清理动态分配的电机对象
    for (auto& pair : motors) {
        delete pair.second.motor;
    }
    motors.clear();

    // 关闭CAN总线
    canbus_sdk::CanBusController::getInstance().closeAllCanBuses();
}


////////////////////////////////////////////////////////////////////////////////////
//                          Calibration Functions                             //
////////////////////////////////////////////////////////////////////////////////////

void entry_calibration(const EntryContext& context) {
    std::cout << "\033[33m==> 电机校准功能 <==\033[0m" << std::endl;

    try {
        std::cout << "\033[36m===============================================\033[0m" << std::endl;
        std::cout << "\033[36m      电机校准程序\033[0m" << std::endl;
        std::cout << "\033[36m===============================================\033[0m" << std::endl;

        // 获取配置文件路径
        std::string config_file_path = canbus_sdk::ConfigParser::getDefaultConfigFilePath();
        std::cout << "\033[33m使用配置文件: " << config_file_path << "\033[0m" << std::endl;

        // 枚举配置中的所有 CAN 总线, 作为"全部校准"的 cali_buses 集合
        std::set<std::string> cali_buses_all;
        {
            canbus_sdk::ConfigParser parser;
            if (parser.parseFromFile(config_file_path)) {
                for (const auto& bus_cfg : parser.getCanBusConfigs()) {
                    cali_buses_all.insert(bus_cfg.name);
                }
            } else {
                std::cout << "\033[31m警告: 解析配置文件失败, cali_buses 为空, 将不执行校准\033[0m" << std::endl;
            }
        }

        // 创建 MotorevoActuator 实例，对配置里所有 CAN 总线执行校准，控制频率 250Hz
        motorevo::MotorevoActuator actuator(
            config_file_path,
            cali_buses_all,
            250
        );

        std::cout << "\033[32mMotorevoActuator 实例创建成功\033[0m" << std::endl;

        // 初始化执行器
        std::cout << "\033[36m正在初始化执行器...\033[0m" << std::endl;
        if (actuator.init() != 0) {
            std::cout << "\033[31m初始化失败，程序退出\033[0m" << std::endl;
            actuator.reset();
            canbus_sdk::CanBusController::getInstance().closeAllCanBuses();
            return;
        }
        std::cout << "\033[32m执行器初始化成功\033[0m" << std::endl;

        // 执行标定: init() 标定模式下不自动标定，需显式调用
        if (!actuator.calibrateAllAndSave()) {
            std::cout << "\033[31m标定失败，程序退出\033[0m" << std::endl;
            actuator.reset();
            canbus_sdk::CanBusController::getInstance().closeAllCanBuses();
            return;
        }
        std::cout << "\033[32m标定完成\033[0m" << std::endl;

        // 获取电机状态信息
        std::cout << "\033[33m=== 电机状态信息 ===\033[0m" << std::endl;
        auto positions = actuator.get_positions();
        auto velocities = actuator.get_velocity();
        auto torques = actuator.get_torque();

        std::cout << "发现 " << positions.size() << " 个电机" << std::endl;

        for (size_t i = 0; i < positions.size(); ++i) {
            std::cout << "电机 " << i << ": ";
            std::cout << "位置=" << positions[i] << " rad";
            if (i < velocities.size()) {
                std::cout << ", 速度=" << velocities[i] << " rad/s";
            }
            if (i < torques.size()) {
                std::cout << ", 力矩=" << torques[i] << " N·m";
            }
            std::cout << std::endl;
        }

        // 等待一段时间让校准完成
        std::cout << "\033[36m正在等待校准完成...\033[0m" << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(3));

        // 关闭执行器
        std::cout << "\033[36m正在关闭执行器...\033[0m" << std::endl;
        actuator.reset();
        std::cout << "\033[32m执行器已关闭\033[0m" << std::endl;

        std::cout << "\n\033[32m===============================================\033[0m" << std::endl;
        std::cout << "\033[32m      校准程序执行完成\033[0m" << std::endl;
        std::cout << "\033[32m===============================================\033[0m" << std::endl;

        // 关闭CAN总线
        canbus_sdk::CanBusController::getInstance().closeAllCanBuses();
        std::cout << "\033[32mCAN总线已关闭\033[0m" << std::endl;

    } catch (const std::exception& e) {
        std::cout << "\033[31m校准程序异常: " << e.what() << "\033[0m" << std::endl;
    }
}

////////////////////////////////////////////////////////////////////////////////////
//                                Main Function                                  //
////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char* argv[]) {
    // 如果没有参数，显示帮助信息
    if (argc == 1) {
        show_help(argv[0]);
        return 0;
    }

    // 创建命令映射表
    std::map<std::string, std::function<void(const EntryContext&)>> entries = {
        {"--negative", entry_negative_setting},
        {"--cali", entry_calibration},
        {"--help", entry_help}
    };

    // 检查第一个参数
    std::string first_arg = argv[1];
    if (entries.find(first_arg) == entries.end()) {
        std::cerr << "错误: 未知选项 '" << first_arg << "'" << std::endl;
        std::cerr << "支持的选项: --negative, --cali, --help" << std::endl;
        return 1;
    }

    // 创建上下文
    EntryContext context(argv[0], argc, argv);
    entries[first_arg](context);

    return 0;
}