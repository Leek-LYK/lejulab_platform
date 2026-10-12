#include "motorevo/motorevo_actuator.h"
#include "motorevo/motor_ctrl.h"
#include "motorevo/motor_log.h"
#include "canbus_sdk/canbus_sdk.h"
#include "canbus_sdk/config_parser.h"

#include <atomic>
#include <iostream>
#include <unistd.h>
#include <pwd.h>
#include <fstream>
#include <iomanip>
#include <algorithm>
#include <filesystem>
#include <map>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <thread>
#include <time.h>
#include <array>

namespace {

// ===== 诊断: setTargets 同步路径耗时统计 (per bus_group_index) =====
// setTargets 可能被多个上层线程调用 (sensor loop / dds cb / sine test),
// 用 relaxed atomic 实现近似累计, max 用 CAS 更新.
constexpr size_t kMaxSetTargetsStatsBuses = 8;
struct SetTargetsBusStats {
    std::atomic<uint64_t> samples{0};
    std::atomic<uint64_t> total_us{0};
    std::atomic<uint64_t> max_us{0};
};
SetTargetsBusStats g_set_targets_stats[kMaxSetTargetsStatsBuses];

constexpr uint64_t kMotorFeedbackTimeoutMs = 50;
constexpr uint8_t kDisabledConfirmFrames = 50;
constexpr uint64_t kHealthLogIntervalMs = 1000;
constexpr bool kMotorHealthStopEnabled = true;  // validated on vcan and suspended/observed hardware

const char* MotorHealthReasonName(motorevo::MotorHealthReason reason) {
    switch (reason) {
        case motorevo::MotorHealthReason::REPORTED_FAULT: return "reported_fault";
        case motorevo::MotorHealthReason::FEEDBACK_TIMEOUT: return "feedback_timeout";
        case motorevo::MotorHealthReason::UNEXPECTED_DISABLED: return "unexpected_disabled";
        case motorevo::MotorHealthReason::NONE: return "none";
    }
    return "unknown";
}

} // namespace

namespace {

// 从文件加载零点偏移, 返回 name→offset 映射.
// 支持两种格式:
//   - 新格式(name map):  arms_zero_position: {Larm_joint_01: 1.2, ...}
//     按名称匹配, 排序无关
//   - 旧格式(序列): arms_zero_position: [1.2, 3.4, ...]
//     按升级前的按 ID 顺序匹配, 首次校准后自动升级为 name-map
std::map<std::string, float> _LoadZeroOffsets(
    const std::vector<std::string>& legacy_motor_names)
{
    std::map<std::string, float> zero_map;
    std::string zero_file_path = motorevo::MotorevoActuator::getDefaultZeroFilePath();

    // 先判断零点文件是否存在，不存在则回退到另一个文件名
    if (!std::filesystem::exists(zero_file_path)) {
        std::string legacy = zero_file_path;
        size_t pos = legacy.rfind("ruiwo_zero.yaml");
        if (pos != std::string::npos) {
            legacy.replace(pos, 15, "arms_zero.yaml");
        } else {
            pos = legacy.rfind("arms_zero.yaml");
            if (pos != std::string::npos) {
                legacy.replace(pos, 15, "ruiwo_zero.yaml");
            }
        }
        if (std::filesystem::exists(legacy)) {
            RWLOG_I("Legacy zero file found: %s", legacy.c_str());
            zero_file_path = legacy;
        }
    }
    if (!std::filesystem::exists(zero_file_path)) {
        RWLOG_WARNING("Zero file does not exist: %s", zero_file_path.c_str());
        return zero_map;
    }

    try {
        YAML::Node zero_config = YAML::LoadFile(zero_file_path);
        auto arm_zero = zero_config["arms_zero_position"];
        if (!arm_zero) {
            RWLOG_WARNING("No arms_zero_position found in zero file: %s", zero_file_path.c_str());
            return zero_map;
        }

        if (arm_zero.IsMap()) {
            // 新格式: name→offset 映射, 排序无关
            for (const auto& kv : arm_zero) {
                zero_map[kv.first.as<std::string>()] = kv.second.as<float>();
            }
            RWLOG_I("Loaded %zu zero offsets (name-map) from file: %s", zero_map.size(), zero_file_path.c_str());
        } else if (arm_zero.IsSequence()) {
            // 旧格式数组是在历史 motor_refs_ 顺序下保存的，不能使用当前
            // ARM/CANFD 的 bus+id 顺序，否则升级后会把零点错配到其他电机。
            size_t n = arm_zero.size();
            for (size_t i = 0; i < n && i < legacy_motor_names.size(); ++i) {
                zero_map[legacy_motor_names[i]] = arm_zero[i].as<float>();
            }
            RWLOG_I("Loaded %zu zero offsets (legacy ID-order sequence) from file: %s", zero_map.size(), zero_file_path.c_str());
        } else {
            RWLOG_WARNING("Unexpected arms_zero_position type in zero file: %s", zero_file_path.c_str());
        }
    } catch (const YAML::Exception& e) {
        RWLOG_WARNING("Failed to load zero file %s: %s", zero_file_path.c_str(), e.what());
    } catch (...) {
        RWLOG_WARNING("Failed to load zero file: %s", zero_file_path.c_str());
    }

    return zero_map;
}

}

namespace motorevo {

MotorevoActuator::MotorevoActuator(const std::string& config_file,
                                   std::set<std::string> cali_buses,
                                   int control_frequency,
                                   bool skip_head_runtime,
                                   bool single_motor_mode)
    : config_file_(config_file)
    , cali_buses_(std::move(cali_buses))
    , running_(false)
    , control_frequency_(control_frequency)
    , skip_head_runtime_(skip_head_runtime)
    , single_motor_mode_(single_motor_mode) {
    // RWLOG_I("MotorevoActuator constructed with config file: %s, cali_buses: %zu, control frequency: %d Hz",
    //          config_file_.c_str(), cali_buses_.size(), control_frequency);
}

MotorevoActuator::~MotorevoActuator() {
    reset();
    RWLOG_I("MotorevoActuator destructor called");
}

int MotorevoActuator::init() {
    // 组装 cali_buses 打印串, 便于排障 (e.g. "bcan2,bcan3" 或 "(none)")
    std::string cali_buses_str;
    if (cali_buses_.empty()) {
        cali_buses_str = "(none)";
    } else {
        for (const auto& b : cali_buses_) {
            if (!cali_buses_str.empty()) cali_buses_str += ",";
            cali_buses_str += b;
        }
    }

    RWLOG_SUCCESS(
        "\n"
        "===============================================\n"
        "Motorevo C++ SDK\n"
        "Calibration Buses: %s\n"
        "===============================================",
        cali_buses_str.c_str()
    );

    /** 配置文件读取与解析 */
    canbus_sdk::ConfigParser parser;
    if (!parser.parseFromFile(config_file_)) { // 会抛异常(无所谓), 配置文件都解析失败,程序就没法玩了
        RWLOG_FAILURE("Init, Failed to parse config file: %s", config_file_.c_str());
        return 1;
    }

    /** 解析每条CAN总线的通信协议 (protocol字段不在CanBusConfig中, 直接读raw YAML) */
    std::unordered_map<std::string, motorevo::MotorProtocol> canbus_protocols;
    try {
        YAML::Node raw_config = YAML::LoadFile(config_file_);
        if (raw_config["canbus_interfaces"] && raw_config["canbus_interfaces"].IsMap()) {
            for (const auto& kv : raw_config["canbus_interfaces"]) {
                std::string name = kv.first.as<std::string>();
                auto bus_node = kv.second;
                if (bus_node["protocol"]) {
                    std::string proto_str = bus_node["protocol"].as<std::string>();
                    if (proto_str == "canfd_broadcast") {
                        canbus_protocols[name] = motorevo::MotorProtocol::CANFD_BROADCAST;
                        RWLOG_I("CAN总线 %s 使用 CAN FD 广播协议", name.c_str());
                    } else if (proto_str != "can_single_frame") {
                        RWLOG_WARNING("CAN总线 %s 未知协议 '%s', 将使用默认CAN单帧协议",
                                      name.c_str(), proto_str.c_str());
                    }
                }
            }
        }
    } catch (const YAML::Exception& e) {
        RWLOG_WARNING("解析protocol配置失败(将使用默认CAN单帧协议): %s", e.what());
    }

    /** 获取CAN总线配置, 没有就返回失败 */
    auto canbus_configs = parser.getCanBusConfigs();
    if (canbus_configs.empty()) {
        RWLOG_FAILURE("Init, No CAN bus configurations found in config file");
        return 1;
    }
    
    /** 循环打开CAN总线，失败则返回 */
    RWLOG_SUCCESS("============================== CAN 总线初始化开始 ==============================");
    canbus_sdk::CanBusController::getInstance().init(); // 重复初始化无所谓
    for (const auto& bus_config : canbus_configs) {
        if (!bus_config.enable) {
            RWLOG_I("CAN总线 %s 已禁用, 跳过", bus_config.name.c_str());
            continue;
        }

        // 打开CAN总线
        auto result = canbus_sdk::CanBusController::getInstance().openCanBus(
            bus_config.name.c_str(), bus_config.type, bus_config.bitrate);
        if (!result.has_value()) {
            RWLOG_FAILURE("打开CAN总线 %s 失败: %s",
                bus_config.name.c_str(),canbus_sdk::errorToString(result.error()));
            return 2;
        }

        uint8_t bus_id = result.value();
        RWLOG_SUCCESS("CAN总线 %s 打开成功, bus_id: %d", bus_config.name.c_str(), bus_id);
    }
    RWLOG_SUCCESS("============================== CAN 总线初始化完成 ==============================");

    /** 设备信息转电机配置,并绑定总线 */
    RWLOG_SUCCESS("=========================== 开始向CAN总线绑定电机设备 ===========================");
    std::vector<MotorControlRef> motor_temp_refs;
    std::map<std::pair<size_t, motorevo::MotorId>, motorevo::RevoMotorConfig_t> revo_motor_configs;
    size_t bus_groups_index = 0;
    for (const auto& canbus_config : canbus_configs) {
        if (!canbus_config.enable) {
            RWLOG_I("CAN总线 %s 已禁用, 跳过设备绑定", canbus_config.name.c_str());
            continue;
        }

        RWLOG_I("Processing CAN bus: %s", canbus_config.name.c_str());

        // 获取该CAN总线上的电机设备
        auto motor_devices = parser.getDevices(canbus_config.name, canbus_sdk::DeviceType::MOTOR);
        if (motor_devices.empty()) {
            RWLOG_WARNING("No motor devices found on CAN bus: %s", canbus_config.name.c_str());
            continue; // 没有电机设备, 跳过
        }

        RWLOG_I("Found %zu motor devices on CAN bus '%s'", motor_devices.size(), canbus_config.name.c_str());

        // 转换设备配置为RevoMotorConfig_t
        for (const auto& device : motor_devices) {
            if (device.motor_params.size() != 7) {
                RWLOG_FAILURE("电机设备 '%s' 的 motor_params 参数数量不正确 (%zu 个元素，期望 7 个), 请检查件'~/.config/lejuconfig/'下的配置文件",
                              device.name.c_str(), device.motor_params.size());
                return 1;
            }

            motorevo::MotorId id = static_cast<motorevo::MotorId>(device.device_id);
            // 初始化电机参数
            motorevo::RevoMotorConfig_t config;
            config.id = id;
            config.name = device.name;
            config.negtive = device.negtive;
            config.ignore = device.ignore;
            // 运行期跳过: 开关来自 kuavo.json 的 skip_head_runtime_comm (经构造参数传入).
            // 开启时头部关节初始化阶段正常回零, 上层控制开始下发命令(setRuntimeSkipActive)后
            // 不再发 PTM/MIT 运动帧 (policy 不控制头部).
            {
                bool is_head = (device.name.find("Head") != std::string::npos)
                            || (device.name.find("head") != std::string::npos);
                config.runtime_skip = skip_head_runtime_ && is_head;
                if (config.runtime_skip) {
                    RWLOG_WARNING("电机 '%s' 设为运行期跳过(初始化正常回零, 上层控制开始后不发运动帧) [kuavo.json:skip_head_runtime_comm]",
                                  device.name.c_str());
                }
            }
            config.zero_offset = 0.0f;      // 这里赋值只是暂时的后面加载零点文件会赋值
            config.ratio = device.ratio;     // 减速比
            config.default_params.vel = device.motor_params[0];
            config.default_params.kp_pos = device.motor_params[1];
            config.default_params.kd_pos = device.motor_params[2];
            config.default_params.tor = device.motor_params[3];
            config.default_params.kp_vel = device.motor_params[4];
            config.default_params.kd_vel = device.motor_params[5];
            config.default_params.ki_vel = device.motor_params[6];
            motor_temp_refs.push_back({bus_groups_index, id, device.name, device.ignore, config.default_params});
            revo_motor_configs[{bus_groups_index, id}] = config;
        }

        // 保存CAN总线组信息
        CanBusGroup can_group;
        can_group.name = canbus_config.name;
        auto proto_it = canbus_protocols.find(canbus_config.name);
        motorevo::MotorProtocol protocol = (proto_it != canbus_protocols.end())
            ? proto_it->second : motorevo::MotorProtocol::CAN_SINGLE_FRAME;
        can_group.motor_control = std::make_shared<motorevo::RevoMotorControl>(canbus_config.name, protocol);
        RWLOG_I("CAN总线 %s 使用协议: %s", canbus_config.name.c_str(),
                protocol == motorevo::MotorProtocol::CANFD_BROADCAST ? "CANFD_BROADCAST" : "CAN_SINGLE_FRAME");
        can_bus_groups_.push_back(can_group);
        bus_groups_index++;
    }
    RWLOG_SUCCESS("=========================== 所有电机设备绑定总线完毕 ===========================");

    if (can_bus_groups_.empty()) {
        RWLOG_FAILURE("No enabled CAN buses with motor devices found");
        return 2;
    }

    // 保存升级前的按 ID 顺序，用于兼容历史匿名数组零点文件。
    // 当前 motor_refs_ 在 ARM/CANFD 上按总线分组排序，不能用它解析旧数组。
    std::vector<MotorControlRef> legacy_sorted_refs = motor_temp_refs;
    // 与旧版本生成匿名数组时使用的 comparator 保持一致；即使重复 ID 的
    // 显示顺序不适合作为新索引，也必须用历史顺序解释已有文件。
    std::sort(legacy_sorted_refs.begin(), legacy_sorted_refs.end(),
              [](const MotorControlRef& a, const MotorControlRef& b) {
                  return a.id < b.id;
              });
    std::vector<std::string> legacy_motor_names;
    legacy_motor_names.reserve(legacy_sorted_refs.size());
    for (const auto& ref : legacy_sorted_refs) legacy_motor_names.push_back(ref.name);

    /** 索引映射构建:索引与电机ID映射 */
    buildIndexMappings(motor_temp_refs);

    /** 零点文件读取 */
    RWLOG_I("=============================== 零点文件读取开始 ===============================");
    std::map<std::string, float> zero_map = _LoadZeroOffsets(legacy_motor_names);
    if(!zero_map.empty()) {
        std::cout << "从配置文件加载零点:\n";
        for (size_t i = 0; i < motor_refs_.size(); ++i) {
            auto &ref = motor_refs_[i];
            float offset = 0.0f;         // 设置零点偏移
            auto it = zero_map.find(ref.name);
            if (it != zero_map.end()) {
                offset = it->second;
            } else if (zero_map.size() > i) {
                // 旧格式匿名数组 fallback: 按 by-id 顺序已在上层解为 name→offset
                // 若名称不在 map 中, 用 magic number
                RWLOG_WARNING("No zero offset found for motor '%s' (ID 0x%02X bus %zu), using magic number", ref.name.c_str(), ref.id, ref.bus_group_index);
                offset = kOFFSET_MAGIC_NUMBER;
            } else {
                RWLOG_WARNING("No zero offset found for motor '%s' (ID 0x%02X bus %zu), using magic number", ref.name.c_str(), ref.id, ref.bus_group_index);
                offset = kOFFSET_MAGIC_NUMBER;
            }
            revo_motor_configs[{ref.bus_group_index, ref.id}].zero_offset = offset; // 从配置文件更新零点偏移
            std::cout << " " << std::fixed << std::setprecision(6) << offset;
        }
        std::cout << std::endl;
    }
    RWLOG_I("=============================== 零点文件读取%s ===============================", zero_map.empty() ? "失败" : "成功");

    /** 控制线程启动: 在 init 之前启动 (moveToZero 依赖控制线程发帧)
     *  broadcast_ready_ 标志保证: 首次 setTargets/updateMotorCmd 前不发 MIT 帧，
     *  避免电机未使能时收到无意义的命令帧产生错误帧 */
    RWLOG_SUCCESS("=============================== 控制线程开始启动 ===============================");
    if (!startControlThread()) {
        RWLOG_FAILURE("Failed to start control thread");
        return 3;
    }
    setControlThreadAffinity(7);  // 绑定到大核 4-7 (isolcpus=4-7)
    RWLOG_SUCCESS("=============================== 控制线程启动成功 ===============================");

    // 初始化电机控制器 (enable/disable/calibrate/moveToZero)
    // init 内部的 enable/disable 通过 sendCanFdBroadcast 直接发控制帧
    // moveToZero 通过 updateMotorCmd → broadcast_ready_=true → 控制线程发 MIT 帧
    for (size_t i = 0; i < can_bus_groups_.size(); ++i) {
        const auto& group = can_bus_groups_[i];

        // 从motor_refs_中找出属于当前CAN总线的电机配置
        std::vector<motorevo::RevoMotorConfig_t> bus_motor_configs;
        for (const auto& ref : motor_refs_) {
            if (ref.bus_group_index == i) {
                bus_motor_configs.push_back(revo_motor_configs[{i, ref.id}]);
            }
        }

        // 初始化电机控制器: 只在 cali_buses_ 包含该总线时执行零点校准
        const bool bus_should_cali = cali_buses_.count(group.name) > 0;

        // 标定模式(非单电机): 推迟标定到用户按 'c' 确认后执行
        // init 阶段仅失能电机, 不做硬件清零/标定/使能/moveToZero
        // 电机保持失能状态, 用户可以安全地 Ctrl+C 退出而不会损坏硬件零点
        bool deferred_cali = bus_should_cali && !single_motor_mode_;
        bool calibrate_arg = deferred_cali ? false : bus_should_cali;
        bool enable_all_arg = deferred_cali ? false : !single_motor_mode_;
        bool move_to_zero_arg = deferred_cali ? false : !single_motor_mode_;

        if (single_motor_mode_) {
            RWLOG_WARNING("[单电机标定模式] CAN 总线 %s 跳过启动整体使能/moveToZero, 电机保持失能", group.name.c_str());
        }
        if (deferred_cali) {
            RWLOG_WARNING("[标定模式] CAN 总线 %s 跳过 init 阶段使能/标定/moveToZero, 电机保持失能, 等待用户按 'c' 确认", group.name.c_str());
        }
        if (!group.motor_control->init(bus_motor_configs, calibrate_arg, move_to_zero_arg, enable_all_arg)) {
            RWLOG_FAILURE("Failed to initialize motor controller for CAN bus: %s", group.name.c_str());
            return 3;
        }
        RWLOG_SUCCESS("Successfully initialized CAN bus: %s with %zu motors (cali=%s, enable=%s)",
                    group.name.c_str(), bus_motor_configs.size(),
                    calibrate_arg ? "True" : "False",
                    enable_all_arg ? "True" : "False");
    }

    /** 零点文件保存: 仅在零点文件原本为空(首次运行)且非标定模式时写入。
     *  标定模式下推迟保存——零点由用户交互(按 'c')确认后通过
     *  calibrateAllAndSave() 保存，避免 Ctrl+C 退出时零点已被覆盖无法撤销。 */
    {
        bool has_deferred_cali = !cali_buses_.empty();
        if (zero_map.empty() && !has_deferred_cali) {
            RWLOG_WARNING("No zero offsets found, writing new zero offsets to file");
            saveZeroToFile();
        } else if (zero_map.empty() && has_deferred_cali) {
            RWLOG_WARNING("标定模式下零点文件为空, 推迟保存到用户按 'c' 确认后");
        }
    }

    /** 完成初始化 */
    RWLOG_SUCCESS("MotorevoActuator 初始化完毕，共 %zu 个CAN总线，%zu 个电机设备", can_bus_groups_.size(), motor_refs_.size());

    // 打印设备信息表格
    std::cout << "\n=== 设备详细信息 ===\n";
    std::cout << "Index\tID\tName\tCAN Bus\tNegtive\tIgnore\tZero Offset\tParams" << std::endl;
    std::cout << std::string(132, '-') << std::endl;

    for (size_t i = 0; i < motor_refs_.size(); ++i) {
        const auto& motor_ref = motor_refs_[i];
        const auto& group = can_bus_groups_[motor_ref.bus_group_index];

        // 从motor_control获取最新的配置
        float zero_offset = 0.0f;
        bool negtive = false;
        if (group.motor_control) {
            auto config = group.motor_control->getMotorConfig(motor_ref.id);
            zero_offset = config.zero_offset;
            negtive = config.negtive;
        }

        const auto& params = motor_ref.rt_params;

        std::cout << i << "\t"
                  << static_cast<int>(motor_ref.id) << "\t"
                  << motor_ref.name << "\t"
                  << group.name << "\t"
                  << (negtive ? "\033[33mYes\033[0m" : "No") << "\t"
                  << (motor_ref.ignore ? "\033[32mYes\033[0m" : "No") << "\t"
                  << std::fixed << std::setprecision(3) << zero_offset << "\t"
                  << std::fixed << std::setprecision(2) << params.vel << ","
                  << params.kp_pos << "," << params.kd_pos << "," << params.tor << ","
                  << params.kp_vel << "," << params.kd_vel << "," << params.ki_vel << std::endl;
    }
    std::cout << std::string(132, '=') << std::endl;

    return 0;
}

std::vector<double> MotorevoActuator::getPositions() {
    if (!running_){
        return {};
    }
    std::vector<double> all_positions(motor_refs_.size(), 0.0);

    for (size_t g = 0; g < can_bus_groups_.size(); ++g) {
        auto positions = can_bus_groups_[g].motor_control->getPositions();
        for (const auto& [id, pos] : positions) {
            all_positions[id2indexs_[g][id]] = static_cast<double>(pos);
        }
    }

    return all_positions;
}

std::vector<double> MotorevoActuator::getVelocities() {
    if (!running_){
        return {};
    }
    std::vector<double> all_velocities(motor_refs_.size(), 0.0);

    for (size_t g = 0; g < can_bus_groups_.size(); ++g) {
        auto velocities = can_bus_groups_[g].motor_control->getVelocities();
        for (const auto& [id, vel] : velocities) {
            all_velocities[id2indexs_[g][id]] = static_cast<double>(vel);
        }
    }

    return all_velocities;
}

std::vector<double> MotorevoActuator::getTorques() {
    if (!running_){
        return {};
    }
    std::vector<double> all_torques(motor_refs_.size(), 0.0);

    for (size_t g = 0; g < can_bus_groups_.size(); ++g) {
        auto torques = can_bus_groups_[g].motor_control->getTorques();
        for (const auto& [id, torque] : torques) {
            all_torques[id2indexs_[g][id]] = static_cast<double>(torque);
        }
    }

    return all_torques;
}

void MotorevoActuator::setRuntimeSkipActive(bool active) {
    for (auto& group : can_bus_groups_) {
        if (group.motor_control) {
            group.motor_control->setRuntimeSkipActive(active);
        }
    }
}

void MotorevoActuator::setTargets(const std::vector<RevoMotorCmd_t>& targets) {
    if (!running_){
        return;
    }
    // 按CAN总线分组目标命令
    std::vector<std::map<motorevo::MotorId, motorevo::RevoMotorCmd_t>> bus_targets(can_bus_groups_.size());

    // 按索引分配目标命令，使用motor_refs_缓存进行O(1)访问
    for (size_t i = 0; i < targets.size() && i < motor_refs_.size(); ++i) {
        const auto& cmd = targets[i];
        const auto& motor_ref = motor_refs_[i];

        // 跳过被忽略的设备
        if (motor_ref.ignore) {
            continue;
        }

        motorevo::MotorId motor_id = motor_ref.id;

        // 直接使用motor_ref中的信息，无需查找
        motorevo::RevoMotorCmd_t revo_cmd;
        revo_cmd.pos = static_cast<float>(cmd.pos);
        revo_cmd.vel = static_cast<float>(cmd.vel);

        // 限制torque在CAN FD协议范围内 (±30 Nm)
        revo_cmd.torque = std::clamp(static_cast<float>(cmd.torque), canfd::kTorqueMin, canfd::kTorqueMax);
        revo_cmd.kp = static_cast<float>(cmd.kp);
        revo_cmd.kd = static_cast<float>(cmd.kd);

        // 直接使用bus_group_index获取对应的CanBusGroup
        bus_targets[motor_ref.bus_group_index][motor_id] = revo_cmd;
    }

    // 直接设置目标命令到控制器
    for (size_t i = 0; i < bus_targets.size(); ++i) {
        if (!bus_targets[i].empty()) {
            // ===== 诊断: 测量 setTargets 同步路径耗时 (含 write 阻塞时间) =====
            auto t0 = std::chrono::steady_clock::now();

            can_bus_groups_[i].motor_control->setTargetPositions(bus_targets[i]);
            // CAN 单帧协议: 仅更新目标, 不在此处调用 write()
            // write() 由 canControlThread 以固定频率 (250Hz) 驱动,
            // 确保底层 CAN 发送频率不受上层 DDS 发布频率影响
            // 广播协议同理, 由 canfdControlThread 独立驱动

            auto t1 = std::chrono::steady_clock::now();
            uint64_t dt_us = static_cast<uint64_t>(
                std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count());
            if (i < kMaxSetTargetsStatsBuses) {
                auto& s = g_set_targets_stats[i];
                s.samples.fetch_add(1, std::memory_order_relaxed);
                s.total_us.fetch_add(dt_us, std::memory_order_relaxed);
                uint64_t prev = s.max_us.load(std::memory_order_relaxed);
                while (dt_us > prev &&
                       !s.max_us.compare_exchange_weak(prev, dt_us,
                                                        std::memory_order_relaxed)) {
                    // 重试
                }
            }
        }
    }
}


static std::string _GetConfigRootPath() {
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
    
    return "";
}

std::string MotorevoActuator::getDefaultZeroFilePath() {
    std::string config_root = _GetConfigRootPath();
    if (config_root.empty()) {
        throw std::runtime_error("Failed to get config root path");
    }
#ifdef __aarch64__
    return config_root + "/ruiwo_zero.yaml";
#else
    return config_root + "/arms_zero.yaml";
#endif
}

bool MotorevoActuator::startControlThread() {
    if (running_) {
        RWLOG_WARNING("Control thread is already running");
        return false;
    }

    if (control_frequency_ <= 0) {
        RWLOG_FAILURE("Control frequency must be positive");
        return false;
    }

    running_ = true;

    canfd_control_thread_ = std::thread(&MotorevoActuator::canfdControlThreadFunc, this);
    can_control_thread_ = std::thread(&MotorevoActuator::canControlThreadFunc, this);

    RWLOG_SUCCESS("Control threads started: CANFD=%dHz, CAN=%dHz",
                  control_frequency_.load(), control_frequency_.load() / 2);
    return true;
}

bool MotorevoActuator::setControlThreadAffinity(int cpu_core) {
    if (!running_) {
        RWLOG_FAILURE("设置 CPU 亲和性失败, 控制线程未运行");
        return false;
    }

    auto setThreadAffinityAndPriority = [](std::thread& t, int core, int priority, const char* name) -> bool {
        if (!t.joinable()) return false;

        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(core, &cpuset);
        int rc = pthread_setaffinity_np(t.native_handle(), sizeof(cpu_set_t), &cpuset);
        if (rc != 0) {
            RWLOG_FAILURE("设置%s CPU亲和性到核心%d失败: %s", name, core, strerror(rc));
            return false;
        }

        struct sched_param sp{};
        sp.sched_priority = priority;
        rc = pthread_setschedparam(t.native_handle(), SCHED_FIFO, &sp);
        if (rc != 0) {
            RWLOG_WARNING("设置%s SCHED_FIFO(pri=%d) 失败: %s", name, priority, strerror(rc));
        } else {
            RWLOG_SUCCESS("%s 已绑定到 CPU%d, SCHED_FIFO 优先级 %d", name, core, priority);
        }
        return true;
    };

    // RK3588 大核布局: core 4(A76), 5(A76), 6(A76), 7(A76)
    // 线程分布:
    //   CANFD 控制线程 → cpu_core (如 core 7)
    //   CAN 控制线程   → cpu_core - 1 (如 core 6)
    //   CAN 发送线程   → cpu_core - 2 (如 core 5)
    //   CAN 接收线程   → cpu_core - 3 (如 core 4)
    int canfd_core  = cpu_core;
    int can_core    = std::max(4, cpu_core - 1);
    int sender_core = std::max(4, cpu_core - 2);
    int recv_core   = std::max(4, cpu_core - 3);

    // 1. CANFD 控制线程 (最高优先级, 500Hz 硬实时)
    setThreadAffinityAndPriority(canfd_control_thread_, canfd_core, 40, "CANFD控制线程");

    // 2. CAN 单帧控制线程 (略低优先级, 250Hz)
    setThreadAffinityAndPriority(can_control_thread_, can_core, 38, "CAN控制线程");

    // 3. CAN 发送线程
    if (canbus_sdk::CanBusController::getInstance().setSenderThreadAffinity(sender_core)) {
        RWLOG_SUCCESS("CAN总线发送线程已绑定到CPU核心%d", sender_core);
    } else {
        RWLOG_WARNING("设置CAN总线发送线程CPU亲和性到核心%d失败", sender_core);
    }

    // 4. CAN 接收线程
    for (size_t i = 0; i < can_bus_groups_.size(); ++i) {
        const std::string& bus_name = can_bus_groups_[i].name;
        if (canbus_sdk::CanBusController::getInstance().setRecvThreadAffinity(bus_name, recv_core)) {
            RWLOG_SUCCESS("CAN总线%s接收线程已绑定到CPU核心%d", bus_name.c_str(), recv_core);
        } else {
            RWLOG_WARNING("设置CAN总线%s接收线程CPU亲和性到核心%d失败", bus_name.c_str(), recv_core);
        }
    }

    RWLOG_SUCCESS("线程CPU分布: CANFD控制=%d, CAN控制=%d, 发送=%d, 接收=%d (isolcpus=4-7)",
                  canfd_core, can_core, sender_core, recv_core);
    return true;
}

void MotorevoActuator::canfdControlThreadFunc() {
    // 预分类: 只取 CANFD 广播总线
    std::vector<size_t> canfd_groups;
    for (size_t i = 0; i < can_bus_groups_.size(); ++i) {
        if (can_bus_groups_[i].motor_control->getProtocol() == motorevo::MotorProtocol::CANFD_BROADCAST)
            canfd_groups.push_back(i);
    }
    if (canfd_groups.empty()) {
        RWLOG_I("CANFD控制线程: 无 CANFD 广播总线, 退出");
        return;
    }

    const int freq = control_frequency_.load();
    const long period_ns = 1000000000L / freq;  // 2ms @ 500Hz
    uint32_t cycle_count = 0;

    RWLOG_SUCCESS("CANFD控制线程启动: %dHz, %zu组, 周期=%ldns", freq, canfd_groups.size(), period_ns);

    struct timespec next_time;
    clock_gettime(CLOCK_MONOTONIC, &next_time);

    while (running_) {
        next_time.tv_nsec += period_ns;
        if (next_time.tv_nsec >= 1000000000L) {
            next_time.tv_sec++;
            next_time.tv_nsec -= 1000000000L;
        }

        auto start_time = std::chrono::steady_clock::now();

        for (size_t idx : canfd_groups) {
            can_bus_groups_[idx].motor_control->write();
        }

        ++cycle_count;

        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);

        auto t_end = std::chrono::steady_clock::now();
        auto total_us = std::chrono::duration_cast<std::chrono::microseconds>(t_end - start_time).count();
        if (total_us > period_ns / 1000 * 2) {
            clock_gettime(CLOCK_MONOTONIC, &next_time);
            RWLOG_W("[CANFD] overrun: total=%ldus, resync, cycle=%u", total_us, cycle_count);
        }

        // 诊断日志 (默认编译关闭; 开启: cmake -DLEJU_ENABLE_DIAG_FILELOG=ON)
#ifdef LEJU_ENABLE_DIAG_FILELOG
        {
            static FILE* logf = fopen("/tmp/canfd_thread_timing.log", "w");
            static auto last_print = std::chrono::steady_clock::now() - std::chrono::seconds(2);
            if (logf && std::chrono::duration_cast<std::chrono::seconds>(t_end - last_print).count() >= 2) {
                fprintf(logf, "[CANFD] write=%ldus total=%ldus cycle=%u freq=%dHz\n",
                        std::chrono::duration_cast<std::chrono::microseconds>(t_end - start_time).count()
                            - std::max(0L, total_us - period_ns / 1000),
                        total_us, cycle_count, freq);
                fflush(logf);
                last_print = t_end;
            }
        }
#endif
    }

    RWLOG_SUCCESS("CANFD控制线程退出");
}

void MotorevoActuator::canControlThreadFunc() {
    // 预分类: 只取 CAN 单帧总线
    std::vector<size_t> can_groups;
    for (size_t i = 0; i < can_bus_groups_.size(); ++i) {
        if (can_bus_groups_[i].motor_control->getProtocol() != motorevo::MotorProtocol::CANFD_BROADCAST)
            can_groups.push_back(i);
    }
    if (can_groups.empty()) {
        RWLOG_I("CAN控制线程: 无 CAN 单帧总线, 退出");
        return;
    }

    // 手臂频率: 默认为 CANFD 频率的一半, 可通过环境变量 CAN_ARM_FREQ 覆盖
    int freq = control_frequency_.load() / 2;
    const char* arm_freq_env = std::getenv("CAN_ARM_FREQ");
    if (arm_freq_env) {
        int env_freq = std::atoi(arm_freq_env);
        if (env_freq >= 50 && env_freq <= 1000) {
            freq = env_freq;
        }
    }
    const long period_ns = 1000000000L / freq;  // 4ms @ 250Hz
    uint32_t cycle_count = 0;

    RWLOG_SUCCESS("CAN控制线程启动: %dHz, %zu组, 周期=%ldns", freq, can_groups.size(), period_ns);

    struct timespec next_time;
    clock_gettime(CLOCK_MONOTONIC, &next_time);

    while (running_) {
        next_time.tv_nsec += period_ns;
        if (next_time.tv_nsec >= 1000000000L) {
            next_time.tv_sec++;
            next_time.tv_nsec -= 1000000000L;
        }

        auto start_time = std::chrono::steady_clock::now();

        for (size_t idx : can_groups) {
            can_bus_groups_[idx].motor_control->write();
        }

        ++cycle_count;

        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);

        auto t_end = std::chrono::steady_clock::now();
        auto total_us = std::chrono::duration_cast<std::chrono::microseconds>(t_end - start_time).count();
        if (total_us > period_ns / 1000 * 2) {
            clock_gettime(CLOCK_MONOTONIC, &next_time);
            RWLOG_W("[CAN] overrun: total=%ldus, resync, cycle=%u", total_us, cycle_count);
        }

        // 诊断日志 (默认编译关闭; 开启: cmake -DLEJU_ENABLE_DIAG_FILELOG=ON)
#ifdef LEJU_ENABLE_DIAG_FILELOG
        {
            static FILE* logf = fopen("/tmp/can_thread_timing.log", "w");
            static auto last_print = std::chrono::steady_clock::now() - std::chrono::seconds(2);
            if (logf && std::chrono::duration_cast<std::chrono::seconds>(t_end - last_print).count() >= 2) {
                fprintf(logf, "[CAN] write=%ldus total=%ldus cycle=%u freq=%dHz\n",
                        std::chrono::duration_cast<std::chrono::microseconds>(t_end - start_time).count()
                            - std::max(0L, total_us - period_ns / 1000),
                        total_us, cycle_count, freq);
                fflush(logf);
                last_print = t_end;
            }
        }

        // ===== 诊断: 手臂路径综合日志 (setTargets 同步耗时 + writeSingleFrame 命中/耗时) =====
        {
            static FILE* arm_logf = fopen("/tmp/arm_diag.log", "w");
            static auto last_arm_print = std::chrono::steady_clock::now() - std::chrono::seconds(2);
            if (arm_logf && std::chrono::duration_cast<std::chrono::seconds>(t_end - last_arm_print).count() >= 2) {
                int64_t ts_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                    t_end.time_since_epoch()).count();
                fprintf(arm_logf, "===== t=%ld ms =====\n", (long)ts_ms);
                for (size_t gi = 0; gi < can_groups.size(); ++gi) {
                    size_t i = can_groups[gi];
                    const auto& grp = can_bus_groups_[i];

                    // setTargets 统计
                    uint64_t st_n  = g_set_targets_stats[i].samples.load(std::memory_order_relaxed);
                    uint64_t st_tu = g_set_targets_stats[i].total_us.load(std::memory_order_relaxed);
                    uint64_t st_mx = g_set_targets_stats[i].max_us.load(std::memory_order_relaxed);
                    double st_avg = st_n > 0 ? (double)st_tu / (double)st_n : 0.0;

                    // writeSingleFrame 统计
                    auto wf = grp.motor_control->getSingleFrameWriteStats();
                    uint64_t wf_n = wf.samples;
                    double wf_avg = wf_n > 0 ? (double)wf.total_us / (double)wf_n : 0.0;
                    uint64_t total_write_calls = wf.hit + wf.miss;
                    double hit_rate = total_write_calls > 0
                                      ? 100.0 * (double)wf.hit / (double)total_write_calls
                                      : 0.0;

                    // write() 来源统计
                    uint64_t from_st = grp.motor_control->hit_from_setTargets_.load(std::memory_order_relaxed);
                    uint64_t from_ct = grp.motor_control->hit_from_canThread_.load(std::memory_order_relaxed);

                    fprintf(arm_logf,
                            "  [%-10s] setTargets: n=%-8lu avg=%6.1fus max=%6luus | "
                            "writeFrame: hit=%-8lu miss=%-8lu hit%%=%5.1f avg=%6.1fus max=%6luus | "
                            "from: setTargets=%-8lu canThread=%-8lu\n",
                            grp.name.c_str(),
                            (unsigned long)st_n, st_avg, (unsigned long)st_mx,
                            (unsigned long)wf.hit, (unsigned long)wf.miss, hit_rate,
                            wf_avg, (unsigned long)wf.max_us,
                            (unsigned long)from_st, (unsigned long)from_ct);
                }
                fflush(arm_logf);
                last_arm_print = t_end;
            }
        }
#endif
    }

    RWLOG_SUCCESS("CAN控制线程退出");
}

bool MotorevoActuator::enableAll() {
    if (!running_){
        RWLOG_FAILURE("enableAll failed: actuator not running");
        return false;
    }
    RWLOG_I("Enabling all motors...");

    std::vector<bool> result;
    for (size_t i = 0; i < can_bus_groups_.size(); ++i) {
        const auto& group = can_bus_groups_[i];
        bool ret = group.motor_control->enableAll();
        result.push_back(ret);
    }

    // 如果全失败返回false
    return std::all_of(result.begin(), result.end(), [](bool success) { return success; });
}

bool MotorevoActuator::disableAll() {
    if (!running_){
        RWLOG_FAILURE("disableAll failed: actuator not running");
        return false;
    }
    RWLOG_I("Disabling all motors...");

    std::vector<bool> result;
    for (size_t i = 0; i < can_bus_groups_.size(); ++i) {
        const auto& group = can_bus_groups_[i];
        bool ret = group.motor_control->disableAll();
        result.push_back(ret);
    }

    // 如果全失败返回false
    return std::all_of(result.begin(), result.end(), [](bool success) { return success; });
}

void MotorevoActuator::buildIndexMappings(const std::vector<MotorControlRef>& motor_refs) {
    // 清空现有映射
    motor_refs_.clear();

    // 初始化id2indexs_二维数组，填充无效值
    for (size_t b = 0; b < MAX_BUS_GROUPS; ++b) {
        for (size_t i = 0; i <= MAX_MOTOR_ID; ++i) {
            id2indexs_[b][i] = static_cast<size_t>(-1);
        }
    }

    // motor_refs_ 的顺序同时用于标定工具的序号显示和各类索引映射。
    // ARM/CANFD 机型的不同总线会复用相同的电机 ID，必须先按总线再按
    // ID 排序；否则仅按 ID 排序会把左右腿/手臂交错，且相同 ID 的顺序
    // 还可能随 std::sort 实现变化，导致标定界面序号与名称混乱。
    // x86/EC_MASTER 保留历史上的按 ID 顺序，兼容旧版序列零点文件。
    std::vector<MotorControlRef> sorted_refs = motor_refs;
#if defined(__aarch64__) || defined(__ARM_ARCH)
    std::sort(sorted_refs.begin(), sorted_refs.end(),
              [](const MotorControlRef& a, const MotorControlRef& b) {
                  if (a.bus_group_index != b.bus_group_index) {
                      return a.bus_group_index < b.bus_group_index;
                  }
                  return a.id < b.id;
              });
#else
    std::stable_sort(sorted_refs.begin(), sorted_refs.end(),
                     [](const MotorControlRef& a, const MotorControlRef& b) {
                         return a.id < b.id;
                     });
#endif

    // 构建映射关系：id2indexs_[bus_group][motor_id] = index
    for (size_t i = 0; i < sorted_refs.size(); ++i) {
        auto& ref = sorted_refs[i];
        id2indexs_[ref.bus_group_index][ref.id] = i;
        motor_refs_.push_back(ref);
    }
    health_log_states_.assign(motor_refs_.size(), HealthLogState{});
    health_issue_seen_.assign(motor_refs_.size(), 0);
    health_issues_scratch_.clear();
    health_issues_scratch_.reserve(8);
}

void MotorevoActuator::saveZeroToFile() {
    std::string config_path = getDefaultZeroFilePath();
    std::string backup_path = config_path + ".bak";

    // 收集所有电机的零点偏移值, 按索引收集后用名称保存 (name→offset 映射)
    std::vector<float> zero_offsets(motor_refs_.size(), 0.0f);

    for (size_t g = 0; g < can_bus_groups_.size(); ++g) {
        auto offsets = can_bus_groups_[g].motor_control->getZeroOffsets();
        for (const auto& [id, offset] : offsets) {
            zero_offsets[id2indexs_[g][id]] = offset;
        }
    }

    // 用电机名称作为 key, offset 作为 value (name→offset 格式)
    // 此格式与排序无关, 跨平台通用
    YAML::Node config;
    YAML::Node ym_zero_position;
    for (size_t i = 0; i < zero_offsets.size(); ++i) {
        ym_zero_position[motor_refs_[i].name] = zero_offsets[i];
    }
    config["arms_zero_position"] = ym_zero_position;

    // 备份配置文件
    if (std::filesystem::exists(config_path)) {
        std::filesystem::copy_file(config_path, backup_path, std::filesystem::copy_options::overwrite_existing);
        RWLOG_I("Backup zero file to %s", backup_path.c_str());
    }

    // 保存配置文件
    std::ofstream file(config_path);
    if (file.is_open()) {
        file << config;
        file.close();
        RWLOG_SUCCESS("Zero position saved to %s", config_path.c_str());
    } else {
        RWLOG_FAILURE("Failed to open zero file for writing: %s", config_path.c_str());
    }
}

void MotorevoActuator::reset() {
    RWLOG_I("Closing MotorevoActuator...");

    // 停止控制线程
    if (!running_) {
        return;
    }

    // 先掉使能所有电机（控制线程还在运行，CAN 收发正常）
    RWLOG_I("Disabling all motors before shutdown...");
    disableAll();

    running_ = false;

    if (canfd_control_thread_.joinable()) {
        canfd_control_thread_.join();
    }
    if (can_control_thread_.joinable()) {
        can_control_thread_.join();
    }

    RWLOG_SUCCESS("Control threads stopped");

    // 清空CAN总线组，手动释放motor_control
    for (auto& group : can_bus_groups_) {
        if (group.motor_control) {
            group.motor_control.reset();
            group.motor_control = nullptr;
        }
    }
    can_bus_groups_.clear();
    motor_refs_.clear();
    health_log_states_.clear();
    health_issue_seen_.clear();
    health_issues_scratch_.clear();
    for (size_t b = 0; b < MAX_BUS_GROUPS; ++b) {
        for (size_t i = 0; i <= MAX_MOTOR_ID; ++i) {
            id2indexs_[b][i] = static_cast<size_t>(-1);
        }
    }

    RWLOG_SUCCESS("RuiwoActuator closed successfully");
}

/******************************************************************************/
/*                          基类接口兼容 (MotorActuatorBase)                  */
/******************************************************************************/

int MotorevoActuator::initialize() {
    return init();
}

int MotorevoActuator::enable() {
    if(!enableAll()) {
        return 1;
    }
    return 0;
}

int MotorevoActuator::disable() {
    if(!disableAll()) {
        return 1;
    }
    return 0;
}

void MotorevoActuator::close() {
    reset();
}

bool MotorevoActuator::disableMotor(int motorIndex) {
    // 检查索引是否有效
    if (motorIndex < 0 || static_cast<size_t>(motorIndex) >= motor_refs_.size()) {
        RWLOG_W("disableMotor Invalid motor index: %d (range: 0-%zu)", motorIndex, motor_refs_.size() - 1);
        return false;
    }

    // 直接从motor_refs_获取对应的电机信息
    const auto& motor_ref = motor_refs_[motorIndex];
    if (motor_ref.ignore) {
        RWLOG_W("Motor at index %d is ignored, cannot disable", motorIndex);
        return true;
    }

    // 获取对应的电机ID和CanBusGroup
    motorevo::MotorId motor_id = motor_ref.id;
    const auto& group = can_bus_groups_[motor_ref.bus_group_index];

    RWLOG_I("Disabling motor at index %d (ID: %d) on CAN总线 %s", motorIndex, motor_id, group.name.c_str());

    int result = group.motor_control->disableMotor(motor_id);
    if (result == 0) {
        RWLOG_SUCCESS("Motor %d disabled successfully", motor_id);
        return true;
    } else {
        RWLOG_FAILURE("Motor %d disable failed with ret code: %d", motor_id, result);
    }
    
    return false;
}

std::vector<double> MotorevoActuator::get_positions() {
    return getPositions();
}

std::vector<double> MotorevoActuator::get_torque() {
    return getTorques();
}

std::vector<double> MotorevoActuator::get_velocity() {
    return getVelocities();
}

void MotorevoActuator::get_all_data(std::vector<double>& pos,
                                     std::vector<double>& vel,
                                     std::vector<double>& torque) {
    if (!running_) return;
    size_t n = motor_refs_.size();
    pos.resize(n, 0.0);
    vel.resize(n, 0.0);
    torque.resize(n, 0.0);

    thread_local std::vector<motorevo::RevoJointSnapshot> snapshot_buf;

    for (size_t g = 0; g < can_bus_groups_.size(); ++g) {
        can_bus_groups_[g].motor_control->getAllJointData(snapshot_buf);
        for (size_t i = 0; i < snapshot_buf.size(); ++i) {
            auto idx = id2indexs_[g][snapshot_buf[i].id];
            pos[idx]    = static_cast<double>(snapshot_buf[i].position);
            vel[idx]    = static_cast<double>(snapshot_buf[i].velocity);
            torque[idx] = static_cast<double>(snapshot_buf[i].torque);
        }
    }
}

std::vector<std::string> MotorevoActuator::getMotorRefNames() const {
    std::vector<std::string> names;
    names.reserve(motor_refs_.size());
    for (const auto& ref : motor_refs_) {
        names.push_back(ref.name);
    }
    return names;
}

void MotorevoActuator::saveAsZeroPosition() {
    return saveZeroToFile();
}

void MotorevoActuator::saveZeroPosition() {
    return saveZeroToFile();
}

void MotorevoActuator::set_teach_pendant_mode(int mode) {
    // 设置0-torque模式
    bool enable = (mode == 1);  // 1：示教模式，0：非示教模式

    // 通知所有CAN总线组设置0-torque模式
    for (auto& bus_group : can_bus_groups_) {
        if (bus_group.motor_control) {
            bus_group.motor_control->setZeroTorqueMode(enable);
        }
    }

    RWLOG_I("Teach pendant mode %s (0-torque mode %s)",
            enable ? "enabled" : "disabled",
            enable ? "enabled" : "disabled");
}

void MotorevoActuator::changeEncoderZeroRound(int index, double direction) {
    if (!running_) {
        RWLOG_FAILURE("changeEncoderZeroRound failed: actuator not running");
        return;
    }

    // 检查关节索引是否有效
    if (index < 0 || static_cast<size_t>(index) >= motor_refs_.size()) {
        RWLOG_W("changeEncoderZeroRound Invalid joint index: %d (range: 0-%zu)", index, motor_refs_.size() - 1);
        return;
    }

    // 获取电机引用
    auto& motor_ref = motor_refs_[index];
    auto& can_group = can_bus_groups_[motor_ref.bus_group_index];

    if (!can_group.motor_control) {
        RWLOG_W("Motor control not available for joint %d", index);
        return;
    }

    // 从motor_control获取当前配置
    auto config = can_group.motor_control->getMotorConfig(motor_ref.id);

    // 计算编码器零点修正值
    float round = 360.0f / (config.ratio) * M_PI / 180.0f;

    // 根据方向调整零点偏移
    if (direction > 0) {
        config.zero_offset += round;
    } else {
        config.zero_offset -= round;
    }

    // 更新电机控制器中的配置
    can_group.motor_control->setMotorConfig(motor_ref.id, config);

    RWLOG_I("ChangeEncoderZeroRound joint %d, direction %.1f, ratio %.1f, round %.3f, new offset: %.3f",
            index, direction, config.ratio, round, config.zero_offset);
}

void MotorevoActuator::adjustZeroPosition(int index, double offset) {
    if (!running_) {
        RWLOG_FAILURE("adjustZeroPosition failed: actuator not running");
        return;
    }
    // 检查关节索引是否有效
    if (index < 0 || static_cast<size_t>(index) >= motor_refs_.size()) {
        RWLOG_W("adjustZeroPosition Invalid joint index: %d (range: 0-%zu)", index, motor_refs_.size() - 1);
        return;
    }

    // 获取电机引用
    auto& motor_ref = motor_refs_[index];
    auto& can_group = can_bus_groups_[motor_ref.bus_group_index];

    if (!can_group.motor_control) {
        RWLOG_W("Motor control not available for joint %d", index);
        return;
    }

    // 从motor_control获取当前配置
    auto config = can_group.motor_control->getMotorConfig(motor_ref.id);

    // 调整零点偏移
    config.zero_offset += static_cast<float>(offset);

    // 更新电机控制器中的配置
    can_group.motor_control->setMotorConfig(motor_ref.id, config);

    RWLOG_I("Adjusted zero position for joint %d by %.3f, new offset: %.3f",
            index, offset, config.zero_offset);
}

bool MotorevoActuator::calibrateSingleMotor(int index) {
    if (index < 0 || static_cast<size_t>(index) >= motor_refs_.size()) {
        RWLOG_W("calibrateSingleMotor Invalid joint index: %d (range: 0-%zu)", index, motor_refs_.size() - 1);
        return false;
    }

    auto& motor_ref = motor_refs_[index];
    auto& can_group = can_bus_groups_[motor_ref.bus_group_index];
    if (!can_group.motor_control) {
        RWLOG_W("Motor control not available for joint %d", index);
        return false;
    }

    if (!can_group.motor_control->calibrateSingleMotor(motor_ref.id)) {
        RWLOG_FAILURE("calibrateSingleMotor failed for joint %d (%s)", index, motor_ref.name.c_str());
        return false;
    }

    // 只重写当前所有电机的实时零点偏移(其它电机内存中的值未被改动), 落盘安全
    saveZeroToFile();
    return true;
}

bool MotorevoActuator::calibrateAllAndSave() {
    RWLOG_I("========== 开始对所有标定总线执行完整标定 ==========");

    for (auto& group : can_bus_groups_) {
        if (cali_buses_.count(group.name) == 0) {
            RWLOG_I("CAN 总线 %s 不在标定范围, 跳过", group.name.c_str());
            continue;
        }

        RWLOG_I("正在标定 CAN 总线: %s", group.name.c_str());
        if (!group.motor_control->calibrateAllMotorsAndHold()) {
            RWLOG_FAILURE("CAN 总线 %s 标定失败", group.name.c_str());
            return false;
        }
        RWLOG_SUCCESS("CAN 总线 %s 标定完成", group.name.c_str());
    }

    // 所有标定总线完成后统一保存到零点文件
    saveZeroToFile();
    RWLOG_SUCCESS("========== 所有标定总线标定并保存完成 ==========");
    return true;
}

std::vector<double> MotorevoActuator::getMotorZeroPoints() {
    std::vector<double> zero_points(motor_refs_.size(), 0.0);

    // 从motor_control获取零点偏移值
    for (size_t g = 0; g < can_bus_groups_.size(); ++g) {
        auto configs = can_bus_groups_[g].motor_control->getZeroOffsets();
        for (const auto& [id, offset] : configs) {
            // 根据电机ID找到对应的索引
            zero_points[id2indexs_[g][id]] = static_cast<double>(offset);
        }
    }

    return zero_points;
}

RuiwoActuatorBase::MotorStateDataVec MotorevoActuator::get_motor_state() {
    MotorStateDataVec motor_states(motor_refs_.size());

    // 初始化，所有电机默认为忽略状态
    for (size_t i = 0; i < motor_refs_.size(); ++i) {
        motor_states[i] = {static_cast<uint8_t>(motor_refs_[i].id), State::Ignored};
    }

    // 从motor_control获取电机状态
    for (size_t g = 0; g < can_bus_groups_.size(); ++g) {
        auto revo_motor_states = can_bus_groups_[g].motor_control->getMotorStates();
        for (const auto& [id, motor_state] : revo_motor_states) {
            // 根据电机ID找到对应的索引
            size_t index = id2indexs_[g][id];

            // 将MotorState映射到State
            State state = (motor_state == motorevo::MotorState::MotorSate) ? State::Enabled : State::Disabled;

            // 如果电机被标记为忽略，则保持忽略状态
            if (!motor_refs_[index].ignore) {
                motor_states[index] = {static_cast<uint8_t>(id), state};
            }
        }
    }

    return motor_states;
}

void MotorevoActuator::get_all_status(std::vector<uint8_t>& status_codes) {
    if (!running_) { status_codes.clear(); return; }

    size_t n = motor_refs_.size();
    status_codes.assign(n, 0);

    constexpr uint64_t kFeedbackTimeoutMs = 1000;

    auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();

    for (size_t g = 0; g < can_bus_groups_.size(); ++g) {
        // 只检测下肢总线(bcan0/bcan1)，手臂/头总线正常也超时
        const auto& name = can_bus_groups_[g].name;
        if (name.find("bcan0") == std::string::npos &&
            name.find("bcan1") == std::string::npos) continue;

        uint64_t last = can_bus_groups_[g].motor_control->getLastFeedbackTimeMs();
        if (last == 0 || now_ms - last <= kFeedbackTimeoutMs) continue;

        for (size_t i = 0; i < n; ++i) {
            if (motor_refs_[i].bus_group_index == g && !motor_refs_[i].ignore)
                status_codes[i] = 1;
        }

        // 总线已失联：停机时仍发送disable，但不再逐电机等待反馈确认。
        can_bus_groups_[g].motor_control->setSkipDisableWait(true);
    }
}

bool MotorevoActuator::check_lowerlimb_fault(std::string& fault_info, bool ready_ok) {
    if (!running_) return false;

    const uint64_t now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    std::fill(health_issue_seen_.begin(), health_issue_seen_.end(), 0);

    bool has_protective_issue = false;
    std::array<bool, MAX_BUS_GROUPS> should_skip_disable{};

    for (size_t g = 0; g < can_bus_groups_.size(); ++g) {
        const auto& group = can_bus_groups_[g];
        if (group.name.find("bcan0") == std::string::npos &&
            group.name.find("bcan1") == std::string::npos) continue;

        const auto summary = group.motor_control->getMotorHealthIssues(
            now_ms, ready_ok, kMotorFeedbackTimeoutMs, kDisabledConfirmFrames,
            health_issues_scratch_);
        if (summary.armed_count > 0
            && summary.timed_out_armed_count == summary.armed_count
            && g < should_skip_disable.size()) {
            should_skip_disable[g] = true;
        }

        for (const auto& issue : health_issues_scratch_) {
            size_t index = static_cast<size_t>(-1);
            if (g < MAX_BUS_GROUPS && issue.id <= MAX_MOTOR_ID) {
                index = id2indexs_[g][issue.id];
            }
            const char* joint_name = "unknown";
            if (index < motor_refs_.size()) {
                joint_name = motor_refs_[index].name.c_str();
                health_issue_seen_[index] = 1;
            }

            const bool log_due = index >= health_log_states_.size()
                || health_log_states_[index].reason != issue.reason
                || now_ms - health_log_states_[index].last_log_ms >= kHealthLogIntervalMs;
            if (log_due) {
                RWLOG_W("[MotorUnavailable] bus=%s id=0x%02X joint=%s reason=%s "
                        "feedback_age=%lums status=%s0x%04X fault=0x%02X stop_enabled=%d",
                        group.name.c_str(), issue.id, joint_name,
                        MotorHealthReasonName(issue.reason),
                        static_cast<unsigned long>(issue.feedback_age_ms),
                        issue.status_word_valid ? "" : "invalid/",
                        issue.status_word, static_cast<uint8_t>(issue.fault_code),
                        kMotorHealthStopEnabled ? 1 : 0);
                if (index < health_log_states_.size()) {
                    health_log_states_[index].reason = issue.reason;
                    health_log_states_[index].last_log_ms = now_ms;
                }
            }

            const bool protective = issue.reason == MotorHealthReason::REPORTED_FAULT
                || kMotorHealthStopEnabled;
            if (protective) {
                has_protective_issue = true;
                char buf[256];
                snprintf(buf, sizeof(buf),
                         "[MotorUnavailable] bus=%s id=0x%02X joint=%s reason=%s "
                         "feedback_age=%lums status=%s0x%04X fault=0x%02X\n",
                         group.name.c_str(), issue.id, joint_name,
                         MotorHealthReasonName(issue.reason),
                         static_cast<unsigned long>(issue.feedback_age_ms),
                         issue.status_word_valid ? "" : "invalid/",
                         issue.status_word, static_cast<uint8_t>(issue.fault_code));
                fault_info += buf;
            }
        }
    }

    for (size_t i = 0; i < health_log_states_.size(); ++i) {
        if (!health_issue_seen_[i]) {
            health_log_states_[i] = HealthLogState{};
        }
    }

    if (has_protective_issue) {
        for (size_t g = 0; g < can_bus_groups_.size() && g < should_skip_disable.size(); ++g) {
            if (should_skip_disable[g]) {
                can_bus_groups_[g].motor_control->setSkipDisableWait(true);
            }
        }
    }
    return has_protective_issue;
}

void MotorevoActuator::set_positions(const std::vector<uint8_t> &index,
                                   const std::vector<double> &positions,
                                   const std::vector<double> &torque,
                                   const std::vector<double> &velocity,
                                   const std::vector<double> &kp_pos,
                                   const std::vector<double> &kd_pos) {
    // 检查是否在运行状态
    if (!running_) {
        // printf("Actuator is not running, cannot set positions\n");
        return;
    }

    // 检查参数维度是否一致
    if (index.size() != positions.size() || index.size() != torque.size() ||
        index.size() != velocity.size()) {
        printf("参数维度不一致: index(%zu), positions(%zu), torque(%zu), velocity(%zu)",
                      index.size(), positions.size(), torque.size(), velocity.size());
        return;
    }

    // 创建目标命令数组，初始化为默认值
    std::vector<RevoMotorCmd_t> targets(motor_refs_.size(), {0.0, 0.0, 0.0, 0.0, 0.0});

    // 按索引设置目标值
    for (size_t i = 0; i < index.size(); ++i) {
        uint8_t idx = index[i];

        // 检查索引有效性
        if (idx >= motor_refs_.size()) {
            printf("set_positions Invalid index %u at position %zu, skipping", idx, i);
            continue;
        }

        targets[idx].pos = positions[i] * M_PI / 180.0;  // 角度转弧度
        targets[idx].vel = velocity[i] * M_PI / 180.0;  // 角度/秒 转 弧度/秒
        targets[idx].torque = torque[i];

        // 逐周期 kp/kd: 上层传入值时优先使用，否则用缓存配置
        const bool use_cmd_gains = !kp_pos.empty() && !kd_pos.empty();
        if (use_cmd_gains) {
            targets[idx].kp = static_cast<float>(kp_pos[i]);
            targets[idx].kd = static_cast<float>(kd_pos[i]);
        } else {
            const auto& motor_ref = motor_refs_[idx];
            targets[idx].kp = motor_ref.rt_params.kp_pos;
            targets[idx].kd = motor_ref.rt_params.kd_pos;
        }
    }

    // 调用现有接口设置目标
    setTargets(targets);
}

void MotorevoActuator::set_torque(const std::vector<uint8_t> &index,
                                 const std::vector<double> &torque) {
    // 检查是否在运行状态
    if (!running_) {
        // printf("Actuator is not running, cannot set torque\n");
        return;
    }

    // 检查输入维度是否一致
    if (index.size() != torque.size()) {
        RWLOG_FAILURE("Parameter size mismatch: index(%zu), torque(%zu)",
                      index.size(), torque.size());
        return;
    }

    // TODO 实现具体的扭矩设置逻辑
    RWLOG_W("set_torque not fully implemented yet");
}

void MotorevoActuator::set_velocity(const std::vector<uint8_t> &index,
                                   const std::vector<double> &velocity) {
    // 检查是否在运行状态
    if (!running_) {
        // printf("Actuator is not running, cannot set velocity\n");
        return;
    }

    // 检查输入维度是否一致
    if (index.size() != velocity.size()) {
        RWLOG_FAILURE("Parameter size mismatch: index(%zu), velocity(%zu)",
                      index.size(), velocity.size());
        return;
    }

    // TODO 实现具体的速度设置逻辑
    RWLOG_W("set_velocity not fully implemented yet");
}

void MotorevoActuator::set_joint_gains(const std::vector<int> &joint_indices,
                                     const std::vector<double> &kp_pos,
                                     const std::vector<double> &kd_pos) {
    // 检查参数维度是否一致
    if (kp_pos.size() != joint_indices.size() ||
        kd_pos.size() != joint_indices.size()) {
        RWLOG_WARNING("Parameter size mismatch: joint_indices(%zu), kp_pos(%zu), kd_pos(%zu)",
                      joint_indices.size(), kp_pos.size(), kd_pos.size());
        return;
    }

    // 按关节索引设置增益参数
    for (size_t i = 0; i < joint_indices.size(); ++i) {
        int joint_idx = joint_indices[i];

        // 检查关节索引是否有效
        if (joint_idx < 0 || static_cast<size_t>(joint_idx) >= motor_refs_.size()) {
            RWLOG_W("set_joint_gains Invalid joint index: %d (range: 0-%zu)", joint_idx, motor_refs_.size() - 1);
            continue;
        }

        // 更新增益参数 (仅在值变化时打印)
        auto& motor_ref = motor_refs_[joint_idx];
        float new_kp = static_cast<float>(kp_pos[i]);
        float new_kd = static_cast<float>(kd_pos[i]);
        if (motor_ref.rt_params.kp_pos != new_kp || motor_ref.rt_params.kd_pos != new_kd) {
            RWLOG_I("Set joint %d kp_pos=%.2f kd_pos=%.2f", joint_idx, kp_pos[i], kd_pos[i]);
            motor_ref.rt_params.kp_pos = new_kp;
            motor_ref.rt_params.kd_pos = new_kd;
        }
    }
}

std::vector<std::vector<double>> MotorevoActuator::get_joint_gains(const std::vector<int> &joint_indices) {
    std::vector<double> kp_pos_values;
    std::vector<double> kd_pos_values;

    // 确定要获取的关节索引
    std::vector<int> indices_to_get;
    if (joint_indices.empty()) {
        // 获取所有关节的增益参数
        indices_to_get.resize(motor_refs_.size());
        for (size_t i = 0; i < motor_refs_.size(); ++i) {
            indices_to_get[i] = static_cast<int>(i);
        }
    } else {
        indices_to_get = joint_indices;
    }

    // 获取指定关节的增益参数
    for (int joint_idx : indices_to_get) {
        // 检查关节索引是否有效
        if (joint_idx < 0 || static_cast<size_t>(joint_idx) >= motor_refs_.size()) {
            RWLOG_W("get_joint_gains Invalid joint index: %d (range: 0-%zu)", joint_idx, motor_refs_.size() - 1);
            continue;
        }

        // 获取电机控制引用
        const auto& motor_ref = motor_refs_[joint_idx];
        kp_pos_values.push_back(static_cast<double>(motor_ref.rt_params.kp_pos));
        kd_pos_values.push_back(static_cast<double>(motor_ref.rt_params.kd_pos));
    }

    return {kp_pos_values, kd_pos_values};
}

} // namespace motorevo
