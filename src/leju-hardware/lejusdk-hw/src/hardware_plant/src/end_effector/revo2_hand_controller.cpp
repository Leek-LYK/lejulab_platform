// cppcheck-suppress-file toomanyconfigs
#include <algorithm>
#include "revo2_hand_controller.h"
#include "dexhand_def.h"
#include "revo2_hand.h"
#include "stark_dexhand.h"
#include "revo2_hand_can_customed.h"
#include "canbus_sdk/canbus_sdk.h"
#include "canbus_sdk/config_parser.h"
#include <iostream>
#include <fstream>
#include "utils/json.hpp"

namespace eef_controller {
using namespace dexhand;

// 辅助函数：检查FingerStatus是否全部为0
bool isFingerStatusAllZeros(const FingerStatus& status) {
    return std::all_of(status.positions.begin(), status.positions.end(), [](uint16_t val) { return val == 0; }) &&
           std::all_of(status.speeds.begin(), status.speeds.end(), [](int16_t val) { return val == 0; }) &&
           std::all_of(status.currents.begin(), status.currents.end(), [](int16_t val) { return val == 0; }) &&
           std::all_of(status.states.begin(), status.states.end(), [](uint16_t val) { return val == 0; });
}

// 双手张开位置
const UnsignedDualHandsArray kDualHandOpenPositions = {kOpenFingerPositions, kOpenFingerPositions};
// 双手闭合位置
const UnsignedDualHandsArray kDualHandClosePositions = {kCloseFingerPositions, kCloseFingerPositions};     



bool Revo2HandController::ParseActionSequenceFile(const std::string & gesture_file_path, std::vector<GestureInfoPtr> &gesture_infos)
{
    nlohmann::json data;
    try {
        std::ifstream file(gesture_file_path);
        if (!file.is_open()) {
            std::cerr << "Failed to open gesture config file: " << gesture_file_path << std::endl;
            return false;
        }

        file >> data;

        for (auto &item : data) {
            GestureInfoPtr gest_info = std::make_shared<GestureInfo>();
            gest_info->gesture_name = item["gesture_name"];
            gest_info->description = item["description"];

            // action_sequences
            const auto &action_sequences = item["action_sequences"];
            for (auto &as : action_sequences) {
                eef_controller::ActionSequenceData as_data;
                as_data.duration_ms = as["duration_ms"];
                if (as["positions"].is_array() && as["positions"].size() == 6) {
                    for (int i = 0; i < 6; i++) {
                        as_data.positions[i] = as["positions"][i];
                    }
                }
                if (as["speeds"].is_array() && as["speeds"].size() == 6) {
                    for (int i = 0; i < 6; i++) {
                        as_data.speeds[i] = as["speeds"][i];
                    }
                }
                if (as["forces"].is_array() && as["forces"].size() == 6) {
                    for (int i = 0; i < 6; i++) {
                        as_data.forces[i] = as["forces"][i];
                    }
                }
                gest_info->action_sequences.emplace_back(as_data);
            }

            // alias
            const auto &alias = item["alias"];
            for (auto &alias_item : alias) {
                gest_info->alias.emplace_back(alias_item);
            }

            gesture_infos.emplace_back(gest_info);
        }
    }
    catch (std::exception &e) {
        std::cout << "parse gesture file failed, e.what(): " << e.what() << "\n";
        return false;
    }

    return true;
}

Revo2HandController::Revo2HandController(const std::string &action_sequences_path, bool is_can_protocol)
    : action_sequences_path_(action_sequences_path), is_can_protocol_(is_can_protocol)
{
    finger_status_ = {
        FingerStatus(),
        FingerStatus()
    };
    finger_touch_status_ = {
        TouchSensorStatusArray(),
        TouchSensorStatusArray()
    };
}

Revo2HandController::~Revo2HandController()
{
    close();
}

bool Revo2HandController::init_revo2_dexhand()
{
    std::array<bool, 2> successes = {true, true};
    const std::string kHandPort = "/dev/stark_serial_revo2";
    right_dexhand_ = Revo2Dexhand::Connect(kHandPort, 0x7f, 460800);
    if (!right_dexhand_) {
        std::cerr << "\033[33m[Revo2HandController] Failed to connect to right dexhand at `" << kHandPort << "` with baudrate 460800\033[0m" << std::endl;
        successes[1] = false;
    } else {
        int count = 3;
        while (count > 0) {
            DeviceInfo_t dev_info;
            if(right_dexhand_->getDeviceInfo(dev_info)) {
                std::cout << "------------------------------------------------------------------\n";
                std::cout << "[Revo2HandController] Connected to right dexhand:\n" << dev_info;
                std::cout << "------------------------------------------------------------------\n";
                break;
            }
            count--;
        }
        if (count <= 0) {
            std::cerr << "\033[33m[Revo2HandController] Failed to get right hand device info, timeout. \033[0m" << std::endl;
        }
    }

    left_dexhand_ = Revo2Dexhand::Connect(kHandPort, 0x7e, 460800);
    if (!left_dexhand_) {
        std::cerr << "\033[33m[Revo2HandController] Failed to connect to left dexhand at `" << kHandPort << "`with baudrate 460800\033[0m" << std::endl;
        successes[0] = false;
    } else {
        int count = 3;
        while (count > 0) {
            DeviceInfo_t dev_info;
            if(left_dexhand_->getDeviceInfo(dev_info)) {
                std::cout << "------------------------------------------------------------------\n";
                std::cout << "[Revo2HandController] Connected to left dexhand:\n" << dev_info;
                std::cout << "------------------------------------------------------------------\n";
                break;
            }
            count--;
        }
        if (count <= 0) {
            std::cerr << "\033[33m[Revo2HandController] Failed to get left hand device info, timeout. \033[0m" << std::endl;
        }
    }

    if (std::none_of(successes.begin(), successes.end(), [](bool v) { return v; })) {
        return false;
    }

    return true;
}

bool Revo2HandController::init_normal_dexhand()
{
    std::array<bool, 2> successes = {true, true};
    const std::string kLeftHandPort = "/dev/stark_serial_L";
    const std::string kRightHandPort = "/dev/stark_serial_R";
    right_dexhand_ = StarkDexhand::Connect(kRightHandPort);
    if (!right_dexhand_) {
        std::cerr << "\033[33m[Revo2HandController] Failed to connect to right dexhand at `" << kRightHandPort << "` with baudrate 115200\033[0m" << std::endl;
        successes[1] = false;
    } else {
        int count = 3;
        while (count > 0) {
            DeviceInfo_t dev_info;
            if(right_dexhand_->getDeviceInfo(dev_info)) {
                std::cout << " ------------------------------------------------------------------\n";
                std::cout << "[Revo2HandController] Connected to right dexhand:\n" << dev_info;
                std::cout << " ------------------------------------------------------------------\n";
                break;
            }
            count--;
        }
        if (count <= 0) {
            std::cerr << "\033[33m[Revo2HandController] Failed to get right hand device info, timeout. \033[0m" << std::endl;
        }
    }

    left_dexhand_ = StarkDexhand::Connect(kLeftHandPort);
    if (!left_dexhand_) {
        std::cerr << "\033[33m[Revo2HandController] Failed to connect to left dexhand at `" << kLeftHandPort << "`with baudrate 115200\033[0m" << std::endl;
        successes[0] = false;
    } else {
        int count = 3;
        while (count > 0) {
            DeviceInfo_t dev_info;
            if(left_dexhand_->getDeviceInfo(dev_info)) {
                std::cout << " ------------------------------------------------------------------\n";
                std::cout << "[Revo2HandController] Connected to left dexhand:\n" << dev_info;
                std::cout << " ------------------------------------------------------------------\n";
                break;
            }
            count--;
        }
        if (count <= 0) {
            std::cerr << "\033[33m[Revo2HandController] Failed to get left hand device info, timeout. \033[0m" << std::endl;
        }
    }

    if (std::none_of(successes.begin(), successes.end(), [](bool v) { return v; })) {
        return false;
    }

    return true;
}

bool Revo2HandController::init_revo2_dexhand_can_customed()
{
    using namespace canbus_sdk;
    auto config_file = ConfigParser::getDefaultConfigFilePath();

    /** 配置文件读取与解析 */
    canbus_sdk::ConfigParser parser;
    if (!parser.parseFromFile(config_file)) { // 会抛异常(无所谓), 配置文件都解析失败,程序就没法玩了
        printf("\033[31m[Revo2HandController] ERROR: Failed to parse config file: %s\033[0m\n", config_file.c_str());
        return false;
    }

    /** 获取CAN总线配置, 没有就返回失败 */
    auto canbus_configs = parser.getCanBusConfigs();
    if (canbus_configs.empty()) {
        printf("\033[31m[Revo2HandController] ERROR: No CAN bus configurations found in config file\033[0m\n");
        return false;
    }

    canbus_sdk::CanBusController::getInstance().init(); // 重复初始化无所谓
    std::vector<DeviceConfig> revo2_devices;
    for (const auto &canbus_config : canbus_configs) { 
        auto devices = parser.getDevices(canbus_config.name, canbus_sdk::DeviceType::REVO2_HAND);
        if(devices.empty()) {
            continue;
        }

        // 找到REVO2 DEXHAND配置在CAN总线上，那么就可以后续的初始化操作
        auto result = canbus_sdk::CanBusController::getInstance().openCanBus(
            canbus_config.name.c_str(), canbus_config.type, canbus_config.bitrate);
        if (result.has_value()) {
            // 总线打开成功，准备把这些设备后续注册到总线
            for (auto &d : devices) {
                revo2_devices.push_back(d);
            }
        }
        else {
            printf("\033[31m[Revo2HandController] ERROR: 打开CAN总线 %s 失败: %s\033[0m\n", canbus_config.name.c_str(),canbus_sdk::errorToString(result.error()));
        }
    }

    if(revo2_devices.empty()) {
        printf("\033[31m[Revo2HandController] ERROR: 没有找到REVO2 DEXHAND设备\033[0m\n");
        return false;
    }

    // 初始化设备!
    // 定义设备初始化函数
    auto init_device = [&revo2_devices](const std::string& device_name, const char* hand_desc) -> std::unique_ptr<Revo2CanDexhand> {
        auto it = std::find_if(revo2_devices.begin(), revo2_devices.end(),
            [&](const canbus_sdk::DeviceConfig& device) { return device.name == device_name; });

        if (it != revo2_devices.end()) {
            // 检查设备是否被忽略
            if (it->ignore) {
                printf("\033[33m[Revo2HandController] WARN: %s Revo2CanDexhand 设备被忽略，跳过初始化\033[0m\n", hand_desc);
                return nullptr;
            }

            auto hand = Revo2CanDexhand::Connect(*it);
            printf("%s[Revo2HandController] %s%s Revo2CanDexhand 设备%s\033[0m\n",
                   hand ? "\033[32m" : "\033[31m", hand ? "成功初始化" : "初始化", hand_desc, hand ? "" : "失败");
            if(hand) {
                DeviceInfo_t dev_info;
                if (hand->getDeviceInfo(dev_info)) {
                    std::cout << " ------------------------------------------------------------------\n";
                    std::cout << "[Revo2HandController] Connected to " << device_name << ":\n" << dev_info;
                    std::cout << " ------------------------------------------------------------------\n";
                }       
            }
            return hand;
        }
        return nullptr;
    };

    // 初始化左右手设备
    right_dexhand_ = init_device("Rhand_revo2_hand", "右手");
    left_dexhand_ = init_device("Lhand_revo2_hand", "左手");

    if (!left_dexhand_ && !right_dexhand_) {
        printf("\033[31m[Revo2HandController] ERROR: 未找到任何可用的REVO2灵巧手设备\033[0m\n");
        return false;
    }

    return true;
}

bool Revo2HandController::init()
{
    std::vector<GestureInfoPtr> gesture_infos;
    if (!this->ParseActionSequenceFile(action_sequences_path_, gesture_infos)) {
        std::cerr << "[Revo2HandController] Failed to parse action sequence file: " << action_sequences_path_ << std::endl;
    }

    for (const auto &gesture_info : gesture_infos) {
        gesture_map_[gesture_info->gesture_name] = gesture_info;
        for (const auto &alias_name : gesture_info->alias) {
            gesture_map_[alias_name] = gesture_info;
        }
    }

    if (is_can_protocol_) {
        // revo2 如果是can通讯协议使用
        if (!init_revo2_dexhand_can_customed()) {
            return false;
        }
    } else {
        if (!init_revo2_dexhand()) {
            return false;
        }
    }

    {
        std::lock_guard<std::mutex> finger_lock(finger_status_mutex_);
        finger_device_present_[0] = static_cast<bool>(left_dexhand_);
        finger_device_present_[1] = static_cast<bool>(right_dexhand_);
    }

    running_ = true;
    control_thread_ = std::thread(&Revo2HandController::control_thread_func, this);
    gesture_thread_ = std::thread(&Revo2HandController::gesture_thread_func, this);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    send_position(kDualHandClosePositions);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    send_position(kDualHandOpenPositions);
    return true;
}

bool Revo2HandController::close()
{
    if (running_) {
        running_ = false;
        task_cv_.notify_all();
        std::cout << "[Revo2HandController] Control thread joining." << std::endl;
        if (control_thread_.joinable()) {
            control_thread_.join();
        }
        std::cout << "[Revo2HandController] Work thread joining." << std::endl;
        task_cv_.notify_all();
        if (gesture_thread_.joinable()) {
            gesture_thread_.join();
        }
        std::cout << "[Revo2HandController] all thread joined." << std::endl;
    }

    left_dexhand_ = nullptr;
    right_dexhand_ = nullptr;

    {
        std::lock_guard<std::mutex> finger_lock(finger_status_mutex_);
        finger_device_present_ = {{false, false}};
        finger_status_seen_ = {{false, false}};
    }

    return true;
}

void Revo2HandController::control_thread_func()
{
    while (running_) {
        if (r_position_updated_ && right_dexhand_) {
            right_dexhand_->setFingerPositions(right_position_);
            r_position_updated_ = false;
        }
        if (l_position_updated_ && left_dexhand_) {
            left_dexhand_->setFingerPositions(left_position_);
            l_position_updated_ = false;
        }
        if (r_speed_updated_ && right_dexhand_) {
            right_dexhand_->setFingerSpeeds(right_speed_);
            r_speed_updated_ = false;
        }
        if (l_speed_updated_ && left_dexhand_) {
            left_dexhand_->setFingerSpeeds(left_speed_);
            l_speed_updated_ = false;
        }

        {
            if (left_dexhand_) {
                dexhand::FingerStatus left_temp_status;
                if (left_dexhand_->getFingerStatus(left_temp_status)) {
                    std::lock_guard<std::mutex> finger_lock(finger_status_mutex_);
                    finger_status_[0] = left_temp_status;
                    finger_status_seen_[0] = true;
                    finger_status_success_at_[0] = std::chrono::steady_clock::now();
                }
            }
            if (right_dexhand_) {
                dexhand::FingerStatus right_temp_status;
                if (right_dexhand_->getFingerStatus(right_temp_status)) {
                    std::lock_guard<std::mutex> finger_lock(finger_status_mutex_);
                    finger_status_[1] = right_temp_status;
                    finger_status_seen_[1] = true;
                    finger_status_success_at_[1] = std::chrono::steady_clock::now();
                }
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1000 / control_frequency_));
    }

    std::cout << "[Revo2HandController] Control loop exited." << std::endl;
}

void Revo2HandController::gesture_thread_func()
{
    while (running_) {
        TaskFunc task;
        {
            std::unique_lock<std::mutex> lock(task_mutex_);
            task_cv_.wait(lock, [this] { return current_task_.has_value() || !running_; });
            if (!running_ && !current_task_.has_value()) {
                return;
            }
            if (current_task_.has_value()) {
                task = std::move(*current_task_);
                current_task_.reset();
            }
        }
        if (task) {
            task();
        }
    }
}

void Revo2HandController::send_left_position(const UnsignedFingerArray &position)
{
    if (!left_dexhand_) {
        std::cerr << "[Revo2HandController] Left dexhand is not connected." << std::endl;
    }
    left_position_ = position;
    l_position_updated_ = true;
}

void Revo2HandController::send_right_position(const UnsignedFingerArray &position)
{
    if (!right_dexhand_) {
        std::cerr << "[Revo2HandController] Right dexhand is not connected." << std::endl;
    }
    right_position_ = position;
    r_position_updated_ = true;
}

void Revo2HandController::send_position(const UnsignedDualHandsArray &finger_positions)
{
    if (left_dexhand_) {
        left_position_ = finger_positions[0];
        l_position_updated_ = true;
    }
    if (right_dexhand_) {
        right_position_ = finger_positions[1];
        r_position_updated_ = true;
    }
}

void Revo2HandController::send_left_speed(const FingerArray &speeds)
{
    if (!left_dexhand_) {
        // std::cerr << "[Revo2HandController] Left dexhand is not connected." << std::endl;
    }
    left_speed_ = speeds;
    // 当前二代手的速度控制无法使用，先禁用！
    // TODO：可能需要使用二代手的协议进行速度控制。
    l_speed_updated_ = false;
}

void Revo2HandController::send_right_speed(const FingerArray &speeds)
{
    if (!right_dexhand_) {
        // std::cerr << "[Revo2HandController] Right dexhand is not connected." << std::endl;
    }
    right_speed_ = speeds;
    // 当前二代手的速度控制无法使用，先禁用！
    // TODO：可能需要使用二代手的协议进行速度控制。
    r_speed_updated_ = false;
}

void Revo2HandController::send_speed(const DualHandsArray &speeds)
{
    if (left_dexhand_) {
        left_speed_ = speeds[0];
        // 当前二代手的速度控制无法使用，先禁用！
        // TODO：可能需要使用二代手的协议进行速度控制。
        l_speed_updated_ = false;
    }
    if (right_dexhand_) {
        right_speed_ = speeds[1];
        // 当前二代手的速度控制无法使用，先禁用！
        // TODO：可能需要使用二代手的协议进行速度控制。
        r_speed_updated_ = false;
    }
}

FingerStatusArray Revo2HandController::get_finger_status()
{
    std::lock_guard<std::mutex> lock(finger_status_mutex_);
    return finger_status_;
}

FingerStatusSnapshot Revo2HandController::get_finger_status_snapshot()
{
    constexpr auto kFreshness = std::chrono::milliseconds(300);
    const auto now = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(finger_status_mutex_);

    FingerStatusSnapshot snapshot;
    snapshot.status = finger_status_;
    for (size_t hand = 0; hand < snapshot.status.size(); ++hand) {
        if (!finger_status_seen_[hand]) {
            continue;
        }
        const auto age = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - finger_status_success_at_[hand]);
        const auto age_count = age.count() < 0 ? 0 : age.count();
        snapshot.sample_age_ms[hand] = static_cast<uint32_t>(
            std::min<int64_t>(age_count, UINT32_MAX));
        snapshot.valid[hand] = finger_device_present_[hand] && age <= kFreshness;
    }
    return snapshot;
}

FingerTouchStatusArray Revo2HandController::get_touch_status()
{
    return finger_touch_status_;
}

bool Revo2HandController::set_hand_force_level(GripForce level)
{
    return set_right_hand_force_level(level) || set_left_hand_force_level(level);
}

bool Revo2HandController::set_right_hand_force_level(GripForce level)
{
    bool ret = false;
    if (right_dexhand_) {
        right_dexhand_->setGripForce(level);
        ret = true;
    }
    return ret;
}

bool Revo2HandController::set_left_hand_force_level(GripForce level)
{
    bool ret = false;
    if (left_dexhand_) {
        left_dexhand_->setGripForce(level);
        ret = true;
    }
    return ret;
}

bool Revo2HandController::set_turbo_mode_enabled(bool enabled)
{
    bool right_success = set_right_hand_turbo_mode_enabled(enabled);
    bool left_success = set_left_hand_turbo_mode_enabled(enabled);
    return right_success && left_success;
}

bool Revo2HandController::set_right_hand_turbo_mode_enabled(bool enabled)
{
    bool ret = false;
    if (right_dexhand_) {
        right_dexhand_->setTurboModeEnabled(enabled);
        ret = true;
    }
    return ret;
}

bool Revo2HandController::set_left_hand_turbo_mode_enabled(bool enabled)
{
    bool ret = false;
    if (left_dexhand_) {
        left_dexhand_->setTurboModeEnabled(enabled);
        ret = true;
    }
    return ret;
}

bool Revo2HandController::is_left_hand_turbo_mode_enabled()
{
    bool ret = false;
    if (left_dexhand_) {
        ret = left_dexhand_->isTurboModeEnabled();
    }
    return ret;
}

bool Revo2HandController::is_right_hand_turbo_mode_enabled()
{
    bool ret = false;
    if (right_dexhand_) {
        ret = right_dexhand_->isTurboModeEnabled();
    }
    return ret;
}

bool Revo2HandController::is_turbo_mode_enabled()
{
    bool left_enabled = false;
    bool right_enabled = false;

    if (left_dexhand_) {
        left_enabled = left_dexhand_->isTurboModeEnabled();
    }
    if (right_dexhand_) {
        right_enabled = right_dexhand_->isTurboModeEnabled();
    }

    if (left_dexhand_ && right_dexhand_) {
        return left_enabled && right_enabled;
    } else if (left_dexhand_) {
        return left_enabled;
    } else if (right_dexhand_) {
        return right_enabled;
    } else {
        return false;
    }
}

std::vector<GestureInfoPtr> Revo2HandController::list_gestures()
{
    std::vector<GestureInfoPtr> gestures;
    for (const auto &gesture : gesture_map_) {
        if (std::find(gestures.begin(), gestures.end(), gesture.second) == gestures.end()) {
            gestures.push_back(gesture.second);
        }
    }
    return gestures;
}

bool Revo2HandController::execute_gestures(const GestureExecuteInfoVec &gesture_tasks, std::string &err_msg)
{
    bool no_gesture = false;
    err_msg = "Not found gesture:";

    BatchGestureTask batch_task;
    for (const auto &task : gesture_tasks) {
        if (gesture_map_.find(task.gesture_name) == gesture_map_.end()) {
            no_gesture = true;
            err_msg += "'" + task.gesture_name + "', ";
            continue;
        }
        batch_task.emplace_back(GestureExecuteTask{task.hand_side, gesture_map_[task.gesture_name]});
    }

    if (batch_task.size() == 0) {
        err_msg = "no gesture to execute.";
        return false;
    }

    std::unique_lock<std::mutex> lock(task_mutex_);
    current_task_ = [this, batch_task]() {
        gesture_executing_ = true;
        abort_running_task_flag_ = false;

        bool batch_task_aborted = false;
        for (auto &task : batch_task) {
            if (!running_)
                return;
            for (auto &as : task.gesture->action_sequences) {
                if (!running_)
                    return;
                if (abort_running_task_flag_) {
                    batch_task_aborted = true;
                    break;
                }

                if (task.hand_side == HandSide::LEFT) {
                    UnsignedFingerArray left_positions;
                    for (int i = 0; i < 6; i++) {
                        left_positions[i] = as.positions[i];
                    }
                    this->send_left_position(left_positions);
                } else if (task.hand_side == HandSide::RIGHT) {
                    UnsignedFingerArray right_positions;
                    for (int i = 0; i < 6; i++) {
                        right_positions[i] = as.positions[i];
                    }
                    this->send_right_position(right_positions);
                } else {
                    UnsignedFingerArray left_positions;
                    UnsignedFingerArray right_positions;
                    for (int i = 0; i < 6; i++) {
                        left_positions[i] = as.positions[i];
                        right_positions[i] = as.positions[i];
                    }
                    this->send_left_position(left_positions);
                    this->send_right_position(right_positions);
                }

                if (!this->sleep_for_100ms(std::max(1, as.duration_ms / 100))) {
                    batch_task_aborted = true;
                    break;
                }
            }
            if (batch_task_aborted) {
                std::cout << "gesture_thread_func: previous task abort:" << task.gesture->gesture_name << "\n";
                break;
            }
        }
        gesture_executing_ = false;
    };
    abort_running_task_flag_ = true;
    task_cv_.notify_all();

    return true;
}

bool Revo2HandController::is_gesture_executing()
{
    return gesture_executing_;
}

bool Revo2HandController::sleep_for_100ms(int ms_count)
{
    int i = 0;
    while (i < ms_count) {
        if (abort_running_task_flag_) {
            std::cout << "sleep_for_100ms: abort!\n";
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        i++;
    }
    return true;
}

} // namespace eef_controller
