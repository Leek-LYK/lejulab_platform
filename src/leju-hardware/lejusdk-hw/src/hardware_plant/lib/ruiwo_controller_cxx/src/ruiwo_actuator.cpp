#include "ruiwo_actuator.h"
#include "ruiwoSDK.h"

#include <pwd.h>
#include <unistd.h>
#include <time.h>
#include <cmath>

#include <filesystem>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <chrono>
#include <atomic>
#include <cstring>
#include <yaml-cpp/yaml.h> 
#include <pthread.h>
// CPU_ZERO/CPU_SET 宏
#include <sched.h>

const int kDisableAddress = 0x00;       // 忽略地址
const float dt = 0.004;                 // 控制周期
const float kMaxSpeed = 2;              // 插值规划的速度
const float velocity_factor = 0.01;     // 速度系数
RUIWOTools ruiwoCpp;

namespace ruiwo_utils {
class RuiwoObserver {
public:
    RuiwoObserver() : rx_msg_count_(0), tx_msg_count_(0),mismatch_count_(0),
     rx_frequency_(0.0), tx_frequency_(0.0) {}

    void increment_rx_count() {
        ++rx_msg_count_;
    }
    void increment_tx_count() {
        ++tx_msg_count_;
    }
    void increment_mismatch_count() {
        mismatch_count_++;
    }
    void update_rx_frequency(float hz) {
        const float alpha = 0.2f; // 平滑因子
        rx_frequency_ = alpha * hz + (1.0f - alpha) * rx_frequency_;
    }
    void update_tx_frequency(float hz) {
        const float alpha = 0.2f; // 平滑因子
        tx_frequency_ = alpha * hz + (1.0f - alpha) * tx_frequency_;
    }

    uint32_t rx_count() {
        return rx_msg_count_;
    }
    
    uint32_t tx_count() {
        return tx_msg_count_;
    }
    
    uint32_t mismatch_count() {
        return mismatch_count_;
    }
    
    float rx_frequency() {
        return rx_frequency_;
    }
    
    float tx_frequency() {
        return tx_frequency_;
    }
private:
    std::atomic<uint32_t> rx_msg_count_{0};      // 接收到的消息数量
    std::atomic<uint32_t> tx_msg_count_{0};      // 发送消息数量
    std::atomic<uint32_t> mismatch_count_{0};    // 匹配失败的消息数量
    // 通信频率监控
    std::atomic<float> rx_frequency_{0.0f};  // 接收消息频率 (Hz)
    std::atomic<float> tx_frequency_{0.0f};  // 发送消息频率 (Hz)
};

std::string GetConfigRootPath() {
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

std::string GetZeroFilePath(){

    std::string path = ruiwo_utils::GetConfigRootPath();
    if(path.empty()){
        std::cerr << "Failed to get home path." <<std::endl;
        return "";
    }
    std::string config_path = path + "/arms_zero.yaml";
    return config_path;
}

} // ruiwo_utils 

ruiwo_utils::RuiwoObserver g_ruiwo_observer;

std::string RuiwoErrCode2string(RuiwoErrCode errcode) {
    switch (errcode) {
        case RuiwoErrCode::NO_FAULT:
            return "无故障";
        case RuiwoErrCode::DC_BUS_OVER_VOLTAGE:
            return "直流母线电压过压";
        case RuiwoErrCode::DC_BUS_UNDER_VOLTAGE:
            return "直流母线电压欠压";
        case RuiwoErrCode::ENCODER_ANGLE_FAULT:
            return "编码器电角度故障";
        case RuiwoErrCode::DRV_DRIVER_FAULT:
            return "DRV驱动器故障";
        case RuiwoErrCode::DC_BUS_CURRENT_OVERLOAD:
            return "直流母线电流过流";
        case RuiwoErrCode::MOTOR_A_PHASE_CURRENT_OVERLOAD:
            return "电机A相电流过载";
        case RuiwoErrCode::MOTOR_B_PHASE_CURRENT_OVERLOAD:
            return "电机B相电流过载";
        case RuiwoErrCode::MOTOR_C_PHASE_CURRENT_OVERLOAD:
            return "电机C相电流过载";
        case RuiwoErrCode::DRIVER_BOARD_OVERHEAT:
            return "驱劝板温度过高";
        case RuiwoErrCode::MOTOR_WINDING_OVERHEAT:
            return "电机线圈过温";
        case RuiwoErrCode::ENCODER_FAILURE:
            return "编码器故障";
        case RuiwoErrCode::CURRENT_SENSOR_FAILURE:
            return "电流传感器故障";
        case RuiwoErrCode::OUTPUT_ANGLE_OUT_OF_RANGE:
            return "输出轴实际角度超过通信范围";
        case RuiwoErrCode::OUTPUT_SPEED_OUT_OF_RANGE:
            return "输出轴速度超过通信范围";
        case RuiwoErrCode::STUCK_PROTECTION:
            return "堵转保护：电机电枢电流(Iq)大于 Stuck Current，同时电机速度小于 StuckVelocity，持续时间超过 Stuck Time 后触发";
        case RuiwoErrCode::CAN_COMMUNICATION_LOSS:
            return "CAN通讯丢失：超过CAN通信超时时间未收到数据帧";
        case RuiwoErrCode::ABS_ENCODER_OFFSET_VERIFICATION_FAILURE:
            return "离轴/对心多圈绝对值编码器接口帧头校验失败，重启，若还失败则请联系售后工程师";
        case RuiwoErrCode::ABSOLUTE_ENCODER_MULTI_TURN_FAILURE:
            return "对心多圈绝对值编码器多圈接口故障，重启，若还失败则请联系售后工程师";
        case RuiwoErrCode::ABSOLUTE_ENCODER_EXTERNAL_INPUT_FAILURE:
            return "对心多圈绝对值编码器外部输入故障，重启，若还失败则请联系售后工程师";
        case RuiwoErrCode::ABSOLUTE_ENCODER_SYSTEM_ANOMALY:
            return "对心多圈绝对值编码器读值故障，重启，若还失败则请联系售后工程师";
        case RuiwoErrCode::ERR_OFFS:
            return "对心多圈绝对值编码器ERR_OFFS，重启，若还失败则请联系售后工程师";
        case RuiwoErrCode::ERR_CFG:
            return "对心多圈绝对值编码器ERR_CFG，重启，若还失败则请联系售后工程师";
        case RuiwoErrCode::ILLEGAL_FIRMWARE_DETECTED:
            return "检测到非法固件，重启，若还失败则请联系售后工程师";
        case RuiwoErrCode::INTEGRATED_STATOR_DRIVER_DAMAGED:
            return "集成式栅极驱动器初始化失败，重启，若还失败则请联系售后工程师";
        default:
            std::cout << "\033[33m[DEBUG] 未处理的错误码: 0x" << std::hex << static_cast<int>(errcode)
                      << std::dec << " (" << static_cast<int>(errcode) << ")\033[0m" << std::endl;
            return "未知故障码: " + std::to_string(static_cast<int>(errcode));
    }
}

static bool isPositionInvalid(double position) {
    // 根据RUIWO电机的机械限制，检查关节位置是否超出合理范围
    // 通常关节位置应在±12.5rad范围内

    const double POSITION_MAX_LIMIT = 12.5;  // 硬件最大位置限制
    const double EPSILON = 1e-10;           // 浮点数比较精度

    // 检查是否为NaN或无穷大
    if (std::isnan(position) || std::isinf(position)) {
        return true;
    }

    // 如果位置绝对值大于等于最大限制（考虑浮点精度），则非法
    if (std::abs(position) - POSITION_MAX_LIMIT > EPSILON) {
        return true;
    }

    return false;
}

static bool hasPositionJump(double previous_pos, double current_pos) {
    // 根据RUIWO电机特性，检查位置是否存在异常跳变
    // 100RPM = 10.47 rad/sec，考虑CAN通讯可能丢几帧，设置安全的跳变阈值

    const double POSITION_JUMP_THRESHOLD = 0.3;  // 位置跳变阈值（弧度）
    const double EPSILON = 1e-10;                // 浮点数比较精度

    // 检查输入值的有效性
    if (std::isnan(current_pos) || std::isinf(current_pos) ||
        std::isnan(previous_pos) || std::isinf(previous_pos)) {
        return true;  // 无效输入认为是异常
    }

    // 计算位置变化的绝对值
    double position_change = std::abs(current_pos - previous_pos);

    // 如果位置变化超过阈值，则认为存在异常跳变
    return position_change > POSITION_JUMP_THRESHOLD + EPSILON;
}

RuiWoActuator::RuiWoActuator(std::string unused, bool is_cali, bool single_motor_mode) :
is_cali_(is_cali), single_motor_mode_(single_motor_mode)
{
}

RuiWoActuator::~RuiWoActuator()
{
    if (thread_running) {
        close();
    }
}

int RuiWoActuator::initialize()
{
    // Print information about RUIWO C++ SDK being used
    std::cout << "===============================================" << std::endl;
    std::cout << "RUIWO C++ SDK" << std::endl;
    std::cout << "Calibration Mode: " << (is_cali_ ? "True" : "False") << std::endl;
    std::cout << "===============================================" << std::endl;

    try {
        initialize_ruiwoSDK(&ruiwoCpp, false);
        std::string path = ruiwo_utils::GetConfigRootPath();
        if (path.empty()) {
            std::cout << "[RUIWO motor] Failed to get config root path." << std::endl;
            return 1;
        }
        
        // check file exist!
        std::string config_path = path + "/config.yaml";
        std::cout << "[RUIWO motor] config_path:" << config_path << "\n";
        if (!std::filesystem::exists(config_path)) {
            std::cout << "[RUIWO motor] config file is not exist, path:" << config_path << "\n";
            std::cout << "[RUIWO motor] 请从`src/kuavo_assets/config` 拷贝对应的 config.yaml 到配置路径`~/.config/lejuconfig` \n";
            return 1; // config error
        }

        // parse config file
        if(!parse_config_file(config_path)) {
            std::cout << "[RUIWO motor] Failed to parse config file." << std::endl;
            return 1; // config error
        }

        std::cout << "---------------INTIALIZED START---------------" << std::endl;
        // thread running.
        thread_running = true;

        unsigned char version[4] = {0x02, 0x02, 0x04, 0x0A};
        auto errorcode = open_canbus(&ruiwoCpp, 1, version);
        if (errorcode != 0) {
            std::cout << "[RUIWO motor]:Canbus status: [ " << errorcode << " ]" << std::endl;
            return 2; // canbus error
        }
        std::cout << "[RUIWO motor]:Canbus status: [ Success ]" << std::endl;
        
        std::vector<int> all_joint_addresses = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C};
        std::cout << "All joint addresses: ";
        for (int addr : all_joint_addresses) {
            std::cout << std::hex << addr << " ";
        }
        std::cout << std::endl;

        // cali_arm 模式下进行多圈清零
        if(is_cali_) {
            std::cout << "[RUIWO motor]: Calibration mode detected, performing multi-turn zeroing..." << std::endl;
            multi_turn_zeroing(all_joint_addresses);
        } else {
            std::cout << "[RUIWO motor]: Normal mode, skipping multi-turn zeroing" << std::endl;
        }

        // 读取零点位置
        std::string zerosPath = ruiwo_utils::GetZeroFilePath();
        std::cout << "[RUIWO motor]:Zero file path: " << zerosPath<< std::endl;
        bool setZero = is_cali_ ? true : false;
        if (std::filesystem::exists(zerosPath) && !setZero) {
            std::ifstream file(zerosPath);
            if (file.is_open()) {
                YAML::Node zerosConfig = YAML::Load(file);
                auto zero_offsets = zerosConfig["arms_zero_position"].as<std::vector<double>>();
                for (int i = 0; i < zero_offsets.size(); i++) {
                    ruiwo_mtr_config_[i].zero_offset = zero_offsets[i];
                }
                if (single_motor_mode_) {
                    std::cout << "\033[33m[单电机标定模式] 跳过启动整体使能, 所有电机保持失能\033[0m" << std::endl;
                } else {
                    auto ec = enable();
                    if(ec != 0) {
                        std::cout << "\033[31m[RUIWO motor]:Failed to enable motors, RetCode:" << std::to_string(ec) << "\033[0m" << std::endl;
                        return 3; // 电机故障
                    }
                }
                file.close();
            } else {
                std::cerr << "Failed to open zero_position file." << std::endl;
                return 1;
            }
        } else if (single_motor_mode_) {
            std::cout << "\033[33m[单电机标定模式] 零点文件不存在也无妨: 跳过启动整体使能, "
                          "标定时对目标电机单独处理\033[0m" << std::endl;
        } else {
            auto ec = enable();
            if(ec != 0) {
                std::cout << "\033[31m[RUIWO motor]:Failed to enable motors, RetCode:" << std::to_string(ec) << "\033[0m" << std::endl;
                std::cout << "\033[31m[RUIWO motor]: 电机零点标定失败，请重试!\033[0m" << std::endl;
                std::cout << "\033[31m[RUIWO motor]: 电机零点标定失败，请重试!\033[0m" << std::endl;
                std::cout << "\033[31m[RUIWO motor]: 电机零点标定失败，请重试!\033[0m" << std::endl;
                return 3; // 电机故障
            }
            if (!setZero) {
                std::cout << "[RUIWO motor]:Warning: zero_position file does not exist, will use current position as zero value." << std::endl;
                saveAsZeroPosition(); // 保存当前为零点位置
            }
            // 标定模式下不保存，零点由用户交互（按 'c'）确认后保存，
            // 避免 Ctrl+C 退出时零点已被覆盖无法撤销
        }

        std::cout << "---------------------------------------------------------------\n";
        for (auto& config : ruiwo_mtr_config_) {
           std::cout << config << std::endl;
        }
        std::cout << "---------------------------------------------------------------\n";
        std::cout << "[RUIWO motor]:Control mode:" << Control_mode_ << std::endl;
        std::cout << "[RUIWO motor]:Multi-turn_Encoder_mode: " << multi_turn_encoder_mode << std::endl;
        std::cout << "[RUIWO motor] 原始手臂关节位置:" << std::endl;
        std::vector<double> zero_pos;
        bool has_invalid_pos = false;
        for (const auto& state : origin_joint_status) {
            if (!state.empty()) {
                zero_pos.push_back(state[1]);
                // 判断 state[1] 是否为非法位置
                if (isPositionInvalid(state[1])) {
                    has_invalid_pos = true;
                }
            }
        }
        
        std::cout << "zero_pos: ";
        for (const auto& pos : zero_pos) {
            std::cout << pos << " ";
        }
        std::cout << std::endl;

        std::cout << "[RUIWO motor] 加上零点后的手臂关节位置:" << std::endl;
        zero_pos.clear();
        {
            std::lock_guard<std::mutex> lock(state_lock);
            for (const auto& state : joint_status) {
                if (!state.empty()) {
                    zero_pos.push_back(state[1]);
                }
            }
        }
        std::cout << "zero_pos: ";
        for (const auto& pos : zero_pos) {
            std::cout << pos << " ";
        }
        std::cout << std::endl;

        // 检测是否有接近或超过12.5rad的关节位置
        if (has_invalid_pos) {
            std::cout << "\033[33m[RUIWO motor]: Error: 检测到接近或超过12.5rad的关节位置，可能存在位置异常\033[0m" << std::endl;
            return 3;
        }
        // Go to zero
        if (single_motor_mode_) {
            std::cout << "\033[33m[单电机标定模式] 跳过启动 go_to_zero, 电机只锁定当前位置\033[0m" << std::endl;
        } else {
            go_to_zero();
            std::cout << "[RUIWO motor]: Moved to zero succeed" << std::endl;
        }
        std::cout << "\n\n";

        std::cout << "[RUIWO motor] 回零之后，手臂关节位置:" << std::endl;
        zero_pos.clear();
        {
            std::lock_guard<std::mutex> lock(state_lock);
            for (const auto& state : joint_status) {
                if (!state.empty()) {
                    zero_pos.push_back(state[1]);
                }
            }
        }
        
        std::cout << "zero_pos: ";
        for (const auto& pos : zero_pos) {
            std::cout << pos << " ";
        }
        std::cout << std::endl;

        std::vector<int> maybe_negative_joint_ids;
        for (size_t i = 0; i < zero_pos.size(); ++i) {
            if (std::abs(zero_pos[i]) > 0.1) {
                maybe_negative_joint_ids.push_back(ruiwo_mtr_config_.at(i).address);
            }
        }

        if (!maybe_negative_joint_ids.empty()) {
            std::cout << "\033[31m[RUIWO motor]: Warning: 下列关节方向可能反了:\n";
            for (const auto& id : maybe_negative_joint_ids) {
                std::cout << id << " ";
            }
            std::cout << "\n请检查并修改 config.yaml\033[0m" << std::endl;
        }
        std::cout << "\n\n";
        {
            std::lock_guard<std::mutex> lock(update_lock);
            target_update = false;
        }
        
        std::cout << "---------------INTIALIZED DONE---------------" << std::endl;

        thread_end = false;
        control_thread_ = std::thread(&RuiWoActuator::control_thread, this);
        recv_thread_ = std::thread(&RuiWoActuator::recv_thread, this);

        // 设置线程CPU亲和性到核心7
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(7, &cpuset);

        int rc1 = pthread_setaffinity_np(control_thread_.native_handle(), sizeof(cpu_set_t), &cpuset);
        if (rc1 != 0) {
            std::cerr << "[RUIWO motor]: 设置control_thread_ CPU亲和性失败，错误码: " << rc1 << " (" << strerror(rc1) << ")" << std::endl;
        } else {
            std::cout << "[RUIWO motor]: 已将control_thread_绑定到CPU核心 7" << std::endl;
        }

        int rc2 = pthread_setaffinity_np(recv_thread_.native_handle(), sizeof(cpu_set_t), &cpuset);
        if (rc2 != 0) {
            std::cerr << "[RUIWO motor]: 设置recv_thread_ CPU亲和性失败，错误码: " << rc2 << " (" << strerror(rc2) << ")" << std::endl;
        } else {
            std::cout << "[RUIWO motor]: 已将recv_thread_绑定到CPU核心 7" << std::endl;
        }
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Failed to initialize RuiWoActuator: " << e.what() << std::endl;
        return 1;
    }

    return 3;
}

bool RuiWoActuator::parse_config_file(const std::string &config_file)
{
    try {
        Unnecessary_go_zero_list.clear();

        YAML::Node motor_config = YAML::LoadFile(config_file);
        const YAML::Node& address_node = motor_config["address"];
        const YAML::Node& online_node = motor_config["online"];
        const YAML::Node& parameter_node = motor_config["parameter"];

        // extract joint names from keys.
        std::vector<std::string> joint_names;
        for (const auto& entry : address_node) {
            std::string key = entry.first.as<std::string>();
            if (key.find("Left_joint_arm") != std::string::npos ||
                key.find("Right_joint_arm") != std::string::npos ||
                key.find("Head_joint") != std::string::npos) {
                joint_names.push_back(key);
            }
        }

        auto isKeyExists = [](const YAML::Node& node, const std::string& key) -> bool {
            return node[key].IsDefined();
        };

        for(auto joint_name : joint_names) {
            if (!isKeyExists(online_node, joint_name)) {
                std::cout << "[RUIWO motor] [ERROR] online_node does not contain:" << joint_name << std::endl;
                return  false;
            }
            if (!isKeyExists(parameter_node, joint_name)) {
                std::cout << "[RUIWO motor] [ERROR] parameter_node does not contain:" << joint_name << std::endl;
                return  false;
            }

            RuiwoMotorConfig_t mtr_config;
            mtr_config.address = address_node[joint_name].as<int>();
            mtr_config.joint_name = joint_name;
            mtr_config.online = online_node[joint_name].as<bool>();
            mtr_config.parameter = RuiwoMotorParam_t(parameter_node[joint_name].as<std::vector<int>>());
            ruiwo_mtr_config_.push_back(mtr_config);
        }
        
        // 其他配置项        
        const YAML::Node& negtive_address = motor_config["negtive_address"];
        if(negtive_address.size()) {
            const YAML::Node& negtive_motor_list = negtive_address[0];
            for (size_t j = 0; j < negtive_motor_list.size(); ++j) {
               int address = negtive_motor_list[j].as<int>();               
               // Find the motor config with matching address and set its negtive flag to true
               auto it = std::find_if(ruiwo_mtr_config_.begin(), ruiwo_mtr_config_.end(), 
                   [address](const RuiwoMotorConfig_t& config) {
                       return config.address == address;
                   });
               if (it != ruiwo_mtr_config_.end()) {
                   it->negtive = true;
               }
            }
        }

        const YAML::Node& low_arm_address = motor_config["low_arm_address"];
        if(low_arm_address.size()) {
            const YAML::Node& low_arm_address_list = low_arm_address[0];
            for (size_t j = 0; j < low_arm_address_list.size(); ++j) {
               int motor_id = low_arm_address_list[j].as<int>();
               Unnecessary_go_zero_list.push_back(motor_id); 
            }
        }
        
        // 控制模式
        Control_mode_ = motor_config["control_mode"].as<std::string>();

        // Get ratio configuration
        try {
            const YAML::Node& ratio_node = motor_config["ratio"];
            if(ratio_node.size()) {
                std::cout << "[RUIWO motor] ratio:";
                const YAML::Node& ratio_config = ratio_node[0];
                for (size_t i = 0; i < ratio_config.size(); ++i) {
                    ratio[i] = ratio_config[i].as<int>();
                    std::cout << ratio[i] << " ";
                }
                std::cout << std::endl;
            }
            else {
                ratio = {36, 36, 36, 25, 25, 25, 36, 36, 36, 25, 25, 25, 36, 36};
                std::cout << "[RUIWO motor] [WARNING] Using default ratio values" << std::endl;
            }   
        } catch (const YAML::Exception& e) {
            // Use default ratio values if not specified in config
            ratio = {36, 36, 36, 25, 25, 25, 36, 36, 36, 25, 25, 25, 36, 36};
            std::cout << "[RUIWO motor] [WARNING] Using default ratio values, exception:" << e.what() << std::endl;
        }

        return true;
    }catch (const YAML::Exception &e) {
        std::cout << "[RUIWO motor]:YAML::Exception:" << e.what() << std::endl;
    }
    catch (const std::exception& e) {
        std::cout << "[RUIWO motor] Exception:" << e.what() << std::endl;
    }

    return false;
}

std::vector<int> RuiWoActuator::get_all_joint_addresses() {
    std::vector<int> all_joint_addresses;
    for (const auto& config : ruiwo_mtr_config_) {
        if (config.online) {
            all_joint_addresses.push_back(config.address);
        }
    }
    return all_joint_addresses;
}

void RuiWoActuator::multi_turn_zeroing(const std::vector<int>& dev_ids) {
    for (int dev_id : dev_ids) {
        try {
            BM_CanMessageTypeDef msg;
            memset(&msg, 0, sizeof(msg));
            msg.id.SID = 0x600 + dev_id; // Arbitration ID
            msg.ctrl.tx.DLC = 8;
            msg.payload[0] = 0x67;
            msg.payload[1] = 0x06;
            msg.payload[2] = 0xFF;
            msg.payload[3] = 0x00;
            msg.payload[4] = 0x00;
            msg.payload[5] = 0x00;
            msg.payload[6] = 0x00;
            msg.payload[7] = 0x76;

            std::cout << "Preparing to send zeroing command to device " << std::hex << dev_id << std::endl;
            std::cout << "Arbitration ID: " << std::hex << msg.id.SID << std::endl;
            std::cout << "DLC: " << std::dec << msg.ctrl.tx.DLC << std::endl;
            std::cout << "Payload: ";
            for (int i = 0; i < msg.ctrl.tx.DLC; ++i) {
                std::cout << std::hex << (int)msg.payload[i] << " ";
            }
            std::cout << std::endl;

            BM_StatusTypeDef error = BM_WriteCanMessage(ruiwoCpp.channel, &msg, 0, ruiwoCpp.dev_info.timeout, NULL);
            if (error != BM_ERROR_OK) {
                char buffer[256] = { 0 };
                BM_GetErrorText(error, buffer, sizeof(buffer), 0);
                std::cerr << "Failed to send zeroing command to device " << std::hex << dev_id << ": " << buffer << std::endl;
            } else {
                std::cout << "Sent zeroing command to device " << std::hex << dev_id << std::endl;
            }
        } catch (const std::exception& e) {
            std::cerr << "Error processing device " << std::hex << dev_id << ": " << e.what() << std::endl;
        }
    }
}

void RuiWoActuator::update_status() {

    auto size = ruiwo_mtr_config_.size();
    std::vector<float> current_positions_copy(size, 0);
    std::vector<float> current_velocity_copy(size, 0);
    std::vector<float> current_torque_copy(size, 0);

    // 读取当前位置
    {
        std::lock_guard<std::mutex> lock(state_lock);

        for (size_t i = 0; i < size; ++i)
        {
            if (ruiwo_mtr_config_.at(i).online)
            {
                const std::vector<float> &motor = joint_status[i];
                current_positions_copy[i] = motor[1];
                current_velocity_copy[i] = motor[2];
                current_torque_copy[i] = motor[3];
            }
        }
    }

    {
        std::lock_guard<std::mutex> lock(recvpos_lock);
        current_positions = current_positions_copy;
    }
    {
        std::lock_guard<std::mutex> lock(recvtor_lock);
        current_torque = current_torque_copy;
    }
    {
        std::lock_guard<std::mutex> lock(recvvel_lock);
        current_velocity = current_velocity_copy;
    }
}

void RuiWoActuator::recv_thread()
{
    while(thread_running) {
        auto start_time = std::chrono::high_resolution_clock::now();
        
        // revceive
        recv_message();

        auto current_time = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration<float>(current_time - start_time).count();
        float sleep_time = std::max<float>(0.0f, dt - elapsed);
        std::this_thread::sleep_for(std::chrono::microseconds(static_cast<long long>(sleep_time * 1000000)));

        // Update debug statistics
        current_time = std::chrono::high_resolution_clock::now();
        elapsed = std::chrono::duration<float>(current_time - start_time).count();
        
        if (elapsed > 0) {
            g_ruiwo_observer.update_rx_frequency(1.0f / elapsed);
            
            // Log frequency every 100 iterations
            static int counter = 0;
            if (++counter % 100 == 0) {
                // std::cout << "[RUIWO motor]: Receive thread average frequency: " 
                //           << g_ruiwo_observer.rx_frequency() << " Hz" << std::endl;
                counter = 0;
            }
        }
    }
}

void RuiWoActuator::recv_message() 
{
    BM_CanMessageTypeDef rx_msg;
    uint32_t port;
    uint32_t timestamp;

    // Wait for notifications with a timeout
    constexpr int Timeout_ms = 1000;
    if (BM_WaitForNotifications(&ruiwoCpp.notification, 1, Timeout_ms) < 0) {
        return;
    }

    BM_StatusTypeDef error = BM_ReadCanMessage(ruiwoCpp.channel, &rx_msg, &port, &timestamp);
    while(error == BM_ERROR_OK) {
        // Debug
        g_ruiwo_observer.increment_rx_count();
        int msg_id = BM_GET_STD_MSG_ID(rx_msg.id);
        if (msg_id != 0) {
            // 检查 arbitration_id 是否在关节地址列表中
            auto iter = std::find_if(ruiwo_mtr_config_.begin(), ruiwo_mtr_config_.end(), [sid = msg_id](const RuiwoMotorConfig_t &config) {
                return config.address == sid;
            });
            if (iter != ruiwo_mtr_config_.end()) {
                int rx_id = static_cast<int>(rx_msg.payload[0]);
                // 双重判断: arbitration_id == rx_id
                auto iter1 = std::find_if(ruiwo_mtr_config_.begin(), ruiwo_mtr_config_.end(), [rx_id](const RuiwoMotorConfig_t &config) {
                    return config.address == rx_id;
                });
                if (rx_id == msg_id && iter1 != ruiwo_mtr_config_.end()) {
                    std::vector<float> motor_state(6);
                    uint8_t errcode = 0;
                    return_motor_state(&rx_msg, motor_state.data(), &errcode, &ruiwoCpp.can_com_param);
                    if(errcode != 0) {
                        std::cout << "\033[31m[RUIWO motor] RX_ID:" << std::dec << rx_id << " Error code: 0x"
                                  << std::hex << static_cast<int>(errcode) << std::dec << " "
                                  << RuiwoErrCode2string(static_cast<RuiwoErrCode>(errcode)) << "\033[0m" << std::endl;
                    }
                    else {
                        size_t motor_index = std::distance(ruiwo_mtr_config_.begin(), iter1);
                        auto current_raw_pos = iter->negtive ? -motor_state[1] : motor_state[1];  // origin state 是带方向的
                        auto previous_raw_pos = origin_joint_status[motor_index][1]; // previous state
                        if(hasPositionJump(previous_raw_pos, current_raw_pos)) {
                            std::cout << "\033[33m[RUIWO motor] RX_ID:" << rx_id
                                      << " 检测到位置异常跳变: " << previous_raw_pos
                                      << " -> " << current_raw_pos << "\033[0m" << std::endl;
                        }
                        else {
                            set_joint_state(motor_index, motor_state);
                            update_status();
                        } // end if jump position
                    } // enndif errcode
                }
                else {
                    std::cout << "[RUIWO motor] rx_id mismatch "<< rx_id <<" : rx_msg.id.SID : "<< msg_id << "\n";
                }
            }
            else{
                std::cout << "[RUIWO motor] 没有找到 sid :"<< msg_id << "\n";
            } // end if iter
        }
        
        // recview message utils recv queue empty.
        error = BM_ReadCanMessage(ruiwoCpp.channel, &rx_msg, &port, &timestamp);
    }
    if(error == BM_ERROR_QRCVEMPTY) {
        return;
    }
    else if (error != BM_ERROR_OK) {
        std::cerr << "Failed to receive message, error code: " << error << std::endl;
        return;
    }
}

void RuiWoActuator::control_thread()
{
    std::cout << "[RUIWO motor]:Threadstart succeed" << std::endl;
    std::vector<float> target_positions_copy;
    std::vector<float> target_torque_copy;
    std::vector<float> target_velocity_copy;
    std::vector<float> target_kp_copy;
    std::vector<float> target_kd_copy;

    try
    {
        while (thread_running) {
            // update_status_asynchronous();
            // update_status();

            std::vector<int> index(ruiwo_mtr_config_.size());
            for (size_t i = 0; i < ruiwo_mtr_config_.size(); ++i) {
                index[i] = i;
            }

            {
                std::lock_guard<std::mutex> lock(update_lock);
                if (!target_update) {
                    continue;
                }
            }
            
            auto start_time = std::chrono::high_resolution_clock::now();
            {
                std::lock_guard<std::mutex> lock_sendpos(sendpos_lock);
                std::lock_guard<std::mutex> lock_sendvel(sendvel_lock);
                std::lock_guard<std::mutex> lock_sendtor(sendtor_lock);

                target_positions_copy = target_positions;
                target_torque_copy = target_torque;
                target_velocity_copy = target_velocity;
                target_kp_copy = target_kp_pos;
                target_kd_copy = target_kd_pos;
            }
            const bool has_cmd_gains = target_has_cmd_gains_.load();

            {
                std::lock_guard<std::mutex> lock(update_lock);
                target_update = false;
            }

            // send_positions(index, target_positions_copy, target_torque_copy, target_velocity_copy);
            if (torque_only_mode_) {
                // CST 模式：kp=0, kd=0，仅下发前馈力矩
                std::vector<float> zero_gains(ruiwo_mtr_config_.size(), 0.0f);
                send_positions_No_response(index, target_positions_copy, target_torque_copy, target_velocity_copy, &zero_gains, &zero_gains);
            } else if (has_cmd_gains) {
                // CSP 模式：使用本次 cmd 携带的逐周期 kp/kd
                send_positions_No_response(index, target_positions_copy, target_torque_copy, target_velocity_copy, &target_kp_copy, &target_kd_copy);
            } else {
                // CSP 模式：沿用 ruiwo_mtr_config_[].parameter 里缓存的 kp/kd
                send_positions_No_response(index, target_positions_copy, target_torque_copy, target_velocity_copy);
            }
            
            auto current_time = std::chrono::high_resolution_clock::now();
            auto elapsed = std::chrono::duration<float>(current_time - start_time).count();
            float sleep_time = std::max(0.0f, dt - elapsed);
            std::this_thread::sleep_for(std::chrono::microseconds(static_cast<long long>(sleep_time * 1000000)));
            
            // Debug
            current_time = std::chrono::high_resolution_clock::now();
            elapsed = std::chrono::duration<float>(current_time - start_time).count();
            if (elapsed > 0) {
            float frequency = 1.0f / elapsed;
            g_ruiwo_observer.update_tx_frequency(frequency);
            static int ctrl_counter = 0;
            // Print frequency every 100 iterations to avoid flooding the console
            if (++ctrl_counter % 100 == 0) {
                // std::cout << "[RUIWO motor]: Control thread frequency: " << frequency << " Hz (elapsed time: " << elapsed * 1000 << "sleep time:" << sleep_time *1000<< " ms)" << std::endl;
                // std::cout << "[RUIWO motor]: Control thread avg frequency: " << g_ruiwo_observer.tx_frequency() << std::endl;
                ctrl_counter = 0;
            }
        }
        }
    }
    catch (const std::exception &e) {
        std::cout << e.what() << std::endl;
    }

    thread_end = true;
    std::cout << "[RUIWO motor]:Thread end succeed" << std::endl;
}


std::vector<std::vector<float>> RuiWoActuator::interpolate_positions_with_speed(const std::vector<float> &a, const std::vector<float> &b, float speed, float dt)
{
    std::vector<float> a_vec = a;
    std::vector<float> b_vec = b;
    int num_joints = a.size();

    // 计算总时间
    float total_distance = 0;
    for (int i = 0; i < num_joints; ++i)
    {
        float distance = b_vec[i] - a_vec[i];
        total_distance += distance * distance;
    }
    total_distance = std::sqrt(total_distance);
    float total_time = total_distance / speed;

    // 根据总时间和时间步长计算实际的时间步数
    int steps = static_cast<int>(total_time / dt) + 1;

    std::cout << "[RUIWO motor] interpolate_positions_with_speed steps:" << steps << "\n";  

    // 使用线性插值计算每个时间步的关节位置
    std::vector<std::vector<float>> interpolation(steps, std::vector<float>(num_joints));
    for (int i = 0; i < steps; ++i)
    {
        float t = static_cast<float>(i) / std::max(1, (steps - 1));
        for (int j = 0; j < num_joints; ++j)
        {
            interpolation[i][j] = a_vec[j] + t * (b_vec[j] - a_vec[j]);
        }
    }

    return interpolation;
}

void RuiWoActuator::go_to_zero()
{
    std::cout << "[RUIWO motor]:Start moving to zero" << std::endl;
    std::vector<std::vector<float>> state = get_joint_state();
    std::vector<float> current_q(ruiwo_mtr_config_.size(), 0);
    std::vector<float> target_q(ruiwo_mtr_config_.size(), 0);

    for (size_t i = 0; i < ruiwo_mtr_config_.size(); ++i) {
        if (ruiwo_mtr_config_.at(i).online) {
            // std::cout << "[RUIWO motor] i:" << i << " current_positions :" << current_q[i] << " state:" << state[i][1] << "\n";
            current_q[i] = state[i][1];
        }
    }

    // 从当前位置插值到目标位置(零点)
    interpolate_move(current_q, target_positions, kMaxSpeed, dt);

    std::vector<int> indexs(ruiwo_mtr_config_.size());
    for (int i = 0; i < ruiwo_mtr_config_.size(); i++) {
        indexs[i] = i;  //0, 1, ..., 13
    }

    std::vector<float> target_tau(ruiwo_mtr_config_.size(), 0);
    std::vector<float> target_vel(ruiwo_mtr_config_.size(), 0);

    // Lambda function to check if all positions are zero
    auto all_positions_zero = [](const std::vector<float>& positions) -> bool {
        for (const auto& pos : positions) {
            if (std::abs(pos) > 0.005f) {
                return false;
            }
        }
        return true;
    };

    float timeout = 1.5; // 1.5s
    auto start_time = std::chrono::steady_clock::now();
    bool reached_zero = false;

    // 继续发送命令读电机当前位置(距离零点较远的话，插值结束时电机可能还没有完全回到零点)，确保回到零点
    while (!reached_zero) {
        // 发送位置命令
        send_positions(indexs, target_q, target_tau, target_vel);
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(dt * 1000)));

        // 检查是否所有关节都接近零点
        reached_zero = all_positions_zero(this->current_positions);
        // 检查是否超时
        auto current_time = std::chrono::steady_clock::now();
        float elapsed_time = std::chrono::duration<float>(current_time - start_time).count();
        if (elapsed_time > timeout) {
            std::cout << "[RUIWO motor]: Zero position timeout after " << elapsed_time << " seconds" << std::endl;
            break;
        }
    }
}

void RuiWoActuator::interpolate_move(const std::vector<float> &start_positions, 
    const std::vector<float> &target_positions, 
    float speed, float dt)
{
    std::vector<std::vector<float>> interpolation_list = interpolate_positions_with_speed(start_positions, target_positions, speed, dt);
    
    std::vector<float> target_velocity_copy;
    std::vector<float> target_torque_copy;
    {
        std::lock_guard<std::mutex> lock_sendvel(sendvel_lock);
        std::lock_guard<std::mutex> lock_sendtor(sendtor_lock);
        target_velocity_copy = target_velocity;
        target_torque_copy = target_torque;
    }

    std::vector<int> indexs(ruiwo_mtr_config_.size());
    for (int i = 0; i < ruiwo_mtr_config_.size(); i++) {
        indexs[i] = i;  //0, 1, ..., 13
    }

    for (const auto &target_pos : interpolation_list) {
        send_positions(indexs, target_pos, target_torque_copy, target_velocity_copy);
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(dt * 1000)));
    }
}

int RuiWoActuator::enable()
{
    // reset motors
    int ret = disable();
    if(ret != 0) {
        std::cout << "\033[31m[RUIWO motor]: 电机使能前置步骤（清除电机故障码）失败，无法继续使能，返回码:\033[0m" << ret << std::endl;
        return ret;
    }

    //////////  初始化类成员变量 ///////////////////
    size_t joint_size = ruiwo_mtr_config_.size();
    {
        std::lock_guard<std::mutex> lock_sendpos(sendpos_lock);
        std::lock_guard<std::mutex> lock_sendvel(sendvel_lock);
        std::lock_guard<std::mutex> lock_sendtor(sendtor_lock);

        // Init target position, velocity, torque
        target_positions.assign(joint_size, 0);
        target_velocity.assign(joint_size, 0);
        target_torque.assign(joint_size, 0);
        target_kp_pos.assign(joint_size, 0);
        target_kd_pos.assign(joint_size, 0);
    }

    {
        std::lock_guard<std::mutex> lock(recvpos_lock);
        current_positions.assign(joint_size, 0);
    }
    {
        std::lock_guard<std::mutex> lock(recvtor_lock);
        current_torque.assign(joint_size, 0);
    }
    {
        std::lock_guard<std::mutex> lock(recvvel_lock);
        current_velocity.assign(joint_size, 0);
    }
    {
        std::lock_guard<std::mutex> lock(state_lock);
        joint_status.assign(joint_size, std::vector<float>(6, 0));
        origin_joint_status.assign(joint_size, std::vector<float>(6, 0));
    }

    head_low_torque = 0;
    head_high_torque = 0;
    //////////  初始化类成员变量 结束 ///////////////////
      
    float state_list[6] = {0};
    int success_count = 0, ignore_count = 0;
    for(int i = 0; i < ruiwo_mtr_config_.size(); ++i) {
        int address = ruiwo_mtr_config_.at(i).address;
        if (address == kDisableAddress) {
            ruiwo_mtr_config_.at(i).online = false;
            std::cout << "[RUIWO motor]:ID:" << address << " Enable: [Ignored]" << std::endl;
            ignore_count++;
            continue;
        }
        uint8_t errcode = 0;
        // 先 Enter Reset 擦除电机故障码.
        int retcode = enter_reset_state(&ruiwoCpp, address, state_list, &errcode);
        if(retcode == 0 && errcode >= static_cast<uint8_t>(RuiwoErrCode::ABS_ENCODER_OFFSET_VERIFICATION_FAILURE)) {
            RuiwoErrCode motor_err = static_cast<RuiwoErrCode>(errcode);
            std::cout << "\033[31m[RUIWO motor]:ID:" << address << " Enable: [Failed], Errorcode: [0x"
                     << std::hex << static_cast<int>(errcode) << std::dec << "] "
                     << RuiwoErrCode2string(motor_err) << "\033[0m" << std::endl;
            return 2; // 直接返回 2 表示存在电机有严重故障码
        }
        // 使能电机
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        retcode = enter_motor_state(&ruiwoCpp, address, state_list, &errcode);
        if (retcode == 0) {
            if(errcode >= static_cast<uint8_t>(RuiwoErrCode::ABS_ENCODER_OFFSET_VERIFICATION_FAILURE)) {
                RuiwoErrCode motor_err = static_cast<RuiwoErrCode>(errcode);
                std::cout << "\033[31m[RUIWO motor]:ID:" << address << " Enable: [Failed], Errorcode: [0x"
                         << std::hex << static_cast<int>(errcode) << std::dec << "] "
                         << RuiwoErrCode2string(motor_err) << "\033[0m" << std::endl;
                return 2; // 直接返回 2 表示存在电机有严重故障码
            }
            success_count++;
            std::vector<float> state;
            state.assign(state_list, state_list + 6);
            set_joint_state(i, state);
            ruiwo_mtr_config_.at(i).online = true;
            std::cout << "[RUIWO motor]:ID:" << address << " Enable: [Succeed]" << std::endl;
        }
        else {
            std::cout << "[RUIWO motor]:ID:" << address << " Enable: [Failed], Errorcode: [" << retcode << "]" << std::endl;
        }
    }

    if(ignore_count == ruiwo_mtr_config_.size()) {
        return 0;
    }
    else if(success_count == 0) {
        return 1; // 全部失败 ==> 通讯问题
    }

    return 0;
}

int RuiWoActuator::disable()
{
    float state_list[6] = {0};
    int success_count = 0, ignore_count = 0, motor_error_count = 0; // 错误计数
    for (size_t i = 0; i < ruiwo_mtr_config_.size(); ++i) {
        int address = ruiwo_mtr_config_.at(i).address;
        if (address == kDisableAddress) {
            std::cout << "[RUIWO motor]:ID:" << address << " Disable: [Ignored]" << std::endl;
            ignore_count++;
            continue;
        }
        uint8_t errcode;
        int retcode = enter_reset_state(&ruiwoCpp, address, state_list, &errcode);
        if (retcode == 0) {
            // 出现大于 128 无法清除的故障码
            if(errcode >= static_cast<uint8_t>(RuiwoErrCode::ABS_ENCODER_OFFSET_VERIFICATION_FAILURE)) {
                RuiwoErrCode motor_err = static_cast<RuiwoErrCode>(errcode);
                std::cout << "\033[31m[RUIWO motor]:ID:" << address << " Disable: [Failed], Errorcode: [0x"
                         << std::hex << static_cast<int>(errcode) << std::dec << "],错误: "
                         << RuiwoErrCode2string(motor_err) << "\033[0m" << std::endl;   
                motor_error_count++;
                continue;
            }
            success_count++;
            ruiwo_mtr_config_.at(i).online = false;
            std::cout << "[RUIWO motor]:ID:" << address << " Disable: [Succeed]" << std::endl;
        }
        else {
            std::cout << "[RUIWO motor]:ID:" << address << " Disable: [Failed], Retcode: [" << retcode << "]" << std::endl;
        }
    }

    // 有电机存在大于 128 的故障码(无法清除), 严重
    if(motor_error_count > 0) {
        return 2;  // 优先返回更严重的错误
    }
    
    if(ignore_count == ruiwo_mtr_config_.size()) {
        return 0;
    }
    else if(success_count == 0) {
        return 1; // 全部失败 ==> 通讯问题
    }

    return 0; // success
}

bool RuiWoActuator::disableMotor(int motorIndex)
{
    int errorcode = 1;
    float state_list[6] = {0};

    int address = ruiwo_mtr_config_.at(motorIndex).address;
    if (kDisableAddress == address)
    {
        std::cout << "[RUIWO motor]:ID:" << address << " Disable: [Ignored]" << std::endl;
        // return false;
    }
    uint8_t reset_errcode;
    errorcode = enter_reset_state(&ruiwoCpp, address, state_list, &reset_errcode);
    if (0 == errorcode)
    {
        ruiwo_mtr_config_.at(motorIndex).online = false;
        std::cout << "[RUIWO motor]:ID:" << address << " Disable: [Succeed]" << std::endl;
        // return true;
    }
    else
    {
        std::cout << "[RUIWO motor]:ID:" << address << " Disable: [Failed],Errorcode: [" << errorcode << "]" << std::endl;
    }

    return true;
}

void RuiWoActuator::saveAsZeroPosition() {
    float state_list[6] = {0};
    int fail_count = 0;
    for(int  i = 0; i < ruiwo_mtr_config_.size(); i++) {
        int address = ruiwo_mtr_config_[i].address;
        uint8_t motor_errcode;
        int errorcode = enter_motor_state(&ruiwoCpp, address, state_list, &motor_errcode);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        if (errorcode == 0) {
            errorcode = run_ptm_mode(&ruiwoCpp, address, state_list[1], 0,
                ruiwo_mtr_config_[i].parameter.kp_pos, ruiwo_mtr_config_[i].parameter.kd_pos, 0, state_list);
            ruiwo_mtr_config_[i].zero_offset = origin_joint_status[i][1]; // update zero offset
            std::vector<float> state;
            state.assign(state_list, state_list + 6);
            set_joint_state(i, state);
            std::cout << "[RUIWO motor]:ID: " << address << " Current position: " << origin_joint_status[i][1] << std::endl;
        } else {
            fail_count++;
            std::cout << "\033[31m[RUIWO motor]:ID: " << address << " Failed to get position: " << origin_joint_status[i][0] << ", errorcode: " << errorcode << "\033[0m" << std::endl;
        }
    }

    if (fail_count > 0) {
        std::cout << "\033[33m[RUIWO motor]:Warning: " << fail_count << "/" << ruiwo_mtr_config_.size()
                  << " 个电机读位置失败, 零点文件中对应电机保留旧值，请检查通讯!\033[0m" << std::endl;
    }
    saveZeroPosition();
    update_status();
}

void RuiWoActuator::changeEncoderZeroRound(int index, double direction){

    float round = 360.0 / ratio[index] * M_PI / 180.0;
    round *= direction;
    ruiwo_mtr_config_.at(index).zero_offset += round;

    float state_list[6];
    int address = ruiwo_mtr_config_.at(index).address;
    uint8_t motor_errcode;
    int errorcode = enter_motor_state(&ruiwoCpp, address, state_list, &motor_errcode);
    auto &param = ruiwo_mtr_config_.at(index).parameter; 
    if (errorcode == 0) {
        run_ptm_mode(&ruiwoCpp, address, state_list[1], 0, param.kp_pos, param.kd_pos, 0, state_list);
        
        std::vector<float> state;
        state.assign(state_list, state_list + 6);
        set_joint_state(index, state);
    } else {
        std::cout << "[RUIWO motor]:ID: " << address << " Failed to get position: " << origin_joint_status[index][0] << std::endl;
    }

    update_status();
    std::cout << "[RUIWO motor]:ID: " << address << " Change encoder zero position: " << ruiwo_mtr_config_.at(index).zero_offset << std::endl;
}

void RuiWoActuator::adjustZeroPosition(int index, double offset) {
    // 调整指定索引电机的零点位置
    if (index >= 0 && index < static_cast<int>(ruiwo_mtr_config_.size())) {
        ruiwo_mtr_config_[index].zero_offset += offset;
        std::cout << "[RUIWO motor]:ID: " << ruiwo_mtr_config_[index].address 
                  << " Adjusted zero position by: " << offset 
                  << ", new zero offset: " << ruiwo_mtr_config_[index].zero_offset << std::endl;
    } else {
        std::cerr << "[RUIWO motor]: Invalid motor index: " << index << std::endl;
    }
}

bool RuiWoActuator::calibrateSingleMotor(int index) {
    if (index < 0 || index >= static_cast<int>(ruiwo_mtr_config_.size())) {
        std::cerr << "[RUIWO motor]: calibrateSingleMotor invalid motor index: " << index << std::endl;
        return false;
    }

    int address = ruiwo_mtr_config_[index].address;

    // Step 1: 只对这一个电机清多圈编码器 (先失能状态下清零, 避免瞬间位置突变导致抽搐)
    multi_turn_zeroing({address});
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    // Step 2: 只使能这一个电机, 用当前反馈位置刷新驱动状态 (跟 saveAsZeroPosition 既有写法一致)
    float state_list[6] = {0};
    uint8_t motor_errcode;
    int errorcode = enter_motor_state(&ruiwoCpp, address, state_list, &motor_errcode);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    if (errorcode != 0) {
        std::cerr << "[RUIWO motor]:ID: " << address << " calibrateSingleMotor 使能失败, errorcode: " << errorcode << std::endl;
        return false;
    }

    errorcode = run_ptm_mode(&ruiwoCpp, address, state_list[1], 0,
        ruiwo_mtr_config_[index].parameter.kp_pos, ruiwo_mtr_config_[index].parameter.kd_pos, 0, state_list);
    if (errorcode != 0) {
        std::cerr << "[RUIWO motor]:ID: " << address << " calibrateSingleMotor run_ptm_mode 失败, errorcode: " << errorcode << std::endl;
        return false;
    }
    std::vector<float> state;
    state.assign(state_list, state_list + 6);
    set_joint_state(index, state);

    // Step 3: 读取清零/使能后的原始位置, 计算零点
    bool ok = true;
    if (isPositionInvalid(origin_joint_status[index][1])) {
        std::cerr << "\033[31m[RUIWO motor]:ID: " << address << " calibrateSingleMotor 位置异常(接近或超过12.5rad), 拒绝标定\033[0m" << std::endl;
        ok = false;
    } else {
        ruiwo_mtr_config_[index].zero_offset = origin_joint_status[index][1];
        std::cout << "[RUIWO motor]:ID: " << address << " calibrateSingleMotor new zero offset: "
                   << ruiwo_mtr_config_[index].zero_offset << std::endl;
        update_status();
        // 只重写当前所有电机的实时零点偏移(其它电机内存中的值未被改动), 落盘安全
        saveZeroPosition();
    }

    // Step 4: 标完(不论成功失败)重新失能这一个电机, 回到跟启动时一致的安全静止状态
    uint8_t reset_errcode;
    int disable_errorcode = enter_reset_state(&ruiwoCpp, address, state_list, &reset_errcode);
    if (disable_errorcode != 0) {
        std::cerr << "\033[31m[RUIWO motor]:ID: " << address << " calibrateSingleMotor 标定后重新失能失败, 请手动确认电机状态\033[0m" << std::endl;
    }

    return ok;
}

std::vector<double> RuiWoActuator::getMotorZeroPoints() {
    std::vector<double> zero_points;
    for (const auto& config : ruiwo_mtr_config_) {
        zero_points.push_back(config.zero_offset);
    }
    return zero_points;
}


void RuiWoActuator::saveZeroPosition() {
    std::string config_path = ruiwo_utils::GetZeroFilePath();
    std::string backup_path = config_path + ".bak";
    
    YAML::Node ym_zero_position;
    for(auto&mtr: ruiwo_mtr_config_)
        ym_zero_position.push_back(mtr.zero_offset);
    
    YAML::Node config;
    config["arms_zero_position"] = ym_zero_position;

    // 备份配置文件
    if (std::filesystem::exists(config_path)) {
        std::filesystem::copy_file(config_path, backup_path, std::filesystem::copy_options::overwrite_existing);
        std::cout << "[RUIWO motor]: Backup config file to " << backup_path << std::endl;
    }

    // 保存配置文件
    std::ofstream file(config_path);
    if (file.is_open()) {
        file << config;
        file.close();
        std::cout << "[RUIWO motor]: Zero position saved to " << config_path << std::endl;
    } else {
        std::cerr << "[RUIWO motor]: Failed to open config file for writing." << std::endl;
    }
}

void RuiWoActuator::set_zero()
{
    float state_list[6];
    for(int i = 0; i < ruiwo_mtr_config_.size(); i++) {
        if(ruiwo_mtr_config_.at(i).address == kDisableAddress) {
            continue;
        }

        int errorcode = set_zero_position(&ruiwoCpp, ruiwo_mtr_config_.at(i).address, state_list);
        if (errorcode == 0) {
            std::vector<float> state;
            state.assign(state_list, state_list + 6);
            set_joint_state(i, state);
        }
        else {
            std::cout << "[RUIWO motor] set zero position error, address: " << ruiwo_mtr_config_.at(i).address << "errcode: " << errorcode << std::endl;
        }
        std::cout << "[RUIWO motor]:Set motor zero position, address: " << state_list[1] << std::endl;
    }
}

float RuiWoActuator::measure_head_torque(float pos)
{
    float torque_coefficient = 1;
    float sin_coefficient = -0.45;
    float torque = (torque_coefficient / sin(sin_coefficient)) * sin(pos);
    return torque;
}


void RuiWoActuator::send_positions(const std::vector<int> &index, 
    const std::vector<float> &pos, 
    const std::vector<float> &torque, 
    const std::vector<float> &velocity, 
    const std::vector<float> *current_kp, 
    const std::vector<float> *current_kd)
{
    if (!thread_running) {
        return;
    }

    if (index.size() != pos.size() || index.size() != torque.size() || index.size() != velocity.size()) {
        std::cout << "[RUIWO motor] send_positions error, dimensions do not match. Size of index: " 
        << index.size() << ", Size of pos: " << pos.size() << ", Size of torque: " << torque.size() << ", Size of velocity: " << velocity.size() << std::endl;
        return;
    }

    for (int i : index)
    {
        auto &mtr_config = ruiwo_mtr_config_.at(i);
        if (!mtr_config.online) {
            continue;
        }

        int address = mtr_config.address;
        float q = pos.at(i);
        float v = velocity.at(i);
        float tau = torque.at(i);

        if(mtr_config.negtive) { // Negtive
            q = -(q + mtr_config.zero_offset);
            v = velocity_factor * -v;
            tau = std::max<float>(-10.0f, -tau);
        } else {
            q = pos[i] + mtr_config.zero_offset;
            tau = std::min<float>(10.0f, tau);
            v = velocity_factor * v;
        }
        
        if (address == Head_joint_address[1]) {
            tau = head_high_torque;
        }

        float state_list[6];
        int errorcode = 1;
        auto &param = mtr_config.parameter;
        float kp = (current_kp ? (*current_kp)[i] : param.kp_pos);
        float kd = (current_kd ? (*current_kd)[i] : param.kd_pos);
        if (Control_mode_ == "ptm")
        {
            if(teach_pendant_mode == 1){
                errorcode = run_ptm_mode(&ruiwoCpp, address, 0, 0, 0, 0, 0, state_list);
            }
            else {
                errorcode = run_ptm_mode(&ruiwoCpp, address, q, v, kp, kd, tau, state_list);
            }
        }
        else if (Control_mode_ == "servo")
        {
            errorcode = run_servo_mode(&ruiwoCpp, address, q, v, kp, kd, param.kp_vel, param.kd_vel, param.ki_vel, state_list);
        }
        
        if (errorcode == 0){
            // update state
            std::vector<float> state;
            state.assign(state_list, state_list + 6);
            set_joint_state(i, state);
            update_status();
            if(address == Head_joint_address[1]) {
                this->head_high_torque = this->measure_head_torque(this->current_positions.at(i));
            }
        }
        else
        {
            std::cout << "[RUIWO motor]:ID:" << address << " Send position: [Failed],Errorcode: [" << errorcode << "]" << std::endl;
        }
    }
}

void RuiWoActuator::set_teach_pendant_mode(int mode){

    if(teach_pendant_mode == 1)
        std::cout << "[RUIWO motor]进入teach_pendant模式" << std::endl;
    teach_pendant_mode = mode;
}

void RuiWoActuator::send_positions_No_response(const std::vector<int> &index, 
    const std::vector<float> &pos, 
    const std::vector<float> &torque, 
    const std::vector<float> &velocity, 
    const std::vector<float> *current_kp, 
    const std::vector<float> *current_kd)
{
    if (!thread_running) {
        return;
    }

    if (index.size() != pos.size() || index.size() != torque.size() || index.size() != velocity.size()) {
        std::cout << "[RUIWO motor] send_positions_No_response error, dimensions do not match. Size of index: " 
        << index.size() << ", Size of pos: " << pos.size() << ", Size of torque: " << torque.size() << ", Size of velocity: " << velocity.size() << std::endl;
        return;
    }

    for (int i : index) {
        auto &mtr_config = ruiwo_mtr_config_.at(i);
        if (!mtr_config.online) {
            continue;
        }

        int address = mtr_config.address;
        float q = pos.at(i);
        float v = velocity.at(i);
        float tau = torque.at(i);
        if(mtr_config.negtive) { // Negtive
            q = -(q + mtr_config.zero_offset);
            v = velocity_factor * -v;
            tau = std::max<float>(-10.0f, -tau);
        } else {
            q = pos[i] + mtr_config.zero_offset;
            tau = std::min<float>(10.0f, tau);
            v = velocity_factor * v;
        }
        
        if (address == Head_joint_address[1]) {
            tau = head_high_torque;
        }

        float state_list[6];
        int errorcode = 1;
        auto &param = mtr_config.parameter;
        float kp = (current_kp ? (*current_kp)[i] : param.kp_pos);
        float kd = (current_kd ? (*current_kd)[i] : param.kd_pos);
        if (Control_mode_ == "ptm") {
            if(teach_pendant_mode) {
                g_ruiwo_observer.increment_tx_count(); // Debug
                errorcode = run_ptm_mode_No_response(&ruiwoCpp, address, 0, 0, 0, 0, 0, state_list);
            }
            else {
                g_ruiwo_observer.increment_tx_count(); // Debug
                errorcode = run_ptm_mode_No_response(&ruiwoCpp, address, q, v, kp, kd, tau, state_list);
            }
        }
        else if (Control_mode_ == "servo")
        {
            errorcode = run_servo_mode(&ruiwoCpp, address, q, v, kp, kd, param.kp_vel, param.kd_vel, param.ki_vel, state_list);
        }
        
        if (errorcode == 0){
            // std::cout << "errorcode: " << errorcode << std::endl;
        }
        else
        {
            std::cout << "[RUIWO motor]:ID:" << address << " Send position: [Failed], Errorcode: [" << errorcode << "]" << std::endl;
        }
        // update_status_asynchronous();
        // update_status();
    }
}

void RuiWoActuator::set_positions(const std::vector<uint8_t> &index,
    const std::vector<double> &positions,
    const std::vector<double> &torque,
    const std::vector<double> &velocity,
    const std::vector<double> &kp_pos,
    const std::vector<double> &kd_pos)
{
    if (!thread_running) {
        std::cout << "[RUIWO motor]: [Warn] control thread is not running" << std::endl;
        return;
    }

    // 检查参数的维度是否正确
    if (index.size() != positions.size() || index.size() != torque.size() || index.size() != velocity.size()) {
        std::cout << "[RUIWO motor]: [Error] Dimension mismatch in set_positions:"
            << "  index size: " << index.size()
            << ", positions size: " << positions.size()
            << ", torque size: " << torque.size()
            << ", velocity size: " << velocity.size() << std::endl;
        return;
    }

    const bool use_cmd_gains = !kp_pos.empty() && !kd_pos.empty();
    if (use_cmd_gains &&
        (kp_pos.size() != index.size() || kd_pos.size() != index.size())) {
        std::cout << "[RUIWO motor]: [Error] kp/kd dimension mismatch in set_positions:"
            << " index(" << index.size() << "), kp_pos(" << kp_pos.size()
            << "), kd_pos(" << kd_pos.size() << ")" << std::endl;
        return;
    }

    std::vector<double> rad_position(positions.size());
    std::vector<double> rad_velocity(velocity.size());
    for (size_t i = 0; i < positions.size(); i++) {
        rad_position[i] = positions[i] * M_PI / 180;  // to radians
        rad_velocity[i] = velocity[i] * M_PI / 180;   // to radians/s
    }

    std::vector<float> new_positions, new_torque, new_velocity, new_kp, new_kd;
    {
        std::lock_guard<std::mutex> lock_sendpos(sendpos_lock), lock_sendvel(sendvel_lock), lock_sendtor(sendtor_lock);
        new_positions = target_positions;
        new_torque = target_torque;
        new_velocity = target_velocity;
        new_kp = target_kp_pos;
        new_kd = target_kd_pos;
    }

    for (size_t i = 0; i < ruiwo_mtr_config_.size(); ++i) {
        int address = ruiwo_mtr_config_[i].address;
        if (address == kDisableAddress) {
            continue;
        }

        for (size_t j = 0; j < index.size(); ++j) {
            if (i == static_cast<int>(index[j])) {
                new_positions[i] = rad_position[j];
                new_torque[i] = torque[j];
                new_velocity[i] = rad_velocity[j];
                if (use_cmd_gains) {
                    new_kp[i] = static_cast<float>(kp_pos[j]);
                    new_kd[i] = static_cast<float>(kd_pos[j]);
                }
            }
        }
    }
    {
        std::lock_guard<std::mutex> lock_sendpos(sendpos_lock), lock_sendvel(sendvel_lock), lock_sendtor(sendtor_lock);
        target_positions = new_positions;
        target_torque = new_torque;
        target_velocity = new_velocity;
        if (use_cmd_gains) {
            target_kp_pos = new_kp;
            target_kd_pos = new_kd;
        }
    }
    target_has_cmd_gains_ = use_cmd_gains;

    // 退出 CST 模式，恢复使用配置的 kp/kd
    torque_only_mode_ = false;

    {
        std::lock_guard<std::mutex> lock(update_lock);
        target_update = true;
    }
}

void RuiWoActuator::set_torque(const std::vector<uint8_t> &index, const std::vector<double> &torque)
{
   if (!thread_running) {
        std::cout << "[RUIWO motor]: [Warn] set_torque, control thread is not running!" << std::endl;
        return;
    }

    // 检查参数的维度是否正确
    if (index.size() != torque.size()) {
        std::cout << "[RUIWO motor]: [Error] Dimension mismatch in set_torque:"
            << "  index size: " << index.size() 
            << ", torque size: " << std::endl;
        return;
    }

    std::vector<float> new_torque;
    {
        std::lock_guard<std::mutex> lock(sendtor_lock);
        new_torque = target_torque;
    }

    for (size_t i = 0; i < ruiwo_mtr_config_.size(); ++i) {
        int address = ruiwo_mtr_config_[i].address;
        if (address == kDisableAddress) {
            continue;
        }

        for (size_t j = 0; j < index.size(); ++j) {
            if (address - 1 == static_cast<int>(index[j]))
            {
                if (ruiwo_mtr_config_[i].negtive) {
                    new_torque[i] = -torque[j];
                }
                else
                {
                    new_torque[i] = torque[j];
                }
                break;
            }
            new_torque[i] = std::max<float>(-10.0f, std::min<float>(new_torque[i], 10.0f));
        }
    }
    {
        std::lock_guard<std::mutex> lock(sendtor_lock);
        target_torque = new_torque;
    }

    {
        std::lock_guard<std::mutex> lock(update_lock);
        target_update = true;
    }
}

void RuiWoActuator::set_velocity(const std::vector<uint8_t> &index, const std::vector<double> &velocity)
{
    if (!thread_running)
    {
        return;
    }

    std::vector<float> new_velocity;
    {
        std::lock_guard<std::mutex> lock(sendvel_lock);
        new_velocity = target_velocity;
    }

    for (size_t i = 0; i < ruiwo_mtr_config_.size(); ++i) {
        int address = ruiwo_mtr_config_[i].address;
        if (address == kDisableAddress) {
            continue;
        }
        
        for (size_t j = 0; j < index.size(); ++j)
        {
            if (address - 1 == static_cast<int>(index[j])) {
                if (ruiwo_mtr_config_[i].negtive)
                {
                    new_velocity[i] = -velocity[j];
                }
                else
                {
                    new_velocity[i] = velocity[j];
                }
                break;
            }
        }
    }
    {
        std::lock_guard<std::mutex> lock(sendvel_lock);
        target_velocity = new_velocity;
    }

    {
        std::lock_guard<std::mutex> lock(update_lock);
        target_update = true;
    }
}

std::vector<double> RuiWoActuator::get_positions()
{
    if (!thread_running){
        return {};
    }

    std::vector<double> pose_double;

    {
        // std::lock_guard<std::mutex> lock(recvpos_lock);
        int size = current_positions.size();
        pose_double.resize(size);
        for (size_t i = 0; i < size; i++)
        {
            pose_double[i] = static_cast<double>(current_positions[i]);
        }
    }
    return pose_double;
}

std::vector<double> RuiWoActuator::get_torque()
{
    if (!thread_running) {
        return {};
    }
    std::vector<double> torque_double;


    std::lock_guard<std::mutex> lock(recvtor_lock);
    int size = current_torque.size();
    torque_double.resize(size);
    for (size_t i = 0; i < size; i++)
    {
        torque_double[i] = static_cast<double>(current_torque[i]);
    }
    return torque_double;
}

std::vector<double> RuiWoActuator::get_velocity()
{
    if (!thread_running)
    {
        return {};
    }
    std::vector<double> velocity_double;
    
    std::lock_guard<std::mutex> lock(recvvel_lock);
    int size = current_velocity.size();
    velocity_double.resize(size);
    for (size_t i = 0; i < size; i++)
    {
        velocity_double[i] = static_cast<double>(current_velocity[i]);
    }

    return velocity_double;
}

void RuiWoActuator::set_joint_state(int index, const std::vector<float> &state)
{
    // if (!thread_running) {
    //     return;
    // }

    /**
    * See Details:《Motorevo Driver User Guide v0.2.3》- 6.5.1 反馈帧格式
    * 无论在什么状态和模式下，驱动板在收到报文标识符为 CAN COM ID 的报文时都会进行一
    * 次应答，反馈当前电机的位置(rad)，速度(rad/s)，扭矩(N·m)。应答的报文标识符恒定本机
    * CAN ID，波特率为 1Mbps，DLC 为 8 字节。
    * +---------+---------+---------+---------+---------+---------+---------+---------+---------+
    * | Byte0   | Byte1   | Byte2   | Byte3   | Byte4_H |Byte4_L  | Byte5   | Byte6_H | Byte7   |
    * +---------+---------+---------+---------+---------+---------+---------+---------+---------+
    * | Motor   |position |position |velocity |velocity | torque  | torque  | errcode | tempe-  |
    * | Id      | High    | Low     | High    | Low     | High    | Low     |         | rature  |
    * | 8bits   | 8bits   | 8bits   | 8bits   | 4bits   | 4bits   | 8bits   |         |         |
    * +---------+---------+---------+---------+---------+---------+---------+---------+---------+
    */

    auto new_state = state;
    int address = static_cast<int>(new_state[0]);

    auto iter = std::find_if(ruiwo_mtr_config_.begin(), ruiwo_mtr_config_.end(), 
        [address](const RuiwoMotorConfig_t& config) { return config.address == address && config.negtive; });
    if (iter != ruiwo_mtr_config_.end()) {
        new_state[1] = -new_state[1]; // position
        new_state[2] = -new_state[2]; // velocity
        new_state[3] = -new_state[3]; // torque
    }

    // 保存原始状态
    std::vector<float> origin_states = new_state;

    // 减去零点偏移
    new_state[1] -= ruiwo_mtr_config_.at(index).zero_offset;

    // 添加小的容差范围，避免浮点数误差
    if (std::abs(new_state[1]) < 1e-10) {
        new_state[1] = 0.0;
    }
    if (std::abs(new_state[2]) < 1e-10) {
        new_state[2] = 0.0;
    }
    if (std::abs(new_state[3]) < 1e-10) {
        new_state[3] = 0.0;
    }
   
    {
        std::lock_guard<std::mutex> lock(state_lock);
        joint_status[index] = new_state;
        origin_joint_status[index] = origin_states;
    }
}

std::vector<std::vector<float>> RuiWoActuator::get_joint_state()
{
    if (!thread_running)
    {
        return {};
    }
    std::vector<std::vector<float>> joint_status_copy;
    {
        std::lock_guard<std::mutex> lock(state_lock);
        joint_status_copy = joint_status;
    }
    return joint_status_copy;
}

void RuiWoActuator::close()
{
    thread_running = false;
    if (control_thread_.joinable()) {
        control_thread_.join();
    }

    if(recv_thread_.joinable()) {
        recv_thread_.join();
    }

    auto time0 = std::chrono::system_clock::now();
    auto time0_milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(time0.time_since_epoch()).count();
    while (!thread_end) {
        auto time1 = std::chrono::system_clock::now();
        auto time1_milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(time1.time_since_epoch()).count();
        auto time_diff = time1_milliseconds - time0_milliseconds;
        if (time_diff > 1000)
        {
            std::cout << "[RUIWO motor]:Threadend Timeout" << std::endl;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    disable();
    auto errorcode = close_canbus(&ruiwoCpp);
    if (errorcode == 0) {
        std::cout << "[RUIWO motor]:Canbus status: [Close]" << std::endl;
    }
    else {
        std::cout << "[RUIWO motor]:Close canbus failed, Errorcode: " << "[" << errorcode << "]" << std::endl;
    }
}

RuiWoActuator::MotorStateDataVec RuiWoActuator::get_motor_state()
{
    MotorStateDataVec motor_states;
    for (const auto &motor : ruiwo_mtr_config_) {
        MotorStateData state;
        state.id = static_cast<uint8_t>(motor.address);
        if(motor.online == true) {
            state.state = State::Enabled;
        }   
        else {
            state.state = State::Disabled;
        }

        motor_states.push_back(state); 
    }
    return motor_states;
}

std::vector<std::vector<float>> RuiWoActuator::get_joint_origin_state() {
    if (!thread_running) {
        return {};
    }
    std::lock_guard<std::mutex> lock(state_lock);
    return origin_joint_status;
}

void RuiWoActuator::set_joint_gains(const std::vector<int> &joint_indices, const std::vector<double> &kp_pos, const std::vector<double> &kd_pos)
{
    for (size_t i = 0; i < joint_indices.size(); ++i) {
        int joint_idx = joint_indices[i];
        if (joint_idx < 0 || joint_idx >= static_cast<int>(ruiwo_mtr_config_.size())) {
            std::cout << "[RUIWO motor]: Warning: joint index " << joint_idx << " out of range" << std::endl;
            continue;
        }

        if (!kp_pos.empty() && i < kp_pos.size()) {
            ruiwo_mtr_config_[joint_idx].parameter.kp_pos = static_cast<int>(kp_pos[i]);
            std::cout << "[RUIWO motor]: Set joint " << joint_idx << " kp_pos to " << kp_pos[i] << std::endl;
        }

        if (!kd_pos.empty() && i < kd_pos.size()) {
            ruiwo_mtr_config_[joint_idx].parameter.kd_pos = static_cast<int>(kd_pos[i]);
            std::cout << "[RUIWO motor]: Set joint " << joint_idx << " kd_pos to " << kd_pos[i] << std::endl;
        }
    }
}

std::vector<std::vector<double>> RuiWoActuator::get_joint_gains(const std::vector<int> &joint_indices)
{
    std::vector<double> kp_values, kd_values;
    
    if (joint_indices.empty()) {
        // Return all joints
        for (const auto& config : ruiwo_mtr_config_) {
            kp_values.push_back(static_cast<double>(config.parameter.kp_pos));
            kd_values.push_back(static_cast<double>(config.parameter.kd_pos));
        }
    } else {
        // Return specified joints
        for (int joint_idx : joint_indices) {
            if (joint_idx < 0 || joint_idx >= static_cast<int>(ruiwo_mtr_config_.size())) {
                std::cout << "[RUIWO motor]: Warning: joint index " << joint_idx << " out of range" << std::endl;
                continue;
            }
            kp_values.push_back(static_cast<double>(ruiwo_mtr_config_[joint_idx].parameter.kp_pos));
            kd_values.push_back(static_cast<double>(ruiwo_mtr_config_[joint_idx].parameter.kd_pos));
        }
    }
    
    return {kp_values, kd_values};
}

// void RuiWoActuator::update_status_asynchronous() {
//     try {
//         BM_CanMessageTypeDef rx_msg;
//         uint32_t port;
//         uint32_t timestamp;

//         BM_StatusTypeDef error = BM_ReadCanMessage(ruiwoCpp.channel, &rx_msg, &port, &timestamp);
        
//         if(error == BM_ERROR_QRCVEMPTY) {
//             return;
//         }
//         else if (error != BM_ERROR_OK) {
//             std::cerr << "Failed to receive message, error code: " << error << std::endl;
//             return;
//         }

//         // Increment the received CAN message counter
//         received_can_message_count_++;
        
//         // Publish the updated counter to ROS topic
//         std_msgs::UInt32 count_msg;
//         count_msg.data = received_can_message_count_.load();
//         received_can_count_pub_.publish(count_msg);

//         int msg_id = BM_GET_STD_MSG_ID(rx_msg.id) & 0x7F;
//         if (msg_id != 0) {
//             // 检查 arbitration_id 是否在关节地址列表中
//             auto iter = std::find_if(ruiwo_mtr_config_.begin(), ruiwo_mtr_config_.end(), [sid = msg_id](const RuiwoMotorConfig_t &config) {
//                 return config.address == sid;
//             });
//             if (iter != ruiwo_mtr_config_.end()) {
//                 int rx_id = static_cast<int>(rx_msg.payload[0]&0x7F);
//                 // 双重判断: arbitration_id == rx_id
//                 auto iter1 = std::find_if(ruiwo_mtr_config_.begin(), ruiwo_mtr_config_.end(), [rx_id](const RuiwoMotorConfig_t &config) {
//                     return config.address == rx_id;
//                 });
//                 if (rx_id == msg_id && iter1 != ruiwo_mtr_config_.end()) {
//                     std::vector<float> motor_state(6);
//                     return_motor_state(&rx_msg, motor_state.data(), NULL, &ruiwoCpp.can_com_param);
//                     set_joint_state(rx_id - 1, motor_state);
//                 }
//                 else {
//                     std::cout << "rx_id mismatch "<< rx_id <<" : rx_msg.id.SID : "<< msg_id << "\n";
//                 }
//             }
//             else{
//                 std::cout << "没有找到 sid :"<< msg_id << "\n";
//             }
//         }
//     } catch (const std::exception& e) {
//         std::cerr << "接收线程异常: " << e.what() << std::endl;
//     }
// }
