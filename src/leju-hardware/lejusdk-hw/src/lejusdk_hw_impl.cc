#include "lejusdk_hw/lejusdk_hw.h"
#include "utils/path_utils.h"
#include "utils/file.h"
#include "lejusdk_hw_log.h"

#include "hardware_plant.h"

#include <cstdlib> // for getenv
#include <functional>
#include <vector>
#include <iostream>
#include <cstring> // for strchr
#include <chrono>

// 辅助函数：打印向量维度
template<typename T, typename... Args>
void printVectorDimsHelper(const char* names, const T& first, const Args&... args) {
    std::cout << names << ": " << first.size() << std::endl;
    if constexpr (sizeof...(args) > 0) {
        // 找到下一个逗号的位置
        const char* next_comma = std::strchr(names, ',');
        if (next_comma) {
            printVectorDimsHelper(next_comma + 1, args...);
        }
    }
}

// 宏定义：打印向量的维度信息（支持任意数量参数）
#define PRINT_VECTOR_DIMS(...) \
    do { \
        std::cout << "Vector dimensions:" << std::endl; \
        printVectorDimsHelper(#__VA_ARGS__, __VA_ARGS__); \
    } while(0)

namespace leju {
namespace hw {

HardwareInterface& HardwareInterface::GetInstance() {
    static HardwareInterface instance;
    return instance;
}

ImuType HardwareInterface::GetImuType() {
    std::string imu_config_path = path::imu_type_path();
    std::string imu_type_str;

    HWLOG_I("Reading IMU type from config file: %s", imu_config_path.c_str());
    bool result = FileUtils::ReadToString(imu_config_path, imu_type_str);
    if(!result) {
        HWLOG_WARNING("Failed to read IMU type config file: %s", imu_config_path.c_str());
        return ImuType::None;
    }

    HWLOG_I("IMU type read from config: [%s]", imu_type_str.c_str());
    return String2ImuType(imu_type_str);
}

HwErrorType HardwareInterface::Initialize(const std::string &config_dir) {
    if(!FileUtils::Exists(config_dir) || !FileUtils::IsDirectory(config_dir)) {
        HWLOG_E("Path not Found or Path is not Directory: %s", config_dir.c_str());
        return HwErrorType::PathNotFound;
    }

    // 1. 初始化硬件参数
    HighlyDynamic::HardwareParam hardware_param;
    hardware_param.cali_leg = false;
    hardware_param.cali_arm = false;
    hardware_param.cali     = false;
    hardware_param.teach_pendant_ = false;
    hardware_param.only_half_up_body = false;
    hardware_param.robot_version = RobotVersion::from_env();
    hardware_param.kuavo_assets_path = config_dir;
    
    HWLOG_I("Hardware version: %s", hardware_param.robot_version.to_string().c_str());

    const double dt = 0.001; // TODO(ohh): 更正
    hardware_plant_ = new HighlyDynamic::HardwarePlant(dt, hardware_param);

    if (hardware_plant_->HWPlantInit() != 0) {
        return HwErrorType::Failed;
    }

    hardware_settings_ = new HighlyDynamic::HardwareSettings(hardware_plant_->get_motor_info());

    initialized_.store(true);

    JointData_t joint_data;
    if(this->GetJointState(joint_data) != HwErrorType::Success) {
        HWLOG_FAILURE("Init, Failed to get joint state.");
        initialized_.store(false);
        return HwErrorType::Failed;
    }

    return HwErrorType::Success;
}

void HardwareInterface::Shutdown() {
    // if(!initialized_) {
    //     return;
    // }
    
    initialized_.store(false);
    if(hardware_plant_) {
        hardware_plant_->HWPlantDeInit();
        delete hardware_plant_;
        hardware_plant_ = nullptr;
        delete hardware_settings_;
        hardware_settings_ = nullptr;
    }
}

HwErrorType HardwareInterface::SetJointCommand(const JointCommand_t &joint_command) {
    // 检查是否已初始化
    if (!initialized_) {
        return HwErrorType::NotInitialized;
    }
    // 检查维度
    if (!joint_command.isValid()) {
        PRINT_VECTOR_DIMS(joint_command.q,
            joint_command.v,
            joint_command.tau,
            joint_command.kp,
            joint_command.kd,
            joint_command.modes);
        return HwErrorType::DimensionMismatch;
    }

    /////////////// JointSpace 2 MotorSpace /////////////////////
    uint32_t num_joint = joint_command.q.size();
    Eigen::VectorXd cmd_out(num_joint * 5);
    Eigen::VectorXd cmd_input(num_joint * 5);
    std::vector<int> control_modes(num_joint);
    Eigen::VectorXd joint_kp(num_joint);
    Eigen::VectorXd joint_kd(num_joint);

    for (uint32_t i = 0; i < num_joint; ++i) {
        joint_kp(i) =joint_command.kp.at(i);
        joint_kd(i) = joint_command.kd.at(i);
        control_modes[i] = static_cast<int>(joint_command.modes.at(i));

        cmd_input[num_joint * 0 + i] = joint_command.q.at(i);          
        cmd_input[num_joint * 1 + i] = joint_command.v.at(i);     
        cmd_input[num_joint * 2 + i] = joint_command.tau.at(i);       
        cmd_input[num_joint * 3 + i] = hardware_settings_->max_current.at(i);  // tau_max  
        // cmd_input[num_joint * 3 + i] = joint_command.tau_max.at(i);     // tau_max
        cmd_input[num_joint * 4 + i] = joint_command.tau_ratio.at(i);   // tau_ratio
        
    }

    // Use HardwarePlant's command processing
    hardware_plant_->cmds2Cmdr(cmd_input, num_joint, cmd_out, num_joint);

    // Handle NaN values
    if (cmd_out.hasNaN()) {
        std::cerr << "Command output has NaN values, setting to zero" << std::endl;
        return HwErrorType::Failed;
    }
    ////////////////////////////////////////////////////////////////

    // Motor Command Write
    hardware_plant_->writeCommand(cmd_out, num_joint, control_modes, joint_kp, joint_kd);

    return HwErrorType::Success;
    #undef CHECK_JOINT_COMMAND_DIMS
}

HwErrorType HardwareInterface::GetJointState(JointData_t &joint_data) {
    if (!initialized_) {
        return HwErrorType::NotInitialized;
    }

    using Clock = std::chrono::steady_clock;
    auto t0 = Clock::now();

    SensorData_t sensor_data_motor, sensor_data_joint;
    sensor_data_joint.resizeJoint(hardware_plant_->get_num_actuated());
    // Read current sensor data using parent class method
    if (!hardware_plant_->readSensor(sensor_data_motor)) {
        return HwErrorType::Failed;
    }
    auto t1 = Clock::now();

    hardware_plant_->motor2joint(sensor_data_motor, sensor_data_joint);
    auto t2 = Clock::now();
    hardware_plant_->setState(sensor_data_motor, sensor_data_joint);

    // Fill joint_data from sensor_data_joint
    size_t num_joints = sensor_data_joint.joint_q.size();
    joint_data.resize(num_joints);
    for (size_t i = 0; i < num_joints; ++i) {
        joint_data.q.at(i) = sensor_data_joint.joint_q(i);
        joint_data.v.at(i) = sensor_data_joint.joint_v(i);
        joint_data.vd.at(i) = sensor_data_joint.joint_vd(i);
        joint_data.tau.at(i) = sensor_data_joint.joint_current(i);
    }
    auto t3 = Clock::now();

    // 周期性打印 GetJointState 各阶段耗时
    {
        auto us = [](auto a, auto b) {
            return std::chrono::duration_cast<std::chrono::microseconds>(b - a).count();
        };
        auto total = us(t0, t3);
        if (total > 300) {
            static auto last_dbg = Clock::now() - std::chrono::seconds(5);
            auto now = Clock::now();
            if (std::chrono::duration_cast<std::chrono::seconds>(now - last_dbg).count() >= 5) {
                printf("[GetJointState] readSensor=%ldus motor2joint=%ldus setState+copy=%ldus total=%ldus\n",
                       us(t0,t1), us(t1,t2), us(t2,t3), total);
                last_dbg = now;
            }
        }
    }

    return HwErrorType::Success;
}

HwErrorType HardwareInterface::GetMotorState(JointData_t &motor_data) {
    if (!initialized_) {
        return HwErrorType::NotInitialized;
    }

    SensorData_t snap;
    if (!hardware_plant_->GetMotorSensorSnapshot(snap)) {
        return HwErrorType::Failed;
    }
    const size_t n = snap.joint_q.size();
    motor_data.resize(n);
    for (size_t i = 0; i < n; ++i) {
        motor_data.q[i]   = snap.joint_q(i);
        motor_data.v[i]   = snap.joint_v(i);
        motor_data.vd[i]  = snap.joint_vd(i);
        motor_data.tau[i] = snap.joint_current(i);   // 原始电机 tau，未乘 c2t
    }
    return HwErrorType::Success;
}

HwErrorType HardwareInterface::GetMotorCmd(JointCommand_t &motor_cmd) {
    if (!initialized_) {
        return HwErrorType::NotInitialized;
    }

    HighlyDynamic::HardwarePlant::MotorCmdSnapshot snap;
    if (!hardware_plant_->GetMotorCmdSnapshot(snap)) {
        return HwErrorType::Failed;
    }
    const size_t n = snap.q.size();
    motor_cmd.resize(n);
    for (size_t i = 0; i < n; ++i) {
        motor_cmd.q[i]         = snap.q[i];
        motor_cmd.v[i]         = snap.v[i];
        motor_cmd.tau[i]       = snap.tau[i];
        motor_cmd.kp[i]        = snap.kp[i];
        motor_cmd.kd[i]        = snap.kd[i];
        motor_cmd.tau_max[i]   = snap.tau_max[i];
        motor_cmd.tau_ratio[i] = 1.0;                 // motor_cmd 是 post-tau_ratio
        motor_cmd.modes[i]     = snap.modes[i];
    }
    return HwErrorType::Success;
}

HwErrorType HardwareInterface::JointMoveTo(const std::vector<double> &goal_pos, 
    double speed, double dt /*= 1e-3*/, 
    double current_limit /*=-1*/) {
    if (!initialized_) {
        return HwErrorType::NotInitialized;
    }

    if(goal_pos.size() != hardware_plant_->get_num_actuated()) {
        return HwErrorType::DimensionMismatch;
    }

    hardware_plant_->jointMoveTo(goal_pos, speed, dt, current_limit);

    return HwErrorType::Success;
}

// HwErrorType HardwareInterface::SetMotorCommand(const MotorCommand_t &motor_command) {
//     // 检查是否已初始化
//     if (!initialized_) {
//         return HwErrorType::NotInitialized;
//     }

//     // TODO(ohh): 实现！
//     std::cout << "Not implemented yet: SetMotorCommand" << std::endl;

//     return HwErrorType::Success;
// }

// HwErrorType HardwareInterface::GetMotorState(MotorData_t &motor_data) {
//     // 检查是否已初始化
//     if (!initialized_) {
//         return HwErrorType::NotInitialized;
//     }

//     // TODO(ohh): 实现！
//     std::cout << "Not implemented yet: GetMotorState" << std::endl;

//     return HwErrorType::Success;
// }

HwErrorType HardwareInterface::GetRawJointState(JointData_t &joint_data) {
    if (!initialized_) {
        return HwErrorType::NotInitialized;
    }

    std::vector<double> pos_rad, vel_rad;
    hardware_plant_->getRawMotorData(pos_rad, vel_rad);

    joint_data.resize(pos_rad.size());
    for (size_t i = 0; i < pos_rad.size(); i++) {
        joint_data.q[i] = pos_rad[i];
        joint_data.v[i] = vel_rad[i];
    }
    return HwErrorType::Success;
}

HwErrorType HardwareInterface::GetImuData(ImuData_t &imu_data) {
    if (!initialized_) {
        return HwErrorType::NotInitialized;
    }

    imu_data.reset();
    auto read_ret = hardware_plant_->readImuData(imu_data);
    if(-2 == read_ret) {
        return HwErrorType::ReadSensorFailed;
    }
    else if(-1 == read_ret) {
        return HwErrorType::ImuUnavailable;
    }
    else if(0 == read_ret) {
        return HwErrorType::Success;
    }

    return HwErrorType::Failed;
}

uint32_t HardwareInterface::GetMotorNumber() const {
    if (!initialized_) {
        return 0; // warning: not initialized
    }
    return hardware_plant_->get_num_actuated();
}

bool HardwareInterface::HasECMasterMotors() const {
    if (!initialized_) {
        return false;
    }
    return hardware_plant_->getCountECMasters() > 0;
}

bool HardwareInterface::HasLowerLimbMotorFault(std::string &fault_info, bool ready_ok) {
    if (!initialized_) {
        return false;
    }
    return hardware_plant_->checkLowerLimbMotorFault(fault_info, ready_ok);
}

void HardwareInterface::SetBypassAnkleSolver(bool v) {
    if (hardware_plant_) {
        hardware_plant_->setBypassAnkleSolver(v);
    }
}

void HardwareInterface::SetRuntimeSkipActive(bool active) {
    if (hardware_plant_) {
        hardware_plant_->setRuntimeSkipActive(active);
    }
}

HwErrorType HardwareInterface::SetHandCommand(const HandCommand_t &hand_command) {
    if (!initialized_) {
        return HwErrorType::NotInitialized;
    }
    if (!hand_command.isValid()) {
        return HwErrorType::DimensionMismatch;
    }

    // 将 12 维手部命令转换为 EndEffectorData（左右手各6维）
    std::vector<EndEffectorData> eef_data;
    eef_data.push_back({EndEffectorType::revo2,
                        Eigen::VectorXd::Zero(6),
                        Eigen::VectorXd::Zero(6),
                        Eigen::VectorXd::Zero(6)});
    eef_data.push_back({EndEffectorType::revo2,
                        Eigen::VectorXd::Zero(6),
                        Eigen::VectorXd::Zero(6),
                        Eigen::VectorXd::Zero(6)});

    for (int f = 0; f < 6; f++) {
        eef_data[0].position[f] = hand_command.position[f];
        eef_data[1].position[f] = hand_command.position[f + 6];
    }

    hardware_plant_->endEffectorCommand(eef_data);
    return HwErrorType::Success;
}

HwErrorType HardwareInterface::GetHandState(HandState_t &hand_state) {
    if (!initialized_) {
        return HwErrorType::NotInitialized;
    }

    eef_controller::FingerStatusSnapshot snapshot;
    if (!hardware_plant_->getEndEffectorState(snapshot)) {
        return HwErrorType::Failed;
    }

    const auto& left = snapshot.status[0];
    const auto& right = snapshot.status[1];
    hand_state.left_valid = snapshot.valid[0];
    hand_state.right_valid = snapshot.valid[1];
    hand_state.left_sample_age_ms = snapshot.sample_age_ms[0];
    hand_state.right_sample_age_ms = snapshot.sample_age_ms[1];

    for (int f = 0; f < 6; f++) {
        hand_state.position[f]     = left.positions[f];
        hand_state.velocity[f]     = left.speeds[f];
        hand_state.current[f]      = left.currents[f];
        hand_state.state[f]        = static_cast<uint8_t>(left.states[f]);

        hand_state.position[f + 6] = right.positions[f];
        hand_state.velocity[f + 6] = right.speeds[f];
        hand_state.current[f + 6]  = right.currents[f];
        hand_state.state[f + 6]    = static_cast<uint8_t>(right.states[f]);
    }

    return HwErrorType::Success;
}

} // namespace hw
} // namespace leju
