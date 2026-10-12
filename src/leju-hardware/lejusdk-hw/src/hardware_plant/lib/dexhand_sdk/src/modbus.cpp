#include "stark-sdk.h"
#include "modbus.h"

#include <stdint.h>
#include <vector>
#include <iostream>
#include <algorithm>

namespace dexhand {

std::ostream& operator<<(std::ostream& os, const TouchSensorStatus_t& status) {
    os << "Normal Force 1: " << status.normal_force1 << "\n";
    os << "Normal Force 2: " << status.normal_force2 << "\n";
    os << "Normal Force 3: " << status.normal_force3 << "\n";
    os << "Tangential Force 1: " << status.tangential_force1 << "\n";
    os << "Tangential Force 2: " << status.tangential_force2 << "\n";
    os << "Tangential Force 3: " << status.tangential_force3 << "\n";
    os << "Tangential Direction 1: " << status.tangential_direction1 << "\n";
    os << "Tangential Direction 2: " << status.tangential_direction2 << "\n";
    os << "Tangential Direction 3: " << status.tangential_direction3 << "\n";
    os << "Self Proximity 1: " << status.self_proximity1 << "\n";
    os << "Self Proximity 2: " << status.self_proximity2 << "\n";
    os << "Mutual Proximity: " << status.mutual_proximity << "\n";
    os << "Status: " << status.status << "\n";
    return os;
}

std::ostream& operator<<(std::ostream& os, const FingerStatusPtr& status)
{
    if (!status) {
        return os;
    }
    os << "Positions: ";
    for (auto pos : status->positions) {
        os << pos << " ";
    }
    os << ", Speeds:";
    for (auto speed : status->speeds) {
        os << speed << " ";
    }
    os << ", Currents:";
    for (auto current : status->currents) {
        os << current << " ";
    }
    os << "\n";
    return os;
}

// void convert_stark_sdk_touch_sensor_status(const TouchSensorStatus& status, TouchSensorStatus_t& touch_sensor_status)
// {
//     touch_sensor_status.normal_force1 = status.normal_force1;
//     touch_sensor_status.normal_force2 = status.normal_force2;
//     touch_sensor_status.normal_force3 = status.normal_force3;
//     touch_sensor_status.tangential_force1 = status.tangential_force1;
//     touch_sensor_status.tangential_force2 = status.tangential_force2;
//     touch_sensor_status.tangential_force3 = status.tangential_force3;
//     touch_sensor_status.tangential_direction1 = status.tangential_direction1;
//     touch_sensor_status.tangential_direction2 = status.tangential_direction2;
//     touch_sensor_status.tangential_direction3 = status.tangential_direction3;
//     touch_sensor_status.self_proximity1 = status.self_proximity1;
//     touch_sensor_status.self_proximity2 = status.self_proximity2;
//     touch_sensor_status.mutual_proximity = status.mutual_proximity;
//     touch_sensor_status.status = status.status;
// }

// std::unique_ptr<ModbusDexhand> ModbusDexhand::Connect(
//         const std::string& port,
//         uint8_t slave_id,
//         uint32_t baudrate
//     ) {
//         init_cfg(StarkFirmwareType::STARK_FIRMWARE_TYPE_V2_STANDARD, StarkProtocolType::STARK_PROTOCOL_TYPE_MODBUS, LogLevel::LOG_LEVEL_INFO);
//         DeviceHandler* handle = ::modbus_open(port.c_str(), baudrate, slave_id);
//         if (!handle) {
//             return nullptr;
//         }
//     return std::unique_ptr<ModbusDexhand>(new ModbusDexhand(handle, slave_id));
// }


ModbusDexhand::ModbusDexhand(DeviceHandler* handle, uint8_t slave_id_):DexHandBase(), mb_handle_(handle), slave_id_(slave_id_) {
}

ModbusDexhand::~ModbusDexhand() {
    if (mb_handle_) {
        std::cout  << "[TouchDexhand] close modbus handle. \n";
        ::modbus_close(mb_handle_);
    }
    mb_handle_ = nullptr;
}

DexHandFwType ModbusDexhand::getDexHandFwType() {
    return DexHandFwType::V1_TOUCH;
}

bool ModbusDexhand::getDeviceInfo(DeviceInfo_t& info) {
    info.sku_type = DexHandType::SKU_TYPE_NONE;
    info.serial_number = "";
    info.firmware_version = "";

    CDeviceInfo* device_info = nullptr;
    bool success = false;

    try {
        device_info = stark_get_device_info(mb_handle_, slave_id_);
        if (!device_info) {
            // std::cerr << "[TouchDexhand] Failed to get device info !" << std::endl;
            return false;
        }

        info.sku_type = static_cast<DexHandType>(device_info->sku_type);
        info.serial_number = device_info->serial_number;
        info.firmware_version = device_info->firmware_version;
        success = true;
    }catch(std::exception &e) {
        std::cout << "[TouchDexhand] getDeviceInfo exception:" << e.what() << "\n";
    }

    if(device_info != nullptr) {
        free_device_info(device_info);
    }
    return success;
}

void ModbusDexhand::setFingerPositions(const UnsignedFingerArray &positions)
{
    try {
        // API 0~100 → SDK 0~1000 (×10)
        UnsignedFingerArray clamped_positions;
        for (size_t i = 0; i < positions.size(); ++i) {
            clamped_positions[i] = 10 * std::clamp(positions[i], static_cast<uint16_t>(0), static_cast<uint16_t>(100));
        }

        stark_set_finger_positions(mb_handle_, slave_id_, clamped_positions.data(), clamped_positions.size());

    } catch(std::exception &e) {
        std::cerr << "[TouchDexhand] setFingerPositions exception:" << e.what() << "\n";
    }
}

void ModbusDexhand::setFingerSpeeds(const FingerArray &speeds)
{
    try{
        // API -100~100 → SDK -1000~1000 (×10)
        FingerArray scaled_speeds;
        for (size_t i = 0; i < speeds.size(); ++i) {
            scaled_speeds[i] = 10 * std::clamp(speeds[i], static_cast<int16_t>(-100), static_cast<int16_t>(100));
        }
        stark_set_finger_speeds(mb_handle_, slave_id_, scaled_speeds.data(), scaled_speeds.size());
    }catch(std::exception &e) {
        std::cerr << "[TouchDexhand] setFingerSpeeds exception:" << e.what() << "\n";
    }
}

bool ModbusDexhand::getFingerStatus(FingerStatus& status)
{
    status.clear(); // Clear data before getting new values
    CMotorStatusData* motor_status = nullptr;
    bool success = false;
    try {
        motor_status = stark_get_motor_status(mb_handle_, slave_id_);
        if(!motor_status){
            return false;
        }

        // SDK 0~1000 → API 0~100 (÷10)
        for (size_t i = 0; i < 6; ++i) {
            status.positions[i] = std::clamp<uint16_t>(static_cast<uint16_t>(motor_status->positions[i] / 10), 0, 100);
            status.speeds[i] = std::clamp<int16_t>(static_cast<int16_t>(motor_status->speeds[i] / 10), -100, 100);
            status.currents[i] = motor_status->currents[i];
            status.states[i] = motor_status->states[i];
        }

        success = true;
    }catch(std::exception &e) {
        std::cerr << "[TouchDexhand] getFingerStatus exception: " << e.what() << std::endl;
    }
    if(motor_status != nullptr) {
        free_motor_status_data(motor_status);
    }
    return success;
}

void ModbusDexhand::setGripForce(GripForce level)
{
    try{
        stark_set_force_level(mb_handle_, slave_id_, static_cast<::ForceLevel>(level));
    }catch(std::exception &e) {
        std::cerr << "[TouchDexhand] setGripForce exception: " << e.what() << std::endl;
    }
}

GripForce ModbusDexhand::getGripForce()
{
    auto force_level = stark_get_force_level(mb_handle_, slave_id_);
    return static_cast<GripForce>(force_level);
}

void ModbusDexhand::setTurboModeEnabled(bool enabled)
{
    try {
        stark_set_turbo_mode_enabled(mb_handle_, slave_id_, enabled);
    }catch(std::exception &e) {
        std::cerr << "[TouchDexhand] setTurboModeEnabled exception: " << e.what() << std::endl;
    }
}

bool ModbusDexhand::isTurboModeEnabled()
{
    return stark_get_turbo_mode_enabled(mb_handle_, slave_id_);
}

TurboConfig_t ModbusDexhand::getTurboConfig()
{
    CTurboConfig* config = stark_get_turbo_config(mb_handle_, slave_id_);
    TurboConfig_t turbo_config{
        .interval = config->interval,
        .duration = config->duration
    };
    free_turbo_config(config);
    return turbo_config;
}


void ModbusDexhand::runActionSequence(ActionSequenceId_t seq_id)
{   
    try{
        stark_run_action_sequence(mb_handle_, slave_id_, static_cast<ActionSequenceId>(seq_id));
    }catch(std::exception &e) {
        std::cerr << "[TouchDexhand] runActionSequence exception: " << e.what() << std::endl;
    }
}

bool ModbusDexhand::setActionSequence(ActionSequenceId_t seq_id, const ActionSeqDataTypeVec &sequences)
{
    if (seq_id < ACTION_SEQUENCE_ID_CUSTOM_GESTURE1 || seq_id > ACTION_SEQUENCE_ID_CUSTOM_GESTURE6) {
        std::cerr << "[TouchDexhand] Invalid sequence id: " << seq_id << std::endl;
        return false;
    }

    /// - `0`: 动作序列索引
    /// - `2000`: 动作序列持续时间，单位毫秒
    /// - `0, 0, 100, 100, 100, 100`: 6 个手指位置
    /// - `10, 20, 30, 40, 50, 60`: 6 个手指速度
    /// - `5, 10, 15, 20, 25, 30`: 6 个手指力量
    std::vector<uint16_t> sequences_data(sequences.size() * 20);
    for (int i = 0; i < sequences.size(); i++) {
        int start_index = i * 20;
        sequences_data[start_index++] = i; // index
        sequences_data[start_index++] = sequences[i].duration_ms;

        for (int j = 0; j < 6; j++) {
            sequences_data[start_index++] = static_cast<uint16_t>(sequences[i].positions[j]);
        }
        for (int j = 0; j < 6; j++) {
            sequences_data[start_index++] = static_cast<uint16_t>(sequences[i].speeds[j]);
        }
        for (int j = 0; j < 6; j++) {
            sequences_data[start_index++] = static_cast<uint16_t>(sequences[i].forces[j]);
        }
    }
    
    // std::cout << "[TouchDexhand] set action sequence, data length:" << sequences_data.size() << std::endl;
    // std::cout << "[TouchDexhand] set action sequence, length:" << sequences.size() << std::endl;
    try {
        stark_set_action_sequence(mb_handle_, slave_id_, usesRevo2MotorApi(), static_cast<ActionSequenceId>(seq_id), sequences_data.data(), sequences.size());
    }catch(std::exception &e) {
        std::cerr << "[TouchDexhand] setActionSequence exception: " << e.what() << std::endl;
        return false;
    }
    return true;
}
} // namesapce dexhand