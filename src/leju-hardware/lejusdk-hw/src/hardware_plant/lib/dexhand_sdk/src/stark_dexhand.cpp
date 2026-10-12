#include "stark-sdk.h"
#include "stark_dexhand.h"

#include <stdint.h>
#include <vector>
#include <iostream>
#include <algorithm>

namespace dexhand {

std::unique_ptr<StarkDexhand> StarkDexhand::Connect(const std::string& port)
{
    init_logging(LogLevel::LOG_LEVEL_ERROR);

    uint8_t protocol = 0;
    uint8_t hardware_type = 0;
    uint8_t slave_id = 1;
    DeviceHandler* handle = nullptr;

    // 1. Auto-detect (modbus: revo1adv, revo2, etc.)
    CDetectedDeviceList* list = stark_auto_detect(false, port.c_str(), static_cast<StarkProtocolType>(0));
    if (list && list->count > 0) {
        const CDetectedDevice& dev = list->devices[0];
        protocol = static_cast<uint8_t>(dev.protocol);
        hardware_type = static_cast<uint8_t>(dev.hardware_type);
        slave_id = dev.slave_id;

        std::cout << "[StarkDexhand] Detected device on " << port
                  << " protocol=" << (int)protocol
                  << " hw_type=" << (int)hardware_type
                  << " slave_id=" << (int)slave_id << std::endl;

        handle = init_from_detected(&dev);
        free_detected_device_list(list);

        if (!handle) {
            std::cerr << "[StarkDexhand] init_from_detected failed for " << port << std::endl;
            return nullptr;
        }
    } else {
        if (list) {
            free_detected_device_list(list);
        }

        // 2. Fallback: protobuf (legacy revo1)
        //    NOTE: SDK bug — protobuf_open after stark_auto_detect may cause
        //    Tokio runtime panic. Will be fixed in a future SDK release.
        std::cout << "[StarkDexhand] Auto-detect failed on " << port
                  << ", falling back to protobuf protocol" << std::endl;
        slave_id = 10;
        handle = protobuf_open(port.c_str(), slave_id, 115200);
        protocol = static_cast<uint8_t>(StarkProtocolType::STARK_PROTOCOL_TYPE_PROTOBUF);
        hardware_type = static_cast<uint8_t>(StarkHardwareType::STARK_HARDWARE_TYPE_REVO1_PROTOBUF);

        if (!handle) {
            std::cerr << "[StarkDexhand] All connection methods failed for " << port << std::endl;
            return nullptr;
        }
    }

    auto dexhand = std::unique_ptr<StarkDexhand>(
        new StarkDexhand(handle, slave_id, protocol, hardware_type));

    // Verify connection by getting device info
    DeviceInfo_t dev_info;
    if (!dexhand->getDeviceInfo(dev_info)) {
        std::cerr << "[StarkDexhand] Failed to get device info on " << port << std::endl;
        return nullptr;
    }
    std::cout << "[StarkDexhand] Connected: " << dev_info;
    return dexhand;
}

StarkDexhand::StarkDexhand(DeviceHandler* handle, uint8_t slave_id, uint8_t protocol, uint8_t hardware_type)
    : DexHandBase(), handle_(handle), slave_id_(slave_id), protocol_(protocol), hardware_type_(hardware_type) {
}

StarkDexhand::~StarkDexhand() {
    if (handle_) {
        std::cout << "[StarkDexhand] closing device handle.\n";
        close_device_handler(handle_, protocol_);
    }
    handle_ = nullptr;
}

DexHandFwType StarkDexhand::getDexHandFwType() {
    if (stark_uses_revo1_motor_api(hardware_type_)) {
        return DexHandFwType::V1_STANDARD;
    }
    return DexHandFwType::V2_STANDARD;
}

bool StarkDexhand::getDeviceInfo(DeviceInfo_t& info) {
    info.sku_type = DexHandType::SKU_TYPE_NONE;
    info.serial_number = "";
    info.firmware_version = "";

    CDeviceInfo* device_info = nullptr;
    bool success = false;
    try {
        device_info = stark_get_device_info(handle_, slave_id_);
        if (!device_info) {
            return false;
        }
        info.sku_type = static_cast<DexHandType>(device_info->sku_type);
        info.serial_number = device_info->serial_number;
        info.firmware_version = device_info->firmware_version;
        // Update hardware_type from actual device info
        hardware_type_ = static_cast<uint8_t>(device_info->hardware_type);
        success = true;
    } catch(std::exception &e) {
        std::cout << "[StarkDexhand] getDeviceInfo exception:" << e.what() << "\n";
    }
    if (device_info != nullptr) {
        free_device_info(device_info);
    }
    return success;
}

void StarkDexhand::setFingerPositions(const UnsignedFingerArray &positions)
{
    try {
        // API 0~100 → SDK 0~1000 (×10)
        UnsignedFingerArray scaled;
        for (size_t i = 0; i < positions.size(); ++i) {
            scaled[i] = 10 * std::clamp(positions[i], static_cast<uint16_t>(0), static_cast<uint16_t>(100));
        }
        stark_set_finger_positions(handle_, slave_id_, scaled.data(), scaled.size());
    } catch(std::exception &e) {
        std::cerr << "[StarkDexhand] setFingerPositions exception:" << e.what() << "\n";
    }
}

void StarkDexhand::setFingerSpeeds(const FingerArray &speeds)
{
    try {
        // API -100~100 → SDK -1000~1000 (×10)
        FingerArray scaled;
        for (size_t i = 0; i < speeds.size(); ++i) {
            scaled[i] = 10 * std::clamp(speeds[i], static_cast<int16_t>(-100), static_cast<int16_t>(100));
        }
        stark_set_finger_speeds(handle_, slave_id_, scaled.data(), scaled.size());
    } catch(std::exception &e) {
        std::cerr << "[StarkDexhand] setFingerSpeeds exception:" << e.what() << "\n";
    }
}

bool StarkDexhand::getFingerStatus(FingerStatus& status)
{
    status.clear();
    CMotorStatusData* motor_status = nullptr;
    bool success = false;
    try {
        motor_status = stark_get_motor_status(handle_, slave_id_);
        if (!motor_status) {
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
    } catch(std::exception &e) {
        std::cerr << "[StarkDexhand] getFingerStatus exception: " << e.what() << std::endl;
    }
    if (motor_status != nullptr) {
        free_motor_status_data(motor_status);
    }
    return success;
}

void StarkDexhand::setGripForce(GripForce level)
{
    try {
        stark_set_force_level(handle_, slave_id_, static_cast<::ForceLevel>(level));
    } catch(std::exception &e) {
        std::cerr << "[StarkDexhand] setGripForce exception: " << e.what() << std::endl;
    }
}

GripForce StarkDexhand::getGripForce()
{
    auto force_level = stark_get_force_level(handle_, slave_id_);
    return static_cast<GripForce>(force_level);
}

void StarkDexhand::setTurboModeEnabled(bool enabled)
{
    try {
        stark_set_turbo_mode_enabled(handle_, slave_id_, enabled);
    } catch(std::exception &e) {
        std::cerr << "[StarkDexhand] setTurboModeEnabled exception: " << e.what() << std::endl;
    }
}

bool StarkDexhand::isTurboModeEnabled()
{
    return stark_get_turbo_mode_enabled(handle_, slave_id_);
}

TurboConfig_t StarkDexhand::getTurboConfig()
{
    CTurboConfig* config = stark_get_turbo_config(handle_, slave_id_);
    TurboConfig_t turbo_config{
        .interval = config->interval,
        .duration = config->duration
    };
    free_turbo_config(config);
    return turbo_config;
}

void StarkDexhand::runActionSequence(ActionSequenceId_t seq_id)
{
    try {
        stark_run_action_sequence(handle_, slave_id_, static_cast<ActionSequenceId>(seq_id));
    } catch(std::exception &e) {
        std::cerr << "[StarkDexhand] runActionSequence exception: " << e.what() << std::endl;
    }
}

bool StarkDexhand::setActionSequence(ActionSequenceId_t seq_id, const ActionSeqDataTypeVec &sequences)
{
    if (seq_id < ACTION_SEQUENCE_ID_CUSTOM_GESTURE1 || seq_id > ACTION_SEQUENCE_ID_CUSTOM_GESTURE6) {
        std::cerr << "[StarkDexhand] Invalid sequence id: " << seq_id << std::endl;
        return false;
    }

    std::vector<uint16_t> sequences_data(sequences.size() * 20);
    for (size_t i = 0; i < sequences.size(); i++) {
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

    bool uses_revo2 = !stark_uses_revo1_motor_api(hardware_type_);
    try {
        stark_set_action_sequence(handle_, slave_id_, uses_revo2, static_cast<ActionSequenceId>(seq_id), sequences_data.data(), sequences.size());
    } catch(std::exception &e) {
        std::cerr << "[StarkDexhand] setActionSequence exception: " << e.what() << std::endl;
        return false;
    }
    return true;
}

} // namespace dexhand
