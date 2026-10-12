#include "canbus_sdk/canbus_sdk.h"
#include "canbus_sdk/canbus_log.h"
#include <unordered_map>
#include <cassert>

namespace canbus_sdk {

uint8_t dlc_to_payload_length(uint8_t dlc) {
    // Convert CAN DLC (Data Length Code) to actual payload length
    // Standard CAN frames: DLC 0-8 maps to 0-8 bytes
    if (dlc <= 8) {
        return dlc;
    }
    // For CAN FD, DLC 9-15 maps to 12, 16, 20, 24, 32, 48, 64 bytes
    switch (dlc) {
        case 9: return 12;
        case 10: return 16;
        case 11: return 20;
        case 12: return 24;
        case 13: return 32;
        case 14: return 48;
        case 15: return 64;
        default: return 0; // Invalid DLC
    }
}

uint8_t payload_length_to_dlc(uint8_t payload_length) {
    // Convert payload length to CAN DLC (Data Length Code)
    // For standard CAN frames: 0-8 bytes maps to DLC 0-8
    if (payload_length <= 8) {
        return payload_length;
    }
    // For CAN FD: map payload lengths to DLC values
    if (payload_length <= 12) return 9;
    if (payload_length <= 16) return 10;
    if (payload_length <= 20) return 11;
    if (payload_length <= 24) return 12;
    if (payload_length <= 32) return 13;
    if (payload_length <= 48) return 14;
    if (payload_length <= 64) return 15;
    
    // Invalid payload length
    return 0;
}

CanMessageFrame* createCanMessageFrame() {
    try {
        CanMessageFrame* frame = new CanMessageFrame();
        if (frame) {
            // Initialize the frame to ensure clean state
            memset(frame, 0, sizeof(CanMessageFrame));
        }
        return frame;
    } catch (const std::bad_alloc& e) {
        // Memory allocation failed
        LOG_E("Failed to allocate memory for CanMessageFrame: %s", e.what());
        return nullptr;
    } catch (...) {
        // Other exceptions
        LOG_E("Unknown exception occurred while creating CanMessageFrame");
        return nullptr;
    }
}

void freeCanMessageFrame(CanMessageFrame* frame) {
    if (frame != nullptr) {
        // Check if the pointer is valid before deletion
        try {
            delete frame;
        } catch (...) {
            // Log or handle deletion errors if necessary
            // In production code, you might want to log this error
            LOG_E("freeCanMessageFrame error: Exception occurred while deleting frame");
        }
    }
}

const char* errorToString(CanBusError error) {
    switch (error) {
        case CanBusError::SUCCESS: return "SUCCESS";
        case CanBusError::ERROR_UNINITIALIZED: return "ERROR_UNINITIALIZED";
        case CanBusError::ERROR_INVALID_PARAMETER: return "ERROR_INVALID_PARAMETER";
        case CanBusError::ERROR_NO_AVAILABLE_SLOT: return "ERROR_NO_AVAILABLE_SLOT";
        case CanBusError::ERROR_BUS_NOT_FOUND: return "ERROR_BUS_NOT_FOUND";
        case CanBusError::ERROR_DEVICE_ALREADY_EXISTS: return "ERROR_DEVICE_ALREADY_EXISTS";
        case CanBusError::ERROR_NO_AVAILABLE_DEVICE_SLOT: return "ERROR_NO_AVAILABLE_DEVICE_SLOT";
        case CanBusError::ERROR_DEVICE_NOT_FOUND: return "ERROR_DEVICE_NOT_FOUND";
        case CanBusError::ERROR_MESSAGE_QUEUE_FULL: return "ERROR_MESSAGE_QUEUE_FULL";
        case CanBusError::ERROR_HARDWARE_FAILURE: return "ERROR_HARDWARE_FAILURE";
        case CanBusError::ERROR_MEMORY_ALLOCATION: return "ERROR_MEMORY_ALLOCATION";
        case CanBusError::ERROR_TIMEOUT: return "ERROR_TIMEOUT";
        case CanBusError::ERROR_PERMISSION_DENIED: return "ERROR_PERMISSION_DENIED";
        case CanBusError::ERROR_DEVICE_DISCONNECTED: return "ERROR_DEVICE_DISCONNECTED";
        case CanBusError::ERROR_INTERNAL_ERROR: return "ERROR_INTERNAL_ERROR";
        case CanBusError::ERROR_DEVICE_NOT_REGISTERED: return "ERROR_DEVICE_NOT_REGISTERED";
        case CanBusError::ERROR_BUS_NOT_ACTIVE: return "ERROR_BUS_NOT_ACTIVE";
        case CanBusError::ERROR_DEVICE_REGISTRATION_FAILED: return "ERROR_DEVICE_REGISTRATION_FAILED";
        case CanBusError::ERROR_NOT_IMPLEMENTED: return "ERROR_NOT_IMPLEMENTED";
        default: return "UNKNOWN_ERROR";
    }
}

const char* errorToString(ErrorType error) {
    return errorToString(static_cast<CanBusError>(error));
}

// ===== CanBusModelType 转换映射 =====
static const std::unordered_map<CanBusModelType, const char*> canbus_model_to_string_map = {
    {CanBusModelType::BUSMUST_A, "BUSMUST_A"},
    {CanBusModelType::BUSMUST_B, "BUSMUST_B"},
    {CanBusModelType::LEJU_CAN_A, "LEJU_CAN_A"},
    {CanBusModelType::LEJU_CAN_B, "LEJU_CAN_B"},
    {CanBusModelType::SOCKETCAN, "SOCKETCAN"},
    {CanBusModelType::UNKNOWN, "UNKNOWN"}
};

// ===== DeviceType 转换映射 =====
static const std::unordered_map<DeviceType, const char*> device_type_to_string_map = {
    {DeviceType::MOTOR, "motor"},
    {DeviceType::REVO1_HAND, "revo1_hand"},
    {DeviceType::REVO2_HAND, "revo2_hand"},
    {DeviceType::LEJUCLAW, "lejuclaw"},
    {DeviceType::UNKNOWN, "unknown"}
};

const char* to_string(CanBusModelType type) {
    auto it = canbus_model_to_string_map.find(type);
    return (it != canbus_model_to_string_map.end()) ? it->second : "UNKNOWN";
}

const char* to_string(DeviceType type) {
    auto it = device_type_to_string_map.find(type);
    return (it != device_type_to_string_map.end()) ? it->second : "unknown";
}

CanBusModelType canbus_model_from_string(const std::string& str) {
    for (auto it = canbus_model_to_string_map.begin(); it != canbus_model_to_string_map.end(); ++it) {
        if (str == it->second) {
            return it->first;
        }
    }
    // If string not found, this is a critical configuration error - assert
    assert(false && "Unsupported CAN bus model type string");
    return CanBusModelType::UNKNOWN; // never reached
}

DeviceType device_type_from_string(const std::string& str) {
    for (auto it = device_type_to_string_map.begin(); it != device_type_to_string_map.end(); ++it) {
        if (str == it->second) {
            return it->first;
        }
    }
    // If string not found, this is a critical configuration error - assert
    assert(false && "Unsupported device type string");

    return DeviceType::UNKNOWN; //  never reached
}


// Destructor definition for unique_ptr<Impl>
CanBusController::~CanBusController() = default;

// Custom deleter implementation for Impl
void CanBusController::ImplDeleter::operator()(Impl* ptr) const {
    delete ptr;
}

} // namespace canbus_sdk