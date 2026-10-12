#include "stark-sdk.h"
#include "touch_dexhand.h"

#include <stdint.h>
#include <vector>
#include <iostream>

namespace dexhand {

void convert_stark_sdk_touch_sensor_status(const CTouchFingerItem& status, TouchSensorStatus_t& touch_sensor_status)
{
    touch_sensor_status.normal_force1 = status.normal_force1;
    touch_sensor_status.normal_force2 = status.normal_force2;
    touch_sensor_status.normal_force3 = status.normal_force3;
    touch_sensor_status.tangential_force1 = status.tangential_force1;
    touch_sensor_status.tangential_force2 = status.tangential_force2;
    touch_sensor_status.tangential_force3 = status.tangential_force3;
    touch_sensor_status.tangential_direction1 = status.tangential_direction1;
    touch_sensor_status.tangential_direction2 = status.tangential_direction2;
    touch_sensor_status.tangential_direction3 = status.tangential_direction3;
    touch_sensor_status.self_proximity1 = status.self_proximity1;
    touch_sensor_status.self_proximity2 = status.self_proximity2;
    touch_sensor_status.mutual_proximity = status.mutual_proximity;
    touch_sensor_status.status = status.status;
}

std::unique_ptr<TouchDexhand> TouchDexhand::Connect(
        const std::string& port,
        uint8_t slave_id,
        uint32_t baudrate
    ) {
        init_logging(LogLevel::LOG_LEVEL_ERROR);

        DeviceHandler* handle = nullptr;

        // 1. Auto-detect (handles revo1touch and revo1advancedtouch with different id/baudrate)
        CDetectedDeviceList* list = stark_auto_detect(false, port.c_str(), static_cast<StarkProtocolType>(0));
        if (list && list->count > 0) {
            const CDetectedDevice& dev = list->devices[0];
            uint8_t detected_slave_id = dev.slave_id;

            std::cout << "[TouchDexhand] Detected device on " << port
                      << " protocol=" << (int)dev.protocol
                      << " hw_type=" << (int)dev.hardware_type
                      << " slave_id=" << (int)dev.slave_id << std::endl;

            handle = init_from_detected(&dev);
            free_detected_device_list(list);

            if (!handle) {
                std::cerr << "[TouchDexhand] init_from_detected failed for " << port << std::endl;
                return nullptr;
            }
            return std::unique_ptr<TouchDexhand>(new TouchDexhand(handle, detected_slave_id));
        }

        if (list) {
            free_detected_device_list(list);
        }

        // 2. Fallback: direct modbus_open with given params (legacy revo1touch)
        std::cout << "[TouchDexhand] Auto-detect failed on " << port
                  << ", trying modbus_open with baudrate=" << baudrate << std::endl;
        handle = ::modbus_open(port.c_str(), baudrate);
        if (handle) {
            std::cout << "[TouchDexhand] Connected via modbus_open on " << port << std::endl;
            return std::unique_ptr<TouchDexhand>(new TouchDexhand(handle, slave_id));
        }

        std::cerr << "[TouchDexhand] All connection methods failed for " << port << std::endl;
        return nullptr;
}


TouchDexhand::TouchDexhand(DeviceHandler* handle, uint8_t slave_id_):ModbusDexhand(handle, slave_id_) {
    if (mb_handle_) {
    }
}

TouchDexhand::~TouchDexhand() {
    if (mb_handle_) {
        std::cout  << "[TouchDexhand] close modbus handle. \n";
        ::modbus_close(mb_handle_);
    }
    mb_handle_ = nullptr;
}

DexHandFwType TouchDexhand::getDexHandFwType() {
    return DexHandFwType::V1_TOUCH;
}


TouchSensorStatusArray TouchDexhand::getTouchStatus()
{
    TouchSensorStatusArray touch_status;
    CTouchFingerData* status = nullptr;
    try {
        status = stark_get_touch_status(mb_handle_, slave_id_);
        if(!status) {
            return TouchSensorStatusArray();
        }

        for (int i = 0; i < 5; i++) {
            convert_stark_sdk_touch_sensor_status(status->items[i], touch_status[i]);
        }
    }catch(std::exception &e) {
        std::cerr << "[TouchDexhand] getTouchStatus exception: " << e.what() << std::endl;
    } 

    if(status != nullptr) {
        free_touch_finger_data(status);
    }
    return touch_status;
}


void TouchDexhand::resetTouchSensor(uint8_t bits /* = 0xFF */)
{
    try{
        stark_reset_touch_sensor(mb_handle_, slave_id_, bits);
    }catch(std::exception &e) {
        std::cerr << "[TouchDexhand] resetTouchSensor exception: " << e.what() << std::endl;
    }     
}

void TouchDexhand::enableTouchSensor(uint8_t bits /* = 0xFF */)
{
    try{
        stark_enable_touch_sensor(mb_handle_, slave_id_, bits);
    }catch(std::exception &e) {
        std::cerr << "[TouchDexhand] enableTouchSensor exception: " << e.what() << std::endl;
    }
}

} // namesapce dexhand