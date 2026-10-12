#include "stark-sdk.h"
#include "touch_dexhand.h"

#include <stdint.h>
#include <vector>
#include <iostream>

namespace dexhand {

void convert_stark_sdk_touch_sensor_status(const TouchSensorStatus& status, TouchSensorStatus_t& touch_sensor_status)
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
        init_cfg(StarkFirmwareType::STARK_FIRMWARE_TYPE_V1_TOUCH, StarkProtocolType::STARK_PROTOCOL_TYPE_MODBUS, LogLevel::LOG_LEVEL_INFO);
        ModbusHandle* handle = ::modbus_open(port.c_str(), baudrate, slave_id);
        if (!handle) {
            return nullptr;
        }
    return std::unique_ptr<TouchDexhand>(new TouchDexhand(handle, slave_id));
}


TouchDexhand::TouchDexhand(ModbusHandle* handle, uint8_t slave_id_):ModbusDexhand(handle, slave_id_) {
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
    TouchStatusData* status = nullptr;
    try {
        TouchStatusData* status = modbus_get_touch_status(mb_handle_, slave_id_);
        if(!status) {
            return TouchSensorStatusArray();
        }

        for (int i = 0; i < 5; i++) {
            convert_stark_sdk_touch_sensor_status(status->data[i], touch_status[i]);
        }
    }catch(std::exception &e) {
        std::cerr << "[TouchDexhand] getTouchStatus exception: " << e.what() << std::endl;
    } 

    if(status != nullptr) {
        free_touch_status_data(status);
    }
    return touch_status;
}


void TouchDexhand::resetTouchSensor(uint8_t bits /* = 0xFF */)
{
    try{
        modbus_reset_touch_sensor(mb_handle_, slave_id_, bits);
    }catch(std::exception &e) {
        std::cerr << "[TouchDexhand] resetTouchSensor exception: " << e.what() << std::endl;
    }     
}

void TouchDexhand::enableTouchSensor(uint8_t bits /* = 0xFF */)
{
    try{
        modbus_enable_touch_sensor(mb_handle_, slave_id_, bits);
    }catch(std::exception &e) {
        std::cerr << "[TouchDexhand] enableTouchSensor exception: " << e.what() << std::endl;
    }
}

} // namesapce dexhand