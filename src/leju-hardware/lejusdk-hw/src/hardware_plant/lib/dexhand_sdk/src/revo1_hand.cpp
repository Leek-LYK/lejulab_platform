#include "stark-sdk.h"
#include "revo1_hand.h"

#include <stdint.h>
#include <vector>
#include <iostream>
#include <algorithm>

namespace dexhand {

std::unique_ptr<Revo1Dexhand> Revo1Dexhand::Connect(
        const std::string& port,
        uint8_t slave_id,
        uint32_t baudrate
    ) {
        init_logging(LogLevel::LOG_LEVEL_ERROR);
        auto cfg = auto_detect_modbus_revo1(port.c_str(), true);
        if (cfg == NULL) {
            std::cout << "Failed to auto detect modbus revo1" << std::endl;
            return nullptr;
        }
        DeviceHandler* handle = modbus_open(cfg->port_name, cfg->baudrate);
        free_device_config(cfg);
        if (!handle) {
            return nullptr;
        }

    auto revo1_dexhand = std::unique_ptr<Revo1Dexhand>(new Revo1Dexhand(handle, slave_id));
    /* !!! IMPORTANT !!! 必须要调用获取设备信息识别一下设备! */
    DeviceInfo_t dev_info;
    if(!revo1_dexhand->getDeviceInfo(dev_info)) {
        return nullptr;
    }
    return revo1_dexhand;
}

Revo1Dexhand::Revo1Dexhand(DeviceHandler* handle, uint8_t slave_id_):ModbusDexhand(handle, slave_id_) {
}

Revo1Dexhand::~Revo1Dexhand() {
    if (mb_handle_) {
        std::cout  << "[Revo1Dexhand] close modbus handle. \n";
        ::modbus_close(mb_handle_);
    }
    mb_handle_ = nullptr;
}

DexHandFwType Revo1Dexhand::getDexHandFwType() {
    return DexHandFwType::V1_STANDARD;
}

void Revo1Dexhand::setGripForce(GripForce level) {
    std::cout << "[Revo1Dexhand] revo1灵巧手不支持设置抓力等级，此函数调用将被忽略" << std::endl;
}

GripForce Revo1Dexhand::getGripForce() {
    std::cout << "[Revo1Dexhand] revo1灵巧手不支持获取抓力等级，返回默认值FORCE_LEVEL_NORMAL" << std::endl;
    return GripForce::FORCE_LEVEL_NORMAL;
}

} // namespace dexhand
