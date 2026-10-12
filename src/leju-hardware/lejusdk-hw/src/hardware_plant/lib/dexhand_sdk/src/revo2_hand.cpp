#include "stark-sdk.h"
#include "revo2_hand.h"

#include <stdint.h>
#include <vector>
#include <iostream>
#include <algorithm>

namespace dexhand {

std::unique_ptr<Revo2Dexhand> Revo2Dexhand::Connect(
        const std::string& port,
        uint8_t slave_id,
        uint32_t baudrate
    ) {
        init_logging(LogLevel::LOG_LEVEL_ERROR);
        auto cfg = auto_detect_modbus_revo2(port.c_str(), true);
        if (cfg == NULL) {
            std::cout << "Failed to auto detect modbus revo2" << std::endl;
            return nullptr;
        }
        DeviceHandler* handle = modbus_open(cfg->port_name, cfg->baudrate);
        free_device_config(cfg);
        if (!handle) {
            return nullptr;
        }

    auto revo2_dexhand = std::unique_ptr<Revo2Dexhand>(new Revo2Dexhand(handle, slave_id));
    /* !!! IMPORTANT !!! 必须要调用获取设备信息识别一下设备! */
    DeviceInfo_t dev_info;
    if(!revo2_dexhand->getDeviceInfo(dev_info)) {
        return nullptr;
    }
    
    // 设置所有手指的保护电流为 1000mA
    const uint16_t protected_current = 1000; // 1000mA
    const StarkFingerId finger_ids[] = {
        STARK_FINGER_ID_THUMB,
        STARK_FINGER_ID_THUMB_AUX,
        STARK_FINGER_ID_INDEX,
        STARK_FINGER_ID_MIDDLE,
        STARK_FINGER_ID_RING,
        STARK_FINGER_ID_PINKY
    };
    
    for (auto finger_id : finger_ids) {
        stark_set_finger_protected_current(handle, slave_id, finger_id, protected_current);
    }
    std::cout << "[Revo2Dexhand] 已设置所有手指的保护电流为 " << protected_current << " mA" << std::endl;
    
    return revo2_dexhand;
}

Revo2Dexhand::Revo2Dexhand(DeviceHandler* handle, uint8_t slave_id_):ModbusDexhand(handle, slave_id_) {
}

Revo2Dexhand::~Revo2Dexhand() {
    if (mb_handle_) {
        std::cout  << "[Revo2Dexhand] close modbus handle. \n";
        ::modbus_close(mb_handle_);
    }
    mb_handle_ = nullptr;
}

DexHandFwType Revo2Dexhand::getDexHandFwType() {
    return DexHandFwType::V2_STANDARD;
}

void Revo2Dexhand::setGripForce(GripForce level) {
    std::cout << "[Revo2Dexhand] Revo2灵巧手不支持设置抓力等级，此函数调用将被忽略" << std::endl;
}

GripForce Revo2Dexhand::getGripForce() {
    std::cout << "[Revo2Dexhand] Revo2灵巧手不支持获取抓力等级，返回默认值FORCE_LEVEL_NORMAL" << std::endl;
    return GripForce::FORCE_LEVEL_NORMAL;
}

} // namespace dexhand
