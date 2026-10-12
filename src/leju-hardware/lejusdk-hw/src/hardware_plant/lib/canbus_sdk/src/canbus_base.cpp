#include "canbus_base.h"
#ifdef HAS_BMAPI
#include "bm_canbus_wrapper.h"
#endif
#include "leju_canbus_wrapper.h"
#include "socketcan_wrapper.h"

namespace canbus_sdk {

std::unique_ptr<CanBusBase> CanBusBase::create(const std::string& canbus_name, CanBusModelType model_type, const CanBusBitrate& bitrate)
{
    // 根据型号类型创建对应的CAN总线实现
    switch (model_type) {
#ifdef HAS_BMAPI
        case CanBusModelType::BUSMUST_A:
            return std::unique_ptr<CanBusBase>(new BMCanbusWrapper(canbus_name, model_type, bitrate));
        case CanBusModelType::BUSMUST_B:
            return std::unique_ptr<CanBusBase>(new BMCanbusWrapper(canbus_name, model_type, bitrate));
#endif
        case CanBusModelType::LEJU_CAN_A:
            return std::unique_ptr<CanBusBase>(new LejuCanbusWrapper(canbus_name, model_type, bitrate));
        case CanBusModelType::LEJU_CAN_B:
            return std::unique_ptr<CanBusBase>(new LejuCanbusWrapper(canbus_name, model_type, bitrate));
        case CanBusModelType::SOCKETCAN:
            return std::unique_ptr<CanBusBase>(new SocketCanWrapper(canbus_name, model_type, bitrate));
        default:
            // 未知型号，返回空指针
            return nullptr;
    }
}

const std::string& CanBusBase::getCanbusName() const
{
    return canbus_name_;
}

CanBusModelType CanBusBase::getModelType() const
{
    return model_type_;
}

} // namespace canbus_sdk