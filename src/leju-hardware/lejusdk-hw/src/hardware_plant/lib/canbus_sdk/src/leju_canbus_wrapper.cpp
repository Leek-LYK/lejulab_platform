#include "leju_canbus_wrapper.h"
#include "canbus_sdk/result.h"
#include "canbus_sdk/canbus_log.h"
#include "canbus_sdk/canbus_sdk.h"

#include "CANFD_leju.h"

#include <cstring>

#define PRINT_SEND_DATA 0

namespace canbus_sdk {

// ===== Inner结构体：封装LEJU CANable设备 =====
struct LejuCanbusWrapper::Inner {
    std::unique_ptr<CANable> can_device;  ///< LEJU CANable设备对象

    Inner() : can_device(nullptr) {}

    void reset() {
        can_device.reset();
    }
};

// ===== 构造函数和析构函数 =====

LejuCanbusWrapper::LejuCanbusWrapper(const std::string& canbus_name,
                                     CanBusModelType model_type,
                                     const CanBusBitrate& bitrate)
    : CanBusBase(canbus_name, model_type, bitrate),
      inner_(new Inner()) {
    LOG_D("LejuCanbusWrapper constructor called for bus: %s", canbus_name.c_str());
}

LejuCanbusWrapper::~LejuCanbusWrapper() {
    LOG_D("Destroying LejuCanbusWrapper for bus: %s", canbus_name_.c_str());
    if (inner_ && inner_->can_device) {
        inner_->can_device->Close();
        inner_->reset();
    }
}

// ===== 初始化 =====

Result<bool> LejuCanbusWrapper::init() {
    LOG_I("[LejuCanbusWrapper] Initializing LEJU CAN bus: %s", canbus_name_.c_str());

    try {
        // 创建CANable设备对象（启用CAN FD）
        inner_->can_device.reset(new CANable(canbus_name_, true));
        
        // 初始化设备
        if (!inner_->can_device->Init()) {
            LOG_E("[LejuCanbusWrapper] Failed to initialize CANable device");
            inner_->reset();
            return Result<bool>::error(static_cast<ErrorType>(LEJU_ERROR_INIT));
        }

        LOG_I("[LejuCanbusWrapper] LEJU CAN bus %s initialized successfully", canbus_name_.c_str());
        LOG_I("[LejuCanbusWrapper] Bitrate - Standard: %d kbps, Data: %d kbps", 
              bitrate_.nbitrate, bitrate_.dbitrate);
        
        return Result<bool>::ok(true);
        
    } catch (const std::exception& e) {
        LOG_E("[LejuCanbusWrapper] Exception during initialization: %s", e.what());
        inner_->reset();
        return Result<bool>::error(static_cast<ErrorType>(LEJU_ERROR_UNKNOWN));
    }
}

// ===== 发送消息 =====

Result<ErrorType> LejuCanbusWrapper::writeCanMessage(const CanMessageFrame& frame) {
    if (!inner_ || !inner_->can_device) {
        LOG_E("[LejuCanbusWrapper] CAN device not initialized");
        return Result<ErrorType>::error(static_cast<ErrorType>(LEJU_ERROR_INIT));
    }

    // 转换数据格式：CanMessageFrame -> LEJU格式
    uint32_t can_id = frame.id.SID;  // 使用标准ID
    
    // 获取实际数据长度
    uint8_t payload_length = dlc_to_payload_length(frame.ctrl.tx.dlc);
    
    // 复制payload数据到vector
    std::vector<uint8_t> data;
    data.reserve(payload_length);
    for (uint8_t i = 0; i < payload_length && i < sizeof(frame.payload); i++) {
        data.push_back(frame.payload[i]);
    }

#if PRINT_SEND_DATA
    printf("LEJU TX: ID=0x%03X, DLC=%d, Len=%d, Data: ", 
           can_id, frame.ctrl.tx.dlc, payload_length);
    for (const auto& byte : data) {
        printf("%02X ", byte);
    }
    printf("\n");
#endif

    // 调用LEJU驱动发送
    LEJU_StatusTypeDef status = inner_->can_device->Send(can_id, data);
    
    if (status != LEJU_ERROR_OK) {
        LOG_W("[LejuCanbusWrapper] Send failed with error code: %d", status);
        return Result<ErrorType>::error(static_cast<ErrorType>(status));
    }

    CanMessageFrame* tef_frame = createCanMessageFrame();
    if (!tef_frame) {
        LOG_E("[LejuCanbusWrapper] Failed to allocate CanMessageFrame");
        return Result<ErrorType>::ok(static_cast<ErrorType>(LEJU_ERROR_OK)); // fixme
    }
    *tef_frame = frame;

    if (tef_callback_) {
        tef_callback_(tef_frame, tef_context_);
        // 注意：回调函数负责释放frame内存
    } else {
        // 如果没有回调，释放frame
        freeCanMessageFrame(tef_frame);
    }

    return Result<ErrorType>::ok(static_cast<ErrorType>(LEJU_ERROR_OK));
}

// ===== 设置回调函数 =====

void LejuCanbusWrapper::setMessageReceiveCallback(MessageCallback callback, 
                                                   const CallbackContext* context) {
    callback_ = callback;
    context_ = context;
    LOG_D("[LejuCanbusWrapper] Message receive callback set");
}

void LejuCanbusWrapper::setTefEventCallback(TefEeventCallback callback, 
                                            const CallbackContext* context) {
    tef_callback_ = callback;
    tef_context_ = context;
    LOG_D("[LejuCanbusWrapper] TEF event callback set");
}

// ===== 接收消息 =====

Result<ErrorType> LejuCanbusWrapper::receivedCanMessages() {
    if (!inner_ || !inner_->can_device) {
        LOG_E("[LejuCanbusWrapper] CAN device not initialized");
        return Result<ErrorType>::error(static_cast<ErrorType>(LEJU_ERROR_INIT));
    }

    int message_count = 0;
    constexpr int MAX_MESSAGES_PER_POLL = 100;  // 每次轮询最多处理100条消息

    // 循环接收消息，直到队列为空
    for (int i = 0; i < MAX_MESSAGES_PER_POLL; i++) {
        Recv_frame leju_frame;
        LEJU_StatusTypeDef status = inner_->can_device->Receive(leju_frame);

        // 超时或队列为空则退出
        if (status == LEJU_ERROR_BUSTIMEOUT) {
            break;
        }

        // 其他错误
        if (status != LEJU_ERROR_OK) {
            if (status == LEJU_ERROR_PARSE) {
                LOG_W("[LejuCanbusWrapper] Failed to parse received frame");
                continue;  // 跳过无效帧，继续接收
            }
            LOG_E("[LejuCanbusWrapper] Receive error: %d", status);
            return Result<ErrorType>::error(static_cast<ErrorType>(status));
        }  

        // 成功接收到消息
        message_count++;

#if PRINT_SEND_DATA
        printf("LEJU RX: ID=0x%03X, DLC=%d, Data: ", 
               leju_frame.id, leju_frame.dlc);
        for (const auto& byte : leju_frame.data) {
            printf("%02X ", byte);
        }
        printf("\n");
#endif

        // 转换为SDK格式：LEJU Recv_frame -> CanMessageFrame
        CanMessageFrame* frame = createCanMessageFrame();
        if (!frame) {
            LOG_E("[LejuCanbusWrapper] Failed to allocate CanMessageFrame");
            continue;
        }

        // 填充CAN ID
        frame->id.SID = leju_frame.id & 0x7FF;      // 标准ID（11位）
        frame->id.EID = (leju_frame.id >> 11) & 0x3FFFF;  // 扩展ID（18位）
        frame->id.SID11 = 0;
        frame->id.unimplemented1 = 0;

        // 设置DLC
        uint8_t dlc_code = payload_length_to_dlc(leju_frame.dlc);
        frame->ctrl.rx.dlc = dlc_code;

        // 复制payload数据
        size_t copy_len = std::min(leju_frame.data.size(), sizeof(frame->payload));
        for (size_t j = 0; j < copy_len; j++) {
            frame->payload[j] = leju_frame.data[j];
        }
        // 剩余部分填充0
        if (copy_len < sizeof(frame->payload)) {
            memset(&frame->payload[copy_len], 0, sizeof(frame->payload) - copy_len);
        }

        // 调用回调函数
        if (callback_) {
            callback_(frame, context_);
            // 注意：回调函数负责释放frame内存
        } else {
            // 如果没有回调，释放frame
            freeCanMessageFrame(frame);
        }
    }

    return Result<ErrorType>::ok(static_cast<ErrorType>(message_count));
}

} // namespace canbus_sdk

