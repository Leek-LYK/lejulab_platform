#include "bm_canbus_wrapper.h"
#include "canbus_sdk/result.h"
#include "canbus_sdk/canbus_log.h"
#include "canbus_sdk/canbus_sdk.h"

#include "bmapi.h"

#include <iostream>
#include <algorithm>
#include <cstring>
#include <string>
#include <mutex>
#include <iomanip>
#include <iostream>
#include <chrono>

#define  PRINT_SEND_DATA 0
namespace canbus_sdk {

#define GET_BM_ERROR_TEXT(err, buffer) \
    memset(buffer, 0, sizeof(buffer)); \
    BM_GetErrorText(err, buffer, sizeof(buffer), 0);

class BusmustLibrary {
public:
    static BusmustLibrary& getInstance() {
        static BusmustLibrary instance;
        return instance;
    }

    BM_StatusTypeDef init() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!initialized_) {
            initialized_ = true;
            // Initialize BMAPI library, this function shall be called before any other API calls 
            // and shall only be called once.
            auto err = BM_Init();
            if(err != BM_ERROR_OK) {
                char buffer[256]{};
                GET_BM_ERROR_TEXT(err, buffer);
                LOG_E("[BusmustLibrary] BM_Init Error 0X%08X: %s.", err, buffer);
                return err;
            }

            BM_SetLogLevel(BM_LOG_INF);

            // Print BUSMUST version
            union bmapi_version_num_t {
                struct __attribute__((packed)) {
                uint32_t build : 16;    //bit15-00 = build
                uint32_t revision : 8;  // bit23-16 = revision
                uint32_t minor : 4;     // bit27-24 = minor
                uint32_t major : 4;     // bit31-28 = major
                }; 
                uint32_t packed;
            };

            uint32_t bmapi_version =  BM_GetVersion();
            bmapi_version_num_t version_num;
            version_num.packed = bmapi_version;
            LOG_I("[BusmustLibrary] BUSMUST VERSION (major.minor.revision.build): %d.%d.%d.%d", version_num.major, 
                version_num.minor, version_num.revision, version_num.build);
        }

        return BM_ERROR_OK;
    }

    void uninit() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (initialized_) {
            initialized_ = false;
            // Un-initialize BMAPI library, this function shall be called after any 
            // other API calls and shall only be called once.
            BM_UnInit();
        }
    }

private:
    BusmustLibrary() {}
    ~BusmustLibrary() {
        if (initialized_)
            BM_UnInit();
    }

    bool initialized_ = false;
    std::mutex mutex_;
};
// Define ChannelRecoveryConfigs structure in cpp file to hide implementation details
struct BMCanbusWrapper::ChannelRecoveryConfigs {
    uint64_t previousRecoveryTs;                           ///< 上次恢复时间戳
    BM_ChannelInfoTypeDef info;                           ///< 通道信息
    BM_CanModeTypeDef mode;                               ///< 工作模式
    BM_BitrateTypeDef bitrate;                            ///< 比特率配置
    BM_TerminalResistorTypeDef tres;                      ///< 终端电阻配置
    BM_RxFilterTypeDef rxfilters[2];                      ///< 接收过滤器数组
    BM_TxTaskTypeDef txtasks[64];                         ///< 发送任务数组
};

// Define Inner structure in cpp file to hide implementation details
struct BMCanbusWrapper::Inner {
    BM_ChannelHandle channel_handle;
    BM_NotificationHandle notification_handle;
    ChannelRecoveryConfigs recovery_configs;              ///< 通道恢复配置

    Inner() : channel_handle(nullptr), notification_handle(nullptr) {
        memset(&recovery_configs, 0, sizeof(recovery_configs));
        recovery_configs.previousRecoveryTs = 0;
    }

    void reset() {
        channel_handle = nullptr;
        notification_handle = nullptr;
        memset(&recovery_configs, 0, sizeof(recovery_configs));
        recovery_configs.previousRecoveryTs = 0;
    }
};

BMCanbusWrapper::BMCanbusWrapper(const std::string& canbus_name,
    CanBusModelType model_type,
    const CanBusBitrate& bitrate)
    : CanBusBase(canbus_name, model_type, bitrate),
    inner_(new Inner()) {
}

BMCanbusWrapper::~BMCanbusWrapper()
{
    if (inner_ && inner_->channel_handle) {
        BM_StatusTypeDef error = BM_Close(inner_->channel_handle);
        if (error != BM_ERROR_OK) {
            char buffer[256] = { 0 };
            GET_BM_ERROR_TEXT(error, buffer);
            LOG_E("[~CanbusWrapper] BM_Close Error 0X%08X: %s.", error, buffer);
        }

        inner_->reset();
    }
}

constexpr size_t VERSION_SIZE = 4;
uint8_t  kBusmustAVersion [VERSION_SIZE] = {0x02, 0x02, 0x04, 0x0A}; // A 款 0x0202040A
uint8_t  kBusmustBVersion [VERSION_SIZE] = {0x02, 0x02, 0x04, 0x0B}; // B 款 0x0202040B

Result<bool> BMCanbusWrapper::init()
{
    char buffer[256] = { 0 };
    BM_StatusTypeDef err = BM_ERROR_OK;

    /* Step1: Initialize BMAPI library before any other operation */
    err = BusmustLibrary::getInstance().init();
    if(err != BM_ERROR_OK) {
        LOG_E("[BMCanbusWrapper] init, BM_Init Error");
        return Result<bool>::error(static_cast<ErrorType>(err));
    }

    /* Step2: Enumerate CAN bus channels */
    BM_ChannelInfoTypeDef channelinfos[32]{};
    int nchannels = sizeof(channelinfos) / sizeof(channelinfos[0]);
    err = BM_Enumerate(channelinfos, &nchannels);
    if (err != BM_ERROR_OK) {
        GET_BM_ERROR_TEXT(err, buffer);
        LOG_E("[CanbusWrapper] init, BM_Enumerate Error 0X%08X: %s.", err, buffer);
        return Result<bool>::error(static_cast<ErrorType>(err));
    }

    /* Step3: Find the target channel */
    uint8_t* target_version = nullptr;
    const char* target_model_name = nullptr;
    if(model_type_ == CanBusModelType::BUSMUST_A) {
        target_version = kBusmustAVersion;
        target_model_name = "BUSMUST_A";
    } else if(model_type_ == CanBusModelType::BUSMUST_B) {
        target_version = kBusmustBVersion;
        target_model_name = "BUSMUST_B";
    } else {
        LOG_E("[BMCanbusWrapper] Invalid model type");
        return Result<bool>::error(static_cast<ErrorType>(BM_ERROR_UNKNOWN));
    }
    
    // Print target version information
    LOG_I("[BMCanbusWrapper] Target name:%s, Target model: %s, Target version: %02X%02X%02X%02X", 
        canbus_name_.c_str(), target_model_name, target_version[0], target_version[1], target_version[2], target_version[3]);

    auto print_hex_array = [](const uint8_t* arr, size_t len) {
        for (size_t k = 0; k < len; ++k) {
            printf("%02X", arr[k]);
        }
    };

    int target_channel = -1;
    for (int i = 0; i < nchannels; i++) {
        printf("Device %d Information:\n", i + 1);
        printf("  Name: %s\n", channelinfos[i].name);
        printf("  Serial Number: ");
        print_hex_array(channelinfos[i].sn, sizeof(channelinfos[i].sn));
        printf("\n");

        printf("  UID: ");
        print_hex_array(channelinfos[i].uid, sizeof(channelinfos[i].uid));
        printf("\n");

        printf("  Version: ");
        print_hex_array(channelinfos[i].version, sizeof(channelinfos[i].version));
        printf("\n");

        printf("  VID: 0x%04X\n", channelinfos[i].vid);
        printf("  PID: 0x%04X\n", channelinfos[i].pid);
        printf("  Port ID: %d\n", channelinfos[i].port);
        printf("  Capabilities: 0x%04X\n", channelinfos[i].cap);

        // TODO(chenmingfu): match channel name 匹配 通道名称
        // if (strcmp(channelinfos[i].name, canbus_name_.c_str()) != 0) {
        //     continue;
        // }
        
        // match can model
        auto version_match = [](const uint8_t* device_version, const uint8_t* target_version) {
            for (size_t j = 0; j < VERSION_SIZE; j++) {
                if (device_version[j] != target_version[j]) {
                    return false;
                }
            }
            return true;
        };
        
        if (version_match(channelinfos[i].version, target_version)) {
            LOG_I("[BMCanbusWrapper] Found target channel: %s", channelinfos[i].name);
            target_channel = i;
            break;
        }
    }

    if(target_channel == -1) {
        LOG_E("[BMCanbusWrapper] Not found target channel");
        
        // Provide helpful guidance when no target device is found
        LOG_WARNING("🔍【设备查找提示】未找到目标BUSMUST设备！");
        LOG_WARNING("📋 已检测到 %d 个设备，但没有匹配的目标型号", nchannels);
        LOG_WARNING("🔧 请检查：");
        LOG_WARNING("   1. BUSMUST设备是否正确连接");
        LOG_WARNING("   2. 当前用户是否有权限访问设备，没有请执行`sudo usermod -a -G dialout $USER` 或以 root 用户运行程序");
        
        return Result<bool>::error(static_cast<ErrorType>(BM_ERROR_UNKNOWN));
    }

    /* Step4: Open the target channel */
    BM_BitrateTypeDef bm_bitrate{};
    memset(&bm_bitrate, 0, sizeof(bm_bitrate));
    bm_bitrate.nbitrate = bitrate_.nbitrate;        /* 仲裁段波特率 */
    bm_bitrate.dbitrate = bitrate_.dbitrate;        /* 数据段波特率 */
    bm_bitrate.nsamplepos = bitrate_.nsamplepos;    /* 仲裁段采样点 */
    bm_bitrate.dsamplepos = bitrate_.dsamplepos;    /* 数据段采样点 */

    BM_CanModeTypeDef can_mode = BM_CAN_NORMAL_MODE;
    // BM_CanModeTypeDef can_mode = BM_CAN_INTERNAL_LOOPBACK_MODE;
    err = BM_OpenEx(&inner_->channel_handle, &channelinfos[target_channel], 
                        can_mode, BM_TRESISTOR_120, &bm_bitrate, NULL, 0);
    if (err != BM_ERROR_OK) {
        inner_->reset();
        GET_BM_ERROR_TEXT(err, buffer);
        LOG_E("[BMCanbusWrapper] init, BM_OpenEx Error 0X%08X: %s.", err, buffer);
        
        // Check for common permission-related errors when opening device
        if (err == BM_ERROR_NODRIVER || err == BM_ERROR_UNKNOWN || err == BM_ERROR_ILLHW) {
            LOG_WARNING("🚨【设备打开失败】可能是权限问题，请检查：");
            LOG_WARNING("   1. USB设备权限: sudo usermod -a -G dialout $USER");
            LOG_WARNING("   2. 设备是否被其他程序占用");
            LOG_WARNING("   3. USB线缆连接是否正常");
            LOG_WARNING("   4. 尝试重新插拔设备");
        }
        
        return Result<bool>::error(static_cast<ErrorType>(err));
    }

    /* Step5: Get notification handle */
    err = BM_GetNotification(inner_->channel_handle, &inner_->notification_handle);
    if (err != BM_ERROR_OK) {
        inner_->reset();
        GET_BM_ERROR_TEXT(err, buffer);
        LOG_E("[BMCanbusWrapper] init, BM_GetNotification Error 0X%08X: %s.", err, buffer);
        return Result<bool>::error(static_cast<ErrorType>(err));
    }

    /* Step6: Initialize channel recovery configuration */
    memcpy(&inner_->recovery_configs.info, &channelinfos[target_channel], sizeof(channelinfos[target_channel]));
    inner_->recovery_configs.mode = can_mode;
    inner_->recovery_configs.tres = BM_TRESISTOR_120;
    memcpy(&inner_->recovery_configs.bitrate, &bm_bitrate, sizeof(bm_bitrate));
    memset(inner_->recovery_configs.rxfilters, 0, sizeof(inner_->recovery_configs.rxfilters));
    memset(inner_->recovery_configs.txtasks, 0, sizeof(inner_->recovery_configs.txtasks));

    return Result<bool>::ok(true);
}

Result<ErrorType> BMCanbusWrapper::writeCanMessage(const CanMessageFrame& frame) {
    BM_CanMessageTypeDef msg{};
    memset(&msg, 0, sizeof(msg));
    
    // Set CAN ID
    BM_SET_STD_MSG_ID(msg.id, frame.id.SID);
    msg.ctrl.tx.DLC = frame.ctrl.tx.dlc;
    
    // Copy payload data (determine length from DLC)
    uint8_t payload_length = dlc_to_payload_length(frame.ctrl.tx.dlc);
    uint8_t copy_length = (payload_length > sizeof(frame.payload)) ? sizeof(frame.payload) : payload_length;
    memcpy(msg.payload, frame.payload, copy_length);
#if PRINT_SEND_DATA
    printf("BUS TX: ID=0x%03X, DLC=%d, Data: ", frame.id.SID, frame.ctrl.tx.dlc);
    for (int i = 0; i < copy_length && i < 8; ++i) {
        printf("%02X ", msg.payload[i]);
    }
    printf("\n");
#endif

    constexpr int timeout_ms = 0; // 0  异步
    BM_StatusTypeDef error = BM_WriteCanMessage(inner_->channel_handle, &msg, 0, timeout_ms, NULL);
    if (error == BM_ERROR_BUSTIMEOUT)  {
        recoverFromError();
    }
    else if (error != BM_ERROR_OK) {
        auto currentTs = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
        char buffer[256] = { 0 };
        GET_BM_ERROR_TEXT(error, buffer);
        LOG_WARNING("[%ld] [BMCanbusWrapper] writeCanMessage, BM_WriteCanMessage Error 0X%08X: %s.", currentTs, error, buffer);
        recoverFromError();
        return Result<ErrorType>::error(static_cast<ErrorType>(error));
    }
    
    return Result<ErrorType>::ok(static_cast<ErrorType>(BM_ERROR_OK));
}

void BMCanbusWrapper::setMessageReceiveCallback(MessageCallback callback, const CallbackContext* context) {
    callback_ = callback;
    context_ = context;
}

void BMCanbusWrapper::setTefEventCallback(TefEeventCallback callback, const CallbackContext* context) {
    tef_callback_ = callback;
    tef_context_ = context;
}

Result<ErrorType> BMCanbusWrapper::receivedCanMessages() {
    int message_count = 0;

    // Wait for notifications with a timeout
    constexpr int Timeout_ms = 1000;
    if (BM_WaitForNotifications(&inner_->notification_handle, 1, Timeout_ms) < 0) {
        return Result<ErrorType>::ok(message_count);
    }

    // Read message from CAN bus using BM_Read instead of BM_ReadCanMessage
    BM_DataTypeDef rx_data = { 0 };
    BM_StatusTypeDef error = BM_ERROR_OK;

    while((error = BM_Read(inner_->channel_handle, &rx_data)) == BM_ERROR_OK) {
        // Check if the data has BM_ACK_DATA flag bit
        bool flag = false;
        if ((rx_data.header.type & (BM_ACK_DATA | BM_CAN_FD_DATA)) == (BM_ACK_DATA | BM_CAN_FD_DATA)) {
            // 调用TEF回调处理ACK消息
            if (tef_callback_) {
                BM_CanMessageTypeDef* can_msg = reinterpret_cast<BM_CanMessageTypeDef*>(rx_data.payload);
                uint32_t can_id = BM_GET_STD_MSG_ID(can_msg->id);
                uint8_t dlc = can_msg->ctrl.rx.DLC;

                // 创建TEF事件帧
                CanMessageFrame* tef_frame = createCanMessageFrame();
                if (tef_frame) {
                    tef_frame->id.SID = can_id;
                    tef_frame->id.EID = BM_GET_EXT_MSG_ID(can_msg->id);
                    tef_frame->id.SID11 = 0;
                    tef_frame->id.unimplemented1 = 0;
                    tef_frame->ctrl.rx.dlc = dlc;
                    memcpy(tef_frame->payload, can_msg->payload, 64);

                    // Call the TEF callback if registered
                    if (tef_callback_) {
                        // Note: The callback function is responsible for freeing the frame
                        tef_callback_(tef_frame, tef_context_);
                    } else {
                        // If no callback is registered, free the frame
                        freeCanMessageFrame(tef_frame);
                    }
                }
            }
            // LOG_D("[BMCanbusWrapper] received ack message");
        }
        else if (rx_data.header.type & BM_CAN_FD_DATA) {
            flag = true;
        }

        // LOG_D("[BMCanbusWrapper] receivedCanMessages, rx_data.header.type:%d", rx_data.header.type);

        if (flag) {
            BM_CanMessageTypeDef* can_msg = reinterpret_cast<BM_CanMessageTypeDef*>(rx_data.payload);
            uint32_t can_id = BM_GET_STD_MSG_ID(can_msg->id);
            uint8_t dlc = can_msg->ctrl.rx.DLC;
#if PRINT_SEND_DATA
            printf("BUS RX: ID=0x%03X, DLC=%d, Data: ", can_id, dlc);
            for (int i = 0; i < dlc && i < 8; ++i) {
                printf("%02X ", can_msg->payload[i]);
            }
            printf("\n");
#endif
            message_count++;

            // Convert BM_DataTypeDef to CanMessageFrame (heap allocated)
            CanMessageFrame* frame = createCanMessageFrame();
            if (!frame) {
                LOG_E("[BMCanbusWrapper] Failed to allocate CanMessageFrame");
                // Read next message
                error = BM_Read(inner_->channel_handle, &rx_data);
                continue;
            }

            // Extract CAN message information from BM_DataTypeDef
            frame->id.SID = BM_GET_STD_MSG_ID(can_msg->id);
            frame->id.EID = BM_GET_EXT_MSG_ID(can_msg->id);
            frame->id.SID11 = 0; // Reserved
            frame->id.unimplemented1 = 0; // Reserved

            // Set RX DLC from received message
            frame->ctrl.rx.dlc = can_msg->ctrl.rx.DLC;

            // Copy payload data (full 64 bytes)
            memcpy(frame->payload, can_msg->payload, 64);

            // Call the callback if registered
            if (callback_) {
                callback_(frame, context_);
                // Note: The callback function is now responsible for freeing the frame
            } else {
                // If no callback is registered, free the frame
                freeCanMessageFrame(frame);
            }
        }
    }

    if(error == BM_ERROR_QRCVEMPTY) {
        return Result<ErrorType>::ok(message_count);
    }
    else if (error != BM_ERROR_OK) {
        LOG_E("Failed to receive message, error code: %d", error);
        return Result<ErrorType>::error(error);
    }

    return Result<ErrorType>::ok(message_count);
}


void BMCanbusWrapper::recoverFromError()
{
 /* THIS FUNCTION COPY FROM BUS MUST EXAMPLE */
    // Note: It takes some time to run BM_Close and BM_OpenEx,
    // and channel handles will be invalidated when recovering from error,
    // make sure no other threads are using channel handles (e.g. calling BM_Write) during the recovery,
    // or, use single-threaded design, just like this demo.
    auto currentTs = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
    uint64_t elapsed = currentTs - inner_->recovery_configs.previousRecoveryTs;
    if (elapsed >= 1000ULL)
    {
        BM_CanStatusInfoTypedef canStatus;
        BM_StatusTypeDef status = BM_GetStatus(inner_->channel_handle, &canStatus);
        // cppcheck-suppress unreadVariable
        BM_StatusTypeDef error = BM_ERROR_OK;
        if (status & BM_ERROR_INITIALIZE)
        {
            // BM_Init() is not called yet.
            // Read our SDK documentation for details.
            LOG_WARNING("[%ld] [BMCanbusWrapper] recoverFromError CAN模块遇到错误自动恢复, BM_ERROR_INITIALIZE", currentTs);
        }
        else if (status & BM_ERROR_ILLPARAMVAL)
        {
            // Channel handle is invalid.
            // Maybe it's not opened yet (using BM_OpenEx) or already closed?
            LOG_WARNING("[%ld] [BMCanbusWrapper] recoverFromError CAN模块遇到错误自动恢复, BM_ERROR_ILLPARAMVAL", currentTs);
            auto &rc = inner_->recovery_configs;
            error = BM_OpenEx(&inner_->channel_handle, &rc.info, rc.mode, rc.tres, &rc.bitrate, &rc.rxfilters[0], 0);
            if (error != BM_ERROR_OK) {
                LOG_E("[BMCanbusWrapper] recoverFromError BM_OpenEx failed with error: %d", error);
            }
            rc.previousRecoveryTs = currentTs;
        }
        else if (status & BM_ERROR_ILLOPERATION)
        {
            // USB Device operation failed.
            // Maybe the device is unplugged from host PC?

            LOG_WARNING("[%ld] [BMCanbusWrapper] recoverFromError CAN模块遇到错误自动恢复, BM_ERROR_ILLOPERATION", currentTs);
            // Close the channel directly
            BM_Close(inner_->channel_handle);
            inner_->channel_handle = NULL;

            // Try to reopen the channel
            auto &rc = inner_->recovery_configs;
            error = BM_OpenEx(&inner_->channel_handle, &rc.info, rc.mode, rc.tres, &rc.bitrate, &rc.rxfilters[0], 0);
            if (error != BM_ERROR_OK) {
                LOG_E("[BMCanbusWrapper] recoverFromError BM_OpenEx failed with error: %d", error);
            }
            rc.previousRecoveryTs = currentTs;
        }
        else if ((status & BM_ERROR_BUSOFF) || (status & BM_ERROR_BUSPASSIVE) ||
            (status == BM_ERROR_OK && (canStatus.TXBO || canStatus.TXBP)))
        {
            // BUSOFF
            // Maybe the remote CAN device is disconnected,
            // or you might want to check your bitrate, sample-position, tres configuration.
            LOG_WARNING("[%ld] [BMCanbusWrapper] recoverFromError CAN模块遇到错误自动恢复, BUSOFF RECOVERY", currentTs);
            // Sometimes BUSOFF might be reported from a good channel.
            // We only perform BUSOFF recovery on the channels that are reported to have tx problems (e.g. tx timeout).
            BM_CanModeTypeDef oldmode;
			// BM_GetCanMode(inner_->channel_handle, &oldmode);
            BM_SetCanMode(inner_->channel_handle, BM_CAN_INTERNAL_LOOPBACK_MODE);
            static BM_CanMessageTypeDef dummyMessages[256] = { 0 };
            uint32_t nmessages = (uint32_t)(sizeof(dummyMessages) / sizeof(dummyMessages[0]));
            for (uint32_t k = 0; k < nmessages; k++)
            {
                dummyMessages[k].ctrl.tx.DLC = 1;
            }
            BM_WriteMultipleCanMessage(inner_->channel_handle, dummyMessages, &nmessages, NULL, 1000, NULL);
            BM_SetCanMode(inner_->channel_handle, BM_CAN_NORMAL_MODE);
            BM_ClearBuffer(inner_->channel_handle);
            inner_->recovery_configs.previousRecoveryTs = currentTs;
        }
    }
}

#undef GET_BM_ERROR_TEXT

} // namespace canbus_sdk