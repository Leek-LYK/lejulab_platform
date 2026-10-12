#include "stark-sdk.h"
#include "revo2_hand_can_customed.h"
#include "canbus_sdk/canbus_sdk.h"

#include <stdint.h>
#include <vector>
#include <iostream>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <unordered_map>

namespace dexhand {

// 日志宏定义
#define REVO2_LOG_ERROR(fmt, ...) printf("\033[31m[Revo2CanDexhand] ERROR: " fmt "\033[0m\n", ##__VA_ARGS__)
#define REVO2_LOG_WARN(fmt, ...) printf("\033[33m[Revo2CanDexhand] WARN: " fmt "\033[0m\n", ##__VA_ARGS__)
#define REVO2_LOG_INFO(fmt, ...) printf("[Revo2CanDexhand] INFO: " fmt "\n", ##__VA_ARGS__)

// CAN消息队列结构实现
struct Revo2CanMessageQueue {
    static const size_t MAX_QUEUE_SIZE = 1024;
    std::queue<canbus_sdk::CanMessageFrame> queue;
    std::mutex mutex;
    std::condition_variable cv;

    void push(const canbus_sdk::CanMessageFrame& frame) {
        std::lock_guard<std::mutex> lock(mutex);
        if (queue.size() >= MAX_QUEUE_SIZE) {
            // 队列已满，丢弃最旧的消息
            queue.pop();
        }
        queue.push(frame);
        cv.notify_one();
    }

    // 获取队列第一条消息（不弹出）
    bool front(canbus_sdk::CanMessageFrame* frame_out) {
        std::lock_guard<std::mutex> lock(mutex);
        if (!queue.empty()) {
            *frame_out = queue.front();
            return true;
        }
        return false;
    }

    // 弹出一条消息
    bool try_pop(canbus_sdk::CanMessageFrame* frame_out, int timeout_ms = 200) {
        std::unique_lock<std::mutex> lock(mutex);
        if (cv.wait_for(lock, std::chrono::milliseconds(timeout_ms), [this] { return !queue.empty(); })) {
            if (!queue.empty()) {
                *frame_out = queue.front();
                queue.pop();
                return true;
            }
        }
        return false;
    }
};    

///////////////////////////////////////////////////////////////////////////////
// 封装全局映射关系
///////////////////////////////////////////////////////////////////////////////

struct Revo2HandContext {
    canbus_sdk::BusId bus_id;
    struct Revo2CanMessageQueue *message_queue;
};

class Revo2HandContextManager {
public:
    static bool create(uint8_t slave_id, canbus_sdk::BusId bus_id) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (contexts_.find(slave_id) != contexts_.end()) {
            return false; // 已存在
        }
        contexts_[slave_id].bus_id = bus_id;
        contexts_[slave_id].message_queue = new Revo2CanMessageQueue();
        return true;
    }

    static bool has(uint8_t slave_id) {
        std::lock_guard<std::mutex> lock(mutex_);
        return contexts_.find(slave_id) != contexts_.end();
    }

    static Revo2HandContext* get(uint8_t slave_id) {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = contexts_.find(slave_id);
        if (it != contexts_.end()) {
            return &it->second;
        }
        return nullptr;
    }

    static canbus_sdk::BusId getBusId(uint8_t slave_id) {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = contexts_.find(slave_id);
        if (it != contexts_.end()) {
            return it->second.bus_id;
        }
        return -1; // -1表示无效的bus_id
    }

    static Revo2CanMessageQueue* getMessageQueue(uint8_t slave_id) {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = contexts_.find(slave_id);
        if (it != contexts_.end()) {
            return it->second.message_queue;
        }
        return nullptr;
    }

    static bool remove(uint8_t slave_id) {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = contexts_.find(slave_id);
        if (it != contexts_.end()) {
            // 释放消息队列内存
            if (it->second.message_queue) {
                delete it->second.message_queue;
                it->second.message_queue = nullptr;
            }
            contexts_.erase(it);
            return true;
        }
        return false;
    }

private:
    static std::mutex mutex_;
    static std::unordered_map<uint8_t, Revo2HandContext> contexts_;
};

// 静态成员初始化
std::mutex Revo2HandContextManager::mutex_;
std::unordered_map<uint8_t, Revo2HandContext> Revo2HandContextManager::contexts_;

///////////////////////////////////////////////////////////////////////////////
// Helper Functions for BrainCo Revo2 Hand SDK
///////////////////////////////////////////////////////////////////////////////
void setup_can_revo2_callbacks()
{
  // 需要由接入方实现，开启CAN FD设备，和以下读写回调
  set_can_tx_callback([](uint8_t slave_id,
                         uint32_t can_id,
                         const uint8_t *data,
                         uintptr_t data_len) -> int
                      {
#if DEBUG_PRINT_CAN_DATA
                        uint8_t device_id = (can_id >> 7) & 0x0F;  // 7-10位是Device ID
                        uint8_t cmd = (can_id >> 3) & 0x0F;        // 3-6位是Command
                        uint8_t channel = can_id & 0x07;           // 0-2位是Channel
                        printf("CAN TX: Slave ID: %d, CAN ID: 0x%x, cmd:0x%X, Data Length: %lu, Data: ", slave_id, can_id, cmd, data_len);
                        for (uintptr_t i = 0; i < data_len; ++i) {
                            printf("%02X ", data[i]);
                        }
                        printf("\n");
#endif
                        using namespace canbus_sdk;

                        // 查找对应的 bus_id
                        canbus_sdk::BusId bus_id = Revo2HandContextManager::getBusId(slave_id);
                        if (bus_id == -1) {
                            REVO2_LOG_ERROR("Cannot find bus_id for slave_id %d", slave_id);
                            return -1;
                        }

                        CanMessageFrame can_frame{};
                        can_frame.id.SID = static_cast<uint32_t>(can_id);
                        can_frame.ctrl.tx.dlc = payload_length_to_dlc(8);
                        memcpy(can_frame.payload, data, data_len);

                        // 处理发送结果
                        auto result = CanBusController::getInstance().sendMessage(bus_id, can_frame);
                        return result.has_value() ? 0: -1; // Return 0 to indicate success
                      });
                  
  set_can_rx_callback([](uint8_t slave_id,
                         uint32_t expected_can_id,
                         uint8_t expected_frames,
                         uint32_t *can_id_out,
                         uint8_t *data_out,
                         uintptr_t *data_len_out) -> int
                      {
                        // 查找对应的消息队列
                        Revo2CanMessageQueue* message_queue = Revo2HandContextManager::getMessageQueue(slave_id);
                        if (!message_queue) {
                            REVO2_LOG_ERROR("Cannot find message queue for slave_id %d", slave_id);
                            return -1;
                        }

                        // 从消息队列中读取数据
                        canbus_sdk::CanMessageFrame first_frame;
                        if (!message_queue->try_pop(&first_frame)) {
                        //   REVO2_LOG_ERROR("Failed to read from CAN message queue for slave_id %d", slave_id);
                          return -1;
                        }
                          
                        constexpr int MAX_BUF_SIZE = 8;
                        canbus_sdk::CanMessageFrame received_frames[MAX_BUF_SIZE];
                        received_frames[0] = first_frame;
                        int frame_count = 1;

                        // 解析第一条消息的CAN ID和cmd
                        uint32_t base_can_id = received_frames[0].id.SID;
                        uint8_t base_cmd = (base_can_id >> 3) & 0x0F;

                        // 如果cmd是0x0B，继续获取后续连续的具有相同CAN ID和cmd的消息
                        if (base_cmd == 0x0B) {
                            while (frame_count < MAX_BUF_SIZE) {
                                canbus_sdk::CanMessageFrame peek_frame;
                                if (!message_queue->front(&peek_frame))
                                    break;

                                uint32_t next_can_id = peek_frame.id.SID;
                                uint8_t next_cmd = (next_can_id >> 3) & 0x0F;

                                // 检查是否具有相同的CAN ID和cmd
                                if (next_can_id != base_can_id || next_cmd != base_cmd)
                                    break;

                                // 弹出这条消息
                                canbus_sdk::CanMessageFrame next_frame;
                                if (!message_queue->try_pop(&next_frame))
                                    break;

                                received_frames[frame_count] = next_frame;
                                frame_count++;
                            }
                        }

                        // 复制数据
                        int idx = 0;
                        int total_dlc = 0;
                        for (int i = 0; i < frame_count; i++)
                        {
                          canbus_sdk::CanMessageFrame &frame = received_frames[i];
                          int can_dlc = frame.ctrl.rx.dlc;
                          for (int j = 0; j < can_dlc; j++)
                          {
                            data_out[idx++] = frame.payload[j];
                          }
                          total_dlc += can_dlc;
                        }

                        *can_id_out = received_frames[0].id.SID & 0x1FFFFFFF;
                        *data_len_out = total_dlc;
                        return 0;
                      });
}

void can_receive_revo2_callback(canbus_sdk::CanMessageFrame* frame, const canbus_sdk::CallbackContext* context)
{
    uint32_t device_id = (frame->id.SID >> 7) & 0x0F;
#if DEBUG_PRINT_CAN_DATA
    uint8_t cmd = (frame->id.SID >> 3) & 0x0F;        // 3-6位是Command
    uint8_t channel = frame->id.SID & 0x07;           // 0-2位是Channel
    printf("\033[32mCAN FD Received: ID: 0x%X, Device_id:0x%X, CMD :0x%X, channel:0x%X, DLC: %d, Data: ", 
    frame->id.SID, device_id, cmd, channel,frame->ctrl.rx.dlc);
    for (int i = 0; i < frame->ctrl.rx.dlc; ++i)
    {
    printf("%02x ", frame->payload[i]);
    }
    printf("\033[0m\n");
#endif

    // 查找对应的消息队列并推送消息
    Revo2CanMessageQueue* message_queue = Revo2HandContextManager::getMessageQueue(device_id);
    if (message_queue) {
    message_queue->push(*frame);
    }

    freeCanMessageFrame(frame);
}

///////////////////////////////////////////////////////////////////////////////
// Revo2CanDexhand
///////////////////////////////////////////////////////////////////////////////


std::unique_ptr<Revo2CanDexhand> Revo2CanDexhand::Connect(
    const canbus_sdk::DeviceConfig& config
    ) {

    /************ Init Gobal Callbacks ************/
    static std::once_flag init_flag;
    std::call_once(init_flag, []() {
        setup_can_revo2_callbacks();
        init_logging(LogLevel::LOG_LEVEL_WARN);
    });
    /***********************************************/
    using namespace canbus_sdk;
    if(config.device_type != DeviceType::REVO2_HAND) {
        REVO2_LOG_ERROR("Device type is not REVO2_HAND");
        return nullptr;
    }

    auto &bus_name = config.canbus_name;
    auto& canbus_controller = canbus_sdk::CanBusController::getInstance();
    // 需要外部已经打开总线 这里不负责打开，只负责注册
    // 先通过canbus_name获取bus_id，检查总线是否存在
    auto bus_id_result = canbus_controller.getBusIdByName(bus_name);
    if (!bus_id_result.has_value()) {
        REVO2_LOG_ERROR("Cannot find bus_id for canbus_name '%s'", bus_name.c_str());
        return nullptr;
    }
    canbus_sdk::BusId bus_id = bus_id_result.value();

    // 总线存在，则创建设备句柄

    // Matcher : 总线管理使用根据CANID分发消息
    auto Matcher = [](uint32_t can_id, uint32_t device_id) -> bool {
        ///////////////////////////////////////
        //  device_id : 4 bit 1~14
        //  cmd :       4 bit 1~15
        //  channel : 3 bit 0~7
        // 0001,0000,000 ~ 1110,1111,111 0X80 ~ 0x77F
        ///////////////////////////////////////
        constexpr uint32_t kHandMinCanId = 0X80, kHandMaxCanId = 0X77F;
        if (kHandMinCanId <= can_id && can_id <= kHandMaxCanId) {
          uint32_t id = (can_id >> 7) & 0x0F;  // 7-10位是Device ID
          uint8_t cmd = (can_id >> 3) & 0x0F;        // 3-6位是Command
        //   uint8_t channel = can_id & 0x07;           // 0-2位是Channel
        // 根据《revo2 can通信协议.pdf》手册只有 0x05-0x0B的读取命令
        if(!(cmd >= 0x05 && cmd <= 0x0B)) return false;
          return id == device_id;
        }
        return false;
    };
  
    canbus_sdk::CallbackParams callback_params{
        .msg_callback = can_receive_revo2_callback,
        .msg_cb_ctx = nullptr,
        .tef_callback = nullptr,
        .tef_cb_ctx = nullptr
    };


    // 注册设备到CAN总线
    // DeviceInfo 有用户声明的构造函数(非聚合),不能用指派初始化器,显式构造
    auto register_result = canbus_controller.registerDevice(
        canbus_sdk::DeviceInfo(config.name, config.device_type, config.device_id, Matcher),
        bus_name, callback_params);

    if (!register_result.has_value()) {
        auto error = register_result.error();
        if (error == static_cast<canbus_sdk::ErrorType>(canbus_sdk::CanBusError::ERROR_DEVICE_ALREADY_EXISTS)) {
            REVO2_LOG_WARN("Device already exists for device_id %d on bus '%s'", config.device_id, bus_name.c_str());
        } else {
            REVO2_LOG_ERROR("Failed to register device for device_id %d, error: %d", config.device_id, static_cast<int>(error));
            return nullptr;
        }
    }

    // 预先注册映射关系 - 使用获取到的bus_id
    if (!Revo2HandContextManager::create(config.device_id, bus_id)) {
        REVO2_LOG_ERROR("Failed to create hand context for device_id %d", config.device_id);
        return nullptr;
    }

    DeviceHandler* handle = init_device_handler(StarkProtocolType::STARK_PROTOCOL_TYPE_CAN, 0);
    if (!handle) {
        REVO2_LOG_ERROR("Failed to create device handler");
        return nullptr;
    }
    
    auto revo2_can_dexhand = std::unique_ptr<Revo2CanDexhand>(new Revo2CanDexhand(handle, config.device_id, config));
    /* !!! IMPORTANT !!! 必须要调用获取设备信息识别一下设备! */
    DeviceInfo_t dev_info;
    if(!revo2_can_dexhand->getDeviceInfo(dev_info)) {
        REVO2_LOG_ERROR("Failed to get device info");
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
        stark_set_finger_protected_current(handle, config.device_id, finger_id, protected_current);
    }
    REVO2_LOG_INFO("已设置所有手指的保护电流为 %d mA", protected_current);
    
    return revo2_can_dexhand;
}

Revo2CanDexhand::Revo2CanDexhand(DeviceHandler* handle, uint8_t slave_id_, const canbus_sdk::DeviceConfig& config)
    : ModbusDexhand(handle, slave_id_), config_(config) {
}

Revo2CanDexhand::~Revo2CanDexhand() {
    // 释放设备句柄
    if (mb_handle_) {
        REVO2_LOG_INFO("Closing Revo2CanDexhand handle for device %d", config_.device_id);
        close_device_handler(mb_handle_, static_cast<uint8_t>(StarkProtocolType::STARK_PROTOCOL_TYPE_CAN));
    }
    mb_handle_ = nullptr;
        
    // 反注册设备
    auto& canbus_controller = canbus_sdk::CanBusController::getInstance();
    auto unregister_result = canbus_controller.unregisterDevice(config_.device_type, config_.device_id, config_.canbus_name);
    if (!unregister_result.has_value()) {
        REVO2_LOG_WARN("Failed to unregister device %d from bus '%s', error: %d",
                       config_.device_id, config_.canbus_name.c_str(), static_cast<int>(unregister_result.error()));
    } else {
        REVO2_LOG_INFO("Device %d unregistered from bus '%s'", config_.device_id, config_.canbus_name.c_str());
    }

    // 释放上下文映射和消息队列
    if (!Revo2HandContextManager::remove(config_.device_id)) {
        REVO2_LOG_WARN("Failed to remove context for device %d", config_.device_id);
    }
}

DexHandFwType Revo2CanDexhand::getDexHandFwType() {
    return DexHandFwType::V2_STANDARD;
}


void Revo2CanDexhand::setFingerPositions(const UnsignedFingerArray &positions)
{
    try {
        // 使用std::clamp限制位置值在0~100范围内
        UnsignedFingerArray clamped_positions;
        for (size_t i = 0; i < positions.size(); ++i) {
            clamped_positions[i] = 10 * std::clamp(positions[i], static_cast<uint16_t>(0), static_cast<uint16_t>(100));
        }

        stark_set_finger_positions(mb_handle_, slave_id_, clamped_positions.data(), clamped_positions.size());

    } catch(std::exception &e) {
        std::cerr << "[TouchDexhand] setFingerPositions exception:" << e.what() << "\n";
    }
}

void Revo2CanDexhand::setFingerPositionsAndDurations(const UnsignedFingerArray &positions, const UnsignedFingerArray &durations) {
    try {
        // 使用std::clamp限制位置值在0~100范围内，然后乘以10
        UnsignedFingerArray scaled_positions;
        for (size_t i = 0; i < positions.size(); ++i) {
            scaled_positions[i] = 10 * std::clamp(positions[i], static_cast<uint16_t>(0), static_cast<uint16_t>(100));
        }

        // 使用std::clamp限制持续时间在1~2000毫秒范围内
        UnsignedFingerArray clamped_durations;
        for (size_t i = 0; i < durations.size(); ++i) {
            clamped_durations[i] = std::clamp(durations[i], static_cast<uint16_t>(1), static_cast<uint16_t>(2000));
        }

        stark_set_finger_positions_and_durations(mb_handle_, slave_id_, scaled_positions.data(), clamped_durations.data(), scaled_positions.size());

    } catch(std::exception &e) {
        REVO2_LOG_ERROR("setFingerPositionsAndDurations exception: %s", e.what());
    }
}

void Revo2CanDexhand::setGripForce(GripForce level) {
    REVO2_LOG_WARN("Revo2灵巧手不支持设置抓力等级，此函数调用将被忽略");
}

GripForce Revo2CanDexhand::getGripForce() {
    REVO2_LOG_WARN("Revo2灵巧手不支持获取抓力等级，返回默认值FORCE_LEVEL_NORMAL");
    return GripForce::FORCE_LEVEL_NORMAL;
}

}