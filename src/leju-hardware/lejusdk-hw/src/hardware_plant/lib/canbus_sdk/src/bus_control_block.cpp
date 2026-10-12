#include "bus_control_block.h"
#include "canbus_base.h"
#include "canbus_sdk/canbus_sdk.h"
#include "canbus_sdk/canbus_log.h"
#include "canbus_sdk/canbus_sdk_def.h"

#include <memory>
#include <mutex>
#include <thread>
#include <chrono>
#include <unordered_map>

// 宏控制：是否启用CAN线程统计信息打印（频率、消息计数等）
#ifndef DEBUG_CAN_RECV_THREAD_STATS
#define DEBUG_CAN_RECV_THREAD_STATS 1
#endif

namespace canbus_sdk {

/*
 * BusControlBlock 架构图
 * ======================
 *
 *                      +-------------------+
 *                      | BusControlBlock   |
 *                      | (总线控制块)      |
 *                      +---------+---------+
 *                                | 包含
 *                                v
 *                      +-------------------+
 *                      | BMCanbusWrapper   |
 *                      | (硬件封装器)      |
 *                      +---------+---------+
 *                                |
 *                                v
 *                      +-------------------+
 *                      |    BUSMUST API   |
 *                      |   (硬件驱动)      |
 *                      +-------------------+
 *
 * 设备管理关系:
 * ==============
 *
 *                      +-------------------+
 *                      | BusControlBlock   |
 *                      |                   |
 *                      |  +-------------+ | <-- devices_mutex_
 *                      |  | devices_     | |
 *                      |  |  (unordered_ | |
 *                      |  |   map)       | |
 *                      |  +------+------+ |
 *                      |         |          |
 *           +----------+---------+  +---------+----------+
 *           |          |          |  |          |          |
 *     +-----+----+ +-----+----+ +-----+----+  (更多设备...)
 *     |ID:1 |Reg1| |ID:2 |Reg2| |ID:3 |Reg3|
 *     +-----+----+ +-----+----+ +-----+----+
 *        |           |           |
 *        v           v           v
 *   +--------+  +--------+  +--------+
 *   |回调1   |  |回调2   |  |回调3   |
 *   +--------+  +--------+  +--------+
 *
 * 线程安全保护:
 * =============
 *
 *   +-------------------+     +-------------------+
 *   |  devices_mutex_  |     |  接收线程        |
 *   | (保护设备map)    |     |  (异步处理)      |
 *   +-------------------+     +-------------------+
 *           |                       |
 *           v                       v
 *   +-------------------+     +-------------------+
 *   |   devices_ map    |     |   消息分发        |
 *   |   (ID->Reg映射)   |     |   (回调机制)      |
 *   +-------------------+     +-------------------+
 *           |                       |
 *           v                       v
 *   +-------------------+     +-------------------+
 *   |  DeviceRegistration|     |   消息路由        |
 *   |   (查找O(1))      |     |   (根据ID查找)    |
 *   +-------------------+     +-------------------+
 *
 * 消息流向:
 * =========
 *
 *   外部调用 -> BusControlBlock -> BMCanbusWrapper -> 硬件
 *   硬件 -> BMCanbusWrapper -> BusControlBlock -> 设备回调 -> 外部处理
 *                                         |
 *                                         v
 *                                  根据消息ID查找map
 *                                -> DeviceRegistration -> 对应回调
 */

// Define static member variables
const int BusControlBlock::DEFAULT_RECV_FREQUENCY_MS;

BusControlBlock::BusControlBlock(const std::string& name, CanBusModelType model_type, const CanBusBitrate& bitrate)
    : bus_name_(name), model_type_(model_type), bitrate_(bitrate), canbus_(nullptr), is_active_(false), device_count_(0), recv_thread_cpu_core_(-1), recv_frequency_ms_(DEFAULT_RECV_FREQUENCY_MS), callback_(nullptr) {
    LOG_D("BusControlBlock constructor called for bus: %s", name.c_str());
    // 初始化回调上下文
    callback_context_.userdata = nullptr;
    tef_callback_context_.userdata = nullptr;
}

std::unique_ptr<BusControlBlock> BusControlBlock::create(const std::string& name, CanBusModelType model_type, const CanBusBitrate& bitrate) {
    LOG_I("Creating BusControlBlock for bus: %s, model: %d, bitrate: %d", 
          name.c_str(), static_cast<int>(model_type), bitrate.nbitrate);
    auto block = std::unique_ptr<BusControlBlock>(new BusControlBlock(name, model_type, bitrate));
    if (!block->init()) {
        LOG_E("Failed to initialize BusControlBlock for bus: %s, model: %d, bitrate: %d", 
              name.c_str(), static_cast<int>(model_type), bitrate.nbitrate);
        return nullptr;
    }
    LOG_I("BusControlBlock created successfully for bus: %s, model: %d, bitrate: %d, data_bitrate: %d", 
          name.c_str(), static_cast<int>(model_type), bitrate.nbitrate, bitrate.dbitrate);
    return block;
}

bool BusControlBlock::init() {
    LOG_D("Initializing BusControlBlock for bus: %s", bus_name_.c_str());
    try {
        // 使用工厂函数创建CAN总线实例
        canbus_ = CanBusBase::create(bus_name_, model_type_, bitrate_);
        if (!canbus_) {
            LOG_E("Failed to create CAN bus instance for bus: %s, model: %d",
                  bus_name_.c_str(), static_cast<int>(model_type_));
            return false;
        }
        LOG_D("Created CAN bus instance for bus: %s", bus_name_.c_str());

        // 初始化包装器
        Result<bool> init_result = canbus_->init();
        if (!init_result || !init_result.value()) {
            LOG_E("Failed to initialize CAN bus for bus: %s", bus_name_.c_str());
            canbus_.reset();
            return false;  // 初始化失败
        }
        LOG_D("CAN bus initialized successfully for bus: %s", bus_name_.c_str());
        
        // 设置消息回调，将this作为context传递
        callback_context_.userdata = static_cast<void*>(this);
        canbus_->setMessageReceiveCallback(internalMessageCallback, &callback_context_);
        LOG_D("Message callback set for bus: %s", bus_name_.c_str());

        // 设置TEF事件回调，使用独立的TEF context
        tef_callback_context_.userdata = static_cast<void*>(this);
        canbus_->setTefEventCallback(internalTefEventCallback, &tef_callback_context_);
        LOG_D("TEF event callback set for bus: %s", bus_name_.c_str());
        
        // 标记为活动
        is_active_ = true;
        
        // 启动接收线程
        startRecvThread();
        LOG_I("BusControlBlock initialized successfully for bus: %s", bus_name_.c_str());
        
        return true;
        
    } catch (const std::exception& e) {
        LOG_E("Exception during BusControlBlock initialization for bus %s: %s", bus_name_.c_str(), e.what());
        canbus_.reset();
        return false;  // 创建过程中发生异常
    }
}

BusControlBlock::~BusControlBlock() {
    LOG_D("Destroying BusControlBlock for bus: %s", bus_name_.c_str());
    
    // 清理所有注册的设备
    clearAllDevices();
    
    stopRecvThread();
    canbus_.reset();  // 智能指针自动释放内存
    LOG_D("BusControlBlock destroyed for bus: %s", bus_name_.c_str());
}

DeviceRegistration* BusControlBlock::findDevice(DeviceUniqueId unique_id) {
    if (unique_id.getType() == DeviceType::UNKNOWN) {
        return nullptr;
    }

    std::lock_guard<std::mutex> lock(devices_mutex_);
    auto it = devices_.find(unique_id);
    if (it != devices_.end() && it->second.is_registered) {
        return &it->second;
    }
    return nullptr;
}

DeviceRegistration* BusControlBlock::findDevice(DeviceType type, DeviceId device_id) {
    return findDevice(DeviceUniqueId(type, device_id));
}

bool BusControlBlock::hasDevice(DeviceUniqueId unique_id) const {
    if (unique_id.getType() == DeviceType::UNKNOWN) {
        return false;
    }
    std::lock_guard<std::mutex> lock(devices_mutex_);
    auto it = devices_.find(unique_id);
    return it != devices_.end() && it->second.is_registered;
}

bool BusControlBlock::hasDevice(DeviceType type, DeviceId device_id) const {
    return hasDevice(DeviceUniqueId(type, device_id));
}


bool BusControlBlock::addDevice(const DeviceInfo& device_info,
                                const CallbackParams& callback_params) {
    LOG_D("Adding device %s (ID: 0x%08X) to bus %s",
          device_info.device_name.c_str(), DeviceUniqueId(device_info.device_type, device_info.device_id).value, bus_name_.c_str());

    // 验证unique_id范围
    if (device_info.device_type == DeviceType::UNKNOWN) {
        LOG_W("Invalid device type UNKNOWN for bus %s", bus_name_.c_str());
        return false;
    }

    // 检查设备是否已注册
    if (hasDevice(device_info.device_type, device_info.device_id)) {
        LOG_W("Device %s (ID: 0x%08X) already registered on bus %s",
              device_info.device_name.c_str(), DeviceUniqueId(device_info.device_type, device_info.device_id).value, bus_name_.c_str());
        return false;
    }

    // 使用哈希表存储设备
    {
        std::lock_guard<std::mutex> lock(devices_mutex_);
        DeviceRegistration reg;
        reg.setRegistration(device_info, callback_params);
        devices_[DeviceUniqueId(device_info.device_type, device_info.device_id)] = reg;
        device_count_++;
    }

    LOG_I("Device %s (ID: 0x%08X) added successfully to bus %s. Total devices: %d",
          device_info.device_name.c_str(), DeviceUniqueId(device_info.device_type, device_info.device_id).value, bus_name_.c_str(), device_count_);
    return true;
}

bool BusControlBlock::removeDevice(DeviceUniqueId unique_id) {
    LOG_D("Removing device ID 0x%08X from bus %s", unique_id.value, bus_name_.c_str());

    std::string device_name;
    {
        std::lock_guard<std::mutex> lock(devices_mutex_);
        auto it = devices_.find(unique_id);
        if (it == devices_.end() || !it->second.is_registered) {
            LOG_W("Device ID 0x%08X not found on bus %s", unique_id.value, bus_name_.c_str());
            return false;
        }

        device_name = it->second.device_info.device_name;
        it->second.clearRegistration();
        devices_.erase(it);

        // 防止下溢
        if (device_count_ > 0) {
            device_count_--;
        } else {
            LOG_W("Device count underflow detected on bus %s, count was already 0", bus_name_.c_str());
        }
    }

    LOG_I("Device %s (ID: 0x%08X) removed from bus %s. Remaining devices: %d",
          device_name.c_str(), unique_id.value, bus_name_.c_str(), device_count_);
    return true;
}

bool BusControlBlock::removeDevice(DeviceType type, DeviceId device_id) {
    return removeDevice(DeviceUniqueId(type, device_id));
}


void BusControlBlock::clearAllDevices() {
    LOG_D("Clearing all devices from bus %s", bus_name_.c_str());
    
    std::lock_guard<std::mutex> lock(devices_mutex_);
    
    // 清理所有设备注册信息
    for (auto& pair : devices_) {
        pair.second.clearRegistration();
    }
    
    // 清空设备映射
    devices_.clear();
    device_count_ = 0;
    
    LOG_D("All devices cleared from bus %s", bus_name_.c_str());
}

Result<ErrorType> BusControlBlock::sendMessage(const CanMessageFrame& frame) {
    if (!is_active_ || !canbus_) {
        LOG_W("Cannot send message on inactive bus %s", bus_name_.c_str());
        return Result<ErrorType>::error(static_cast<ErrorType>(CanBusError::ERROR_BUS_NOT_FOUND));
    }
    
    return canbus_->writeCanMessage(frame);
}

void BusControlBlock::startRecvThread() {
    if (!recv_running_.exchange(true, std::memory_order_acquire)) {
        LOG_D("Starting receive thread for bus: %s", bus_name_.c_str());
        recv_thread_ = std::thread(&BusControlBlock::recvThreadFunction, this);

        // 如果之前设置了CPU核心绑定，则重新应用
        if (recv_thread_cpu_core_ >= 0) {
            setRecvThreadAffinity(recv_thread_cpu_core_);
        }
    }
}

void BusControlBlock::stopRecvThread() {
    if (recv_thread_.joinable()) {
        LOG_D("Stopping receive thread for bus: %s", bus_name_.c_str());
        try {
            recv_running_.store(false, std::memory_order_release);
            recv_thread_.join();
            LOG_D("Receive thread stopped for bus: %s", bus_name_.c_str());
        } catch (const std::exception& e) {
            LOG_E("Exception while stopping receive thread for bus %s: %s", 
                  bus_name_.c_str(), e.what());
            // 即使出现异常，也要确保线程不再运行
            recv_running_.store(false, std::memory_order_release);
            if (recv_thread_.joinable()) {
                recv_thread_.detach(); // 最后的手段，避免程序阻塞
            }
        }
    }
}

bool BusControlBlock::setRecvThreadAffinity(int cpu_core) {
    if (cpu_core < 0) {
        LOG_W("Invalid CPU core %d for bus %s", cpu_core, bus_name_.c_str());
        return false;
    }

    if (!recv_thread_.joinable()) {
        LOG_W("Receive thread not running for bus %s, cannot set affinity", bus_name_.c_str());
        return false;
    }

    // 获取线程native句柄
    pthread_t thread = recv_thread_.native_handle();

    // 设置CPU亲和性
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(cpu_core, &cpuset);

    int result = pthread_setaffinity_np(thread, sizeof(cpu_set_t), &cpuset);
    if (result == 0) {
        recv_thread_cpu_core_ = cpu_core;
        LOG_I("Set receive thread affinity for bus %s to CPU core %d", bus_name_.c_str(), cpu_core);
        return true;
    } else {
        LOG_E("Failed to set receive thread affinity for bus %s to CPU core %d: %s",
              bus_name_.c_str(), cpu_core, strerror(result));
        return false;
    }
}

bool BusControlBlock::setRecvThreadFrequency(int frequency_ms) {
    if (frequency_ms <= 0) {
        LOG_W("Invalid receive thread frequency %d ms for bus %s", frequency_ms, bus_name_.c_str());
        return false;
    }

    if (frequency_ms > 1000) {
        LOG_W("Receive thread frequency %d ms is too slow for bus %s, recommend 1-100 ms",
              frequency_ms, bus_name_.c_str());
    }

    recv_frequency_ms_ = frequency_ms;
    LOG_I("Set receive thread frequency for bus %s to %d ms (%.1f Hz)",
          bus_name_.c_str(), frequency_ms, 1000.0 / frequency_ms);
    return true;
}

int BusControlBlock::getRecvThreadFrequency() const {
    return recv_frequency_ms_;
}

namespace {
    /**
     * @brief 从CAN消息帧中提取设备ID
     * @param frame 指向CAN消息帧的指针
     * @return 从消息中提取的设备ID
     */
    DeviceId extractDeviceId(const CanMessageFrame* frame) {
        // 这是基础实现 - 您可能需要根据特定的CAN协议进行自定义
        // 常见模式：
        // 1. 设备ID在CAN ID的低位：frame->id.SID & 0x07FF (11位全部)
        // 2. 设备ID在payload的第一个字节：frame->payload[0]
        // 3. 设备ID编码在扩展ID中：frame->id.EID & 0x07FF
        
        // 现在假设设备ID直接使用11位标准ID (0-2047)
        return frame->id.SID & 0x07FF;  // 0x07FF = 2047，使用完整的11位标准ID
    }
}

void BusControlBlock::internalMessageCallback(CanMessageFrame* frame, const CallbackContext* context) {
    // 从context中获取bus_block指针
    BusControlBlock* bus_block = static_cast<BusControlBlock*>(context->userdata);
    if (bus_block) {
        uint32_t can_id = frame->id.SID;

        // 静默过滤 CAN FD 广播协议的 TX 帧 (SocketCAN 回环导致自发自收)
        // 0x10=Control, 0x20=MIT, 0x40=PosVel, 0x60=Vel
        if (can_id == 0x10 || can_id == 0x20 || can_id == 0x40 || can_id == 0x60) {
            freeCanMessageFrame(frame);
            return;
        }

        // 使用matcher遍历所有设备查找匹配的设备
        MessageCallback callback = nullptr;
        std::shared_ptr<CallbackContext> context_ptr;
        std::string device_name;
        DeviceId device_id = -1;
        bool device_found = false;

        {
            std::lock_guard<std::mutex> lock(bus_block->devices_mutex_);
            for (auto& pair : bus_block->devices_) {
                if (pair.second.isValid()) {
                    const DeviceInfo& device_info = pair.second.device_info;

                    // 使用设备的matcher检查是否匹配
                    bool matches = false;
                    if (device_info.matcher) {
                        matches = device_info.matcher(can_id, device_info.device_id);
                    } else {
                        // 默认匹配逻辑：CAN ID == device_id
                        matches = (can_id == device_info.device_id);
                    }

                    if (matches) {
                        // 提取必要信息，避免在回调中持有锁
                        callback = pair.second.callback;
                        context_ptr = pair.second.context.lock();  // 尝试从弱引用获取强引用
                        device_name = device_info.device_name;
                        device_id = pair.first.getDeviceId();
                        device_found = true;
                        break;  // 找到匹配的设备就退出循环
                    }
                }
            }
        }

        // 分发消息到对应的设备
        if (device_found && callback) {
            // printf("Dispatching message to device %s (ID: 0x%04X) on bus %s, CAN ID: 0x%04X\n",
            //       device_name.c_str(), device_id, bus_block->bus_name_.c_str(), can_id);
            // 使用提取的信息调用回调，避免在回调中持有锁
            // 注意：回调函数负责在适当的时候释放frame内存
            // 允许context_ptr为nullptr，回调函数应该能处理这种情况
            callback(frame, context_ptr ? context_ptr.get() : nullptr);
        } else {
            if (!device_found) {
                printf("No registered device matched CAN ID 0x%04X on bus %s\n",
                      can_id, bus_block->bus_name_.c_str());
            } else if (!callback) {
                printf("Invalid callback for device %s (ID: 0x%04X) on bus %s\n",
                      device_name.c_str(), device_id, bus_block->bus_name_.c_str());
            }
            // 没有设备处理，释放frame
            freeCanMessageFrame(frame);
        }
    } else {
        printf("Invalid bus block pointer in internalMessageCallback\n");
        // 未找到对应的总线，释放消息帧
        freeCanMessageFrame(frame);
    }
}

void BusControlBlock::internalTefEventCallback(CanMessageFrame* frame, const CallbackContext* context) {
    // 从context中获取bus_block指针
    BusControlBlock* bus_block = static_cast<BusControlBlock*>(context->userdata);
    if (bus_block) {
        uint32_t can_id = frame->id.SID;
        // printf("Received TEF event on bus %s with CAN ID: 0x%04X\n", bus_block->bus_name_.c_str(), can_id);

        // 使用matcher遍历所有设备查找匹配的设备
        TefEeventCallback tef_callback = nullptr;
        std::shared_ptr<CallbackContext> context_ptr;
        std::string device_name;
        DeviceId device_id = -1;
        bool device_found = false;

        {
            std::lock_guard<std::mutex> lock(bus_block->devices_mutex_);
            for (auto& pair : bus_block->devices_) {
                if (pair.second.is_registered && pair.second.tef_callback != nullptr) {
                    const DeviceInfo& device_info = pair.second.device_info;

                    // 使用设备的matcher检查是否匹配
                    bool matches = false;
                    if (device_info.matcher) {
                        matches = device_info.matcher(can_id, device_info.device_id);
                    } else {
                        // 默认匹配逻辑：CAN ID == device_id
                        matches = (can_id == device_info.device_id);
                    }

                    if (matches) {
                        // 提取必要信息，避免在回调中持有锁
                        tef_callback = pair.second.tef_callback;
                        context_ptr = pair.second.tef_context.lock();  // 尝试从弱引用获取强引用
                        device_name = device_info.device_name;
                        device_id = pair.first.getDeviceId();
                        device_found = true;
                        break;  // 找到匹配的设备就退出循环
                    }
                }
            }
        }

        // 分发消息到对应的设备
        if (device_found && tef_callback) {
            // printf("Dispatching TEF event to device %s (ID: 0x%04X) on bus %s, CAN ID: 0x%04X\n",
            //       device_name.c_str(), device_id, bus_block->bus_name_.c_str(), can_id);
            // 使用提取的信息调用回调，避免在回调中持有锁
            // 注意：回调函数负责在适当的时候释放frame内存
            // 允许context_ptr为nullptr，回调函数应该能处理这种情况
            tef_callback(frame, context_ptr ? context_ptr.get() : nullptr);
        } else {
            if (!device_found) {
                // printf("No registered device with TEF callback matched CAN ID 0x%04X on bus %s\n",
                //       can_id, bus_block->bus_name_.c_str());
            } else if (!tef_callback) {
                printf("Device %s (ID: %d) has no valid TEF callback on bus %s\n",
                      device_name.c_str(), device_id, bus_block->bus_name_.c_str());
            }
            // 没有设备处理，释放frame
            freeCanMessageFrame(frame);
        }
    } else {
        printf("Invalid bus block pointer in internalTefEventCallback\n");
        // 未找到对应的总线，释放消息帧
        freeCanMessageFrame(frame);
    }
}

void BusControlBlock::recvThreadFunction() {
    LOG_D("Receive thread started for bus: %s", bus_name_.c_str());

#if DEBUG_CAN_RECV_THREAD_STATS
    // 添加频率统计变量
    uint64_t cycle_count = 0;
    auto last_frequency_log_time = std::chrono::steady_clock::now();
    const uint64_t frequency_log_interval = 5000; // 可配置的日志打印间隔
#endif

    while (recv_running_.load(std::memory_order_acquire)) {
        // 记录开始时间
        auto cycle_start_time = std::chrono::steady_clock::now();

        if (canbus_) {
            // 处理接收到的消息，内部触发回调`internalMessageCallback`，回调函数会分发消息到对应的设备
            Result<ErrorType> result = canbus_->receivedCanMessages();
            if (!result) {
                LOG_E("Error receiving CAN messages on bus %s: %d",
                      bus_name_.c_str(), result.error());
                // TODO(chenmingfu): 后续需要添加错误处理
            }
            else {
                // TODO(chenmingfu): 考虑添加CANBUS统计信息，用于监控CANBUS的通信质量
            }
        }

        using Microseconds = std::chrono::microseconds;
        // 计算函数执行时间
        auto current_time = std::chrono::steady_clock::now();
        // 计算需要休眠的时间（最小为0），使用动态设置的频率
        Microseconds elapsed_time = std::chrono::duration_cast<Microseconds>(current_time - cycle_start_time);
                Microseconds sleep_time = std::max(Microseconds::zero(),
                                 Microseconds(recv_frequency_ms_ * 1000) - elapsed_time);
        std::this_thread::sleep_for(sleep_time);

        // 每frequency_log_interval次循环打印一次实际频率（由宏控制）
#if DEBUG_CAN_RECV_THREAD_STATS
        cycle_count++;
        if (cycle_count % frequency_log_interval == 0) {
            auto now = std::chrono::steady_clock::now();
            auto elapsed_since_last_log = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_frequency_log_time);
            double actual_frequency = static_cast<double>(frequency_log_interval) * 1000.0 / elapsed_since_last_log.count();

            LOG_I("Receive thread actual frequency for bus %s: %.2f Hz (target: %.2f Hz, cycles: %lu)",
                  bus_name_.c_str(), actual_frequency, 1000.0 / recv_frequency_ms_, cycle_count);

            last_frequency_log_time = now;
        }
#endif
    }
    LOG_D("Receive thread exiting for bus: %s", bus_name_.c_str());
}

} // namespace canbus_sdk