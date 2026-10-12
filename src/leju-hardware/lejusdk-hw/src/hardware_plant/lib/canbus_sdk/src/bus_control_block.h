#ifndef CANBUS_CONTROL_BLOCK_H
#define CANBUS_CONTROL_BLOCK_H

#include "canbus_sdk/canbus_sdk_def.h"
#include "canbus_sdk/result.h"
#include "canbus_base.h"

#include <mutex>
#include <thread>
#include <atomic>
#include <functional>
#include <chrono>
#include <memory>
#include <unordered_map>
#include <pthread.h>

namespace canbus_sdk {

/**
 * @brief 设备注册信息
 */
struct DeviceRegistration {
    DeviceInfo device_info;                  ///< 设备信息
    MessageCallback callback;                ///< 消息回调函数
    TefEeventCallback tef_callback;          ///< TEF事件回调函数
    std::weak_ptr<CallbackContext> context;  ///< 回调上下文弱引用指针
    std::shared_ptr<CallbackContext> strong_context; ///< 强引用用于内部使用
    std::weak_ptr<CallbackContext> tef_context;  ///< TEF回调上下文弱引用指针
    std::shared_ptr<CallbackContext> strong_tef_context; ///< TEF强引用用于内部使用
    bool is_registered;                      ///< 设备是否已注册

    DeviceRegistration() : callback(nullptr), tef_callback(nullptr), is_registered(false) {}

    /**
     * @brief 设置设备注册信息
     * @param info 设备信息
     * @param callback_params 回调参数结构，包含消息回调和TEF回调
     */
    void setRegistration(const DeviceInfo& info, const CallbackParams& callback_params) {
        device_info = info;
        callback = callback_params.msg_callback;
        tef_callback = callback_params.tef_callback;

        // 设置消息回调上下文
        if (callback_params.msg_cb_ctx) {
            strong_context = std::make_shared<CallbackContext>(*callback_params.msg_cb_ctx);
            context = strong_context;  // 设置弱引用
        } else {
            strong_context.reset();
            context.reset();
        }

        // 设置TEF回调上下文
        if (callback_params.tef_cb_ctx) {
            strong_tef_context = std::make_shared<CallbackContext>(*callback_params.tef_cb_ctx);
            tef_context = strong_tef_context;  // 设置弱引用
        } else {
            strong_tef_context.reset();
            tef_context.reset();
        }

        is_registered = true;
    }
    
    /**
     * @brief 清除设备注册信息
     */
    void clearRegistration() {
        device_info = DeviceInfo();
        callback = nullptr;
        tef_callback = nullptr;
        strong_context.reset();
        context.reset();
        strong_tef_context.reset();
        tef_context.reset();
        is_registered = false;
    }
    
    /**
     * @brief 检查设备是否有效（已注册且有消息回调）
     * @return 如果设备有效返回true，否则返回false
     */
    bool isValid() const {
        return is_registered && callback != nullptr;
    }
};

/**
 * @brief CAN总线控制块 - 存储已打开CAN总线连接的信息
 * 类似于PCB/TCB模式，用于管理总线状态和资源
 */
class BusControlBlock {
public:
    /**
     * @brief 创建并初始化总线控制块
     * @param name CAN总线接口名称
     * @param model_type CAN总线硬件型号
     * @param bitrate CAN总线比特率配置
     * @return 成功返回unique_ptr，失败返回nullptr
     */
    static std::unique_ptr<BusControlBlock> create(const std::string& name, CanBusModelType model_type, const CanBusBitrate& bitrate);
    
    ~BusControlBlock();
    
    // 获取总线信息
    const std::string& busName() const { return bus_name_; }
    CanBusModelType modelType() const { return model_type_; }
    const CanBusBitrate& bitrate() const { return bitrate_; }
    bool active() const { return is_active_; }
    int deviceCount() const { return device_count_; }
    
    /**
     * @brief 检查指定设备唯一ID是否已注册
     * @param unique_id 要检查的设备唯一ID
     * @return 如果设备已注册返回true，否则返回false
     */
    bool hasDevice(DeviceUniqueId unique_id) const;

    /**
     * @brief 检查指定设备类型和ID是否已注册
     * @param type 设备类型
     * @param device_id 要检查的设备ID
     * @return 如果设备已注册返回true，否则返回false
     */
    bool hasDevice(DeviceType type, DeviceId device_id) const;

      
    /**
     * @brief 向此总线添加设备
     * @param device_info 设备信息
     * @param callback_params 回调参数结构，包含消息回调和TEF回调
     * @return 添加成功返回true，否则返回false
     */
    bool addDevice(const DeviceInfo& device_info, const CallbackParams& callback_params);
    
    /**
     * @brief 从此总线移除设备
     * @param unique_id 要移除的设备唯一ID
     * @return 移除成功返回true，否则返回false
     */
    bool removeDevice(DeviceUniqueId unique_id);

    /**
     * @brief 从此总线移除设备
     * @param type 设备类型
     * @param device_id 要移除的设备ID
     * @return 移除成功返回true，否则返回false
     */
    bool removeDevice(DeviceType type, DeviceId device_id);

        
    /**
     * @brief 向CAN总线发送消息
     * @param frame 要发送的CAN消息帧
     * @return 发送结果，成功返回ErrorType=0，失败返回错误码
     */
    Result<ErrorType> sendMessage(const CanMessageFrame& frame);

    /**
     * @brief 设置接收线程CPU亲和性
     * @param cpu_core CPU核心编号
     * @return 设置是否成功
     */
    bool setRecvThreadAffinity(int cpu_core);

    /**
     * @brief 设置接收线程频率
     * @param frequency_ms 接收线程频率（毫秒）
     * @return 设置是否成功
     */
    bool setRecvThreadFrequency(int frequency_ms);

    /**
     * @brief 获取当前接收线程频率
     * @return 当前接收线程频率（毫秒）
     */
    int getRecvThreadFrequency() const;

private:
    // 私有构造函数，只能通过create()方法创建
    BusControlBlock(const std::string& name, CanBusModelType model_type, const CanBusBitrate& bitrate);
    
    /**
     * @brief 初始化总线控制块
     * @return 初始化成功返回true，否则返回false
     */
    bool init();
    
    /**
     * @brief 根据设备唯一ID查找设备注册信息 (O(1)查找)
     * @param unique_id 要查找的设备唯一ID
     * @return 指向设备注册信息的指针，如果未找到则返回nullptr
     */
    DeviceRegistration* findDevice(DeviceUniqueId unique_id);

    /**
     * @brief 根据设备类型和ID查找设备注册信息 (O(1)查找)
     * @param type 设备类型
     * @param device_id 要查找的设备ID
     * @return 指向设备注册信息的指针，如果未找到则返回nullptr
     */
    DeviceRegistration* findDevice(DeviceType type, DeviceId device_id);
    
    /**
     * @brief 启动此总线的接收线程
     */
    void startRecvThread();
    
    /**
     * @brief 停止此总线的接收线程
     */
    void stopRecvThread();

    /**
     * @brief 清理所有注册的设备
     */
    void clearAllDevices();

    /**
     * @brief 接收线程函数 - 以固定频率读取消息
     */
    void recvThreadFunction();
    
    // 内部接收消息回调函数，用于将消息分发给设备
    static void internalMessageCallback(CanMessageFrame* frame, const CallbackContext* context);

    // 内部TEF事件回调函数，用于将TEF事件分发给设备
    static void internalTefEventCallback(CanMessageFrame* frame, const CallbackContext* context);
    
    // 总线基本信息
    std::string bus_name_;                    ///< CAN总线接口名称
    CanBusModelType model_type_;              ///< CAN总线硬件型号
    CanBusBitrate bitrate_;                   ///< CAN总线比特率配置
    std::unique_ptr<CanBusBase> canbus_;      ///< CanBusBase实例的智能指针
    bool is_active_;                          ///< 此总线连接是否活动
    int device_count_;                        ///< 此总线上注册的设备数量
    
    // 接收线程管理
    std::thread recv_thread_;                ///< 此总线的接收线程
    std::atomic<bool> recv_running_{false};  ///< 接收线程是否正在运行
    int recv_thread_cpu_core_;               ///< 接收线程绑定的CPU核心，-1表示未绑定
    int recv_frequency_ms_;                  ///< 接收线程频率（毫秒）
    static const int DEFAULT_RECV_FREQUENCY_MS = 2;   ///< 默认接收线程频率（毫秒）
    
    // 设备注册存储 - 使用设备唯一ID避免ID冲突
    std::unordered_map<DeviceUniqueId, DeviceRegistration> devices_;  ///< 设备唯一ID到注册信息的映射
    mutable std::mutex devices_mutex_;       ///< 设备映射访问互斥锁
    
    // 回调相关
    CallbackContext callback_context_;       ///< 消息回调上下文（需要保证生命周期）
    MessageCallback callback_;               ///< 消息回调函数
    CallbackContext tef_callback_context_;   ///< TEF事件回调上下文（需要保证生命周期）
};

} // namespace canbus_sdk

#endif // CANBUS_CONTROL_BLOCK_H