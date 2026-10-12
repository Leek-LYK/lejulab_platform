#ifndef _CANBUS_BASE_H_
#define _CANBUS_BASE_H_

#include "canbus_sdk/canbus_sdk_def.h"
#include "canbus_sdk/result.h"

#include <string>
#include <memory>

namespace canbus_sdk {

/**
 * @brief CAN总线抽象基类
 *
 * 该类定义了CAN总线设备的通用接口，所有具体的CAN总线实现类都需要继承此基类。
 * 提供了初始化、发送消息、接收消息、回调设置等核心功能的抽象接口。
 */
class CanBusBase
{
public:
    /**
     * @brief 创建CAN总线实例的工厂函数
     * @param canbus_name CAN总线设备名称
     * @param model_type CAN总线型号类型
     * @param bitrate CAN总线比特率配置
     * @return 成功返回CanBusBase unique_ptr，失败返回nullptr
     */
    static std::unique_ptr<CanBusBase> create(const std::string& canbus_name, CanBusModelType model_type, const CanBusBitrate& bitrate);

    /**
     * @brief 构造函数
     * @param canbus_name CAN总线设备名称
     * @param model_type CAN总线型号类型
     * @param bitrate CAN总线比特率配置
     */
    CanBusBase(const std::string& canbus_name, CanBusModelType model_type, const CanBusBitrate& bitrate)
        : model_type_(model_type), bitrate_(bitrate), canbus_name_(canbus_name),
          callback_(nullptr), context_(nullptr), tef_callback_(nullptr), tef_context_(nullptr)
    {}

    /**
     * @brief 虚析构函数
     */
    virtual ~CanBusBase() = default;

    /**
     * @brief 初始化CAN总线
     *
     * 执行CAN总线的完整初始化序列，具体实现由子类完成。
     *
     * @return Result<bool> 成功返回true，失败返回错误码
     */
    virtual Result<bool> init() = 0;

    /**
     * @brief 向CAN总线写入消息（异步）
     *
     * 异步发送CAN消息，函数在将消息排队后立即返回，
     * 不等待传输完成。
     *
     * @param frame 要发送的CAN消息帧
     * @return Result<ErrorType> 成功返回成功码，失败返回错误码
     */
    virtual Result<ErrorType> writeCanMessage(const CanMessageFrame& frame) = 0;

    /**
     * @brief 设置消息接收回调函数
     *
     * @param callback 接收到消息时调用的回调函数
     * @param context 传递给回调的可选用户上下文（可为nullptr）
     */
    virtual void setMessageReceiveCallback(MessageCallback callback, const CallbackContext* context = nullptr) = 0;

    /**
     * @brief 设置TEF事件回调函数
     *
     * @param callback 接收到TEF事件时调用的回调函数
     * @param context 传递给回调的可选用户上下文（可为nullptr）
     */
    virtual void setTefEventCallback(TefEeventCallback callback, const CallbackContext* context = nullptr) = 0;

    /**
     * @brief 处理接收到的CAN总线消息
     *
     * 此函数在接收到消息时调用已注册的回调函数
     * @return Result<ErrorType> 成功返回处理的消息数量，失败返回错误码
     */
    virtual Result<ErrorType> receivedCanMessages() = 0;

    /**
     * @brief 获取CAN总线名称
     * @return const std::string& CAN总线设备名称/标识符
     */
    const std::string& getCanbusName() const;

    /**
     * @brief 获取CAN总线型号类型
     * @return CanBusModelType CAN总线型号类型
     */
    CanBusModelType getModelType() const;

protected:
    CanBusModelType model_type_;     ///< CAN总线型号类型 (BUSMUST_*)
    CanBusBitrate bitrate_;          ///< CAN总线比特率配置
    std::string canbus_name_;        ///< CAN总线设备名称/标识符
    MessageCallback callback_;        ///< 接收消息的回调函数
    const CallbackContext* context_; ///< 传递给回调函数的用户上下文
    TefEeventCallback tef_callback_;  ///< TEF事件回调函数
    const CallbackContext* tef_context_; ///< 传递给TEF回调的用户上下文
};

} // namespace canbus_sdk

#endif // _CANBUS_BASE_H_