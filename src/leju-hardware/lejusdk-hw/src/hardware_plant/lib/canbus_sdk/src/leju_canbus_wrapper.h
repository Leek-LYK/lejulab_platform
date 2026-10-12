#ifndef _LEJU_CANBUS_WRAPPER_H_
#define _LEJU_CANBUS_WRAPPER_H_

#include "canbus_sdk/canbus_sdk_def.h"
#include "canbus_sdk/result.h"
#include "canbus_base.h"

#include <string>
#include <memory>

namespace canbus_sdk {

class LejuCanbusWrapper;
using LejuCanbusWrapperPtr = std::shared_ptr<LejuCanbusWrapper>;

/**
 * @brief LEJU CAN总线包装类
 *
 * 该类封装了LEJU CANable设备的操作，提供初始化、发送消息、
 * 接收消息等功能。支持CAN FD协议。
 */
class LejuCanbusWrapper final : public CanBusBase
{
public:
    /**
     * @brief 构造函数
     * @param canbus_name CAN总线设备名称 (例如: "can0", "can1")
     * @param model_type CAN总线型号类型
     * @param bitrate CAN总线比特率配置
     */
    LejuCanbusWrapper(const std::string& canbus_name, 
                      CanBusModelType model_type, 
                      const CanBusBitrate& bitrate);
    
    virtual ~LejuCanbusWrapper();

    /**
     * @brief 初始化CAN总线包装器
     *
     * 执行初始化序列：
     * 1. 打开串口设备
     * 2. 配置波特率（标准段和数据段）
     * 3. 打开CAN通道
     *
     * @return Result<bool> 成功返回true，失败返回错误码
     */
    Result<bool> init() override final;

    /**
     * @brief 向CAN总线写入消息
     *
     * @param frame 要发送的CAN消息帧
     * @return Result<ErrorType> 成功返回成功码，失败返回错误码
     */
    Result<ErrorType> writeCanMessage(const CanMessageFrame& frame) override final;

    /**
     * @brief 设置消息接收回调函数
     *
     * @param callback 接收到消息时调用的回调函数
     * @param context 传递给回调的可选用户上下文（可为nullptr）
     */
    void setMessageReceiveCallback(MessageCallback callback, 
                                   const CallbackContext* context = nullptr) override final;

    /**
     * @brief 设置TEF事件回调函数
     *
     * @param callback 接收到TEF事件时调用的回调函数
     * @param context 传递给回调的可选用户上下文（可为nullptr）
     */
    void setTefEventCallback(TefEeventCallback callback, 
                            const CallbackContext* context = nullptr) override final;

    /**
     * @brief 处理接收到的CAN总线消息
     *
     * 此函数轮询接收消息并调用已注册的回调函数
     * @return Result<ErrorType> 成功返回处理的消息数量，失败返回错误码
     */
    Result<ErrorType> receivedCanMessages() override final;

private:
    struct Inner;  // 前向声明 - 实现细节隐藏（Pimpl模式）
    std::unique_ptr<Inner> inner_;  ///< 实现细节
};

} // namespace canbus_sdk

#endif // _LEJU_CANBUS_WRAPPER_H_

