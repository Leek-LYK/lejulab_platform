#ifndef _BM_CANBUS_WRAPPER_H_
#define _BM_CANBUS_WRAPPER_H_

#include "canbus_sdk/canbus_sdk_def.h"
#include "canbus_sdk/result.h"
#include "canbus_base.h"

#include <string>
#include <memory>
#include <atomic>
#include <functional>
#include <vector>

namespace canbus_sdk {

class BMCanbusWrapper;
using BMCanbusWrapperPtr = std::shared_ptr<BMCanbusWrapper>;

/**
 * @brief BUSMUST CAN总线包装类
 *
 * 该类封装了BUSMUST CAN总线设备的操作，提供初始化、发送消息、
 * 接收消息等功能。使用异步发送模式，支持回调机制处理接收到的消息。
 */
class BMCanbusWrapper final : public CanBusBase
{
public:
    /**
     * @brief 构造函数
     * @param canbus_name CAN总线设备名称 "BM-CANFD-X1(6729) CH1", "BM-CANFD-X1(6108) CH1"
     * @param model_type CAN总线型号类型
     * @param bitrate CAN总线比特率配置
     */
    BMCanbusWrapper(const std::string& canbus_name, CanBusModelType model_type, const CanBusBitrate& bitrate);
    virtual ~BMCanbusWrapper();

    /**
     * @brief 初始化CAN总线包装器
     *
     * 此函数执行完整的初始化序列：
     * 1. 初始化BUSMUST库
     * 2. 枚举可用的CAN总线通道
     * 3. 根据型号类型找到目标通道
     * 4. 以指定比特率打开通道
     * 5. 获取消息接收的通知句柄
     *
     * @return Result<bool> 成功返回true，失败返回错误码
     */
    Result<bool> init() override final;

    /**
     * @brief 向CAN总线写入消息（异步）
     *
     * 此函数异步发送CAN消息。函数在将消息排队后立即返回，
     * 不等待传输完成。使用receivedCanMessages()来处理接收到的消息。
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
    void setMessageReceiveCallback(MessageCallback callback, const CallbackContext* context = nullptr) override final;

    /**
     * @brief 设置TEF事件回调函数
     *
     * @param callback 接收到TEF事件时调用的回调函数
     * @param context 传递给回调的可选用户上下文（可为nullptr）
     */
    void setTefEventCallback(TefEeventCallback callback, const CallbackContext* context = nullptr) override final;

    /**
     * @brief 处理接收到的CAN总线消息
     *
     * 此函数在接收到消息时调用已注册的回调函数
     * @return Result<ErrorType> 成功返回处理的消息数量，失败返回错误码
     */
    Result<ErrorType> receivedCanMessages() override final;

private:
    /**
     * @brief 从错误中恢复CAN总线连接
     *
     * 此函数处理各种CAN总线错误状态，包括：
     * - 设备未初始化错误
     * - 无效通道句柄错误
     * - USB设备操作失败错误
     * - BUSOFF和BUSPASSIVE状态
     *
     * 恢复策略包括重新打开通道、重置设备、清除缓冲区等操作。
     * 注意：恢复过程需要一定时间，确保在此期间没有其他线程使用通道句柄。
     */
    void recoverFromError();

    struct ChannelRecoveryConfigs;  // 前向声明 - 恢复配置细节隐藏
    struct Inner;                   // 前向声明 - 实现细节隐藏
    std::unique_ptr<Inner> inner_;  ///< 实现细节（pimpl模式）
};

} // namespace canbus_sdk

#endif // _BM_CANBUS_WRAPPER_H_