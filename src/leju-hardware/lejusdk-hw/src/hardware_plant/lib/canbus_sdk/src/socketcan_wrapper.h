#ifndef _SOCKETCAN_WRAPPER_H_
#define _SOCKETCAN_WRAPPER_H_

#include "canbus_sdk/canbus_sdk_def.h"
#include "canbus_sdk/result.h"
#include "canbus_base.h"

#include <string>
#include <memory>

namespace canbus_sdk {

class SocketCanWrapper;
using SocketCanWrapperPtr = std::shared_ptr<SocketCanWrapper>;

/**
 * @brief Linux SocketCAN 总线包装类
 *
 * 封装 Linux 原生 SocketCAN 接口，支持标准 CAN 和 CAN FD。
 * 适用于 RK3588 等使用内核 CAN 驱动的平台。
 * 接口名称格式：can0, can1, vcan0 等。
 */
class SocketCanWrapper final : public CanBusBase
{
public:
    /**
     * @brief 构造函数
     * @param canbus_name CAN 接口名称 (例如: "can0", "can1")
     * @param model_type CAN 总线型号类型 (SOCKETCAN)
     * @param bitrate CAN 总线比特率配置
     */
    SocketCanWrapper(const std::string& canbus_name,
                     CanBusModelType model_type,
                     const CanBusBitrate& bitrate);

    virtual ~SocketCanWrapper();

    /**
     * @brief 初始化 SocketCAN 接口
     *
     * 执行初始化序列：
     * 1. 创建 CAN 原始套接字
     * 2. 启用 CAN FD（如果接口支持）
     * 3. 设置接收过滤器（接收所有帧）
     * 4. 绑定到指定 CAN 接口
     *
     * @return Result<bool> 成功返回 true，失败返回错误码
     * @note CAN 接口必须已通过 ip link 配置好（up 状态且波特率已设置）
     */
    Result<bool> init() override final;

    /**
     * @brief 向 CAN 总线写入消息
     * @param frame 要发送的 CAN 消息帧
     * @return Result<ErrorType> 成功返回成功码，失败返回错误码
     */
    Result<ErrorType> writeCanMessage(const CanMessageFrame& frame) override final;

    /**
     * @brief 设置消息接收回调函数
     */
    void setMessageReceiveCallback(MessageCallback callback,
                                   const CallbackContext* context = nullptr) override final;

    /**
     * @brief 设置 TEF 事件回调函数
     */
    void setTefEventCallback(TefEeventCallback callback,
                            const CallbackContext* context = nullptr) override final;

    /**
     * @brief 处理接收到的 CAN 总线消息
     *
     * 使用非阻塞 recv 从 socket 读取所有待处理帧，
     * 通过回调函数分发。
     *
     * @return Result<ErrorType> 成功返回处理的消息数量，失败返回错误码
     */
    Result<ErrorType> receivedCanMessages() override final;

private:
    struct Inner;
    std::unique_ptr<Inner> inner_;
};

} // namespace canbus_sdk

#endif // _SOCKETCAN_WRAPPER_H_
