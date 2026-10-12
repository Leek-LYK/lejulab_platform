#ifndef REVO2_HAND_H
#define REVO2_HAND_H

#include <mutex>
#include "dexhand_base.h"
#include "modbus.h"

namespace dexhand {

/**
 * @brief 基于Modbus 通信的  [Revo2灵巧手]
 *
 */
class Revo2Dexhand: public ModbusDexhand {
public:
    ~Revo2Dexhand();

    /**
     * @brief 获取灵巧手固件类型  => 固定返回 DexHandFwType::V2_STANDARD
     * @return 固件类型
     */
    DexHandFwType getDexHandFwType() override;
    void setGripForce(GripForce level) override final;
    GripForce getGripForce() override final;

    bool usesRevo2MotorApi() const override { return true; }

    /**
     * @brief 连接并创建Revo2灵巧手设备实例
     *
     * 通过串口连接到Revo2灵巧手设备，建立通信链路并创建设备实例。
     * 支持多种波特率选择，适用于不同的串口通信需求。
     *
     * @param port 串口设备路径，例如："/dev/ttyUSB0"
     * @param slave_id 设备ID，范围为1~255，0为广播地址
     * @param baudrate 串口波特率，支持：115200, 57600, 19200, 460800
     * @return 成功返回设备实例指针，失败返回nullptr
     */
    static std::unique_ptr<Revo2Dexhand> Connect(
        const std::string& port,
        uint8_t slave_id,
        uint32_t baudrate
    );

private:
    /**
     * @brief 私有构造函数，用于Connect方法创建实例
     * @param handle 设备句柄
     * @param slave_id_ 从机地址
     */
    explicit Revo2Dexhand(DeviceHandler* handle, uint8_t slave_id_);
};

}

#endif // REVO2_HAND_H
