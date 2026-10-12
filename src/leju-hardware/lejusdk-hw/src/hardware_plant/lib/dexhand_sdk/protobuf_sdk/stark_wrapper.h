#ifndef STARK_WRAPPER_H
#define STARK_WRAPPER_H

#include <stdbool.h>
#include <stdint.h>
#include <dlfcn.h>
#include <sys/stat.h>
#include <unistd.h>
#include "dist/include/stark_sdk.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 包含 protobuf_dexhand.cpp 中实际使用的 stark_ 接口函数指针的结构体
 * 用于统一管理和调用 stark SDK 的必要接口函数
 */
typedef struct {
    // SDK 初始化和配置接口
    const char* (*get_sdk_version)(void);
    void (*set_log_level)(StarkLogLevel logLevel);
    void (*set_log_callback)(StarkLogCB cb);
    void (*set_write_data_callback)(WriteDataCB cb);

    // 设备管理接口
    StarkDevice* (*create_serial_device)(const char *uuid, const int device_id);

    // 设备配置接口
    int (*set_error_callback)(StarkDevice *device, StarkErrorCB cb);

    // 数据通信接口
    int (*did_receive_data)(StarkDevice *device, const uint8_t *data, int size);

    // 设备信息获取接口
    void (*get_motorboard_info)(StarkDevice *device, MotorboardInfoCB cb);

    // 手指状态和设置接口
    void (*get_finger_status)(StarkDevice *device, FingerStatusCB cb);
    void (*set_finger_positions)(StarkDevice *device, const int finger_positions[6]);
    void (*set_finger_speeds)(StarkDevice *device, const int finger_speeds[6]);

    // 握力等级接口
    void (*get_force_level)(StarkDevice *device, StarkIntValueCB cb);
    void (*set_force_level)(StarkDevice *device, int force_level);

    // 涡轮模式接口
    void (*get_turbo_mode)(StarkDevice *device, StarkIntValueCB cb);
    void (*set_turbo_mode)(StarkDevice *device, int turbo_mode);
    void (*get_turbo_conf)(StarkDevice *device, StarkValue2CB cb);

    // 动作序列接口
    void (*run_action_sequence)(StarkDevice *device, int action_id);
    void (*transfer_action_sequence)(StarkDevice *device, int action_id, int action_num, uint16_t **action_sequence);

} StarkWrapper;

/**
 * @brief 初始化 StarkWrapper 实例，动态加载so库并获取所有函数指针
 * @param so_path stark SDK的so文件路径
 * @return StarkWrapper* 指向初始化完成的 StarkWrapper 结构体的指针，失败返回NULL
 */
StarkWrapper* stark_wrapper_init(const char* so_path);

/**
 * @brief 清理 StarkWrapper 实例，释放资源
 * @param wrapper 要清理的 StarkWrapper 实例
 */
void stark_wrapper_cleanup(StarkWrapper* wrapper);

#ifdef __cplusplus
}
#endif

#endif // STARK_WRAPPER_H