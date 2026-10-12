# CAN总线SDK

C++ CAN总线通信SDK，提供统一的CAN总线管理，支持注册电机和Revo2灵巧手等设备。

```
canbus_sdk/
├── src/                    # 实现文件
│   ├── canbus_sdk.cpp      # 主要SDK逻辑
│   ├── canbus_ctrl.cpp     # 控制器实现
│   ├── bus_control_block.cpp  # 总线管理
│   ├── bm_canbus_wrapper.cpp  # 硬件抽象
│   └── config_parser.cpp   # 配置解析
├── include/canbus_sdk/     # 公共头文件
├── examples/               # 使用示例
├── tests/                  # 测试套件
└── *.yaml                 # 配置文件
```

## 功能特性

- **多设备支持**：电机、Revo1/Revo2灵巧手、LejuClaw夹爪
- **硬件兼容性**：支持BUSMUST A/B CAN适配器
- **现代C++设计**：线程安全、RAII、移动语义
- **高性能**：无锁队列、异步消息处理
- **健壮的错误处理**：`Result<T>`模板类实现全面错误管理
- **配置驱动**：基于YAML的设备和总线配置
- **实时控制**：CPU亲和性调优、线程频率控制

## 快速开始

### 1. 安装依赖

```bash
sudo apt-get install libyaml-cpp-dev pkg-config libusb-1.0-0-dev libgtest-dev
```

### 2. 构建SDK

```bash
cd canbus_sdk
mkdir build && cd build
cmake ..
make
```

### 3. 基本使用

```cpp
#include "canbus_sdk/canbus_sdk.h"

using namespace canbus_sdk;

// 初始化CAN总线控制器
CanBusController& controller = CanBusController::getInstance();
controller.init();

// 配置CAN总线比特率
CanBusBitrate bitrate = {1000, 2000, 75, 75}; // 1M/2M bps, 75%采样点

// 打开CAN总线连接
auto result = controller.openCanBus("can0", CanBusModelType::BUSMUST_A, bitrate);
if (!result) {
    printf("打开CAN总线失败: %s\n", errorToString(result.error()));
    return -1;
}

BusId bus_id = result.value();

// 定义设备信息和回调函数
DeviceInfo device_info(
    "my_motor",
    DeviceType::MOTOR,
    0x01,
    [](uint32_t can_id, uint32_t device_id) {
        return (can_id & 0xFF) == device_id;
    }
);

// 注册设备并设置消息回调
CallbackParams callbacks;
callbacks.msg_callback = [](CanMessageFrame* frame, const CallbackContext* context) {
    printf("接收到消息: ID=0x%03X, DLC=%d\n", frame->id.SID, frame->ctrl.rx.dlc);
    freeCanMessageFrame(frame);
};

auto register_result = controller.registerDevice(device_info, "can0", callbacks);
if (!register_result) {
    printf("注册设备失败: %s\n", errorToString(register_result.error()));
}

// 发送消息
CanMessageFrame frame;
frame.id.SID = 0x100;
frame.ctrl.tx.dlc = 8;
memset(frame.payload, 0xAA, 8);

controller.sendMessage(bus_id, frame);

// 清理资源
controller.closeAllCanBuses();
```

## API参考

### CanBusController

实现单例模式的主要接口类。

#### 主要方法

```cpp
// 单例访问
static CanBusController& getInstance();

// 总线管理
Result<BusId> openCanBus(const std::string& bus_name, CanBusModelType type, const CanBusBitrate& bitrate);
Result<ErrorType> closeCanBus(const std::string& bus_name);
Result<ErrorType> closeAllCanBuses();

// 设备管理
Result<ErrorType> registerDevice(const DeviceInfo& info, const std::string& bus_name, const CallbackParams& callbacks);
Result<ErrorType> unregisterDevice(DeviceType type, DeviceId id, const std::string& bus_name);

// 消息操作
Result<ErrorType> sendMessage(const std::string& bus_name, const CanMessageFrame& frame);
Result<ErrorType> sendMessage(BusId bus_id, const CanMessageFrame& frame);

// 性能调优
bool setSenderThreadAffinity(int cpu_core);
bool setRecvThreadAffinity(const std::string& bus_name, int cpu_core);
bool setRecvThreadFrequency(const std::string& bus_name, int frequency_ms);
```

### 错误处理

SDK使用`Result<T>`模板类实现健壮的错误处理：

```cpp
Result<BusId> result = controller.openCanBus("can0", CanBusModelType::BUSMUST_A, bitrate);

if (result) {
    BusId bus_id = result.value();
    // 成功 - 使用bus_id
} else {
    ErrorType error = result.error();
    const char* error_msg = errorToString(error);
    // 处理错误
}
```

### 设备唯一ID

`DeviceUniqueId`联合体使用位域防止ID冲突：

```cpp
DeviceUniqueId id(DeviceType::REVO2_HAND, 0x01);
printf("设备类型: %d, ID: %d\n", id.getType(), id.getDeviceId());
```

## 示例

### 运行示例程序

在`examples/revo2_hand/`目录中提供了可运行的示例程序：

```bash
cd build
cmake .. && make

# 运行单手控制示例
sudo ./examples/revo2_hand/revo2_can_customed

# 运行双手协调示例
sudo ./examples/revo2_hand/dual_revo2_can_customed
```

## 测试

`tests/`目录中的综合测试套件：

```bash
cd build
cmake .. && make

# 运行基础功能测试
./tests/bm_canbus_wrapper_simple_test
./tests/config_parser_test

# 运行完整功能测试
./tests/bm_canbus_wrapper_test
./tests/canbus_control_block_test
./tests/device_unique_id_test
./tests/register_device_test

# 运行性能和压力测试
./tests/race_condition_test
./tests/extreme_pressure_test
./tests/mpsc_ring_buffer_test
```

