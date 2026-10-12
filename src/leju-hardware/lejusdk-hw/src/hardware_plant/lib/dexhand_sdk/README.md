# 灵巧手SDK

对强脑灵巧手SDK的统一封装，提供统一的C++接口，支持Modbus、Protobuf、CAN总线通信。

```
dexhand_sdk/
├── src/                        # 实现文件
│   ├── revo2_hand.cpp          # Revo2灵巧手实现
│   ├── revo2_hand_can_customed.cpp  # CAN自定义Revo2实现
│   ├── modbus.cpp              # Modbus通信实现
│   ├── protobuf_dexhand.cpp     # Protobuf通信实现
│   └── touch_dexhand.cpp       # 触觉灵巧手实现
├── include/                    # 公共头文件
│   ├── revo2_hand.h           # Revo2灵巧手接口
│   ├── revo2_hand_can_customed.h  # CAN自定义接口
│   ├── modbus.h               # Modbus通信接口
│   ├── protobuf_dexhand.h     # Protobuf通信接口
│   ├── touch_dexhand.h       # 触觉灵巧手接口
│   └── dexhand_base.h        # 基础类定义
├── modbus_sdk/                # Modbus SDK库
├── protobuf_sdk/              # Protobuf SDK库
├── tests/                     # 测试程序
└── scripts/                   # 脚本文件
```

## 设备类型

#### 1. ProtobufDexhand (RS485_PROTOBUF)
- **类名**: ProtobufDexhand
- **中文名称**: Kuavo 一代灵巧手
- **头文件**: `protobuf_dexhand.h`
- **通信协议**: RS485 + Protobuf

#### 2. TouchDexhand (V1_TOUCH)
- **类名**: TouchDexhand
- **中文名称**: Kuavo 一代触觉灵巧手
- **头文件**: `touch_dexhand.h`
- **通信协议**: Modbus RTU

#### 3. Revo2Dexhand (V2_STANDARD)
- **类名**: Revo2Dexhand
- **中文名称**: Roban Revo2二代灵巧手
- **头文件**: `revo2_hand.h`
- **通信协议**: Modbus RTU

#### 4. Revo2CanDexhand (V2_STANDARD)
- **类名**: Revo2CanDexhand
- **中文名称**: Roban Revo2二代灵巧手 --> 基于 CAN 通讯
- **头文件**: `revo2_hand_can_customed.h`
- **通信协议**: CAN 

## 编译

由于 dexhand_sdk 依赖 canbus_sdk，需要先编译 canbus_sdk：

```bash
# 编译canbus_sdk
cd ../canbus_sdk
mkdir build && cd build
cmake ..
make

# 返回dexhand_sdk目录并编译
cd ../../dexhand_sdk
mkdir build && cd build
cmake ..
make
```

## 示例和测试

### 运行示例和测试程序

在项目根目录中提供了可运行的示例程序：

```bash
cd build

# 运行CAN通信测试
sudo ./revo2_can_test

# 运行Protobuf通信测试
./protobuf_dexhand_test


# 运行基础测试程序
./dexhand_test 

# 以下是示例输出:
# Usage: ./dexhand_test --touch|--normal|--revo2 [--test [round]] [--scan]
# Options:
#   --touch: 触觉手测试模式
#   --normal: 普通手测试模式
#   --revo2: Roban2 灵巧手测试模式
#   --test [round]: 测试灵巧手运动, round 为测试次数，默认为 5 次
#   --scan: 扫描设备, 识别 ttyUSB 设备, 并尝试控制设备运动
# 例如:
# 触觉手扫描: ./dexhand_test --touch --scan 
# 触觉手测试: ./dexhand_test --touch --test 
# 普通手扫描: ./dexhand_test --normal --scan 
# 普通手测试: ./dexhand_test --normal --test 

```

## 工具

### 设备规则生成工具
```bash
# 运行设备规则生成脚本
./scripts/gen_dexhand_serial_rules.sh
```

SDK支持自动设备识别，需要创建udev规则：

## 支持和文档
- **revo1 一代手文档**：https://www.brainco-hz.com/docs/revolimb-hand/revo1/parameters.html
- **revo2 二代手文档**：https://www.brainco-hz.com/docs/revolimb-hand/index.html
- **原生 SDK 示例**: https://github.com/BrainCoTech/stark-serialport-example