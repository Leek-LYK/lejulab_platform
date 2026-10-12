# lejusdk-recorder

基于 Eclipse CycloneDDS 的 DDS 数据录制工具，使用 MCAP 文件格式存储。

## 功能特性

- **DDS 录制**：自动发现并录制 DDS 网络中的话题到 MCAP 文件
- **动态类型发现**：通过 XTypes 支持任意 DDS 类型的录制
- **话题过滤**：支持配置允许/禁止录制的话题列表
- **MCAP 格式**：行业标准格式，兼容 Foxglove Studio 可视化工具

## 依赖要求

- CMake 3.16+
- C++17 编译器
- CycloneDDS 0.10.2+（需要启用 Topic/Type Discovery）

## 编译

使用 catkin 编译：

```bash
catkin build lejusdk-recorder
```

## 使用方法

```bash
# 基本录制
lejusdk_recorder -o recording.mcap

# 指定域ID和时长
lejusdk_recorder -d 1 -o recording.mcap -t 60

# 使用配置文件
lejusdk_recorder -c config/recorder_example.yaml

# 详细输出
lejusdk_recorder -o recording.mcap -v
```

### 命令行参数

| 参数 | 说明 |
|------|------|
| `-h, --help` | 显示帮助信息 |
| `-c, --config` | 配置文件路径 (YAML) |
| `-o, --output` | 输出 MCAP 文件路径 |
| `-d, --domain` | DDS 域 ID (默认: 0) |
| `-t, --duration` | 录制时长（秒），0 表示无限制 |
| `-v, --verbose` | 详细日志输出 |

## 架构设计

```
┌─────────────────────────────────────────────────────────────────────┐
│                         DdsRecorder                                  │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────────┐  │
│  │ TopicDiscovery  │  │ TypesCollection │  │     McapWriter      │  │
│  │                 │  │                 │  │                     │  │
│  │ - 监听内置话题  │  │ - 收集类型信息  │  │ - Schema管理        │  │
│  │ - 发现发布者    │  │ - 生成IDL定义   │  │ - Channel管理       │  │
│  │ - 获取类型信息  │  │ - 话题-类型映射 │  │ - 消息写入          │  │
│  └────────┬────────┘  └────────┬────────┘  └──────────┬──────────┘  │
│           │                    │                      │             │
│           ▼                    ▼                      ▼             │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │                    RawDataReader(s)                          │   │
│  │              (每个话题一个, 使用 dds_takecdr)                 │   │
│  └─────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────┘
                              │
                              ▼
                    ┌─────────────────┐
                    │   MCAP 文件     │
                    │  (omgidl+cdr)   │
                    └─────────────────┘
```

## MCAP 文件格式

录制的 MCAP 文件结构：

```
MCAP File
├── Header (profile: "cyclone-dds-recorder")
├── Schemas (OMG IDL 定义)
│   ├── Schema 1: HelloWorldData::HelloWorld
│   ├── Schema 2: SensorData::IMU
│   └── ...
├── Channels (话题)
│   ├── Channel 1: /HelloWorldTopic -> Schema 1
│   ├── Channel 2: /IMUTopic -> Schema 2
│   └── ...
├── Messages (CDR 编码的消息数据)
│   ├── Message 1: channel=1, timestamp=..., data=[CDR bytes]
│   └── ...
└── Footer (索引信息)
```

**编码格式**：
- Schema encoding: `omgidl` (OMG IDL 格式)
- Message encoding: `cdr` (DDS 标准序列化)

## Foxglove Studio 兼容性

录制的 MCAP 文件可以在 [Foxglove Studio](https://foxglove.dev/studio) 中打开查看。

## 项目结构

```
lejusdk-recorder/
├── CMakeLists.txt
├── package.xml
├── common/                     # 公共工具库
│   └── include/common/
│       ├── Log.hpp            # 日志
│       ├── Types.hpp          # 公共类型定义
│       └── Config.hpp         # 配置基类
├── cyclone_dds/               # CycloneDDS 抽象层
│   └── include/cyclone_dds/
│       ├── CycloneParticipant.hpp  # DDS 参与者
│       ├── TopicDiscovery.hpp      # 话题发现
│       ├── RawDataReader.hpp       # CDR 数据读取
│       └── TypeSupport.hpp         # 类型支持
├── recorder/                  # 录制器
│   ├── include/recorder/
│   │   ├── DdsRecorder.hpp    # 主录制器
│   │   ├── McapWriter.hpp     # MCAP 写入器
│   │   └── TypesCollection.hpp # 类型收集器
│   └── app/main.cpp
├── config/                    # 配置示例
└── thirdparty/               # 第三方库
    ├── mcap/                 # Foxglove MCAP 库
    ├── nlohmann-json/
    └── optionparser/
```

## 参考资料

- [Eclipse CycloneDDS](https://github.com/eclipse-cyclonedds/cyclonedds)
- [MCAP 文件格式](https://mcap.dev/)
- [Foxglove Studio](https://foxglove.dev/studio)
