# DDS Recorder 实现文档

## 概述

DDS Recorder 是一个基于 CycloneDDS 的数据录制工具，能够动态发现 DDS 网络中的话题并将消息录制为 MCAP 格式文件。该工具支持任意 DDS 类型的录制，无需在编译时知道消息结构。

## 架构图

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
│           │                                                         │
│           ▼                                                         │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │                   Data Polling Thread                        │   │
│  │              (轮询所有reader, 写入MCAP)                       │   │
│  └─────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────┘
                              │
                              ▼
                    ┌─────────────────┐
                    │   MCAP 文件     │
                    │  (omgidl+cdr)   │
                    └─────────────────┘
```

## 核心组件

### 1. DdsRecorder (`recorder/src/DdsRecorder.cpp`)

主控制器，协调所有组件的工作。

**职责：**
- 初始化 DDS Participant
- 管理录制生命周期 (start/stop/pause/resume)
- 响应话题发现事件
- 创建和管理 RawDataReader
- 维护录制统计信息

**关键流程：**

```cpp
// 初始化
bool DdsRecorder::init() {
    // 1. 创建 DDS Participant
    participant_ = make_unique<CycloneParticipant>(config);

    // 2. 创建话题发现器
    discovery_ = make_unique<TopicDiscovery>(*participant_);
    discovery_->set_endpoint_discovered_callback([this](auto& ep) {
        on_endpoint_discovered(ep);
    });

    // 3. 创建 MCAP Writer
    writer_ = make_unique<McapWriter>(config.output_config());
}

// 开始录制
bool DdsRecorder::start() {
    writer_->open();           // 打开MCAP文件
    discovery_->start();       // 开始话题发现
    polling_thread_ = ...;     // 启动数据轮询线程
}
```

### 2. TopicDiscovery (`cyclone_dds/src/TopicDiscovery.cpp`)

使用 DDS 内置话题动态发现网络中的发布者。

**工作原理：**
- 订阅 `DCPSPublication` 内置话题
- 当有新的发布者出现时，获取其话题名、类型名和类型信息 (`dds_typeinfo_t`)
- 通过回调通知 DdsRecorder

**关键代码：**

```cpp
void TopicDiscovery::process_endpoint(
    const dds_builtintopic_endpoint_t* sample,
    bool is_writer, bool is_alive)
{
    if (is_alive) {
        DiscoveredEndpoint endpoint;
        endpoint.topic_name = sample->topic_name;
        endpoint.type_name = sample->type_name;
        endpoint.type_info = sample->type_info;  // XTypes类型信息
        endpoint.is_writer = is_writer;

        // 通知发现回调
        discovered_callback_(endpoint);
    }
}
```

### 3. RawDataReader (`cyclone_dds/src/RawDataReader.cpp`)

使用 `dds_takecdr()` 读取原始 CDR 数据，无需反序列化。

**为什么使用 CDR 原始数据：**
- 录制时不需要知道消息的具体结构
- 避免了为每种类型生成反序列化代码
- CDR 是 DDS 标准序列化格式，可以直接存储

**关键代码：**

```cpp
std::vector<RawCdrSample> RawDataReader::take_raw() {
    std::vector<RawCdrSample> result;

    // 使用 dds_takecdr 读取原始CDR数据
    dds_sample_info_t info;
    struct ddsi_serdata* serdata = nullptr;

    while (dds_takecdr(reader_, &serdata, 1, &info, DDS_ANY_STATE) > 0) {
        if (info.valid_data) {
            RawCdrSample sample;
            // 从 serdata 提取 CDR 数据
            sample.cdr_data = extract_cdr_payload(serdata);
            sample.timestamp = info.source_timestamp;
            result.push_back(std::move(sample));
        }
        ddsi_serdata_unref(serdata);
    }

    return result;
}
```

### 4. TypesCollection (`recorder/src/TypesCollection.cpp`)

收集和管理发现的 DDS 类型信息。

**职责：**
- 存储类型名称和类型信息的映射
- 生成 OMG IDL 格式的 Schema 定义
- 管理话题到类型的映射

**Schema 生成：**

```cpp
bool TypesCollection::add_type(const std::string& type_name, ...) {
    CollectedType collected;
    collected.type_name = type_name;
    collected.schema_encoding = "omgidl";  // OMG IDL格式

    // 生成IDL定义 (用于Foxglove等工具解析)
    collected.schema_data = TypeSupport::generate_idl_for_type(type_name);

    types_[type_name] = collected;
}
```

### 5. McapWriter (`recorder/src/McapWriter.cpp`)

将 DDS 数据写入 MCAP 文件格式。

**MCAP 结构：**
```
MCAP File
├── Header (profile: "cyclone-dds-recorder")
├── Schemas (OMG IDL 定义)
│   ├── Schema 1: SensorData::IMU
│   ├── Schema 2: SensorData::JointState
│   └── ...
├── Channels (话题)
│   ├── Channel 1: /IMUTopic -> Schema 1
│   ├── Channel 2: /JointStateTopic -> Schema 2
│   └── ...
├── Messages (CDR 编码的消息数据)
│   ├── Message 1: channel=1, timestamp=..., data=[CDR bytes]
│   ├── Message 2: channel=2, timestamp=..., data=[CDR bytes]
│   └── ...
└── Footer (索引信息)
```

**关键代码：**

```cpp
// 注册 Schema
mcap::SchemaId McapWriter::register_schema(
    const std::string& type_name,
    const std::string& schema_encoding,  // "omgidl"
    const std::string& schema_data)      // IDL文本
{
    mcap::Schema schema;
    schema.name = type_name;
    schema.encoding = schema_encoding;
    schema.data = to_bytes(schema_data);

    writer_->addSchema(schema);
    return schema.id;
}

// 注册 Channel
mcap::ChannelId McapWriter::register_channel(
    const std::string& topic_name,
    mcap::SchemaId schema_id,
    const std::string& message_encoding)  // "cdr"
{
    mcap::Channel channel;
    channel.topic = topic_name;
    channel.schemaId = schema_id;
    channel.messageEncoding = message_encoding;

    writer_->addChannel(channel);
    return channel.id;
}

// 写入消息
bool McapWriter::write_message(
    mcap::ChannelId channel_id,
    uint64_t timestamp,
    const void* data,
    size_t size)
{
    mcap::Message msg;
    msg.channelId = channel_id;
    msg.logTime = timestamp;
    msg.publishTime = timestamp;
    msg.data = reinterpret_cast<const std::byte*>(data);
    msg.dataSize = size;

    return writer_->write(msg).ok();
}
```

## 数据流程

### 录制流程

```
1. 启动录制
   DdsRecorder::start()
   ├── McapWriter::open()          // 创建MCAP文件
   ├── TopicDiscovery::start()     // 开始监听内置话题
   └── 启动 polling_thread

2. 话题发现
   TopicDiscovery 发现新发布者
   └── DdsRecorder::on_endpoint_discovered()
       ├── TypesCollection::add_type()      // 收集类型信息
       ├── McapWriter::register_schema()    // 注册IDL Schema
       ├── McapWriter::register_channel()   // 注册Channel
       └── setup_reader_for_topic()         // 创建RawDataReader

3. 数据轮询
   polling_thread 循环执行:
   └── 对每个 RawDataReader:
       ├── reader->take_raw()               // 读取CDR数据
       └── McapWriter::write_message()      // 写入MCAP

4. 停止录制
   DdsRecorder::stop()
   ├── polling_active_ = false
   ├── TopicDiscovery::stop()
   └── McapWriter::close()                  // 写入索引,关闭文件
```

### MCAP 文件格式

```
┌────────────────────────────────────────────────┐
│ MCAP Header                                     │
│   Magic: 0x89 'M' 'C' 'A' 'P' '0' '\r' '\n'   │
│   Profile: "cyclone-dds-recorder"              │
│   Library: "libmcap 2.1.2"                     │
├────────────────────────────────────────────────┤
│ Schema Records                                  │
│   Schema 1:                                     │
│     name: "SensorData::JointState"             │
│     encoding: "omgidl"                         │
│     data: "module SensorData { struct ... }"   │
├────────────────────────────────────────────────┤
│ Channel Records                                 │
│   Channel 1:                                    │
│     topic: "JointStateTopic"                   │
│     schema_id: 1                               │
│     message_encoding: "cdr"                    │
├────────────────────────────────────────────────┤
│ Message Records (in Chunks)                    │
│   Message:                                      │
│     channel_id: 1                              │
│     log_time: 1234567890123456789 (ns)         │
│     data: [CDR encoded bytes]                  │
├────────────────────────────────────────────────┤
│ Footer (Summary + Index)                       │
└────────────────────────────────────────────────┘
```

## 关键技术点

### 1. 动态类型发现

CycloneDDS 支持 XTypes (Extensible Types)，通过 `dds_typeinfo_t` 可以获取运行时类型信息：

```cpp
// 从内置话题获取类型信息
const dds_typeinfo_t* type_info = sample->type_info;

// 使用类型信息创建动态Topic描述符
dds_topic_descriptor_t* desc;
dds_create_topic_descriptor(
    DDS_FIND_SCOPE_GLOBAL,
    participant,
    type_info,
    timeout,
    &desc);
```

### 2. CDR 原始数据读取

使用 `dds_takecdr()` 而非 `dds_take()` 读取数据：

```cpp
// dds_take() - 需要类型定义，返回反序列化的结构体
void* samples[1];
dds_take(reader, samples, ...);  // 需要知道消息类型

// dds_takecdr() - 返回原始CDR字节流
struct ddsi_serdata* serdata;
dds_takecdr(reader, &serdata, ...);  // 不需要知道消息类型
```

### 3. Schema 格式选择

MCAP 支持多种 Schema 格式，对于 DDS 录制：

| Schema Encoding | Message Encoding | 说明 |
|-----------------|------------------|------|
| `omgidl`        | `cdr`            | 推荐：标准DDS格式，Foxglove支持 |
| `ros2msg`       | `cdr`            | ROS2消息定义格式 |
| `jsonschema`    | `json`           | JSON格式（需转换CDR->JSON） |

本项目使用 `omgidl` + `cdr` 组合，这是 DDS 的标准格式。

## 文件结构

```
recorder/
├── include/recorder/
│   ├── DdsRecorder.hpp      # 主录制器
│   ├── McapWriter.hpp       # MCAP写入器
│   ├── TypesCollection.hpp  # 类型收集器
│   └── Configuration.hpp    # 配置定义
├── src/
│   ├── DdsRecorder.cpp
│   ├── McapWriter.cpp
│   ├── TypesCollection.cpp
│   └── Configuration.cpp
└── app/
    └── main.cpp             # 命令行入口

cyclone_dds/
├── include/cyclone_dds/
│   ├── TopicDiscovery.hpp   # 话题发现
│   ├── RawDataReader.hpp    # CDR数据读取
│   ├── CycloneParticipant.hpp
│   └── TypeSupport.hpp      # IDL生成
└── src/
    ├── TopicDiscovery.cpp
    ├── RawDataReader.cpp
    ├── CycloneParticipant.cpp
    └── TypeSupport.cpp
```

## 使用示例

```bash
# 基本录制
./dds_recorder -o recording.mcap

# 指定域ID和时长
./dds_recorder -d 1 -o recording.mcap --duration 60

# 话题过滤
./dds_recorder -o recording.mcap --topics "IMU*,Joint*"
```

## 依赖

### CycloneDDS

**版本要求**: `0.10.2` 或更高版本

**必要特性**:
- `TYPE_DISCOVERY` - 动态类型发现支持
- `TOPIC_DISCOVERY` - 话题发现支持
- XTypes 支持 - 用于获取运行时类型信息

**编译选项**:
```bash
cmake -DENABLE_TYPE_DISCOVERY=ON \
      -DENABLE_TOPIC_DISCOVERY=ON \
      ..
```

**关键 API**:
| API | 版本 | 用途 |
|-----|------|------|
| `dds_takecdr()` | 0.8+ | 读取原始 CDR 数据 |
| `dds_create_topic_descriptor()` | 0.10+ | 从 typeinfo 创建动态 topic |
| `dds_builtintopic_endpoint_t.type_info` | 0.10+ | 获取 XTypes 类型信息 |
| `dds_delete_topic_descriptor()` | 0.10+ | 释放动态 topic 描述符 |

**注意**: CycloneDDS 0.9.x 及更早版本不支持完整的动态类型发现，无法使用本录制工具。

### 其他依赖

- **libmcap 2.1.x**: 官方 Foxglove MCAP C++ 库 (header-only)
- **spdlog 1.x**: 日志库
- **yaml-cpp 0.6+**: 配置文件解析（可选）

## Foxglove 兼容性

录制的 MCAP 文件可以在 Foxglove Studio 中打开：

1. Schema encoding 使用 `omgidl`（OMG IDL 格式）
2. Message encoding 使用 `cdr`（DDS 标准序列化）
3. IDL 定义包含完整的结构体信息，支持数组类型

```
Schema示例 (omgidl):
module SensorData {
    struct JointState {
        @key long joint_id;
        double timestamp;
        string name;
        double position[26];
        double velocity[26];
        double effort[26];
    };
};
```
