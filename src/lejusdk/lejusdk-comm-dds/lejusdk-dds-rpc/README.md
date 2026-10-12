# CycloneDDS RPC Framework

基于 CycloneDDS 实现的 RPC over DDS 框架，遵循 OMG DDS-RPC 规范。

## 项目位置

本框架位于 `src/lejusdk/lejusdk-comm-dds/lejusdk-dds-rpc/`，与现有 DDS 通信模块保持一致：

```
src/lejusdk/lejusdk-comm-dds/
├── lejusdk-dds-idl/        # IDL 定义
├── lejusdk-topic-pubsub/   # 发布订阅
└── lejusdk-dds-rpc/        # RPC 服务 (本框架)
```

## 目录结构

```
lejusdk-dds-rpc/
├── README.md                    # 本文档
├── CMakeLists.txt               # CMake 构建配置
├── cmake/
│   └── dds_rpc-config.cmake.in  # CMake 包配置模板
├── include/
│   └── dds_rpc/
│       ├── rpc_types.hpp        # 基础类型和类型约束
│       ├── requester.hpp        # 客户端 Requester
│       ├── replier.hpp          # 服务端 Replier
│       └── service.hpp          # 便捷包装类
├── idl/
│   ├── rpc_types.idl            # RPC 基础类型 IDL
│   └── calculator_service.idl   # 示例服务 IDL
└── examples/
    ├── calculator_server.cpp    # 服务端示例
    └── calculator_client.cpp    # 客户端示例
```

## 快速开始

### 1. 构建

```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . -j$(nproc)
```

### 2. 运行示例

```bash
# 终端 1: 启动服务端
./calculator_server

# 终端 2: 运行客户端
./calculator_client
```

## 核心组件

### Requester (客户端)

```cpp
#include <dds_rpc/requester.hpp>

// 创建 Requester
dds_rpc::Requester<MyRequest, MyReply> requester(
    participant, "MyService", &MyRequest_desc, &MyReply_desc);

// 等待服务可用
requester.wait_for_service();

// 同步调用
MyRequest req{};
auto reply = requester.send_request(req);

// 异步调用
auto future = requester.send_request_future(req);
auto reply = future.get();
```

### Replier (服务端)

```cpp
#include <dds_rpc/replier.hpp>

// 创建 Replier
dds_rpc::Replier<MyRequest, MyReply> replier(
    participant, "MyService", &MyRequest_desc, &MyReply_desc);

// 设置处理器
replier.set_handler([](const MyRequest& req) {
    MyReply reply{};
    // 处理请求...
    return reply;
});

// 启动服务
replier.start();  // 后台线程
// 或
replier.run();    // 阻塞运行
```

### 便捷包装类

```cpp
#include <dds_rpc/service.hpp>

// 服务端 (自动管理 Participant)
dds_rpc::ServiceServer<MyRequest, MyReply> server(
    "MyService", &MyRequest_desc, &MyReply_desc,
    [](const MyRequest& req) { return MyReply{}; });
server.run();

// 客户端
dds_rpc::ServiceClient<MyRequest, MyReply> client(
    "MyService", &MyRequest_desc, &MyReply_desc);
auto reply = client.call(request);
```

## 服务重复检测

框架默认**不允许同名服务重复注册**，避免多个服务端同时处理请求导致的问题：

```cpp
// 默认行为：第二个服务端启动时抛出异常
Replier<Req, Rep> replier1(participant, "Calculator", ...);  // ✓ 成功
Replier<Req, Rep> replier2(participant, "Calculator", ...);  // ✗ 抛出 DuplicateServiceException

// 如需允许重复（特殊场景）
RpcQosConfig config;
config.allow_duplicate_service = true;
Replier<Req, Rep> replier(participant, "Calculator", ..., config);
```

**配置选项：**

| 选项 | 默认值 | 说明 |
|------|--------|------|
| `allow_duplicate_service` | `false` | 是否允许同名服务重复注册 |
| `discovery_wait_ms` | `300` | 服务发现等待时间（毫秒） |

## 类型约束

Request 和 Reply 类型必须包含符合规范的 header 成员：

```cpp
// Request 类型要求
struct MyRequest {
    dds_rpc::RequestHeader header;  // 必须
    // 业务数据...
};

// Reply 类型要求
struct MyReply {
    dds_rpc::ReplyHeader header;    // 必须
    // 业务数据...
};
```

如果类型不符合要求，编译时会报错：

```
error: static assertion failed: TRequest must have a 'header' member
error: static assertion failed: TReply::header must have a 'remote_ex' member
```

## 定义服务

### 1. 编写 IDL

```idl
#include "rpc_types.idl"

module my_service {
    struct MyRequest {
        dds_rpc::RequestHeader header;
        int32 data;
    };

    struct MyReply {
        dds_rpc::ReplyHeader header;
        int32 result;
    };
};
```

### 2. 生成代码

```bash
idlc -l c my_service.idl
```

### 3. 实现服务

参考 `examples/` 目录下的示例代码。

## 架构

```
┌─────────────┐                              ┌─────────────┐
│   Client    │                              │   Server    │
│  Requester  │                              │   Replier   │
└──────┬──────┘                              └──────┬──────┘
       │                                            │
       │  Request (request_id)                      │
       │ ──────────────────────────────────────────►│
       │          Topic: rq/<service>/Request       │
       │                                            │
       │  Reply (related_request_id)                │
       │◄────────────────────────────────────────── │
       │          Topic: rr/<service>/Reply         │
       │                                            │
```

## 依赖

- CycloneDDS >= 0.10.0
- C++17 编译器
- CMake >= 3.16

## 详细文档

完整的技术文档请参阅: [cyclonedds_rpc_implementation.md](./cyclonedds_rpc_implementation.md)

## 许可证

MIT License
