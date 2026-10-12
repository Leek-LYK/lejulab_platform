# CycloneDDS RPC over DDS 实现技术文档

> **快速使用**: 可直接使用的头文件和示例代码位于本目录的 [include/](./include/) 和 [examples/](./examples/) 目录

## 1. 概述

### 1.1 背景

DDS (Data Distribution Service) 是 OMG 组织定义的数据分发服务标准，提供发布/订阅模式的通信机制。然而，DDS 原生不支持请求-响应（Request-Reply）模式，而这种模式在服务调用场景中非常常见。

OMG 定义了 DDS-RPC 规范来解决这个问题，但 CycloneDDS 并未内置实现该规范。本文档描述如何基于 CycloneDDS 实现符合 DDS-RPC 规范的服务调用框架。

### 1.2 目标

- 实现类似 ROS 2 Service 的请求-响应模式
- 遵循 OMG DDS-RPC 规范
- 支持同步和异步调用
- 提供类型安全的模板化 API
- 支持超时和异常处理

### 1.3 核心概念

| 概念 | 说明 |
|------|------|
| **Requester** | 客户端角色，发送请求并等待响应 |
| **Replier** | 服务端角色，接收请求并发送响应 |
| **RequestHeader** | 请求头，包含请求标识和实例名 |
| **ReplyHeader** | 响应头，包含关联的请求标识和异常码 |
| **SampleIdentity** | 唯一标识一个请求，由 GUID + 序列号组成 |

## 2. 架构设计

### 2.1 整体架构

```
┌─────────────────────────────────────────────────────────────────┐
│                        应用层 (Application)                      │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│   ┌─────────────────────┐         ┌─────────────────────┐       │
│   │      Client         │         │       Server        │       │
│   │  ┌───────────────┐  │         │  ┌───────────────┐  │       │
│   │  │   Requester   │  │         │  │    Replier    │  │       │
│   │  │  <Req, Rep>   │  │         │  │  <Req, Rep>   │  │       │
│   │  └───────┬───────┘  │         │  └───────┬───────┘  │       │
│   └──────────┼──────────┘         └──────────┼──────────┘       │
│              │                               │                   │
├──────────────┼───────────────────────────────┼───────────────────┤
│              │         RPC 封装层            │                   │
│              │                               │                   │
│   ┌──────────▼──────────┐         ┌──────────▼──────────┐       │
│   │  Request Publisher  │         │  Request Subscriber │       │
│   └──────────┬──────────┘         └──────────┬──────────┘       │
│              │                               │                   │
│   ┌──────────▼──────────┐         ┌──────────▼──────────┐       │
│   │  Reply Subscriber   │         │   Reply Publisher   │       │
│   └──────────┬──────────┘         └──────────┬──────────┘       │
│              │                               │                   │
├──────────────┼───────────────────────────────┼───────────────────┤
│              │                               │                   │
│              └───────────┬───────────────────┘                   │
│                          │                                       │
│              ┌───────────▼───────────┐                          │
│              │      CycloneDDS       │                          │
│              │   (Pub/Sub Layer)     │                          │
│              └───────────────────────┘                          │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

### 2.2 Topic 命名约定

遵循 DDS-RPC 规范的 Topic 命名：

```
请求 Topic: rq/<ServiceName>/Request
响应 Topic: rr/<ServiceName>/Reply
```

示例：
- 服务名: `Calculator`
- 请求 Topic: `rq/Calculator/Request`
- 响应 Topic: `rr/Calculator/Reply`

### 2.3 请求-响应匹配机制

```
┌─────────────┐                              ┌─────────────┐
│   Client    │                              │   Server    │
├─────────────┤                              ├─────────────┤
│             │  Request (request_id=123)    │             │
│  Requester  │ ─────────────────────────────► Replier    │
│             │                              │             │
│             │  Reply (related_id=123)      │             │
│             │ ◄───────────────────────────── │           │
└─────────────┘                              └─────────────┘

SampleIdentity {
    writer_guid[16]     // 客户端唯一标识
    sequence_number     // 递增序列号
}

匹配规则: reply.related_request_id == request.request_id
```

### 2.4 数据流

```
[Client]                                           [Server]
   │                                                   │
   │  1. 生成 request_id (GUID + SeqNum)               │
   │  2. 填充 RequestHeader                            │
   │  3. 发布到 rq/<service>/Request                   │
   │ ─────────────────────────────────────────────────►│
   │                                                   │
   │                            4. 接收请求            │
   │                            5. 处理业务逻辑        │
   │                            6. 设置 related_id     │
   │                            7. 发布到 rr/<srv>/Reply
   │◄───────────────────────────────────────────────── │
   │                                                   │
   │  8. 接收响应                                      │
   │  9. 匹配 request_id                               │
   │ 10. 返回结果                                      │
   │                                                   │
```

## 3. IDL 定义

### 3.1 基础 RPC 类型 (rpc_types.idl)

```idl
// rpc_types.idl
// DDS-RPC 基础类型定义

module dds_rpc {

    /**
     * 样本标识 - 唯一标识一个请求
     * 由写入者 GUID 和序列号组成
     */
    struct SampleIdentity {
        octet writer_guid[16];      // 128-bit GUID
        int64 sequence_number;      // 递增序列号
    };

    /**
     * 请求头 - 每个请求必须包含
     */
    struct RequestHeader {
        SampleIdentity request_id;  // 请求唯一标识
        string instance_name;       // 服务实例名称 (可选)
    };

    /**
     * 远程异常码
     */
    enum RemoteExceptionCode {
        REMOTE_EX_OK,                    // 成功
        REMOTE_EX_UNSUPPORTED,           // 不支持的操作
        REMOTE_EX_INVALID_ARGUMENT,      // 无效参数
        REMOTE_EX_OUT_OF_RESOURCES,      // 资源不足
        REMOTE_EX_UNKNOWN_OPERATION,     // 未知操作
        REMOTE_EX_UNKNOWN_EXCEPTION      // 未知异常
    };

    /**
     * 响应头 - 每个响应必须包含
     */
    struct ReplyHeader {
        SampleIdentity related_request_id;  // 关联的请求标识
        RemoteExceptionCode remote_ex;      // 远程异常码
    };

};
```

### 3.2 服务定义示例 (calculator_service.idl)

```idl
// calculator_service.idl
// 计算器服务示例

#include "rpc_types.idl"

module example {

    /**
     * 计算器请求
     */
    struct CalculatorRequest {
        dds_rpc::RequestHeader header;  // 必须包含请求头
        int32 operand_a;                // 操作数 A
        int32 operand_b;                // 操作数 B
        string operation;               // 操作类型: "add", "subtract", "multiply", "divide"
    };

    /**
     * 计算器响应
     */
    struct CalculatorReply {
        dds_rpc::ReplyHeader header;    // 必须包含响应头
        int32 result;                   // 计算结果
        string error_message;           // 错误信息 (如果有)
    };

};
```

### 3.3 IDL 编译

```bash
# 使用 CycloneDDS idlc 编译
idlc -l c rpc_types.idl
idlc -l c calculator_service.idl

# 生成 C++ 代码 (需要 cyclonedds-cxx)
idlcxx rpc_types.idl
idlcxx calculator_service.idl
```

## 4. 核心实现

### 4.1 项目结构

```
cyclonedds_rpc/
├── CMakeLists.txt
├── idl/
│   ├── rpc_types.idl
│   └── calculator_service.idl
├── include/
│   └── dds_rpc/
│       ├── rpc_types.hpp
│       ├── requester.hpp
│       ├── replier.hpp
│       └── service.hpp
├── src/
│   └── rpc_types.cpp
├── examples/
│   ├── calculator_server.cpp
│   └── calculator_client.cpp
└── tests/
    └── test_rpc.cpp
```

### 4.2 基础类型实现 (rpc_types.hpp)

```cpp
// include/dds_rpc/rpc_types.hpp
#pragma once

#include <dds/dds.h>
#include <cstdint>
#include <cstring>
#include <string>
#include <random>
#include <atomic>
#include <chrono>
#include <functional>
#include <memory>

namespace dds_rpc {

//=============================================================================
// 基础类型定义
//=============================================================================

/**
 * @brief 样本标识结构
 * 用于唯一标识一个请求
 */
struct SampleIdentity {
    uint8_t writer_guid[16];    ///< 写入者 GUID (128-bit)
    int64_t sequence_number;    ///< 序列号

    SampleIdentity() : sequence_number(0) {
        std::memset(writer_guid, 0, sizeof(writer_guid));
    }

    bool operator==(const SampleIdentity& other) const {
        return std::memcmp(writer_guid, other.writer_guid, 16) == 0
            && sequence_number == other.sequence_number;
    }

    bool operator!=(const SampleIdentity& other) const {
        return !(*this == other);
    }
};

/**
 * @brief 远程异常码枚举
 */
enum class RemoteExceptionCode : int32_t {
    OK = 0,                     ///< 成功
    UNSUPPORTED = 1,            ///< 不支持的操作
    INVALID_ARGUMENT = 2,       ///< 无效参数
    OUT_OF_RESOURCES = 3,       ///< 资源不足
    UNKNOWN_OPERATION = 4,      ///< 未知操作
    UNKNOWN_EXCEPTION = 5,      ///< 未知异常
    TIMEOUT = 6                 ///< 超时
};

/**
 * @brief 请求头结构
 */
struct RequestHeader {
    SampleIdentity request_id;  ///< 请求标识
    std::string instance_name;  ///< 服务实例名
};

/**
 * @brief 响应头结构
 */
struct ReplyHeader {
    SampleIdentity related_request_id;  ///< 关联的请求标识
    RemoteExceptionCode remote_ex;      ///< 远程异常码

    ReplyHeader() : remote_ex(RemoteExceptionCode::OK) {}
};

//=============================================================================
// 工具函数
//=============================================================================

/**
 * @brief 生成随机 GUID
 * @param guid 输出 GUID 数组
 */
inline void generate_guid(uint8_t guid[16]) {
    static thread_local std::random_device rd;
    static thread_local std::mt19937_64 gen(rd());

    uint64_t* ptr = reinterpret_cast<uint64_t*>(guid);
    ptr[0] = gen();
    ptr[1] = gen();
}

/**
 * @brief 生成请求 Topic 名称
 * @param service_name 服务名称
 * @return Topic 名称
 */
inline std::string request_topic_name(const std::string& service_name) {
    return "rq/" + service_name + "/Request";
}

/**
 * @brief 生成响应 Topic 名称
 * @param service_name 服务名称
 * @return Topic 名称
 */
inline std::string reply_topic_name(const std::string& service_name) {
    return "rr/" + service_name + "/Reply";
}

//=============================================================================
// SampleIdentity 哈希支持 (用于 std::unordered_map)
//=============================================================================

struct SampleIdentityHash {
    std::size_t operator()(const SampleIdentity& id) const {
        std::size_t h = 0;
        for (int i = 0; i < 16; ++i) {
            h ^= std::hash<uint8_t>{}(id.writer_guid[i]) << (i % 8);
        }
        h ^= std::hash<int64_t>{}(id.sequence_number) << 1;
        return h;
    }
};

//=============================================================================
// 类型约束检查 (编译时静态检测)
//=============================================================================

namespace traits {

/**
 * @brief 检测类型是否有 header 成员
 */
template<typename T, typename = void>
struct has_header : std::false_type {};

template<typename T>
struct has_header<T, std::void_t<decltype(std::declval<T>().header)>>
    : std::true_type {};

template<typename T>
inline constexpr bool has_header_v = has_header<T>::value;

/**
 * @brief 检测 Request 类型的 header 是否有 request_id 成员
 */
template<typename T, typename = void>
struct has_request_id : std::false_type {};

template<typename T>
struct has_request_id<T, std::void_t<decltype(std::declval<T>().header.request_id)>>
    : std::true_type {};

template<typename T>
inline constexpr bool has_request_id_v = has_request_id<T>::value;

/**
 * @brief 检测 Reply 类型的 header 是否有 related_request_id 成员
 */
template<typename T, typename = void>
struct has_related_request_id : std::false_type {};

template<typename T>
struct has_related_request_id<T,
    std::void_t<decltype(std::declval<T>().header.related_request_id)>>
    : std::true_type {};

template<typename T>
inline constexpr bool has_related_request_id_v = has_related_request_id<T>::value;

/**
 * @brief 检测 Reply 类型的 header 是否有 remote_ex 成员
 */
template<typename T, typename = void>
struct has_remote_ex : std::false_type {};

template<typename T>
struct has_remote_ex<T, std::void_t<decltype(std::declval<T>().header.remote_ex)>>
    : std::true_type {};

template<typename T>
inline constexpr bool has_remote_ex_v = has_remote_ex<T>::value;

/**
 * @brief 验证 Request 类型是否符合 RPC 规范
 *
 * Request 类型必须满足:
 * - 有 header 成员
 * - header 有 request_id 成员 (SampleIdentity 类型)
 */
template<typename TRequest>
struct is_valid_request {
    static constexpr bool value =
        has_header_v<TRequest> &&
        has_request_id_v<TRequest>;
};

template<typename TRequest>
inline constexpr bool is_valid_request_v = is_valid_request<TRequest>::value;

/**
 * @brief 验证 Reply 类型是否符合 RPC 规范
 *
 * Reply 类型必须满足:
 * - 有 header 成员
 * - header 有 related_request_id 成员 (SampleIdentity 类型)
 * - header 有 remote_ex 成员 (RemoteExceptionCode 类型)
 */
template<typename TReply>
struct is_valid_reply {
    static constexpr bool value =
        has_header_v<TReply> &&
        has_related_request_id_v<TReply> &&
        has_remote_ex_v<TReply>;
};

template<typename TReply>
inline constexpr bool is_valid_reply_v = is_valid_reply<TReply>::value;

} // namespace traits

/**
 * @brief 静态断言宏 - 用于在模板类中检查类型约束
 */
#define DDS_RPC_STATIC_ASSERT_REQUEST(TRequest) \
    static_assert(traits::has_header_v<TRequest>, \
        "TRequest must have a 'header' member"); \
    static_assert(traits::has_request_id_v<TRequest>, \
        "TRequest::header must have a 'request_id' member of type SampleIdentity")

#define DDS_RPC_STATIC_ASSERT_REPLY(TReply) \
    static_assert(traits::has_header_v<TReply>, \
        "TReply must have a 'header' member"); \
    static_assert(traits::has_related_request_id_v<TReply>, \
        "TReply::header must have a 'related_request_id' member of type SampleIdentity"); \
    static_assert(traits::has_remote_ex_v<TReply>, \
        "TReply::header must have a 'remote_ex' member of type RemoteExceptionCode")

/**
 * @brief 组合检查宏 - 同时检查 Request 和 Reply
 */
#define DDS_RPC_STATIC_ASSERT_TYPES(TRequest, TReply) \
    DDS_RPC_STATIC_ASSERT_REQUEST(TRequest); \
    DDS_RPC_STATIC_ASSERT_REPLY(TReply)

//=============================================================================
// QoS 配置
//=============================================================================

/**
 * @brief RPC QoS 配置
 */
struct RpcQosConfig {
    bool reliable = true;                           ///< 可靠传输
    int32_t history_depth = 100;                    ///< 历史深度
    dds_duration_t deadline = DDS_INFINITY;         ///< 截止时间
    dds_duration_t lifespan = DDS_SECS(60);        ///< 生命周期

    /**
     * @brief 应用 QoS 到 Writer
     */
    dds_qos_t* create_writer_qos() const {
        dds_qos_t* qos = dds_create_qos();
        if (reliable) {
            dds_qset_reliability(qos, DDS_RELIABILITY_RELIABLE, DDS_SECS(1));
        }
        dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, history_depth);
        dds_qset_durability(qos, DDS_DURABILITY_TRANSIENT_LOCAL);
        return qos;
    }

    /**
     * @brief 应用 QoS 到 Reader
     */
    dds_qos_t* create_reader_qos() const {
        dds_qos_t* qos = dds_create_qos();
        if (reliable) {
            dds_qset_reliability(qos, DDS_RELIABILITY_RELIABLE, DDS_SECS(1));
        }
        dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, history_depth);
        dds_qset_durability(qos, DDS_DURABILITY_TRANSIENT_LOCAL);
        return qos;
    }
};

//=============================================================================
// 异常定义
//=============================================================================

/**
 * @brief RPC 异常基类
 */
class RpcException : public std::exception {
public:
    explicit RpcException(const std::string& message,
                          RemoteExceptionCode code = RemoteExceptionCode::UNKNOWN_EXCEPTION)
        : message_(message), code_(code) {}

    const char* what() const noexcept override { return message_.c_str(); }
    RemoteExceptionCode code() const noexcept { return code_; }

private:
    std::string message_;
    RemoteExceptionCode code_;
};

class TimeoutException : public RpcException {
public:
    TimeoutException() : RpcException("Request timeout", RemoteExceptionCode::TIMEOUT) {}
};

class ServiceNotFoundException : public RpcException {
public:
    explicit ServiceNotFoundException(const std::string& service)
        : RpcException("Service not found: " + service, RemoteExceptionCode::UNKNOWN_OPERATION) {}
};

} // namespace dds_rpc
```

### 4.3 Requester 实现 (requester.hpp)

```cpp
// include/dds_rpc/requester.hpp
#pragma once

#include "rpc_types.hpp"
#include <optional>
#include <future>
#include <mutex>
#include <condition_variable>
#include <unordered_map>
#include <queue>

namespace dds_rpc {

/**
 * @brief 请求者模板类
 *
 * 用于发送请求并接收响应的客户端组件
 *
 * @tparam TRequest 请求类型 (必须包含符合规范的 header 成员)
 * @tparam TReply 响应类型 (必须包含符合规范的 header 成员)
 *
 * 类型约束:
 * - TRequest 必须有 header.request_id (SampleIdentity)
 * - TReply 必须有 header.related_request_id (SampleIdentity)
 * - TReply 必须有 header.remote_ex (RemoteExceptionCode)
 */
template<typename TRequest, typename TReply>
class Requester {
    // =========== 编译时类型检查 ===========
    DDS_RPC_STATIC_ASSERT_TYPES(TRequest, TReply);

public:
    using RequestType = TRequest;
    using ReplyType = TReply;
    using ReplyCallback = std::function<void(const TReply&, RemoteExceptionCode)>;

    /**
     * @brief 构造函数
     *
     * @param participant DDS Participant
     * @param service_name 服务名称
     * @param request_desc 请求类型描述符
     * @param reply_desc 响应类型描述符
     * @param qos_config QoS 配置
     */
    Requester(dds_entity_t participant,
              const std::string& service_name,
              const dds_topic_descriptor_t* request_desc,
              const dds_topic_descriptor_t* reply_desc,
              const RpcQosConfig& qos_config = RpcQosConfig())
        : participant_(participant)
        , service_name_(service_name)
        , sequence_number_(0)
        , running_(true)
    {
        // 生成客户端 GUID
        generate_guid(client_guid_);

        // 创建请求 Topic
        std::string req_topic_name = request_topic_name(service_name);
        request_topic_ = dds_create_topic(
            participant, request_desc, req_topic_name.c_str(), nullptr, nullptr);
        if (request_topic_ < 0) {
            throw RpcException("Failed to create request topic");
        }

        // 创建请求 Writer
        dds_qos_t* writer_qos = qos_config.create_writer_qos();
        request_writer_ = dds_create_writer(participant, request_topic_, writer_qos, nullptr);
        dds_delete_qos(writer_qos);
        if (request_writer_ < 0) {
            throw RpcException("Failed to create request writer");
        }

        // 创建响应 Topic
        std::string rep_topic_name = reply_topic_name(service_name);
        reply_topic_ = dds_create_topic(
            participant, reply_desc, rep_topic_name.c_str(), nullptr, nullptr);
        if (reply_topic_ < 0) {
            throw RpcException("Failed to create reply topic");
        }

        // 创建响应 Reader
        dds_qos_t* reader_qos = qos_config.create_reader_qos();
        reply_reader_ = dds_create_reader(participant, reply_topic_, reader_qos, nullptr);
        dds_delete_qos(reader_qos);
        if (reply_reader_ < 0) {
            throw RpcException("Failed to create reply reader");
        }

        // 创建 WaitSet
        waitset_ = dds_create_waitset(participant);
        read_condition_ = dds_create_readcondition(reply_reader_, DDS_DATA_AVAILABLE);
        dds_waitset_attach(waitset_, read_condition_, 0);

        // 启动响应接收线程
        receive_thread_ = std::thread(&Requester::receive_loop, this);
    }

    /**
     * @brief 析构函数
     */
    ~Requester() {
        running_ = false;
        if (receive_thread_.joinable()) {
            receive_thread_.join();
        }

        dds_waitset_detach(waitset_, read_condition_);
        dds_delete(read_condition_);
        dds_delete(waitset_);
        dds_delete(reply_reader_);
        dds_delete(request_writer_);
        dds_delete(reply_topic_);
        dds_delete(request_topic_);
    }

    // 禁用拷贝
    Requester(const Requester&) = delete;
    Requester& operator=(const Requester&) = delete;

    /**
     * @brief 获取服务名称
     */
    const std::string& service_name() const { return service_name_; }

    /**
     * @brief 同步发送请求
     *
     * @param request 请求对象 (header 会被自动填充)
     * @param timeout 超时时间
     * @return 响应对象
     * @throw TimeoutException 超时
     * @throw RpcException 其他错误
     */
    TReply send_request(TRequest request,
                        std::chrono::milliseconds timeout = std::chrono::seconds(5)) {
        // 填充请求头
        SampleIdentity req_id = fill_request_header(request);

        // 创建 Promise
        auto promise = std::make_shared<std::promise<TReply>>();
        auto future = promise->get_future();

        // 注册等待回调
        {
            std::lock_guard<std::mutex> lock(pending_mutex_);
            pending_requests_[req_id] = [promise](const TReply& reply, RemoteExceptionCode ex) {
                if (ex == RemoteExceptionCode::OK) {
                    promise->set_value(reply);
                } else {
                    promise->set_exception(std::make_exception_ptr(
                        RpcException("Remote exception", ex)));
                }
            };
        }

        // 发送请求
        dds_return_t rc = dds_write(request_writer_, &request);
        if (rc != DDS_RETCODE_OK) {
            std::lock_guard<std::mutex> lock(pending_mutex_);
            pending_requests_.erase(req_id);
            throw RpcException("Failed to send request: " + std::to_string(rc));
        }

        // 等待响应
        auto status = future.wait_for(timeout);
        if (status == std::future_status::timeout) {
            std::lock_guard<std::mutex> lock(pending_mutex_);
            pending_requests_.erase(req_id);
            throw TimeoutException();
        }

        return future.get();
    }

    /**
     * @brief 异步发送请求
     *
     * @param request 请求对象
     * @param callback 响应回调
     * @return 请求标识
     */
    SampleIdentity send_request_async(TRequest request, ReplyCallback callback) {
        // 填充请求头
        SampleIdentity req_id = fill_request_header(request);

        // 注册回调
        {
            std::lock_guard<std::mutex> lock(pending_mutex_);
            pending_requests_[req_id] = std::move(callback);
        }

        // 发送请求
        dds_return_t rc = dds_write(request_writer_, &request);
        if (rc != DDS_RETCODE_OK) {
            std::lock_guard<std::mutex> lock(pending_mutex_);
            pending_requests_.erase(req_id);
            throw RpcException("Failed to send request");
        }

        return req_id;
    }

    /**
     * @brief 异步发送请求 (返回 future)
     *
     * @param request 请求对象
     * @param timeout 超时时间
     * @return future 对象
     */
    std::future<TReply> send_request_future(TRequest request,
                        std::chrono::milliseconds timeout = std::chrono::seconds(5)) {
        return std::async(std::launch::async, [this, request, timeout]() mutable {
            return send_request(std::move(request), timeout);
        });
    }

    /**
     * @brief 取消等待中的请求
     *
     * @param request_id 请求标识
     * @return 是否成功取消
     */
    bool cancel_request(const SampleIdentity& request_id) {
        std::lock_guard<std::mutex> lock(pending_mutex_);
        return pending_requests_.erase(request_id) > 0;
    }

    /**
     * @brief 获取等待中的请求数量
     */
    size_t pending_count() const {
        std::lock_guard<std::mutex> lock(pending_mutex_);
        return pending_requests_.size();
    }

    /**
     * @brief 等待服务可用
     *
     * @param timeout 超时时间
     * @return 服务是否可用
     */
    bool wait_for_service(std::chrono::milliseconds timeout = std::chrono::seconds(10)) {
        auto deadline = std::chrono::steady_clock::now() + timeout;

        while (std::chrono::steady_clock::now() < deadline) {
            dds_publication_matched_status_t status;
            if (dds_get_publication_matched_status(request_writer_, &status) == DDS_RETCODE_OK) {
                if (status.current_count > 0) {
                    return true;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        return false;
    }

private:
    /**
     * @brief 填充请求头
     */
    SampleIdentity fill_request_header(TRequest& request) {
        std::memcpy(request.header.request_id.writer_guid, client_guid_, 16);
        request.header.request_id.sequence_number = ++sequence_number_;
        request.header.instance_name = service_name_;
        return request.header.request_id;
    }

    /**
     * @brief 响应接收循环
     */
    void receive_loop() {
        while (running_) {
            dds_attach_t triggered;
            dds_return_t rc = dds_waitset_wait(waitset_, &triggered, 1, DDS_MSECS(100));

            if (rc > 0) {
                process_replies();
            }
        }
    }

    /**
     * @brief 处理接收到的响应
     */
    void process_replies() {
        void* samples[16];
        dds_sample_info_t infos[16];
        std::memset(samples, 0, sizeof(samples));

        int32_t n = dds_take(reply_reader_, samples, infos, 16, 16);

        for (int32_t i = 0; i < n; ++i) {
            if (infos[i].valid_data) {
                TReply* reply = static_cast<TReply*>(samples[i]);

                ReplyCallback callback;
                {
                    std::lock_guard<std::mutex> lock(pending_mutex_);
                    auto it = pending_requests_.find(reply->header.related_request_id);
                    if (it != pending_requests_.end()) {
                        callback = std::move(it->second);
                        pending_requests_.erase(it);
                    }
                }

                if (callback) {
                    callback(*reply, reply->header.remote_ex);
                }
            }
        }

        if (n > 0) {
            dds_return_loan(reply_reader_, samples, n);
        }
    }

    // 成员变量
    dds_entity_t participant_;
    std::string service_name_;
    uint8_t client_guid_[16];
    std::atomic<int64_t> sequence_number_;
    std::atomic<bool> running_;

    dds_entity_t request_topic_;
    dds_entity_t request_writer_;
    dds_entity_t reply_topic_;
    dds_entity_t reply_reader_;
    dds_entity_t waitset_;
    dds_entity_t read_condition_;

    std::thread receive_thread_;
    mutable std::mutex pending_mutex_;
    std::unordered_map<SampleIdentity, ReplyCallback, SampleIdentityHash> pending_requests_;
};

} // namespace dds_rpc
```

### 4.4 Replier 实现 (replier.hpp)

```cpp
// include/dds_rpc/replier.hpp
#pragma once

#include "rpc_types.hpp"
#include <thread>
#include <atomic>
#include <functional>

namespace dds_rpc {

/**
 * @brief 响应者模板类
 *
 * 用于接收请求并发送响应的服务端组件
 *
 * @tparam TRequest 请求类型 (必须包含符合规范的 header 成员)
 * @tparam TReply 响应类型 (必须包含符合规范的 header 成员)
 *
 * 类型约束:
 * - TRequest 必须有 header.request_id (SampleIdentity)
 * - TReply 必须有 header.related_request_id (SampleIdentity)
 * - TReply 必须有 header.remote_ex (RemoteExceptionCode)
 */
template<typename TRequest, typename TReply>
class Replier {
    // =========== 编译时类型检查 ===========
    DDS_RPC_STATIC_ASSERT_TYPES(TRequest, TReply);

public:
    using RequestType = TRequest;
    using ReplyType = TReply;

    /**
     * @brief 请求处理器类型
     *
     * 返回响应对象，header 会被自动填充
     * 抛出异常会被捕获并转换为错误响应
     */
    using RequestHandler = std::function<TReply(const TRequest&)>;

    /**
     * @brief 高级请求处理器类型
     *
     * 可以直接设置响应头的异常码
     */
    using AdvancedRequestHandler = std::function<void(const TRequest&, TReply&)>;

    /**
     * @brief 构造函数
     */
    Replier(dds_entity_t participant,
            const std::string& service_name,
            const dds_topic_descriptor_t* request_desc,
            const dds_topic_descriptor_t* reply_desc,
            const RpcQosConfig& qos_config = RpcQosConfig())
        : participant_(participant)
        , service_name_(service_name)
        , running_(false)
    {
        // 创建请求 Topic
        std::string req_topic_name = request_topic_name(service_name);
        request_topic_ = dds_create_topic(
            participant, request_desc, req_topic_name.c_str(), nullptr, nullptr);
        if (request_topic_ < 0) {
            throw RpcException("Failed to create request topic");
        }

        // 创建请求 Reader
        dds_qos_t* reader_qos = qos_config.create_reader_qos();
        request_reader_ = dds_create_reader(participant, request_topic_, reader_qos, nullptr);
        dds_delete_qos(reader_qos);
        if (request_reader_ < 0) {
            throw RpcException("Failed to create request reader");
        }

        // 创建响应 Topic
        std::string rep_topic_name = reply_topic_name(service_name);
        reply_topic_ = dds_create_topic(
            participant, reply_desc, rep_topic_name.c_str(), nullptr, nullptr);
        if (reply_topic_ < 0) {
            throw RpcException("Failed to create reply topic");
        }

        // 创建响应 Writer
        dds_qos_t* writer_qos = qos_config.create_writer_qos();
        reply_writer_ = dds_create_writer(participant, reply_topic_, writer_qos, nullptr);
        dds_delete_qos(writer_qos);
        if (reply_writer_ < 0) {
            throw RpcException("Failed to create reply writer");
        }

        // 创建 WaitSet
        waitset_ = dds_create_waitset(participant);
        read_condition_ = dds_create_readcondition(request_reader_, DDS_DATA_AVAILABLE);
        dds_waitset_attach(waitset_, read_condition_, 0);
    }

    /**
     * @brief 析构函数
     */
    ~Replier() {
        stop();

        dds_waitset_detach(waitset_, read_condition_);
        dds_delete(read_condition_);
        dds_delete(waitset_);
        dds_delete(reply_writer_);
        dds_delete(request_reader_);
        dds_delete(reply_topic_);
        dds_delete(request_topic_);
    }

    // 禁用拷贝
    Replier(const Replier&) = delete;
    Replier& operator=(const Replier&) = delete;

    /**
     * @brief 获取服务名称
     */
    const std::string& service_name() const { return service_name_; }

    /**
     * @brief 设置请求处理器
     */
    void set_handler(RequestHandler handler) {
        handler_ = std::move(handler);
    }

    /**
     * @brief 设置高级请求处理器
     */
    void set_advanced_handler(AdvancedRequestHandler handler) {
        advanced_handler_ = std::move(handler);
    }

    /**
     * @brief 启动服务 (在新线程中运行)
     */
    void start() {
        if (running_) return;

        running_ = true;
        worker_thread_ = std::thread(&Replier::run_loop, this);
    }

    /**
     * @brief 停止服务
     */
    void stop() {
        running_ = false;
        if (worker_thread_.joinable()) {
            worker_thread_.join();
        }
    }

    /**
     * @brief 检查服务是否运行中
     */
    bool is_running() const { return running_; }

    /**
     * @brief 处理单个请求 (用于手动控制)
     *
     * @param timeout 等待超时
     * @return 是否处理了请求
     */
    bool process_one(std::chrono::milliseconds timeout = std::chrono::milliseconds(100)) {
        dds_attach_t triggered;
        dds_return_t rc = dds_waitset_wait(
            waitset_, &triggered, 1,
            static_cast<dds_duration_t>(timeout.count()) * DDS_NSECS_IN_MSEC);

        if (rc > 0) {
            return take_and_process();
        }
        return false;
    }

    /**
     * @brief 在当前线程中运行服务
     *
     * 阻塞直到 stop() 被调用
     */
    void run() {
        running_ = true;
        run_loop();
    }

    /**
     * @brief 获取已处理的请求数
     */
    uint64_t processed_count() const { return processed_count_; }

private:
    /**
     * @brief 主循环
     */
    void run_loop() {
        while (running_) {
            process_one(std::chrono::milliseconds(100));
        }
    }

    /**
     * @brief 取出并处理请求
     */
    bool take_and_process() {
        void* samples[1];
        dds_sample_info_t infos[1];
        samples[0] = nullptr;

        int32_t n = dds_take(request_reader_, samples, infos, 1, 1);

        if (n > 0 && infos[0].valid_data) {
            TRequest* request = static_cast<TRequest*>(samples[0]);

            TReply reply{};
            reply.header.related_request_id = request->header.request_id;
            reply.header.remote_ex = RemoteExceptionCode::OK;

            try {
                if (advanced_handler_) {
                    advanced_handler_(*request, reply);
                } else if (handler_) {
                    reply = handler_(*request);
                    // 确保响应头正确设置
                    reply.header.related_request_id = request->header.request_id;
                    reply.header.remote_ex = RemoteExceptionCode::OK;
                } else {
                    reply.header.remote_ex = RemoteExceptionCode::UNKNOWN_OPERATION;
                }
            } catch (const RpcException& e) {
                reply.header.remote_ex = e.code();
            } catch (const std::invalid_argument&) {
                reply.header.remote_ex = RemoteExceptionCode::INVALID_ARGUMENT;
            } catch (...) {
                reply.header.remote_ex = RemoteExceptionCode::UNKNOWN_EXCEPTION;
            }

            // 发送响应
            dds_write(reply_writer_, &reply);
            ++processed_count_;

            dds_return_loan(request_reader_, samples, n);
            return true;
        }

        return false;
    }

    // 成员变量
    dds_entity_t participant_;
    std::string service_name_;
    std::atomic<bool> running_;
    std::atomic<uint64_t> processed_count_{0};

    dds_entity_t request_topic_;
    dds_entity_t request_reader_;
    dds_entity_t reply_topic_;
    dds_entity_t reply_writer_;
    dds_entity_t waitset_;
    dds_entity_t read_condition_;

    std::thread worker_thread_;
    RequestHandler handler_;
    AdvancedRequestHandler advanced_handler_;
};

} // namespace dds_rpc
```

### 4.5 便捷服务包装 (service.hpp)

```cpp
// include/dds_rpc/service.hpp
#pragma once

#include "requester.hpp"
#include "replier.hpp"
#include <memory>

namespace dds_rpc {

/**
 * @brief 服务客户端包装类
 *
 * 提供更简洁的 API，自动管理 DDS 实体生命周期
 */
template<typename TRequest, typename TReply>
class ServiceClient {
public:
    ServiceClient(const std::string& service_name,
                  const dds_topic_descriptor_t* request_desc,
                  const dds_topic_descriptor_t* reply_desc,
                  int domain_id = 0)
        : domain_id_(domain_id)
    {
        participant_ = dds_create_participant(domain_id, nullptr, nullptr);
        if (participant_ < 0) {
            throw RpcException("Failed to create participant");
        }

        requester_ = std::make_unique<Requester<TRequest, TReply>>(
            participant_, service_name, request_desc, reply_desc);
    }

    ~ServiceClient() {
        requester_.reset();
        dds_delete(participant_);
    }

    /**
     * @brief 调用服务 (同步)
     */
    TReply call(TRequest request,
                std::chrono::milliseconds timeout = std::chrono::seconds(5)) {
        return requester_->send_request(std::move(request), timeout);
    }

    /**
     * @brief 调用服务 (异步)
     */
    std::future<TReply> call_async(TRequest request,
                std::chrono::milliseconds timeout = std::chrono::seconds(5)) {
        return requester_->send_request_future(std::move(request), timeout);
    }

    /**
     * @brief 等待服务可用
     */
    bool wait_for_service(std::chrono::milliseconds timeout = std::chrono::seconds(10)) {
        return requester_->wait_for_service(timeout);
    }

    Requester<TRequest, TReply>& requester() { return *requester_; }

private:
    int domain_id_;
    dds_entity_t participant_;
    std::unique_ptr<Requester<TRequest, TReply>> requester_;
};

/**
 * @brief 服务端包装类
 */
template<typename TRequest, typename TReply>
class ServiceServer {
public:
    using Handler = typename Replier<TRequest, TReply>::RequestHandler;

    ServiceServer(const std::string& service_name,
                  const dds_topic_descriptor_t* request_desc,
                  const dds_topic_descriptor_t* reply_desc,
                  Handler handler,
                  int domain_id = 0)
        : domain_id_(domain_id)
    {
        participant_ = dds_create_participant(domain_id, nullptr, nullptr);
        if (participant_ < 0) {
            throw RpcException("Failed to create participant");
        }

        replier_ = std::make_unique<Replier<TRequest, TReply>>(
            participant_, service_name, request_desc, reply_desc);
        replier_->set_handler(std::move(handler));
    }

    ~ServiceServer() {
        replier_.reset();
        dds_delete(participant_);
    }

    void start() { replier_->start(); }
    void stop() { replier_->stop(); }
    void run() { replier_->run(); }
    bool is_running() const { return replier_->is_running(); }

    Replier<TRequest, TReply>& replier() { return *replier_; }

private:
    int domain_id_;
    dds_entity_t participant_;
    std::unique_ptr<Replier<TRequest, TReply>> replier_;
};

} // namespace dds_rpc
```

## 5. 使用示例

### 5.1 计算器服务 - 服务端

```cpp
// examples/calculator_server.cpp
#include <dds_rpc/service.hpp>
#include <iostream>
#include <signal.h>

// 假设 IDL 生成的头文件
#include "CalculatorRequest.h"
#include "CalculatorReply.h"

// 全局运行标志
static volatile sig_atomic_t running = 1;

void signal_handler(int sig) {
    (void)sig;
    running = 0;
}

int main() {
    std::cout << "Calculator Service Starting..." << std::endl;

    // 设置信号处理
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    try {
        // 创建 Participant
        dds_entity_t participant = dds_create_participant(0, nullptr, nullptr);
        if (participant < 0) {
            std::cerr << "Failed to create participant" << std::endl;
            return 1;
        }

        // 创建 Replier
        dds_rpc::Replier<example_CalculatorRequest, example_CalculatorReply> replier(
            participant,
            "CalculatorService",
            &example_CalculatorRequest_desc,
            &example_CalculatorReply_desc
        );

        // 设置请求处理器
        replier.set_handler([](const example_CalculatorRequest& req) {
            example_CalculatorReply reply{};

            std::cout << "Received request: " << req.operand_a
                      << " " << req.operation
                      << " " << req.operand_b << std::endl;

            if (std::strcmp(req.operation, "add") == 0) {
                reply.result = req.operand_a + req.operand_b;
            } else if (std::strcmp(req.operation, "subtract") == 0) {
                reply.result = req.operand_a - req.operand_b;
            } else if (std::strcmp(req.operation, "multiply") == 0) {
                reply.result = req.operand_a * req.operand_b;
            } else if (std::strcmp(req.operation, "divide") == 0) {
                if (req.operand_b == 0) {
                    throw dds_rpc::RpcException("Division by zero",
                        dds_rpc::RemoteExceptionCode::INVALID_ARGUMENT);
                }
                reply.result = req.operand_a / req.operand_b;
            } else {
                throw dds_rpc::RpcException("Unknown operation",
                    dds_rpc::RemoteExceptionCode::UNKNOWN_OPERATION);
            }

            reply.error_message = "";
            std::cout << "Sending reply: " << reply.result << std::endl;

            return reply;
        });

        // 启动服务
        std::cout << "Calculator Service Ready. Press Ctrl+C to exit." << std::endl;

        // 在主线程运行
        while (running) {
            replier.process_one(std::chrono::milliseconds(100));
        }

        std::cout << "\nShutting down..." << std::endl;
        std::cout << "Processed " << replier.processed_count() << " requests." << std::endl;

        dds_delete(participant);

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
```

### 5.2 计算器服务 - 客户端

```cpp
// examples/calculator_client.cpp
#include <dds_rpc/service.hpp>
#include <iostream>
#include <string>

#include "CalculatorRequest.h"
#include "CalculatorReply.h"

int main() {
    std::cout << "Calculator Client Starting..." << std::endl;

    try {
        // 创建 Participant
        dds_entity_t participant = dds_create_participant(0, nullptr, nullptr);
        if (participant < 0) {
            std::cerr << "Failed to create participant" << std::endl;
            return 1;
        }

        // 创建 Requester
        dds_rpc::Requester<example_CalculatorRequest, example_CalculatorReply> requester(
            participant,
            "CalculatorService",
            &example_CalculatorRequest_desc,
            &example_CalculatorReply_desc
        );

        // 等待服务可用
        std::cout << "Waiting for service..." << std::endl;
        if (!requester.wait_for_service(std::chrono::seconds(5))) {
            std::cerr << "Service not available" << std::endl;
            dds_delete(participant);
            return 1;
        }
        std::cout << "Service found!" << std::endl;

        // 测试用例
        struct TestCase {
            int32_t a;
            int32_t b;
            const char* op;
        };

        TestCase tests[] = {
            {10, 5, "add"},
            {10, 5, "subtract"},
            {10, 5, "multiply"},
            {10, 5, "divide"},
            {10, 0, "divide"},  // 测试除零错误
        };

        for (const auto& test : tests) {
            example_CalculatorRequest request{};
            request.operand_a = test.a;
            request.operand_b = test.b;
            request.operation = const_cast<char*>(test.op);

            std::cout << "\nCalling: " << test.a << " " << test.op << " " << test.b << std::endl;

            try {
                auto reply = requester.send_request(request, std::chrono::seconds(5));
                std::cout << "Result: " << reply.result << std::endl;
            } catch (const dds_rpc::TimeoutException&) {
                std::cout << "Request timed out!" << std::endl;
            } catch (const dds_rpc::RpcException& e) {
                std::cout << "RPC Error: " << e.what() << std::endl;
            }
        }

        // 异步调用示例
        std::cout << "\n--- Async Call Example ---" << std::endl;

        example_CalculatorRequest async_req{};
        async_req.operand_a = 100;
        async_req.operand_b = 25;
        async_req.operation = const_cast<char*>("add");

        auto future = requester.send_request_future(async_req);

        std::cout << "Request sent, doing other work..." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        try {
            auto reply = future.get();
            std::cout << "Async result: " << reply.result << std::endl;
        } catch (const std::exception& e) {
            std::cout << "Async error: " << e.what() << std::endl;
        }

        dds_delete(participant);

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    std::cout << "\nClient finished." << std::endl;
    return 0;
}
```

### 5.3 使用便捷包装类

```cpp
// examples/simple_example.cpp
#include <dds_rpc/service.hpp>
#include <iostream>

#include "CalculatorRequest.h"
#include "CalculatorReply.h"

// 服务端
void run_server() {
    dds_rpc::ServiceServer<example_CalculatorRequest, example_CalculatorReply> server(
        "Calculator",
        &example_CalculatorRequest_desc,
        &example_CalculatorReply_desc,
        [](const example_CalculatorRequest& req) {
            example_CalculatorReply reply{};
            reply.result = req.operand_a + req.operand_b;
            return reply;
        }
    );

    server.run();  // 阻塞运行
}

// 客户端
void run_client() {
    dds_rpc::ServiceClient<example_CalculatorRequest, example_CalculatorReply> client(
        "Calculator",
        &example_CalculatorRequest_desc,
        &example_CalculatorReply_desc
    );

    client.wait_for_service();

    example_CalculatorRequest req{};
    req.operand_a = 10;
    req.operand_b = 20;

    auto reply = client.call(req);
    std::cout << "Result: " << reply.result << std::endl;
}
```

## 6. 构建系统

### 6.1 CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.16)
project(cyclonedds_rpc VERSION 1.0.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找 CycloneDDS
find_package(CycloneDDS REQUIRED)

# IDL 编译
set(IDL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/idl)
set(IDL_FILES
    ${IDL_DIR}/rpc_types.idl
    ${IDL_DIR}/calculator_service.idl
)

# 生成的源文件
set(GENERATED_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated)
file(MAKE_DIRECTORY ${GENERATED_DIR})

foreach(IDL_FILE ${IDL_FILES})
    get_filename_component(IDL_NAME ${IDL_FILE} NAME_WE)
    set(GENERATED_C ${GENERATED_DIR}/${IDL_NAME}.c)
    set(GENERATED_H ${GENERATED_DIR}/${IDL_NAME}.h)

    add_custom_command(
        OUTPUT ${GENERATED_C} ${GENERATED_H}
        COMMAND CycloneDDS::idlc -l c -o ${GENERATED_DIR} ${IDL_FILE}
        DEPENDS ${IDL_FILE}
        COMMENT "Generating code from ${IDL_NAME}.idl"
    )

    list(APPEND GENERATED_SOURCES ${GENERATED_C})
    list(APPEND GENERATED_HEADERS ${GENERATED_H})
endforeach()

# 生成的代码库
add_library(rpc_idl STATIC ${GENERATED_SOURCES})
target_include_directories(rpc_idl PUBLIC ${GENERATED_DIR})
target_link_libraries(rpc_idl PUBLIC CycloneDDS::ddsc)

# RPC 头文件库 (header-only)
add_library(dds_rpc INTERFACE)
target_include_directories(dds_rpc INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(dds_rpc INTERFACE
    CycloneDDS::ddsc
    pthread
)

# 示例: 服务端
add_executable(calculator_server examples/calculator_server.cpp)
target_link_libraries(calculator_server PRIVATE dds_rpc rpc_idl)

# 示例: 客户端
add_executable(calculator_client examples/calculator_client.cpp)
target_link_libraries(calculator_client PRIVATE dds_rpc rpc_idl)

# 测试
enable_testing()
add_executable(test_rpc tests/test_rpc.cpp)
target_link_libraries(test_rpc PRIVATE dds_rpc rpc_idl)
add_test(NAME test_rpc COMMAND test_rpc)

# 安装
install(DIRECTORY include/dds_rpc DESTINATION include)
install(TARGETS dds_rpc EXPORT dds_rpc-targets)
install(EXPORT dds_rpc-targets
    FILE dds_rpc-config.cmake
    DESTINATION lib/cmake/dds_rpc
)
```

### 6.2 构建和运行

```bash
# 创建构建目录
mkdir build && cd build

# 配置
cmake .. -DCMAKE_BUILD_TYPE=Release

# 构建
cmake --build . -j$(nproc)

# 运行服务端 (终端1)
./calculator_server

# 运行客户端 (终端2)
./calculator_client
```

## 7. 类型约束检查

### 7.1 概述

为确保用户定义的 Request/Reply 类型符合 DDS-RPC 规范，框架在编译时进行静态类型检查。如果类型不满足要求，编译器会给出清晰的错误信息。

### 7.2 类型要求

**Request 类型必须包含:**
```cpp
struct MyRequest {
    RequestHeader header;  // 必须: 包含 request_id
    // ... 业务数据
};
```

**Reply 类型必须包含:**
```cpp
struct MyReply {
    ReplyHeader header;    // 必须: 包含 related_request_id 和 remote_ex
    // ... 业务数据
};
```

### 7.3 编译错误示例

**错误示例 1: 缺少 header 成员**
```cpp
// 错误的定义 - 没有 header
struct BadRequest {
    int32_t data;
};

// 编译时会报错:
// error: static assertion failed: TRequest must have a 'header' member
Requester<BadRequest, MyReply> requester(...);
```

**错误示例 2: header 缺少必要字段**
```cpp
// 错误的定义 - header 格式不对
struct BadRequest {
    struct {
        int id;  // 错误: 应该是 SampleIdentity request_id
    } header;
};

// 编译时会报错:
// error: static assertion failed: TRequest::header must have a 'request_id' member of type SampleIdentity
```

**错误示例 3: Reply header 缺少 remote_ex**
```cpp
struct BadReply {
    struct {
        SampleIdentity related_request_id;
        // 缺少 remote_ex
    } header;
};

// 编译时会报错:
// error: static assertion failed: TReply::header must have a 'remote_ex' member of type RemoteExceptionCode
```

### 7.4 正确的类型定义

```cpp
// 方式1: 直接使用框架提供的 Header 类型
struct MyRequest {
    dds_rpc::RequestHeader header;  // 使用标准 header
    std::string payload;
};

struct MyReply {
    dds_rpc::ReplyHeader header;    // 使用标准 header
    int32_t result;
};

// 方式2: 自定义兼容结构 (必须包含必要字段)
struct CustomRequest {
    struct {
        dds_rpc::SampleIdentity request_id;      // 必须
        std::string instance_name;               // 可选
        // 可以添加其他自定义字段
    } header;
    // 业务数据...
};

struct CustomReply {
    struct {
        dds_rpc::SampleIdentity related_request_id;  // 必须
        dds_rpc::RemoteExceptionCode remote_ex;      // 必须
        // 可以添加其他自定义字段
    } header;
    // 业务数据...
};
```

### 7.5 手动类型验证

如果需要在运行时或其他场景验证类型，可以使用 traits:

```cpp
#include <dds_rpc/rpc_types.hpp>

// 编译时检查
if constexpr (dds_rpc::traits::is_valid_request_v<MyRequest>) {
    // 类型有效
}

// 用于 SFINAE
template<typename T>
std::enable_if_t<dds_rpc::traits::is_valid_request_v<T>, void>
process_request(const T& req) {
    // 只接受有效的 Request 类型
}

// C++20 概念 (如果支持)
template<typename T>
concept ValidRequest = dds_rpc::traits::is_valid_request_v<T>;

template<ValidRequest T>
void handle(const T& req) { /* ... */ }
```

## 8. 高级特性

### 8.1 内容过滤优化

使用 ContentFilteredTopic 减少客户端接收的无关响应：

```cpp
// 创建内容过滤 Topic (按客户端 GUID 过滤)
std::string filter_expression =
    "header.related_request_id.writer_guid = '" + guid_to_string(client_guid_) + "'";

dds_entity_t filtered_topic = dds_create_topic_filtered(
    participant,
    reply_topic,
    "ClientFilter",
    filter_expression.c_str()
);
```

### 8.2 多实例服务

支持同一服务的多个实例：

```cpp
// 服务端注册实例名
replier.set_instance_name("instance_1");

// 客户端指定实例
request.header.instance_name = "instance_1";
```

### 8.3 负载均衡

```cpp
// 轮询多个服务实例
class LoadBalancedRequester {
    std::vector<std::unique_ptr<Requester<Req, Rep>>> requesters_;
    std::atomic<size_t> next_index_{0};

public:
    TReply send_request(TRequest request) {
        size_t index = next_index_.fetch_add(1) % requesters_.size();
        return requesters_[index]->send_request(std::move(request));
    }
};
```

### 8.4 请求优先级

```cpp
// 使用 DDS QoS 设置优先级
dds_qos_t* qos = dds_create_qos();
dds_qset_transport_priority(qos, 10);  // 高优先级
```

## 9. 性能优化

### 9.1 批量处理

```cpp
// 服务端批量处理请求
bool take_and_process_batch(int max_batch = 16) {
    void* samples[16];
    dds_sample_info_t infos[16];

    int32_t n = dds_take(reader, samples, infos, max_batch, max_batch);

    for (int32_t i = 0; i < n; ++i) {
        // 处理并发送响应
    }

    dds_return_loan(reader, samples, n);
    return n > 0;
}
```

### 9.2 零拷贝 (Shared Memory)

```cpp
// 配置共享内存传输 (cyclonedds.xml)
/*
<CycloneDDS>
  <Domain>
    <SharedMemory>
      <Enable>true</Enable>
      <LogLevel>info</LogLevel>
    </SharedMemory>
  </Domain>
</CycloneDDS>
*/

// 使用 loan 机制避免拷贝
void* sample;
dds_loan_sample(writer, &sample);
// 填充数据
dds_write(writer, sample);
```

### 9.3 线程池处理

```cpp
#include <thread>
#include <vector>
#include <queue>
#include <condition_variable>

class ThreadPoolReplier {
    std::vector<std::thread> workers_;
    std::queue<TRequest> request_queue_;
    std::mutex queue_mutex_;
    std::condition_variable cv_;

public:
    void start(int num_threads = 4) {
        for (int i = 0; i < num_threads; ++i) {
            workers_.emplace_back([this] { worker_loop(); });
        }
    }

private:
    void worker_loop() {
        while (running_) {
            TRequest request;
            {
                std::unique_lock<std::mutex> lock(queue_mutex_);
                cv_.wait(lock, [this] {
                    return !request_queue_.empty() || !running_;
                });
                if (!running_) break;
                request = std::move(request_queue_.front());
                request_queue_.pop();
            }
            process_request(request);
        }
    }
};
```

## 10. 错误处理最佳实践

### 10.1 客户端错误处理

```cpp
try {
    auto reply = requester.send_request(request, std::chrono::seconds(5));

    // 检查远程异常
    if (reply.header.remote_ex != RemoteExceptionCode::OK) {
        handle_remote_error(reply.header.remote_ex);
    }

} catch (const TimeoutException& e) {
    // 超时重试或降级
    retry_or_fallback();

} catch (const RpcException& e) {
    // 记录错误
    log_error(e.what(), e.code());
}
```

### 10.2 服务端错误处理

```cpp
replier.set_handler([](const TRequest& req) {
    TReply reply{};

    try {
        // 验证输入
        if (!validate_request(req)) {
            throw RpcException("Invalid request",
                RemoteExceptionCode::INVALID_ARGUMENT);
        }

        // 业务逻辑
        reply = process_business_logic(req);

    } catch (const ResourceException& e) {
        throw RpcException(e.what(),
            RemoteExceptionCode::OUT_OF_RESOURCES);
    }

    return reply;
});
```

## 11. 与 ROS 2 对比

| 特性 | 本实现 | ROS 2 Service |
|------|--------|---------------|
| DDS 依赖 | CycloneDDS | 多种 DDS (可选) |
| 消息定义 | IDL | .srv 文件 |
| 服务发现 | DDS 内置 | ROS 2 Graph |
| 生命周期 | 手动管理 | Node 管理 |
| 调试工具 | DDS 工具 | ros2 service |
| 跨语言 | C/C++ | C++/Python/等 |
| 额外依赖 | 无 | ROS 2 运行时 |

## 12. 总结

本文档详细介绍了基于 CycloneDDS 实现 DDS-RPC 规范的完整方案，包括：

1. **架构设计**: 遵循 DDS-RPC 规范的请求-响应模式
2. **核心组件**: Requester 和 Replier 模板类
3. **IDL 定义**: 标准化的请求/响应头结构
4. **便捷封装**: ServiceClient 和 ServiceServer 包装类
5. **性能优化**: 批量处理、零拷贝、线程池
6. **错误处理**: 完善的异常机制

该实现可直接用于生产环境，也可作为更复杂 RPC 框架的基础。

## 附录 A: 项目集成路径

本框架应集成到以下位置：

```
src/lejusdk/lejusdk-comm-dds/lejusdk-dds-rpc/
```

与现有 DDS 通信模块保持一致的结构：

```
src/lejusdk/lejusdk-comm-dds/
├── lejusdk-dds-idl/        # IDL 定义
├── lejusdk-topic-pubsub/   # 发布订阅
└── lejusdk-dds-rpc/        # RPC 服务 (本框架)
    ├── include/
    │   └── dds_rpc/
    │       ├── rpc_types.hpp
    │       ├── requester.hpp
    │       ├── replier.hpp
    │       └── service.hpp
    ├── idl/
    ├── examples/
    └── CMakeLists.txt
```

## 附录 B: 完整头文件

所有头文件位于 [include/dds_rpc/](./include/dds_rpc/) 目录：

```
include/dds_rpc/
├── rpc_types.hpp    # 基础类型 + 类型约束检查
├── requester.hpp    # 请求者实现
├── replier.hpp      # 响应者实现
└── service.hpp      # 便捷包装
```

## 附录 C: 参考资料

1. OMG DDS Specification: https://www.omg.org/spec/DDS/
2. OMG DDS-RPC Specification: https://www.omg.org/spec/DDS-RPC/
3. CycloneDDS Documentation: https://cyclonedds.io/docs/
4. Eclipse Cyclone DDS GitHub: https://github.com/eclipse-cyclonedds/cyclonedds
