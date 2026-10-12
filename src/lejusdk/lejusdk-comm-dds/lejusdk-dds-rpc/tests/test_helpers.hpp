/**
 * @file test_helpers.hpp
 * @brief DDS-RPC 单元测试共享工具和 Fixture
 */
#pragma once

#include <gtest/gtest.h>
#include <dds/dds.h>
#include <atomic>
#include <chrono>
#include <string>
#include <thread>

#include "dds_rpc/rpc_types.hpp"
#include "dds_rpc/requester.hpp"
#include "dds_rpc/replier.hpp"
#include "dds_rpc/service.hpp"
#include "rpc_idl/calculator_service.hpp"

namespace dds_rpc_test {

//=============================================================================
// Domain ID 管理 (隔离测试)
//=============================================================================

/**
 * @brief 获取唯一的 Domain ID
 *
 * 每次调用返回不同的 Domain ID，用于隔离不同测试。
 * 范围：100-199
 */
inline int32_t get_unique_domain_id() {
    static std::atomic<int32_t> counter{100};
    int32_t id = counter.fetch_add(1);
    // 循环使用 100-199 范围
    if (id >= 200) {
        counter.store(100);
        id = 100;
    }
    return id;
}

/**
 * @brief 生成唯一的服务名
 *
 * 每次调用返回不同的服务名，用于隔离不同测试。
 */
inline std::string unique_service_name(const std::string& prefix = "TestService") {
    static std::atomic<int32_t> counter{0};
    return prefix + "_" + std::to_string(counter.fetch_add(1));
}

//=============================================================================
// 计算器处理器
//=============================================================================

/**
 * @brief 计算器请求处理器
 */
struct CalculatorHandler {
    /**
     * @brief 处理计算器请求
     */
    static example::CalculatorReply handle(const example::CalculatorRequest& req) {
        example::CalculatorReply reply{};

        const std::string& op = req.operation();
        int32_t a = req.operand_a();
        int32_t b = req.operand_b();

        if (op == "add") {
            reply.result(a + b);
        } else if (op == "subtract" || op == "sub") {
            reply.result(a - b);
        } else if (op == "multiply" || op == "mul") {
            reply.result(a * b);
        } else if (op == "divide" || op == "div") {
            if (b == 0) {
                throw std::invalid_argument("Division by zero");
            }
            reply.result(a / b);
        } else {
            throw dds_rpc::RpcException(
                "Unknown operation: " + op,
                dds_rpc::RemoteExceptionCode::REMOTE_EX_UNKNOWN_OPERATION);
        }

        return reply;
    }
};

//=============================================================================
// Test Fixture 基类
//=============================================================================

/**
 * @brief DDS 基础 Fixture
 *
 * 自动创建和销毁 DDS Participant。
 */
class DdsFixture : public ::testing::Test {
protected:
    void SetUp() override {
        domain_id_ = get_unique_domain_id();
        participant_ = dds_create_participant(domain_id_, nullptr, nullptr);
        ASSERT_GT(participant_, 0) << "Failed to create DDS participant";
    }

    void TearDown() override {
        if (participant_ > 0) {
            dds_delete(participant_);
            participant_ = 0;
        }
    }

    int32_t domain_id_{0};
    dds_entity_t participant_{0};
};

/**
 * @brief 双 Participant Fixture
 *
 * 用于测试两个端点间的通信（如客户端和服务端）。
 */
class DualParticipantFixture : public ::testing::Test {
protected:
    void SetUp() override {
        domain_id_ = get_unique_domain_id();
        participant1_ = dds_create_participant(domain_id_, nullptr, nullptr);
        ASSERT_GT(participant1_, 0) << "Failed to create participant1";

        participant2_ = dds_create_participant(domain_id_, nullptr, nullptr);
        ASSERT_GT(participant2_, 0) << "Failed to create participant2";
    }

    void TearDown() override {
        if (participant1_ > 0) {
            dds_delete(participant1_);
            participant1_ = 0;
        }
        if (participant2_ > 0) {
            dds_delete(participant2_);
            participant2_ = 0;
        }
    }

    int32_t domain_id_{0};
    dds_entity_t participant1_{0};
    dds_entity_t participant2_{0};
};

/**
 * @brief 端到端测试 Fixture
 *
 * 包含完整的客户端和服务端设置。
 */
class EndToEndFixture : public DualParticipantFixture {
protected:
    void SetUp() override {
        DualParticipantFixture::SetUp();
        service_name_ = unique_service_name("E2ETest");
    }

    std::string service_name_;
};

/**
 * @brief 重复服务检测测试 Fixture
 */
class DuplicateServiceFixture : public DualParticipantFixture {
protected:
    void SetUp() override {
        DualParticipantFixture::SetUp();
        // 使用较短的发现等待时间加速测试
        qos_config_.discovery_wait_ms = 200;
    }

    dds_rpc::RpcQosConfig qos_config_;
};

//=============================================================================
// 工具函数
//=============================================================================

/**
 * @brief 等待条件成立
 *
 * @param condition 条件函数
 * @param timeout 超时时间
 * @return 条件是否在超时前成立
 */
template<typename Condition>
bool wait_for(Condition condition, std::chrono::milliseconds timeout) {
    auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        if (condition()) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return false;
}

/**
 * @brief 等待 DDS 发现完成
 *
 * DDS 发现需要一定时间，测试中经常需要等待。
 */
inline void wait_for_discovery(int ms = 300) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

} // namespace dds_rpc_test
