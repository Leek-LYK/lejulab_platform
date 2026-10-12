/**
 * @file test_replier.cpp
 * @brief Replier 类单元测试
 */

#include <gtest/gtest.h>
#include <thread>
#include <atomic>
#include "test_helpers.hpp"

using namespace dds_rpc;
using namespace dds_rpc_test;
using namespace example;

// 定义类型别名避免宏解析问题
using CalcRequester = Requester<CalculatorRequest, CalculatorReply>;
using CalcReplier = Replier<CalculatorRequest, CalculatorReply>;

//=============================================================================
// Replier 构造测试
//=============================================================================

class ReplierTest : public DdsFixture {
protected:
    void SetUp() override {
        DdsFixture::SetUp();
        service_name_ = unique_service_name("ReplierTest");
    }

    std::string service_name_;
};

TEST_F(ReplierTest, Construction_Success) {
    EXPECT_NO_THROW(CalcReplier replier(participant_, service_name_));
}

TEST_F(ReplierTest, ServiceName_Getter) {
    CalcReplier replier(participant_, service_name_);

    EXPECT_EQ(replier.service_name(), service_name_);
}

TEST_F(ReplierTest, IsRunning_InitialFalse) {
    CalcReplier replier(participant_, service_name_);

    EXPECT_FALSE(replier.is_running());
}

TEST_F(ReplierTest, ProcessedCount_Initial) {
    CalcReplier replier(participant_, service_name_);

    EXPECT_EQ(replier.processed_count(), 0u);
}

//=============================================================================
// 生命周期测试
//=============================================================================

TEST_F(ReplierTest, StartStop_Lifecycle) {
    CalcReplier replier(participant_, service_name_);

    // 初始状态
    EXPECT_FALSE(replier.is_running());

    // 启动
    replier.start();
    EXPECT_TRUE(replier.is_running());

    // 停止
    replier.stop();
    EXPECT_FALSE(replier.is_running());
}

TEST_F(ReplierTest, Start_Idempotent) {
    CalcReplier replier(participant_, service_name_);

    // 多次启动应该是安全的
    replier.start();
    EXPECT_TRUE(replier.is_running());

    replier.start();  // 再次启动
    EXPECT_TRUE(replier.is_running());

    replier.stop();
}

TEST_F(ReplierTest, Stop_Idempotent) {
    CalcReplier replier(participant_, service_name_);

    // 多次停止应该是安全的
    replier.stop();
    EXPECT_FALSE(replier.is_running());

    replier.stop();  // 再次停止
    EXPECT_FALSE(replier.is_running());
}

//=============================================================================
// 处理器测试
//=============================================================================

class ReplierHandlerTest : public DualParticipantFixture {
protected:
    void SetUp() override {
        DualParticipantFixture::SetUp();
        service_name_ = unique_service_name("ReplierHandlerTest");
    }

    std::string service_name_;
};

TEST_F(ReplierHandlerTest, NoHandler_ThrowsException) {
    // 创建服务端，但不设置处理器
    CalcReplier replier(participant1_, service_name_);
    replier.start();

    // 创建客户端
    CalcRequester requester(participant2_, service_name_);
    ASSERT_TRUE(requester.wait_for_service(std::chrono::seconds(5)));

    // 发送请求
    CalculatorRequest request{};
    request.operand_a(10);
    request.operand_b(5);
    request.operation("add");

    // 没有处理器时应该抛出异常
    try {
        CalculatorReply reply = requester.send_request(request);
        FAIL() << "Expected RpcException to be thrown";
    } catch (const RpcException& e) {
        EXPECT_EQ(e.code(), RemoteExceptionCode::REMOTE_EX_UNKNOWN_OPERATION);
    }

    replier.stop();
}

TEST_F(ReplierHandlerTest, Handler_ProcessesRequest) {
    // 创建服务端
    CalcReplier replier(participant1_, service_name_);
    replier.set_handler(CalculatorHandler::handle);
    replier.start();

    // 创建客户端
    CalcRequester requester(participant2_, service_name_);
    ASSERT_TRUE(requester.wait_for_service(std::chrono::seconds(5)));

    // 发送请求
    CalculatorRequest request{};
    request.operand_a(10);
    request.operand_b(5);
    request.operation("add");

    CalculatorReply reply = requester.send_request(request);

    EXPECT_EQ(reply.result(), 15);
    EXPECT_EQ(reply.header().remote_ex(), RemoteExceptionCode::REMOTE_EX_OK);

    // 验证处理计数
    EXPECT_EQ(replier.processed_count(), 1u);

    replier.stop();
}

TEST_F(ReplierHandlerTest, HandlerException_InvalidArgument) {
    // 创建服务端
    CalcReplier replier(participant1_, service_name_);
    replier.set_handler(CalculatorHandler::handle);
    replier.start();

    // 创建客户端
    CalcRequester requester(participant2_, service_name_);
    ASSERT_TRUE(requester.wait_for_service(std::chrono::seconds(5)));

    // 发送除零请求
    CalculatorRequest request{};
    request.operand_a(10);
    request.operand_b(0);
    request.operation("divide");

    // 除零应该抛出异常
    try {
        CalculatorReply reply = requester.send_request(request);
        FAIL() << "Expected RpcException to be thrown";
    } catch (const RpcException& e) {
        EXPECT_EQ(e.code(), RemoteExceptionCode::REMOTE_EX_INVALID_ARGUMENT);
    }

    replier.stop();
}

TEST_F(ReplierHandlerTest, HandlerException_RpcException) {
    // 创建服务端
    CalcReplier replier(participant1_, service_name_);
    replier.set_handler(CalculatorHandler::handle);
    replier.start();

    // 创建客户端
    CalcRequester requester(participant2_, service_name_);
    ASSERT_TRUE(requester.wait_for_service(std::chrono::seconds(5)));

    // 发送未知操作请求
    CalculatorRequest request{};
    request.operand_a(10);
    request.operand_b(5);
    request.operation("unknown");

    // 未知操作应该抛出异常
    try {
        CalculatorReply reply = requester.send_request(request);
        FAIL() << "Expected RpcException to be thrown";
    } catch (const RpcException& e) {
        EXPECT_EQ(e.code(), RemoteExceptionCode::REMOTE_EX_UNKNOWN_OPERATION);
    }

    replier.stop();
}

//=============================================================================
// 高级处理器测试
//=============================================================================

TEST_F(ReplierHandlerTest, AdvancedHandler_Works) {
    // 创建服务端
    CalcReplier replier(participant1_, service_name_);

    // 设置高级处理器
    replier.set_advanced_handler([](const CalculatorRequest& req, CalculatorReply& reply) {
        reply.result(req.operand_a() + req.operand_b());
        reply.error_message("OK");
        // header 由框架自动设置
    });
    replier.start();

    // 创建客户端
    CalcRequester requester(participant2_, service_name_);
    ASSERT_TRUE(requester.wait_for_service(std::chrono::seconds(5)));

    // 发送请求
    CalculatorRequest request{};
    request.operand_a(100);
    request.operand_b(50);
    request.operation("add");

    CalculatorReply reply = requester.send_request(request);

    EXPECT_EQ(reply.result(), 150);
    EXPECT_EQ(reply.error_message(), "OK");
    EXPECT_EQ(reply.header().remote_ex(), RemoteExceptionCode::REMOTE_EX_OK);

    replier.stop();
}

//=============================================================================
// ProcessOne 测试
//=============================================================================

TEST_F(ReplierHandlerTest, ProcessOne_NoRequest) {
    CalcReplier replier(participant1_, service_name_);
    replier.set_handler(CalculatorHandler::handle);

    // 没有请求时应该返回 false
    bool processed = replier.process_one(std::chrono::milliseconds(50));
    EXPECT_FALSE(processed);
}

TEST_F(ReplierHandlerTest, ProcessOne_WithRequest) {
    // 创建服务端
    CalcReplier replier(participant1_, service_name_);
    replier.set_handler(CalculatorHandler::handle);
    // 注意：这里不调用 start()，而是手动调用 process_one()

    // 创建客户端
    CalcRequester requester(participant2_, service_name_);

    // 等待发现
    wait_for_discovery(300);

    // 在单独线程发送请求（因为 send_request 是阻塞的）
    std::atomic<bool> request_done{false};
    std::atomic<int32_t> result{0};

    std::thread sender([&]() {
        CalculatorRequest request{};
        request.operand_a(7);
        request.operand_b(3);
        request.operation("subtract");

        try {
            CalculatorReply reply = requester.send_request(request);
            result = reply.result();
            request_done = true;
        } catch (...) {
            request_done = true;
        }
    });

    // 手动处理一个请求
    bool processed = replier.process_one(std::chrono::milliseconds(2000));
    EXPECT_TRUE(processed);

    // 等待发送线程完成
    sender.join();

    EXPECT_TRUE(request_done);
    EXPECT_EQ(result, 4);  // 7 - 3 = 4
}

//=============================================================================
// 多请求处理测试
//=============================================================================

TEST_F(ReplierHandlerTest, ProcessedCount_Increments) {
    // 创建服务端
    CalcReplier replier(participant1_, service_name_);
    replier.set_handler(CalculatorHandler::handle);
    replier.start();

    // 创建客户端
    CalcRequester requester(participant2_, service_name_);
    ASSERT_TRUE(requester.wait_for_service(std::chrono::seconds(5)));

    // 发送多个请求
    for (int i = 0; i < 5; ++i) {
        CalculatorRequest request{};
        request.operand_a(i);
        request.operand_b(1);
        request.operation("add");

        requester.send_request(request);
    }

    // 验证处理计数
    EXPECT_EQ(replier.processed_count(), 5u);

    replier.stop();
}
