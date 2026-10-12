/**
 * @file test_end_to_end.cpp
 * @brief 端到端集成测试
 *
 * 测试完整的 RPC 调用流程，包括客户端和服务端的协作。
 */

#include <gtest/gtest.h>
#include "test_helpers.hpp"

using namespace dds_rpc;
using namespace dds_rpc_test;
using namespace example;

// 定义类型别名避免宏解析问题
using CalcRequester = Requester<CalculatorRequest, CalculatorReply>;
using CalcReplier = Replier<CalculatorRequest, CalculatorReply>;

//=============================================================================
// 基本端到端测试
//=============================================================================

class EndToEndTest : public EndToEndFixture {};

TEST_F(EndToEndTest, SyncCall_Add) {
    // 创建服务端
    CalcReplier replier(participant1_, service_name_);
    replier.set_handler(CalculatorHandler::handle);
    replier.start();

    // 创建客户端
    CalcRequester requester(participant2_, service_name_);
    ASSERT_TRUE(requester.wait_for_service(std::chrono::seconds(5)));

    // 发送加法请求
    CalculatorRequest request{};
    request.operand_a(10);
    request.operand_b(5);
    request.operation("add");

    CalculatorReply reply = requester.send_request(request);

    EXPECT_EQ(reply.result(), 15);
    EXPECT_EQ(reply.header().remote_ex(), RemoteExceptionCode::REMOTE_EX_OK);

    replier.stop();
}

TEST_F(EndToEndTest, SyncCall_Subtract) {
    CalcReplier replier(participant1_, service_name_);
    replier.set_handler(CalculatorHandler::handle);
    replier.start();

    CalcRequester requester(participant2_, service_name_);
    ASSERT_TRUE(requester.wait_for_service(std::chrono::seconds(5)));

    CalculatorRequest request{};
    request.operand_a(20);
    request.operand_b(8);
    request.operation("subtract");

    CalculatorReply reply = requester.send_request(request);

    EXPECT_EQ(reply.result(), 12);
    EXPECT_EQ(reply.header().remote_ex(), RemoteExceptionCode::REMOTE_EX_OK);

    replier.stop();
}

TEST_F(EndToEndTest, SyncCall_Multiply) {
    CalcReplier replier(participant1_, service_name_);
    replier.set_handler(CalculatorHandler::handle);
    replier.start();

    CalcRequester requester(participant2_, service_name_);
    ASSERT_TRUE(requester.wait_for_service(std::chrono::seconds(5)));

    CalculatorRequest request{};
    request.operand_a(7);
    request.operand_b(6);
    request.operation("multiply");

    CalculatorReply reply = requester.send_request(request);

    EXPECT_EQ(reply.result(), 42);
    EXPECT_EQ(reply.header().remote_ex(), RemoteExceptionCode::REMOTE_EX_OK);

    replier.stop();
}

TEST_F(EndToEndTest, SyncCall_Divide) {
    CalcReplier replier(participant1_, service_name_);
    replier.set_handler(CalculatorHandler::handle);
    replier.start();

    CalcRequester requester(participant2_, service_name_);
    ASSERT_TRUE(requester.wait_for_service(std::chrono::seconds(5)));

    CalculatorRequest request{};
    request.operand_a(100);
    request.operand_b(4);
    request.operation("divide");

    CalculatorReply reply = requester.send_request(request);

    EXPECT_EQ(reply.result(), 25);
    EXPECT_EQ(reply.header().remote_ex(), RemoteExceptionCode::REMOTE_EX_OK);

    replier.stop();
}

//=============================================================================
// 错误处理测试
//=============================================================================

TEST_F(EndToEndTest, SyncCall_DivideByZero) {
    CalcReplier replier(participant1_, service_name_);
    replier.set_handler(CalculatorHandler::handle);
    replier.start();

    CalcRequester requester(participant2_, service_name_);
    ASSERT_TRUE(requester.wait_for_service(std::chrono::seconds(5)));

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

TEST_F(EndToEndTest, SyncCall_UnknownOperation) {
    CalcReplier replier(participant1_, service_name_);
    replier.set_handler(CalculatorHandler::handle);
    replier.start();

    CalcRequester requester(participant2_, service_name_);
    ASSERT_TRUE(requester.wait_for_service(std::chrono::seconds(5)));

    CalculatorRequest request{};
    request.operand_a(10);
    request.operand_b(5);
    request.operation("modulo");  // 不支持的操作

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
// 顺序多请求测试
//=============================================================================

TEST_F(EndToEndTest, MultipleRequests_Sequential) {
    CalcReplier replier(participant1_, service_name_);
    replier.set_handler(CalculatorHandler::handle);
    replier.start();

    CalcRequester requester(participant2_, service_name_);
    ASSERT_TRUE(requester.wait_for_service(std::chrono::seconds(5)));

    const int NUM_REQUESTS = 10;

    // 顺序发送多个请求
    for (int i = 1; i <= NUM_REQUESTS; ++i) {
        CalculatorRequest request{};
        request.operand_a(i);
        request.operand_b(i);
        request.operation("add");

        CalculatorReply reply = requester.send_request(request);
        EXPECT_EQ(reply.result(), i + i);
    }

    EXPECT_EQ(replier.processed_count(), static_cast<uint64_t>(NUM_REQUESTS));

    replier.stop();
}

//=============================================================================
// 多客户端测试
//=============================================================================

class MultiClientEndToEndTest : public ::testing::Test {
protected:
    void SetUp() override {
        domain_id_ = get_unique_domain_id();
        service_name_ = unique_service_name("MultiClientTest");

        server_participant_ = dds_create_participant(domain_id_, nullptr, nullptr);
        client_participant1_ = dds_create_participant(domain_id_, nullptr, nullptr);
        client_participant2_ = dds_create_participant(domain_id_, nullptr, nullptr);

        ASSERT_GT(server_participant_, 0);
        ASSERT_GT(client_participant1_, 0);
        ASSERT_GT(client_participant2_, 0);
    }

    void TearDown() override {
        if (server_participant_ > 0) dds_delete(server_participant_);
        if (client_participant1_ > 0) dds_delete(client_participant1_);
        if (client_participant2_ > 0) dds_delete(client_participant2_);
    }

    int32_t domain_id_{0};
    dds_entity_t server_participant_{0};
    dds_entity_t client_participant1_{0};
    dds_entity_t client_participant2_{0};
    std::string service_name_;
};

TEST_F(MultiClientEndToEndTest, TwoClients_BothSucceed) {
    // 创建服务端
    CalcReplier replier(server_participant_, service_name_);
    replier.set_handler(CalculatorHandler::handle);
    replier.start();

    // 创建两个客户端
    CalcRequester requester1(client_participant1_, service_name_);
    CalcRequester requester2(client_participant2_, service_name_);

    ASSERT_TRUE(requester1.wait_for_service(std::chrono::seconds(5)));
    ASSERT_TRUE(requester2.wait_for_service(std::chrono::seconds(5)));

    // 客户端 1 发送请求
    CalculatorRequest request1{};
    request1.operand_a(100);
    request1.operand_b(50);
    request1.operation("add");

    CalculatorReply reply1 = requester1.send_request(request1);
    EXPECT_EQ(reply1.result(), 150);

    // 客户端 2 发送请求
    CalculatorRequest request2{};
    request2.operand_a(200);
    request2.operand_b(100);
    request2.operation("subtract");

    CalculatorReply reply2 = requester2.send_request(request2);
    EXPECT_EQ(reply2.result(), 100);

    replier.stop();
}

//=============================================================================
// 边界值测试
//=============================================================================

TEST_F(EndToEndTest, BoundaryValues_LargeNumbers) {
    CalcReplier replier(participant1_, service_name_);
    replier.set_handler(CalculatorHandler::handle);
    replier.start();

    CalcRequester requester(participant2_, service_name_);
    ASSERT_TRUE(requester.wait_for_service(std::chrono::seconds(5)));

    // 测试大数值
    CalculatorRequest request{};
    request.operand_a(INT32_MAX / 2);
    request.operand_b(100);
    request.operation("add");

    CalculatorReply reply = requester.send_request(request);

    EXPECT_EQ(reply.result(), INT32_MAX / 2 + 100);
    EXPECT_EQ(reply.header().remote_ex(), RemoteExceptionCode::REMOTE_EX_OK);

    replier.stop();
}

TEST_F(EndToEndTest, BoundaryValues_NegativeNumbers) {
    CalcReplier replier(participant1_, service_name_);
    replier.set_handler(CalculatorHandler::handle);
    replier.start();

    CalcRequester requester(participant2_, service_name_);
    ASSERT_TRUE(requester.wait_for_service(std::chrono::seconds(5)));

    // 测试负数
    CalculatorRequest request{};
    request.operand_a(-10);
    request.operand_b(-5);
    request.operation("multiply");

    CalculatorReply reply = requester.send_request(request);

    EXPECT_EQ(reply.result(), 50);  // (-10) * (-5) = 50
    EXPECT_EQ(reply.header().remote_ex(), RemoteExceptionCode::REMOTE_EX_OK);

    replier.stop();
}

TEST_F(EndToEndTest, BoundaryValues_Zero) {
    CalcReplier replier(participant1_, service_name_);
    replier.set_handler(CalculatorHandler::handle);
    replier.start();

    CalcRequester requester(participant2_, service_name_);
    ASSERT_TRUE(requester.wait_for_service(std::chrono::seconds(5)));

    // 测试零
    CalculatorRequest request{};
    request.operand_a(0);
    request.operand_b(100);
    request.operation("multiply");

    CalculatorReply reply = requester.send_request(request);

    EXPECT_EQ(reply.result(), 0);
    EXPECT_EQ(reply.header().remote_ex(), RemoteExceptionCode::REMOTE_EX_OK);

    replier.stop();
}

//=============================================================================
// 析构安全测试
//=============================================================================

TEST_F(EndToEndTest, DestructorSafe_NoSegfault) {
    CalcReplier replier(participant1_, service_name_);
    replier.set_handler(CalculatorHandler::handle);
    replier.start();

    {
        // 在作用域内创建客户端
        CalcRequester requester(participant2_, service_name_);
        ASSERT_TRUE(requester.wait_for_service(std::chrono::seconds(5)));

        // 发送一个请求
        CalculatorRequest request{};
        request.operand_a(10);
        request.operand_b(5);
        request.operation("add");

        CalculatorReply reply = requester.send_request(request);
        EXPECT_EQ(reply.result(), 15);

        // requester 在此处析构 - 应该安全，不会段错误
    }

    // 服务端应该仍然正常
    EXPECT_TRUE(replier.is_running());

    replier.stop();
}
