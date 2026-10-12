/**
 * @file test_requester.cpp
 * @brief Requester 类单元测试（纯同步版本）
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
// Requester 构造测试
//=============================================================================

class RequesterTest : public DdsFixture {
protected:
    void SetUp() override {
        DdsFixture::SetUp();
        service_name_ = unique_service_name("RequesterTest");
    }

    std::string service_name_;
};

TEST_F(RequesterTest, Construction_Success) {
    EXPECT_NO_THROW(CalcRequester requester(participant_, service_name_));
}

TEST_F(RequesterTest, ServiceName_Getter) {
    CalcRequester requester(participant_, service_name_);

    EXPECT_EQ(requester.service_name(), service_name_);
}

//=============================================================================
// 等待服务测试
//=============================================================================

TEST_F(RequesterTest, WaitForService_NoService_Timeout) {
    CalcRequester requester(participant_, service_name_);

    // 没有服务端时应该超时
    bool found = requester.wait_for_service(std::chrono::milliseconds(100));
    EXPECT_FALSE(found);
}

TEST_F(RequesterTest, WaitForService_WithService_Success) {
    // 先创建服务端
    CalcReplier replier(participant_, service_name_);
    replier.set_handler(CalculatorHandler::handle);
    replier.start();

    // 然后创建客户端
    CalcRequester requester(participant_, service_name_);

    // 应该能找到服务
    bool found = requester.wait_for_service(std::chrono::seconds(5));
    EXPECT_TRUE(found);

    replier.stop();
}

//=============================================================================
// 超时测试
//=============================================================================

TEST_F(RequesterTest, SendRequest_Timeout) {
    CalcRequester requester(participant_, service_name_);

    CalculatorRequest request{};
    request.operand_a(10);
    request.operand_b(5);
    request.operation("add");

    // 没有服务端时应该超时
    EXPECT_THROW(
        requester.send_request(request, std::chrono::milliseconds(100)),
        TimeoutException);
}

//=============================================================================
// 同步请求测试
//=============================================================================

class RequesterSyncTest : public DualParticipantFixture {
protected:
    void SetUp() override {
        DualParticipantFixture::SetUp();
        service_name_ = unique_service_name("RequesterSyncTest");
    }

    std::string service_name_;
};

TEST_F(RequesterSyncTest, SendRequest_Success) {
    // 创建服务端
    CalcReplier replier(participant1_, service_name_);
    replier.set_handler(CalculatorHandler::handle);
    replier.start();

    // 创建客户端
    CalcRequester requester(participant2_, service_name_);
    ASSERT_TRUE(requester.wait_for_service(std::chrono::seconds(5)));

    // 发送请求
    CalculatorRequest request{};
    request.operand_a(20);
    request.operand_b(10);
    request.operation("add");

    CalculatorReply reply = requester.send_request(request);

    EXPECT_EQ(reply.result(), 30);
    EXPECT_EQ(reply.header().remote_ex(), RemoteExceptionCode::REMOTE_EX_OK);

    replier.stop();
}

TEST_F(RequesterSyncTest, SendRequest_RemoteException) {
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

    // 应该抛出异常
    EXPECT_THROW(requester.send_request(request), RpcException);

    replier.stop();
}

//=============================================================================
// 多请求测试
//=============================================================================

TEST_F(RequesterSyncTest, MultipleRequests_Sequential) {
    // 创建服务端
    CalcReplier replier(participant1_, service_name_);
    replier.set_handler(CalculatorHandler::handle);
    replier.start();

    // 创建客户端
    CalcRequester requester(participant2_, service_name_);
    ASSERT_TRUE(requester.wait_for_service(std::chrono::seconds(5)));

    // 发送多个顺序请求
    for (int i = 1; i <= 5; ++i) {
        CalculatorRequest request{};
        request.operand_a(i);
        request.operand_b(i);
        request.operation("add");

        CalculatorReply reply = requester.send_request(request);
        EXPECT_EQ(reply.result(), i + i);
    }

    replier.stop();
}

//=============================================================================
// 析构安全测试
//=============================================================================

TEST_F(RequesterSyncTest, DestructorSafe_NoSegfault) {
    // 创建服务端
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
