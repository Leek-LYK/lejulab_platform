/**
 * @file calculator_client.cpp
 * @brief 计算器客户端示例
 *
 * 演示如何使用 dds_rpc::Requester 调用 RPC 服务。
 */

#include <dds_rpc/requester.hpp>
#include <iostream>

// IDL 生成的头文件
#include "rpc_idl/calculator_service.hpp"

int main() {
    std::cout << "=== Calculator Client ===" << std::endl;

    try {
        // 创建 DDS Participant
        dds_entity_t participant = dds_create_participant(0, nullptr, nullptr);
        if (participant < 0) {
            std::cerr << "Failed to create participant" << std::endl;
            return 1;
        }

        // 创建 Requester (使用 C++ TopicTraits 自动获取类型信息)
        dds_rpc::Requester<example::CalculatorRequest, example::CalculatorReply> requester(
            participant,
            "CalculatorService"
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
            std::string op;
        };

        TestCase tests[] = {
            {10, 5, "add"},
            {10, 5, "subtract"},
            {10, 5, "multiply"},
            {20, 4, "divide"},
            {10, 0, "divide"},      // 测试除零错误
            {10, 5, "modulo"},      // 测试未知操作
        };

        // 同步调用测试
        std::cout << "\n--- Synchronous Calls ---" << std::endl;
        for (const auto& test : tests) {
            example::CalculatorRequest request{};
            request.operand_a(test.a);
            request.operand_b(test.b);
            request.operation(test.op);

            std::cout << "Calling: " << test.a << " " << test.op << " " << test.b << " = ";

            try {
                auto reply = requester.send_request(request, std::chrono::seconds(5));
                std::cout << reply.result() << std::endl;
            } catch (const dds_rpc::TimeoutException&) {
                std::cout << "TIMEOUT" << std::endl;
            } catch (const dds_rpc::RpcException& e) {
                std::cout << "ERROR: " << e.what() << std::endl;
            }
        }

        // 清理
        dds_delete(participant);

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    std::cout << "\nClient finished." << std::endl;
    return 0;
}
