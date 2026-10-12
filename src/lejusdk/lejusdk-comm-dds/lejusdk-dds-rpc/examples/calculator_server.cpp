/**
 * @file calculator_server.cpp
 * @brief 计算器服务端示例
 *
 * 演示如何使用 dds_rpc::Replier 实现一个 RPC 服务端。
 */

#include <dds_rpc/replier.hpp>
#include <iostream>
#include <csignal>

// IDL 生成的头文件
#include "rpc_idl/calculator_service.hpp"

// 全局运行标志
static volatile sig_atomic_t g_running = 1;

void signal_handler(int sig) {
    (void)sig;
    g_running = 0;
}

int main() {
    std::cout << "=== Calculator Service ===" << std::endl;

    // 设置信号处理
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    try {
        // 创建 DDS Participant
        dds_entity_t participant = dds_create_participant(0, nullptr, nullptr);
        if (participant < 0) {
            std::cerr << "Failed to create participant" << std::endl;
            return 1;
        }

        // 创建 Replier (使用 C++ TopicTraits 自动获取类型信息)
        dds_rpc::Replier<example::CalculatorRequest, example::CalculatorReply> replier(
            participant,
            "CalculatorService"
        );

        // 设置请求处理器
        replier.set_handler([](const example::CalculatorRequest& req) {
            example::CalculatorReply reply{};

            std::cout << "Received: " << req.operand_a()
                      << " " << req.operation()
                      << " " << req.operand_b() << std::endl;

            // 根据操作类型计算
            if (req.operation() == "add") {
                reply.result(req.operand_a() + req.operand_b());
            } else if (req.operation() == "subtract") {
                reply.result(req.operand_a() - req.operand_b());
            } else if (req.operation() == "multiply") {
                reply.result(req.operand_a() * req.operand_b());
            } else if (req.operation() == "divide") {
                if (req.operand_b() == 0) {
                    throw dds_rpc::RpcException("Division by zero",
                        dds_rpc::RemoteExceptionCode::REMOTE_EX_INVALID_ARGUMENT);
                }
                reply.result(req.operand_a() / req.operand_b());
            } else {
                throw dds_rpc::RpcException("Unknown operation",
                    dds_rpc::RemoteExceptionCode::REMOTE_EX_UNKNOWN_OPERATION);
            }

            std::cout << "Result: " << reply.result() << std::endl;

            return reply;
        });

        std::cout << "Service ready. Press Ctrl+C to exit." << std::endl;

        // 主循环
        while (g_running) {
            replier.process_one(std::chrono::milliseconds(100));
        }

        std::cout << "\nShutting down..." << std::endl;
        std::cout << "Processed " << replier.processed_count() << " requests." << std::endl;

        // 清理
        dds_delete(participant);

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
