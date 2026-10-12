/**
 * @file rpc_first_call_test.cpp
 * @brief DDS RPC 首次调用延迟测试
 *
 * 模拟正常使用场景：每次创建新的 Requester，wait_for_service，然后发送 RPC。
 * 用于测试首次建立连接的延迟和可靠性。
 *
 * Usage: rpc_first_call_test [options]
 *   --count N         测试次数 (默认: 100)
 *   --timeout MS      超时时间毫秒 (默认: 3000)
 *   --help            显示帮助
 */

#include <dds_rpc/requester.hpp>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <vector>
#include <algorithm>
#include <numeric>
#include <csignal>
#include <string>

#include "rpc_idl/calculator_service.hpp"

volatile bool g_running = true;

void signalHandler(int signum) {
  std::cout << "\nReceived signal " << signum << ", stopping..." << std::endl;
  g_running = false;
}

struct TestResult {
  int index;
  bool success;
  double total_latency_us;      // discovery + RPC
  double discovery_latency_us;
  double rpc_latency_us;
  std::string error;
};

void printUsage(const char* bin) {
  std::cout << "Usage: " << bin << " [options]\n"
            << "Options:\n"
            << "  --count N         Number of test iterations (default: 100)\n"
            << "  --timeout MS      RPC timeout in ms (default: 3000)\n"
            << "  --help            Show this help\n";
}

void printStats(const std::string& name, std::vector<double>& values) {
  if (values.empty()) return;

  std::sort(values.begin(), values.end());
  double sum = std::accumulate(values.begin(), values.end(), 0.0);
  double avg = sum / values.size();
  double min_val = values.front();
  double max_val = values.back();
  double p50 = values[values.size() / 2];
  double p90 = values[static_cast<size_t>(values.size() * 0.9)];
  double p99 = values[static_cast<size_t>(values.size() * 0.99)];

  std::cout << "\n=== " << name << " ===" << std::endl;
  std::cout << "Count:  " << values.size() << std::endl;
  std::cout << "Min:    " << std::fixed << std::setprecision(0) << min_val << " us" << std::endl;
  std::cout << "Max:    " << max_val << " us" << std::endl;
  std::cout << "Avg:    " << avg << " us" << std::endl;
  std::cout << "P50:    " << p50 << " us" << std::endl;
  std::cout << "P90:    " << p90 << " us" << std::endl;
  std::cout << "P99:    " << p99 << " us" << std::endl;
}

int main(int argc, char** argv) {
  signal(SIGINT, signalHandler);
  signal(SIGTERM, signalHandler);

  int count = 100;
  int timeout_ms = 3000;

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--count" && i + 1 < argc) {
      count = std::stoi(argv[++i]);
    } else if (arg == "--timeout" && i + 1 < argc) {
      timeout_ms = std::stoi(argv[++i]);
    } else if (arg == "--help" || arg == "-h") {
      printUsage(argv[0]);
      return 0;
    }
  }

  std::cout << "=== DDS RPC First Call Test ===" << std::endl;
  std::cout << "Test count: " << count << std::endl;
  std::cout << "Timeout:    " << timeout_ms << " ms" << std::endl;
  std::cout << std::endl;

  // 创建 DDS Participant（全局复用，模拟应用程序生命周期）
  dds_entity_t participant = dds_create_participant(0, nullptr, nullptr);
  if (participant < 0) {
    std::cerr << "Failed to create participant" << std::endl;
    return 1;
  }

  std::cout << "DDS Participant created, starting test...\n" << std::endl;

  std::vector<TestResult> results;
  results.reserve(count);

  int success_count = 0;
  int fail_count = 0;
  int timeout_count = 0;

  for (int i = 0; i < count && g_running; ++i) {
    TestResult result;
    result.index = i;
    result.success = false;
    result.total_latency_us = 0;
    result.discovery_latency_us = 0;
    result.rpc_latency_us = 0;

    try {
      // 1. 创建 Requester（不计入延迟统计，模拟 eager loading）
      dds_rpc::Requester<example::CalculatorRequest, example::CalculatorReply> requester(
          participant, "CalculatorService");

      // 2. 等待服务可用（从这里开始计时）
      auto total_start = std::chrono::steady_clock::now();
      auto discovery_start = std::chrono::steady_clock::now();
      if (!requester.wait_for_service(std::chrono::milliseconds(timeout_ms))) {
        auto now = std::chrono::steady_clock::now();
        result.total_latency_us = std::chrono::duration<double, std::micro>(now - total_start).count();
        result.discovery_latency_us = std::chrono::duration<double, std::micro>(now - discovery_start).count();
        result.error = "service not available";
        fail_count++;
        std::cout << "[" << std::setw(4) << i << "] NOT_AVAILABLE "
                  << "discovery=" << std::fixed << std::setprecision(1)
                  << result.discovery_latency_us / 1000 << "ms" << std::endl;
        results.push_back(result);
        continue;
      }
      auto discovery_end = std::chrono::steady_clock::now();
      result.discovery_latency_us = std::chrono::duration<double, std::micro>(
          discovery_end - discovery_start).count();

      // 3. 发送 RPC 请求
      example::CalculatorRequest req{};
      req.operand_a(10);
      req.operand_b(5);
      req.operation("add");

      auto rpc_start = std::chrono::steady_clock::now();
      auto reply = requester.send_request(req, std::chrono::milliseconds(timeout_ms));
      auto rpc_end = std::chrono::steady_clock::now();

      result.rpc_latency_us = std::chrono::duration<double, std::micro>(rpc_end - rpc_start).count();
      result.total_latency_us = std::chrono::duration<double, std::micro>(rpc_end - total_start).count();

      // 验证结果
      if (reply.result() == 15) {
        result.success = true;
        success_count++;
        std::cout << "[" << std::setw(4) << i << "] OK   "
                  << "total=" << std::fixed << std::setprecision(0) << result.total_latency_us << "us "
                  << "discovery=" << result.discovery_latency_us << "us "
                  << "rpc=" << result.rpc_latency_us << "us"
                  << std::endl;
      } else {
        result.error = "wrong result: " + std::to_string(reply.result());
        fail_count++;
        std::cout << "[" << std::setw(4) << i << "] FAIL " << result.error << std::endl;
      }

    } catch (const dds_rpc::TimeoutException&) {
      result.error = "timeout";
      timeout_count++;
      fail_count++;
      std::cout << "[" << std::setw(4) << i << "] TIMEOUT" << std::endl;
    } catch (const std::exception& e) {
      result.error = e.what();
      fail_count++;
      std::cout << "[" << std::setw(4) << i << "] ERROR " << result.error << std::endl;
    }

    results.push_back(result);
  }

  // 统计
  std::cout << "\n=== Test Results ===" << std::endl;
  std::cout << "Total:    " << results.size() << std::endl;
  std::cout << "Success:  " << success_count << " ("
            << std::fixed << std::setprecision(1)
            << (100.0 * success_count / results.size()) << "%)" << std::endl;
  std::cout << "Failed:   " << fail_count << " ("
            << (100.0 * fail_count / results.size()) << "%)" << std::endl;
  std::cout << "Timeout:  " << timeout_count << std::endl;

  // 延迟统计
  std::vector<double> total_latencies;
  std::vector<double> discovery_latencies;
  std::vector<double> rpc_latencies;

  for (const auto& r : results) {
    if (r.success) {
      total_latencies.push_back(r.total_latency_us);
      discovery_latencies.push_back(r.discovery_latency_us);
      rpc_latencies.push_back(r.rpc_latency_us);
    }
  }

  printStats("Total Latency (success)", total_latencies);
  printStats("Discovery Latency", discovery_latencies);
  printStats("RPC Latency", rpc_latencies);

  dds_delete(participant);

  std::cout << "\nTest finished" << std::endl;
  return (fail_count > 0) ? 1 : 0;
}
