/**
 * @file set_mode_stress_test.cpp
 * @brief setArmMode 接口压力测试
 *
 * 测试 setArmMode RPC 调用的可靠性和响应时间
 *
 * Usage: set_mode_stress_test [options]
 *   --count N       测试次数 (默认: 100)
 *   --interval MS   调用间隔毫秒 (默认: 100)
 *   --timeout MS    超时时间毫秒 (默认: 3000)
 */

#include <lejusdk-vr/lejusdk_vr.h>

#include <algorithm>
#include <chrono>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <string>
#include <thread>
#include <vector>

using namespace leju::vr;

volatile bool g_running = true;

void signalHandler(int signum) {
  std::cout << "\nReceived signal " << signum << ", stopping..." << std::endl;
  g_running = false;
}

struct TestResult {
  int index;
  ControlMode mode;
  bool success;
  double latency_ms;
  std::string error;
};

void printUsage(const char* bin) {
  std::cout << "Usage: " << bin << " [options]\n"
            << "Options:\n"
            << "  --count N       Number of test iterations (default: 100)\n"
            << "  --interval MS   Interval between calls in ms (default: 100)\n"
            << "  --timeout MS    RPC timeout in ms (default: 3000)\n"
            << "  --robot TYPE    Robot type: kuavo|roban (default: kuavo)\n"
            << "  --help          Show this help\n";
}

int main(int argc, char** argv) {
  signal(SIGINT, signalHandler);
  signal(SIGTERM, signalHandler);

  // 解析参数
  int count = 100;
  int interval_ms = 100;
  int timeout_ms = 3000;
  std::string robot = "kuavo";

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--count" && i + 1 < argc) {
      count = std::stoi(argv[++i]);
    } else if (arg == "--interval" && i + 1 < argc) {
      interval_ms = std::stoi(argv[++i]);
    } else if (arg == "--timeout" && i + 1 < argc) {
      timeout_ms = std::stoi(argv[++i]);
    } else if (arg == "--robot" && i + 1 < argc) {
      robot = argv[++i];
    } else if (arg == "--help" || arg == "-h") {
      printUsage(argv[0]);
      return 0;
    }
  }

  std::cout << "=== setArmMode Stress Test ===" << std::endl;
  std::cout << "Robot: " << robot << std::endl;
  std::cout << "Test count: " << count << std::endl;
  std::cout << "Interval: " << interval_ms << " ms" << std::endl;
  std::cout << "Timeout: " << timeout_ms << " ms" << std::endl;
  std::cout << std::endl;

  // 创建 API
  std::unique_ptr<VRBaseAPI> api;
  if (robot == "kuavo") {
    api = std::make_unique<KuavoVRAPI>();
  } else if (robot == "roban") {
    api = std::make_unique<RobanVRAPI>();
  } else {
    std::cerr << "Unknown robot type: " << robot << std::endl;
    return 1;
  }

  if (!api->initialize()) {
    std::cerr << "Failed to initialize VR API" << std::endl;
    return 1;
  }

  std::cout << "VR API initialized, starting test...\n" << std::endl;

  // 测试结果
  std::vector<TestResult> results;
  results.reserve(count);

  int success_count = 0;
  int fail_count = 0;
  int timeout_count = 0;

  // 开始测试
  for (int i = 0; i < count && g_running; ++i) {
    // 交替切换模式
    ControlMode target_mode = (i % 2 == 0) ? ControlMode::kExternal : ControlMode::kAuto;
    const char* mode_str = (target_mode == ControlMode::kExternal) ? "External" : "Auto";

    auto start = std::chrono::steady_clock::now();
    bool success = api->setArmMode(target_mode, timeout_ms);
    auto end = std::chrono::steady_clock::now();

    double latency_us = std::chrono::duration<double, std::micro>(end - start).count();
    double latency_ms = latency_us / 1000.0;

    TestResult result;
    result.index = i;
    result.mode = target_mode;
    result.success = success;
    result.latency_ms = latency_ms;

    if (success) {
      success_count++;
      std::cout << "[" << std::setw(4) << i << "] " << std::setw(8) << mode_str
                << " -> OK   latency=" << std::fixed << std::setprecision(0)
                << latency_us << " us" << std::endl;
    } else {
      fail_count++;
      if (latency_ms >= timeout_ms * 0.9) {
        timeout_count++;
        result.error = "timeout";
        std::cout << "[" << std::setw(4) << i << "] " << std::setw(8) << mode_str
                  << " -> TIMEOUT  latency=" << std::fixed << std::setprecision(1)
                  << latency_ms << " ms" << std::endl;
      } else {
        result.error = "failed";
        std::cout << "[" << std::setw(4) << i << "] " << std::setw(8) << mode_str
                  << " -> FAIL latency=" << std::fixed << std::setprecision(1)
                  << latency_ms << " ms" << std::endl;
      }
    }

    results.push_back(result);

    // 等待间隔
    if (i < count - 1 && g_running) {
      std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms));
    }
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
  if (!results.empty()) {
    std::vector<double> latencies;
    std::vector<double> success_latencies;
    for (const auto& r : results) {
      latencies.push_back(r.latency_ms);
      if (r.success) {
        success_latencies.push_back(r.latency_ms);
      }
    }

    std::sort(latencies.begin(), latencies.end());
    double sum = std::accumulate(latencies.begin(), latencies.end(), 0.0);
    double avg = sum / latencies.size();
    double min_val = latencies.front();
    double max_val = latencies.back();
    double p50 = latencies[latencies.size() / 2];
    double p90 = latencies[static_cast<size_t>(latencies.size() * 0.9)];
    double p99 = latencies[static_cast<size_t>(latencies.size() * 0.99)];

    std::cout << "\n=== Latency Statistics (all) ===" << std::endl;
    std::cout << "Min:  " << std::fixed << std::setprecision(0) << min_val * 1000 << " us" << std::endl;
    std::cout << "Max:  " << max_val * 1000 << " us" << std::endl;
    std::cout << "Avg:  " << avg * 1000 << " us" << std::endl;
    std::cout << "P50:  " << p50 * 1000 << " us" << std::endl;
    std::cout << "P90:  " << p90 * 1000 << " us" << std::endl;
    std::cout << "P99:  " << p99 * 1000 << " us" << std::endl;

    if (!success_latencies.empty()) {
      std::sort(success_latencies.begin(), success_latencies.end());
      double success_sum = std::accumulate(success_latencies.begin(), success_latencies.end(), 0.0);
      double success_avg = success_sum / success_latencies.size();
      std::cout << "\n=== Latency Statistics (success only) ===" << std::endl;
      std::cout << "Min:  " << success_latencies.front() * 1000 << " us" << std::endl;
      std::cout << "Max:  " << success_latencies.back() * 1000 << " us" << std::endl;
      std::cout << "Avg:  " << success_avg * 1000 << " us" << std::endl;
    }
  }

  // 恢复 Auto 模式
  std::cout << "\nRestoring arm to Auto mode..." << std::endl;
  api->setArmMode(ControlMode::kAuto);
  api->shutdown();

  std::cout << "Test finished" << std::endl;
  return (fail_count > 0) ? 1 : 0;
}
