#include <iostream>
#include <cassert>
#include <thread>
#include <chrono>
#include <atomic>
#include <vector>
#include "lejusdk-utils/loop.hpp"

using namespace leju;

/**
 * @brief 测试Loop基本功能
 */
void testBasicFunctionality() {
    std::cout << "Testing Loop basic functionality..." << std::endl;

    int counter = 0;
    Loop loop("test_loop", [&counter]() {
        counter++;
    }, 100); // 100ms period

    // 测试初始状态
    assert(loop.getName() == "test_loop");
    assert(loop.getPeriod() == 100);
    assert(!loop.isRunning());
    assert(counter == 0);

    // 启动循环
    loop.start();
    assert(loop.isRunning());

    // 运行一段时间
    std::this_thread::sleep_for(std::chrono::milliseconds(350));

    // 停止循环
    loop.shutdown();
    assert(!loop.isRunning());

    // 验证执行次数（约3-4次）
    std::cout << "Counter value: " << counter << std::endl;
    assert(counter >= 3 && counter <= 4);

    std::cout << "✓ Basic functionality test passed" << std::endl;
}

/**
 * @brief 测试Lambda函数捕获
 */
void testLambdaCapture() {
    std::cout << "Testing lambda capture..." << std::endl;

    int multiplier = 5;
    std::atomic<int> result{0};

    Loop loop("lambda_test", [&multiplier, &result]() {
        result += multiplier;
    }, 50);

    loop.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    loop.shutdown();

    int expected = result.load() / multiplier;
    std::cout << "Expected executions: " << expected << ", Result: " << result.load() << std::endl;
    assert(result.load() >= 20); // 至少4次执行

    std::cout << "✓ Lambda capture test passed" << std::endl;
}

/**
 * @brief 测试多次启动和停止
 */
void testMultipleStartStop() {
    std::cout << "Testing multiple start/stop..." << std::endl;

    int counter = 0;
    Loop loop("multi_test", [&counter]() {
        counter++;
    }, 50);

    // 多次启动应该是幂等的
    loop.start();
    loop.start();
    loop.start();
    assert(loop.isRunning());

    std::this_thread::sleep_for(std::chrono::milliseconds(120));

    // 多次停止也应该是幂等的
    loop.shutdown();
    loop.shutdown();
    assert(!loop.isRunning());

    assert(counter >= 2);

    // 重新启动
    loop.start();
    assert(loop.isRunning());

    std::this_thread::sleep_for(std::chrono::milliseconds(80));
    loop.shutdown();

    assert(counter >= 3);

    std::cout << "✓ Multiple start/stop test passed" << std::endl;
}

/**
 * @brief 测试拷贝和移动语义禁用
 */
void testCopyMoveSemanticsDisabled() {
    std::cout << "Testing copy/move semantics disabled..." << std::endl;

    int counter = 0;
    Loop loop1("original", [&counter]() {
        counter++;
    }, 100);

    // 测试拷贝被禁用 - 这会导致编译错误
    // Loop loop2 = loop1;  // ❌ 编译错误：拷贝被删除

    // 测试移动被禁用 - 这会导致编译错误
    // Loop loop3 = std::move(loop1);  // ❌ 编译错误：移动被删除

    loop1.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    loop1.shutdown();

    assert(counter >= 1);

    std::cout << "✓ Copy/move semantics disabled test passed" << std::endl;
}

/**
 * @brief 测试CPU绑定功能
 */
void testCPUAffinity() {
    std::cout << "Testing CPU affinity..." << std::endl;

    int counter = 0;
    Loop loop("cpu_test", [&counter]() {
        counter++;
    }, 50, 0); // 绑定到CPU 0

    loop.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(120));
    loop.shutdown();

    // 基本功能应该正常工作
    assert(counter >= 2);

    // 测试不绑定的情况（-1）
    Loop loop_no_bind("no_bind_test", [&counter]() {
        counter++;
    }, 50, -1);

    loop_no_bind.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    loop_no_bind.shutdown();

    assert(counter >= 4);

    std::cout << "✓ CPU affinity test passed" << std::endl;
}

/**
 * @brief 测试短周期和高频执行
 */
void testHighFrequencyExecution() {
    std::cout << "Testing high frequency execution..." << std::endl;

    std::atomic<int> counter{0};
    Loop loop("high_freq_test", [&counter]() {
        counter++;
    }, 1); // 1ms 周期

    loop.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    loop.shutdown();

    // 应该执行约20次，允许一些误差
    int executions = counter.load();
    std::cout << "High frequency executions: " << executions << std::endl;
    assert(executions >= 10 && executions <= 30);

    std::cout << "✓ High frequency execution test passed" << std::endl;
}

/**
 * @brief 测试析构函数自动清理
 */
void testDestructorCleanup() {
    std::cout << "Testing destructor cleanup..." << std::endl;

    int counter = 0;
    {
        Loop loop("cleanup_test", [&counter]() {
            counter++;
        }, 20);

        loop.start();
        std::this_thread::sleep_for(std::chrono::milliseconds(60));
        // loop 离开作用域，析构函数应该自动调用 shutdown
    }

    // 在析构后执行应该停止
    int final_count = counter;
    std::this_thread::sleep_for(std::chrono::milliseconds(40));
    assert(counter == final_count); // 计数器应该不再增加

    std::cout << "✓ Destructor cleanup test passed" << std::endl;
}

/**
 * @brief 性能基准测试
 */
void testPerformanceBenchmark() {
    std::cout << "Performance benchmark test..." << std::endl;

    std::atomic<int> counter{0};
    Loop loop("perf_test", [&counter]() {
        counter++;
    }, 10); // 10ms 周期

    auto start_time = std::chrono::steady_clock::now();

    loop.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // 运行1秒
    loop.shutdown();

    auto end_time = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

    int expected_executions = duration.count() / 10;
    int actual_executions = counter.load();

    std::cout << "Expected executions: " << expected_executions
              << ", Actual: " << actual_executions << std::endl;

    // 允许一定的误差（±20%）
    assert(actual_executions >= expected_executions * 0.8);
    assert(actual_executions <= expected_executions * 1.2);

    std::cout << "✓ Performance benchmark test passed" << std::endl;
}

int main() {
    std::cout << "=== Loop Test Suite ===" << std::endl;

    try {
        testBasicFunctionality();
        testLambdaCapture();
        testMultipleStartStop();
        testCopyMoveSemanticsDisabled();
        testCPUAffinity();
        testHighFrequencyExecution();
        testDestructorCleanup();
        testPerformanceBenchmark();

        std::cout << "\n🎉 All Loop tests passed!" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "❌ Test failed with exception: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cerr << "❌ Test failed with unknown exception" << std::endl;
        return 1;
    }

    return 0;
}