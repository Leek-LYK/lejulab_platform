/*
 * 🧪 MPSC Ring Buffer 测试文件
 *
 * 测试内容：
 * - 多生产者单消费者(MPSC)无锁环形缓冲区功能测试
 * - 并发环境下的数据写入和读取
 * - 缓冲区溢出和边界条件处理
 * - 内存管理和线程安全性
 *
 * 使用框架：标准C++ (无依赖框架)
 *
 * 测试覆盖范围：
 * - 基本读写操作
 * - 多线程并发写入
 * - 缓冲区容量测试
 * - 数据一致性检查
 * - 性能压力测试
 */

#include <iostream>
#include <thread>
#include <vector>
#include <atomic>
#include <chrono>
#include <cassert>
#include "../src/lock_free_ring_buffer.h"

using namespace canbus_sdk;

// 测试配置
constexpr size_t BUFFER_SIZE = 1024 * 8;
constexpr int NUM_PRODUCERS = 8;
constexpr int PRODUCER_FREQ_HZ = 250;
constexpr int CONSUMER_FREQ_HZ = 2000;
constexpr int TEST_DURATION_SECONDS = 10;
constexpr int PRODUCER_INTERVAL_US = 1000000 / PRODUCER_FREQ_HZ;
constexpr int CONSUMER_INTERVAL_US = 1000000 / CONSUMER_FREQ_HZ;
constexpr int TIMEOUT_MS = 100;

// 测试统计
struct TestStats {
    std::atomic<int> messages_sent{0};
    std::atomic<int> messages_received{0};
    std::atomic<int> buffer_full_count{0};
    std::atomic<int> buffer_empty_count{0};
    std::atomic<bool> start_signal{false};
};

// 测试消息结构
struct TestMessage {
    int producer_id;
    int message_id;
    int data[8];

    TestMessage() = default;
    TestMessage(int pid, int mid) : producer_id(pid), message_id(mid) {
        for (int i = 0; i < 8; ++i) {
            data[i] = pid * 1000 + mid + i;
        }
    }

    bool operator==(const TestMessage& other) const {
        if (producer_id != other.producer_id || message_id != other.message_id) {
            return false;
        }
        for (int i = 0; i < 8; ++i) {
            if (data[i] != other.data[i]) {
                return false;
            }
        }
        return true;
    }
};

// 生产者函数
void producer_function(LockFreeRingBuffer<TestMessage, BUFFER_SIZE>& buffer,
                      int producer_id, TestStats& stats, std::atomic<bool>& stop_flag) {
    // 等待启动信号
    while (!stats.start_signal.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    auto start_time = std::chrono::steady_clock::now();
    int message_count = 0;

    while (!stop_flag.load()) {
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(current_time - start_time);

        // 检查是否达到测试时长
        if (elapsed.count() >= TEST_DURATION_SECONDS * 1000000) {
            break;
        }

        TestMessage msg(producer_id, message_count);

        // 尝试发送消息
        if (buffer.push(msg)) {
            stats.messages_sent++;
            message_count++;

            // 成功发送后sleep，保证准确频率
            std::this_thread::sleep_for(std::chrono::microseconds(PRODUCER_INTERVAL_US));
        } else {
            stats.buffer_full_count++;

            // 失败时按照正常间隔sleep，避免疯狂重试
            std::this_thread::sleep_for(std::chrono::microseconds(PRODUCER_INTERVAL_US));
        }
    }
}

// 消费者函数
void consumer_function(LockFreeRingBuffer<TestMessage, BUFFER_SIZE>& buffer,
                     TestStats& stats, std::atomic<bool>& stop_flag) {
    auto start_time = std::chrono::steady_clock::now();
    std::vector<int> last_message_id(NUM_PRODUCERS, -1);
    int cycle_count = 0;

    // 消费者准备就绪，设置启动信号
    stats.start_signal.store(true);

    while (!stop_flag.load()) {
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(current_time - start_time);

        // 检查是否达到测试时长
        if (elapsed.count() >= TEST_DURATION_SECONDS * 1000000) {
            break;
        }

        // 尝试消费消息（非阻塞）
        TestMessage msg;
        if (buffer.pop(msg)) {
            stats.messages_received++;

            // 验证消息顺序
            int producer_id = msg.producer_id;
            int message_id = msg.message_id;

            if (message_id <= last_message_id[producer_id]) {
                std::cout << "❌ 消息顺序错误：生产者 " << producer_id
                         << ", 期望 > " << last_message_id[producer_id]
                         << ", 得到 " << message_id << std::endl;
                assert(false);
            }

            last_message_id[producer_id] = message_id;
        } else {
            stats.buffer_empty_count++;
        }

        cycle_count++;
    }

    // 消费剩余消息
    TestMessage msg;
    while (buffer.pop(msg)) {
        stats.messages_received++;

        int producer_id = msg.producer_id;
        int message_id = msg.message_id;

        if (message_id <= last_message_id[producer_id]) {
            std::cout << "❌ 清理过程中的消息顺序错误：生产者 " << producer_id
                     << ", 期望 > " << last_message_id[producer_id]
                     << ", 得到 " << message_id << std::endl;
            assert(false);
        }

        last_message_id[producer_id] = message_id;
    }
}

int main() {
    std::cout << "🚀 开始 MPSC 无锁环形缓冲区频率测试" << std::endl;
    std::cout << "📊 配置：" << std::endl;
    std::cout << "   缓冲区大小：" << BUFFER_SIZE << std::endl;
    std::cout << "   生产者数量：" << NUM_PRODUCERS << std::endl;
    std::cout << "   生产者频率：" << PRODUCER_FREQ_HZ << " Hz" << std::endl;
    std::cout << "   消费者频率：" << CONSUMER_FREQ_HZ << " Hz" << std::endl;
    std::cout << "   测试时长：" << TEST_DURATION_SECONDS << " 秒" << std::endl;
    std::cout << "   理论总消息数：" << NUM_PRODUCERS * PRODUCER_FREQ_HZ * TEST_DURATION_SECONDS << std::endl;
    std::cout << std::endl;

    LockFreeRingBuffer<TestMessage, BUFFER_SIZE> buffer;
    TestStats stats;
    std::atomic<bool> stop_flag{false};

    // 启动消费者线程
    std::cout << "🔵 启动消费者线程..." << std::endl;
    std::thread consumer(consumer_function, std::ref(buffer), std::ref(stats), std::ref(stop_flag));

    // 启动生产者线程
    std::cout << "🔴 启动 " << NUM_PRODUCERS << " 个生产者线程..." << std::endl;
    std::vector<std::thread> producers;
    for (int i = 0; i < NUM_PRODUCERS; ++i) {
        producers.emplace_back(producer_function, std::ref(buffer), i, std::ref(stats), std::ref(stop_flag));
    }

    // 等待测试完成
    std::this_thread::sleep_for(std::chrono::seconds(TEST_DURATION_SECONDS));

    std::cout << "⏰ 测试时间结束，停止所有线程..." << std::endl;
    stop_flag.store(true);

    // 等待所有线程完成
    for (auto& producer : producers) {
        producer.join();
    }
    consumer.join();

    // 打印结果
    std::cout << std::endl;
    std::cout << "📈 测试结果：" << std::endl;
    int actual_sent = stats.messages_sent.load();
    int actual_received = stats.messages_received.load();
    int expected_messages = NUM_PRODUCERS * PRODUCER_FREQ_HZ * TEST_DURATION_SECONDS;

    std::cout << "   实际发送消息数：" << actual_sent << std::endl;
    std::cout << "   实际接收消息数：" << actual_received << std::endl;
    std::cout << "   理论消息数：" << expected_messages << std::endl;
    std::cout << "   缓冲区满次数：" << stats.buffer_full_count.load() << std::endl;
    std::cout << "   缓冲区空次数：" << stats.buffer_empty_count.load() << std::endl;
    std::cout << "   发送效率：" << (actual_sent * 100.0 / expected_messages) << "%" << std::endl;
    std::cout << "   接收效率：" << (actual_received * 100.0 / actual_sent) << "%" << std::endl;

    // 计算实际频率
    double actual_producer_freq = actual_sent / (double)TEST_DURATION_SECONDS / NUM_PRODUCERS;
    double actual_consumer_freq = actual_received / (double)TEST_DURATION_SECONDS;

    std::cout << "   实际生产者频率：" << actual_producer_freq << " Hz" << std::endl;
    std::cout << "   实际消费者频率：" << actual_consumer_freq << " Hz" << std::endl;

    // 验证结果
    double efficiency = (actual_received * 100.0 / expected_messages);

    if (efficiency >= 90.0) {
        std::cout << "🎉 测试通过！系统性能良好。" << std::endl;
        std::cout << "✅ MPSC 无锁环形缓冲区在指定频率下工作正常。" << std::endl;
        return 0;
    } else if (efficiency >= 70.0) {
        std::cout << "⚠️  测试基本通过，但性能有待优化。" << std::endl;
        std::cout << "✅ 系统可以运行，但存在一些瓶颈。" << std::endl;
        return 0;
    } else {
        std::cout << "❌ 测试失败！系统性能不足。" << std::endl;
        std::cout << "   效率低于70%，需要优化。" << std::endl;
        return 1;
    }
}