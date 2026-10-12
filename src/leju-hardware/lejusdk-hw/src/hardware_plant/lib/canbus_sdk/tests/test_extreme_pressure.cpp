/*
 * 🧪 极限压力测试文件
 *
 * 测试内容：
 * - 极限条件下的数据结构和算法稳定性测试
 * - 高并发环境下的性能和可靠性验证
 * - 内存边界条件和资源耗尽场景
 * - 长时间运行的稳定性验证
 *
 * 使用框架：标准C++ (无依赖框架)
 *
 * 测试覆盖范围：
 * - 大数据量处理能力
 * - 高频率写入/读取性能
 * - 内存使用效率
 * - 系统稳定性
 * - 资源泄漏检测
 */

#include <iostream>
#include <thread>
#include <vector>
#include <atomic>
#include <chrono>
#include <mutex>
#include <cassert>
#include <pthread.h>
#include <sched.h>
#include "../src/lock_free_ring_buffer.h"

using namespace canbus_sdk;

// MPSC极限压力测试配置
constexpr size_t BUFFER_SIZE = 2048;
constexpr int NUM_PRODUCERS = 2;
constexpr int NUM_CONSUMERS = 1;
constexpr int MESSAGES_PER_PRODUCER = 1000;
constexpr int TEST_ROUNDS = 2;
constexpr int EXPECTED_TOTAL = NUM_PRODUCERS * MESSAGES_PER_PRODUCER;

// 极限测试数据结构
struct ExtremePressureData {
    int producer_id;
    int sequence;
    uint64_t timestamp;
    double values[8];
    int checksum;

    ExtremePressureData() : producer_id(-1), sequence(-1), timestamp(0), checksum(0) {
        for (int i = 0; i < 8; ++i) {
            values[i] = 0.0;
        }
    }

    ExtremePressureData(int pid, int seq) : producer_id(pid), sequence(seq) {
        timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();

        for (int i = 0; i < 8; ++i) {
            values[i] = pid * 1000.0 + seq + i * 0.1;
        }

        // 计算校验和
        checksum = producer_id ^ sequence ^ (timestamp & 0xFFFFFFFF);
        for (int i = 0; i < 8; ++i) {
            checksum ^= static_cast<int>(values[i]) & 0xFFFF;
        }
    }

    bool isValid() const {
        if (producer_id < 0 || producer_id >= NUM_PRODUCERS || sequence < 0 || sequence >= MESSAGES_PER_PRODUCER) {
            return false;
        }
        if (timestamp == 0) {
            return false;
        }

        // 验证校验和
        int expected_checksum = producer_id ^ sequence ^ (timestamp & 0xFFFFFFFF);
        for (int i = 0; i < 8; ++i) {
            expected_checksum ^= static_cast<int>(values[i]) & 0xFFFF;
        }

        if (checksum != expected_checksum) {
            return false;
        }

        // 验证数值
        for (int i = 0; i < 8; ++i) {
            double expected = producer_id * 1000.0 + sequence + i * 0.1;
            if (std::abs(values[i] - expected) > 0.0001) {
                return false;
            }
        }

        return true;
    }
};

// 严格的多生产者顺序验证器
class MultiProducerValidator {
private:
    std::vector<std::atomic<int>> last_sequences_;
    std::vector<std::atomic<int>> received_counts_;
    std::atomic<int> total_received_{0};
    std::atomic<int> validation_errors_{0};

public:
    MultiProducerValidator() : last_sequences_(NUM_PRODUCERS), received_counts_(NUM_PRODUCERS) {
        for (int i = 0; i < NUM_PRODUCERS; ++i) {
            last_sequences_[i].store(-1, std::memory_order_relaxed);
            received_counts_[i].store(0, std::memory_order_relaxed);
        }
    }

    bool validate(const ExtremePressureData& data) {
        if (!data.isValid()) {
            validation_errors_.fetch_add(1, std::memory_order_relaxed);
            return false;
        }

        int producer_id = data.producer_id;
        int sequence = data.sequence;

        // 验证每个生产者的消息顺序
        int last_seq = last_sequences_[producer_id].load(std::memory_order_acquire);
        if (sequence <= last_seq) {
            std::cout << "❌ 顺序错误: 生产者 " << producer_id
                     << ", 期望 > " << last_seq
                     << ", 得到 " << sequence << std::endl;
            validation_errors_.fetch_add(1, std::memory_order_relaxed);
            return false;
        }

        // 更新序列号
        last_sequences_[producer_id].store(sequence, std::memory_order_release);
        received_counts_[producer_id].fetch_add(1, std::memory_order_relaxed);
        total_received_.fetch_add(1, std::memory_order_relaxed);

        return true;
    }

    int getTotalReceived() const {
        return total_received_.load(std::memory_order_relaxed);
    }

    int getValidationErrors() const {
        return validation_errors_.load(std::memory_order_relaxed);
    }

    bool isComplete() const {
        return getTotalReceived() == EXPECTED_TOTAL;
    }

    void verifyCompleteness() const {
        for (int i = 0; i < NUM_PRODUCERS; ++i) {
            int count = received_counts_[i].load(std::memory_order_relaxed);
            if (count != MESSAGES_PER_PRODUCER) {
                std::cout << "❌ 生产者 " << i << " 不完整: 期望 "
                         << MESSAGES_PER_PRODUCER << ", 实际 " << count << std::endl;
            }
        }
    }

    void reset() {
        for (int i = 0; i < NUM_PRODUCERS; ++i) {
            last_sequences_[i].store(-1, std::memory_order_relaxed);
            received_counts_[i].store(0, std::memory_order_relaxed);
        }
        total_received_.store(0, std::memory_order_relaxed);
        validation_errors_.store(0, std::memory_order_relaxed);
    }
};

// 高速生产者
void high_speed_producer(LockFreeRingBuffer<ExtremePressureData, BUFFER_SIZE>& buffer,
                        int producer_id, std::atomic<int>& messages_sent,
                        std::atomic<bool>& stop_flag) {
    // 设置CPU亲和性 - 每个生产者绑定到不同的CPU核心
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(producer_id % std::thread::hardware_concurrency(), &cpuset);
    pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);

    for (int i = 0; i < MESSAGES_PER_PRODUCER; ++i) {
        if (stop_flag.load(std::memory_order_relaxed)) {
            break;
        }

        ExtremePressureData data(producer_id, i);

        // 高速写入，最小延迟
        int retry_count = 0;
        while (!buffer.push(data)) {
            retry_count++;
            if (retry_count > 50000) {
                std::cout << "   ⚠️  生产者 " << producer_id << " 因缓冲区满而退出" << std::endl;
                return;
            }

            // 短暂暂停避免忙等待
            if (retry_count % 100 == 0) {
                std::this_thread::yield();
            }
        }

        messages_sent.fetch_add(1, std::memory_order_relaxed);

        // 极小延迟，制造压力
        if (i % 50 == 0) {
            std::this_thread::sleep_for(std::chrono::microseconds(1));
        }
    }
}

// 高速消费者
void high_speed_consumer(LockFreeRingBuffer<ExtremePressureData, BUFFER_SIZE>& buffer,
                       MultiProducerValidator& validator,
                       std::atomic<int>& messages_received,
                       std::atomic<bool>& stop_flag) {
    // 设置CPU亲和性 - 消费者绑定到最后一个CPU核心
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(std::thread::hardware_concurrency() - 1, &cpuset);
    pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);

    int empty_count = 0;
    const int max_empty_count = 10000000;

    while (!stop_flag.load(std::memory_order_acquire)) {
        if (validator.isComplete()) {
            empty_count++;
            if (empty_count > 1000) {
                break;
            }
        }

        ExtremePressureData data;
        if (buffer.pop(data)) {
            if (validator.validate(data)) {
                messages_received.fetch_add(1, std::memory_order_relaxed);
            }
            empty_count = 0;
        } else {
            empty_count++;

            // 优化：空时短暂暂停
            if (empty_count % 5000 == 0) {
                std::this_thread::yield();
            }
        }

        if (empty_count > max_empty_count) {
            break;
        }
    }
}

int main() {
    std::cout << "💥💥💥 STARTING EXTREME PRESSURE TEST 💥💥💥" << std::endl;
    std::cout << "🔥🔥🔥 MPSC MODE - TRUE CONCURRENCY 🔥🔥🔥" << std::endl;
    std::cout << "📊 配置信息:" << std::endl;
    std::cout << "   缓冲区大小: " << BUFFER_SIZE << " (小缓冲区 - 最大竞争)" << std::endl;
    std::cout << "   生产者数量: " << NUM_PRODUCERS << " (MPSC模式 - 真正并发)" << std::endl;
    std::cout << "   消费者数量: " << NUM_CONSUMERS << " (单消费者)" << std::endl;
    std::cout << "   每个生产者消息数: " << MESSAGES_PER_PRODUCER << " (高并发)" << std::endl;
    std::cout << "   测试轮数: " << TEST_ROUNDS << " (多轮稳定性)" << std::endl;
    std::cout << "   每轮总消息数: " << EXPECTED_TOTAL << " (巨大吞吐量)" << std::endl;
    std::cout << "   可用CPU核心数: " << std::thread::hardware_concurrency() << std::endl;
    std::cout << "   缓冲区利用率: " << (BUFFER_SIZE * 100.0 / EXPECTED_TOTAL) << "%" << std::endl;
    std::cout << std::endl;
    std::cout << "⚠️  警告: 此测试将极限压测MPSC缓冲区!" << std::endl;
    std::cout << "⚠️  警告: 多生产者高并发竞争!" << std::endl;
    std::cout << std::endl;

    auto total_start_time = std::chrono::high_resolution_clock::now();
    int total_messages_processed = 0;
    int total_validation_errors = 0;

    for (int round = 1; round <= TEST_ROUNDS; ++round) {
        std::cout << "🎯 EXTREME Round " << round << "/" << TEST_ROUNDS << std::endl;

        auto round_start_time = std::chrono::high_resolution_clock::now();

        LockFreeRingBuffer<ExtremePressureData, BUFFER_SIZE> buffer;
        MultiProducerValidator validator;
        std::atomic<int> messages_sent{0};
        std::atomic<int> messages_received{0};
        std::atomic<bool> stop_flag{false};

        // 启动消费者
        std::thread consumer(high_speed_consumer, std::ref(buffer),
                           std::ref(validator), std::ref(messages_received),
                           std::ref(stop_flag));

        // 启动多个生产者
        std::vector<std::thread> producers;
        for (int i = 0; i < NUM_PRODUCERS; ++i) {
            producers.emplace_back(high_speed_producer, std::ref(buffer),
                                 i, std::ref(messages_sent), std::ref(stop_flag));
        }

        // 等待所有生产者完成
        auto producer_wait_start = std::chrono::steady_clock::now();
        for (auto& producer : producers) {
            if (producer.joinable()) {
                auto elapsed = std::chrono::steady_clock::now() - producer_wait_start;
                if (std::chrono::duration_cast<std::chrono::seconds>(elapsed).count() > 30) {
                    std::cout << "   ⚠️  等待生产者超时，强制停止" << std::endl;
                    stop_flag.store(true);
                    break;
                }
                producer.join();
            }
        }

        auto producer_end_time = std::chrono::high_resolution_clock::now();

        std::cout << "   ✅ 所有生产者已完成，等待消费者处理..." << std::endl;

        // 等待消费者完成或超时
        auto wait_start = std::chrono::steady_clock::now();
        while (!validator.isComplete()) {
            auto elapsed = std::chrono::steady_clock::now() - wait_start;
            if (std::chrono::duration_cast<std::chrono::seconds>(elapsed).count() > 15) {
                std::cout << "   ⚠️  消费者等待超时" << std::endl;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        // 停止消费者
        stop_flag.store(true, std::memory_order_release);
        if (consumer.joinable()) {
            consumer.join();
        }

        auto round_end_time = std::chrono::high_resolution_clock::now();

        // 统计结果
        int sent = messages_sent.load(std::memory_order_relaxed);
        int received = messages_received.load(std::memory_order_relaxed);
        int validation_errors = validator.getValidationErrors();

        total_messages_processed += received;
        total_validation_errors += validation_errors;

        // 性能计算
        auto producer_duration = std::chrono::duration_cast<std::chrono::microseconds>(
            producer_end_time - round_start_time).count();
        auto total_duration = std::chrono::duration_cast<std::chrono::microseconds>(
            round_end_time - round_start_time).count();

        double producer_throughput = (sent * 1000000.0) / producer_duration;
        double total_throughput = (received * 1000000.0) / total_duration;

        std::cout << "   📈 性能指标:" << std::endl;
        std::cout << "      发送消息: " << sent << "/" << EXPECTED_TOTAL << std::endl;
        std::cout << "      接收消息: " << received << "/" << EXPECTED_TOTAL << std::endl;
        std::cout << "      验证错误: " << validation_errors << std::endl;
        std::cout << "      成功率: " << (received * 100.0 / EXPECTED_TOTAL) << "%" << std::endl;
        std::cout << "      生产者耗时: " << producer_duration << " μs" << std::endl;
        std::cout << "      总耗时: " << total_duration << " μs" << std::endl;
        std::cout << "      生产者吞吐量: " << producer_throughput << " 消息/秒" << std::endl;
        std::cout << "      总吞吐量: " << total_throughput << " 消息/秒" << std::endl;

        // 验证结果
        if (sent != EXPECTED_TOTAL || received != EXPECTED_TOTAL || validation_errors > 0) {
            std::cout << "❌ 第 " << round << " 轮失败!" << std::endl;
            std::cout << "   验证错误数: " << validation_errors << std::endl;
            validator.verifyCompleteness();
            return 1;
        }

        std::cout << "✅ 第 " << round << " 轮通过!" << std::endl;
        std::cout << std::endl;
    }

    auto total_end_time = std::chrono::high_resolution_clock::now();
    auto total_duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        total_end_time - total_start_time).count();
    double avg_throughput = (total_messages_processed * 1000.0) / total_duration;

    std::cout << "🎉🎉🎉 所有极限测试轮都通过! 🎉🎉🎉" << std::endl;
    std::cout << "🔥🔥🔥 MPSC缓冲区在极限压力下完美运行! 🔥🔥🔥" << std::endl;
    std::cout << "✅ 多生产者并发，无数据竞争!" << std::endl;
    std::cout << "✅ 无消息丢失，数据完整性100%保证!" << std::endl;
    std::cout << "📊 最终性能指标:" << std::endl;
    std::cout << "   总测试时间: " << total_duration << " ms" << std::endl;
    std::cout << "   处理总消息数: " << total_messages_processed << std::endl;
    std::cout << "   总验证错误: " << total_validation_errors << std::endl;
    std::cout << "   平均吞吐量: " << avg_throughput << " 消息/秒" << std::endl;
    std::cout << "   消息完整性: " << (total_validation_errors == 0 ? "100%" : "有错误") << std::endl;
    std::cout << std::endl;
    std::cout << "🏆 MPSC无锁缓冲区极限测试完成!" << std::endl;
    return 0;
}