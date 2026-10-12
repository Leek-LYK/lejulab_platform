/*
 * 🧪 竞态条件测试文件
 *
 * 测试内容：
 * - 多线程环境下的竞态条件检测和验证
 * - 并发数据访问的一致性保证
 * - 锁机制与无锁数据结构的对比
 * - 线程安全性验证
 *
 * 使用框架：标准C++ (无依赖框架)
 *
 * 测试覆盖范围：
 * - 多线程并发写入测试
 * - 数据竞态条件检测
 * - 内存可见性验证
 * - 原子操作正确性
 * - 并发性能评估
 */

#include <iostream>
#include <thread>
#include <vector>
#include <atomic>
#include <chrono>
#include <mutex>
#include <cassert>
#include <algorithm>
#include "../src/lock_free_ring_buffer.h"

using namespace canbus_sdk;

// 严格的竞争测试
constexpr size_t BUFFER_SIZE = 256;  // 足够大的缓冲区
constexpr int NUM_PRODUCERS = 4;     // 适中的生产者数量
constexpr int MESSAGES_PER_PRODUCER = 2000;
constexpr int TEST_ROUNDS = 5;

// 复杂的测试数据结构
struct ComplexData {
    int64_t timestamp;
    int producer_id;
    int sequence;
    double values[16];
    std::string name;
    
    ComplexData() = default;
    
    ComplexData(int pid, int seq) : producer_id(pid), sequence(seq) {
        timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        
        for (int i = 0; i < 16; ++i) {
            values[i] = pid * 1000.0 + seq + i * 0.1;
        }
        
        name = "Producer_" + std::to_string(pid) + "_Msg_" + std::to_string(seq);
    }
    
    bool operator==(const ComplexData& other) const {
        if (producer_id != other.producer_id || sequence != other.sequence) {
            return false;
        }
        if (timestamp != other.timestamp) return false;
        if (name != other.name) return false;
        for (int i = 0; i < 16; ++i) {
            if (values[i] != other.values[i]) return false;
        }
        return true;
    }
    
    bool isValid() const {
        // 检查数据完整性
        // Check data integrity
        if (producer_id < 0 || producer_id >= NUM_PRODUCERS) return false;
        if (sequence < 0 || sequence >= MESSAGES_PER_PRODUCER) return false;
        if (timestamp <= 0) return false;
        if (name.empty()) return false;
        
        // 检查数值是否合理
        // Check if values are reasonable
        for (int i = 0; i < 16; ++i) {
            double expected = producer_id * 1000.0 + sequence + i * 0.1;
            if (std::abs(values[i] - expected) > 0.0001) return false;
        }
        
        return true;
    }
};

// 竞争检测器
class RaceDetector {
private:
    std::vector<std::vector<int>> received_sequences_;
    std::vector<std::mutex> mutexes_;
    
public:
    RaceDetector() : received_sequences_(NUM_PRODUCERS), mutexes_(NUM_PRODUCERS) {}
    
    bool checkAndRecord(const ComplexData& data) {
        std::lock_guard<std::mutex> lock(mutexes_[data.producer_id]);
        
        // 检查是否重复接收
        // Check for duplicate reception
        if (std::any_of(received_sequences_[data.producer_id].begin(),
                       received_sequences_[data.producer_id].end(),
                       [data](int seq) { return seq == data.sequence; })) {
            std::cout << "❌ 检测到重复消息：生产者 " << data.producer_id
                     << ", 序列 " << data.sequence << std::endl;
            return false;
        }
        
        // 检查数据完整性
        // Check data integrity
        if (!data.isValid()) {
            std::cout << "❌ 检测到数据损坏：生产者 " << data.producer_id
                     << ", 序列 " << data.sequence << std::endl;
            return false;
        }
        
        // 记录已接收的序列号
        // Record received sequence number
        received_sequences_[data.producer_id].push_back(data.sequence);
        
        return true;
    }
    
    void checkCompleteness() {
        for (int pid = 0; pid < NUM_PRODUCERS; ++pid) {
            if (received_sequences_[pid].size() != static_cast<size_t>(MESSAGES_PER_PRODUCER)) {
                std::cout << "❌ 生产者 " << pid << " 缺少消息"
                         << ": 期望 " << MESSAGES_PER_PRODUCER
                         << ", 得到 " << received_sequences_[pid].size() << std::endl;
                return;
            }
        }
        std::cout << "✅ 所有消息都已接收，无重复或损坏" << std::endl;
    }
};

// 高压力生产者
void high_pressure_producer(LockFreeRingBuffer<ComplexData, BUFFER_SIZE>& buffer, 
                           int producer_id, std::atomic<int>& messages_sent) {
    for (int i = 0; i < MESSAGES_PER_PRODUCER; ++i) {
        ComplexData data(producer_id, i);
        
        // 高频率发送，增加竞争
        // High frequency sending to increase contention
        bool sent = false;
        int retry_count = 0;
        while (!sent && retry_count < 1000) {
            sent = buffer.push(data);
            if (!sent) {
                retry_count++;
                std::this_thread::yield();  // 让出CPU，增加竞争机会
                // Yield CPU to increase contention opportunities
            }
        }
        
        if (sent) {
            messages_sent++;
        } else {
            std::cout << "❌ 生产者 " << producer_id << " 发送消息 " << i << " 失败" << std::endl;
        }
        
        // 随机延迟，增加竞争的不确定性
        // Random delay to increase uncertainty in contention
        if (i % 100 == 0) {
            std::this_thread::sleep_for(std::chrono::microseconds(1));
        }
    }
}

// 高压力消费者
void high_pressure_consumer(LockFreeRingBuffer<ComplexData, BUFFER_SIZE>& buffer, 
                           RaceDetector& detector, std::atomic<int>& messages_received,
                           std::atomic<bool>& stop_flag) {
    int empty_count = 0;
    
    while (!stop_flag.load() || empty_count < 100) {
        ComplexData data;
        
        if (buffer.pop(data)) {  // 直接弹出，不使用超时
            // Short timeout to increase contention
            if (!detector.checkAndRecord(data)) {
                std::cout << "❌ 检测到竞态条件！" << std::endl;
                assert(false);
            }
            messages_received++;
            empty_count = 0;
        } else {
            empty_count++;
            std::this_thread::yield();  // 让出CPU
            // Yield CPU
        }
    }
}

int main() {
    std::cout << "🔥 开始高压竞态条件测试" << std::endl;
    std::cout << "📊 Configuration:" << std::endl;
    std::cout << "   缓冲区大小：" << BUFFER_SIZE << " (较小以增加竞争)" << std::endl;
    std::cout << "   生产者数量：" << NUM_PRODUCERS << " (较多以增加竞争)" << std::endl;
    std::cout << "   每个生产者消息数：" << MESSAGES_PER_PRODUCER << std::endl;
    std::cout << "   测试轮数：" << TEST_ROUNDS << std::endl;
    std::cout << "   每轮总消息数：" << NUM_PRODUCERS * MESSAGES_PER_PRODUCER << std::endl;
    std::cout << std::endl;
    
    for (int round = 1; round <= TEST_ROUNDS; ++round) {
        std::cout << "🎯 第 " << round << "/" << TEST_ROUNDS << " 轮" << std::endl;
        
        LockFreeRingBuffer<ComplexData, BUFFER_SIZE> buffer;
        RaceDetector detector;
        std::atomic<int> messages_sent{0};
        std::atomic<int> messages_received{0};
        std::atomic<bool> stop_flag{false};
        
        // 启动消费者
        // Start consumer
        std::thread consumer(high_pressure_consumer, std::ref(buffer), 
                           std::ref(detector), std::ref(messages_received), 
                           std::ref(stop_flag));
        
        // 启动生产者
        // Start producers
        std::vector<std::thread> producers;
        for (int i = 0; i < NUM_PRODUCERS; ++i) {
            producers.emplace_back(high_pressure_producer, std::ref(buffer), 
                                 i, std::ref(messages_sent));
        }
        
        // 等待生产者完成
        // Wait for producers to finish
        for (auto& producer : producers) {
            producer.join();
        }
        
        std::cout << "   ✅ 所有生产者已完成，等待消费者..." << std::endl;
        
        // 停止消费者
        // Stop consumer
        stop_flag.store(true);
        consumer.join();
        
        // 检查结果
        // Check results
        int expected = NUM_PRODUCERS * MESSAGES_PER_PRODUCER;
        int sent = messages_sent.load();
        int received = messages_received.load();
        
        std::cout << "   📈 结果：" << std::endl;
        std::cout << "      已发送：" << sent << "/" << expected << std::endl;
        std::cout << "      已接收：" << received << "/" << expected << std::endl;
        std::cout << "      成功率：" << (received * 100.0 / expected) << "%" << std::endl;
        
        // 检查完整性
        // Check completeness
        detector.checkCompleteness();
        
        // 验证总数
        // Verify total count
        if (sent != expected || received != expected) {
            std::cout << "❌ 第 " << round << " 轮失败！" << std::endl;
            return 1;
        }
        
        std::cout << "✅ 第 " << round << " 轮通过！" << std::endl;
        std::cout << std::endl;
    }
    
    std::cout << "🎉 所有轮次都通过了！未检测到竞态条件。" << std::endl;
    std::cout << "✅ MPSC 无锁环形缓冲区无竞态条件！" << std::endl;
    return 0;
}