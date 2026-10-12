#ifndef LOCK_FREE_RING_BUFFER_H
#define LOCK_FREE_RING_BUFFER_H

#include <atomic>
#include <cstddef>
#include <type_traits>

namespace canbus_sdk {

/**
 * @brief 无锁环形缓冲区，借鉴Linux kfifo设计思想
 *
 * 设计考虑（借鉴kfifo）：
 * - 多生产者单消费者模式（MPSC）- 真正的并发支持
 * - 使用原子操作替代锁机制
 * - 内存屏障确保数据可见性
 * - 2的幂次大小便于位运算
 * - 生产者和消费者索引独立管理
 * - 避免复杂的依赖关系
 *
 * @note 在并发访问下，size()、empty()和full()方法是近似值
 * @note 类型T应该具有noexcept的拷贝和移动操作以确保异常安全
 */
template<typename T, size_t SIZE>
class LockFreeRingBuffer {
private:
    static_assert((SIZE & (SIZE - 1)) == 0, "SIZE必须是2的幂次");
    static_assert(SIZE >= 2, "SIZE必须至少为2");

    // 缓存行大小（x86的典型值）
    static constexpr size_t CACHE_LINE_SIZE = 64;

    // 环形缓冲区存储
    T buffer_[SIZE];

    // 读写索引（原子操作，缓存行对齐）
    alignas(CACHE_LINE_SIZE) std::atomic<size_t> head_{0};
    alignas(CACHE_LINE_SIZE) std::atomic<size_t> tail_{0};

    public:
    LockFreeRingBuffer() {
        // 初始化缓冲区为默认值
        for (size_t i = 0; i < SIZE; ++i) {
            buffer_[i] = T();
        }
    }

    ~LockFreeRingBuffer() = default;

    // 删除拷贝操作
    LockFreeRingBuffer(const LockFreeRingBuffer&) = delete;
    LockFreeRingBuffer& operator=(const LockFreeRingBuffer&) = delete;

    /**
     * @brief 向缓冲区推送一个项目（多生产者）
     * @param item 要推送的项目
     * @return 成功返回true，缓冲区满返回false
     */
    bool push(const T& item) {
        // 借鉴kfifo：保守但正确的MPSC实现
        size_t current_tail = tail_.load(std::memory_order_relaxed);
        size_t next_tail = (current_tail + 1) & (SIZE - 1);

        // 检查缓冲区是否已满
        size_t current_head = head_.load(std::memory_order_acquire);
        if (next_tail == current_head) {
            return false;  // 缓冲区满
        }

        // 关键：使用CAS原子性地获取插槽并更新tail指针
        if (!tail_.compare_exchange_weak(
            current_tail,
            next_tail,
            std::memory_order_acquire,
            std::memory_order_relaxed)) {
            return false;  // 其他生产者抢先了
        }

        // 成功获取插槽后，安全地写入数据
        buffer_[current_tail] = item;

        // 确保数据写入对消费者可见
        std::atomic_thread_fence(std::memory_order_release);

        return true;
    }

    /**
     * @brief 使用移动语义向缓冲区推送项目（多生产者）
     * @param item 要推送的项目（将被移动）
     * @return 成功返回true，缓冲区满返回false
     */
    bool push(T&& item) {
        // 借鉴kfifo：真正的MPSC实现
        size_t current_tail = tail_.load(std::memory_order_relaxed);
        size_t next_tail = (current_tail + 1) & (SIZE - 1);

        // 检查缓冲区是否已满
        size_t current_head = head_.load(std::memory_order_acquire);
        if (next_tail == current_head) {
            return false;  // 缓冲区满
        }

        // 关键改进：使用CAS来原子性地更新tail指针
        // 只有在成功获取插槽后，才写入数据
        if (!tail_.compare_exchange_weak(
            current_tail,
            next_tail,
            std::memory_order_acquire,
            std::memory_order_relaxed)) {
            return false;  // 其他生产者抢先了
        }

        // 现在安全地写入数据到保留的插槽
        buffer_[current_tail] = std::move(item);

        // 确保数据写入对消费者可见
        std::atomic_thread_fence(std::memory_order_release);

        return true;
    }

    /**
     * @brief 从缓冲区弹出一个项目（单消费者）
     * @param item 用于存储弹出项目的引用
     * @return 成功返回true，缓冲区空返回false
     */
    bool pop(T& item) {
        // 借鉴kfifo：单消费者使用简单的原子操作
        size_t current_head = head_.load(std::memory_order_relaxed);
        size_t current_tail = tail_.load(std::memory_order_acquire);

        if (current_head == current_tail) {
            return false;  // 缓冲区空
        }

        // 使用内存屏障确保生产者的数据写入对消费者可见
        std::atomic_thread_fence(std::memory_order_acquire);

        // 借鉴kfifo：先读取数据，再更新索引
        item = std::move(buffer_[current_head]);

        // 计算下一个head位置
        size_t next_head = (current_head + 1) & (SIZE - 1);

        // 更新head指针（单消费者不需要CAS）
        head_.store(next_head, std::memory_order_release);

        return true;
    }

    
    /**
     * @brief 获取缓冲区中当前的项目数量
     * @return 近似的项目数量（由于并发访问可能不准确）
     */
    size_t size() const {
        // 借鉴kfifo：简单的减法运算
        size_t head = head_.load(std::memory_order_acquire);
        size_t tail = tail_.load(std::memory_order_acquire);

        return (tail - head) & (SIZE - 1);
    }

    /**
     * @brief 检查缓冲区是否为空
     * @return 空返回true，否则返回false
     */
    bool empty() const {
        // 借鉴kfifo：直接比较索引
        return head_.load(std::memory_order_acquire) == tail_.load(std::memory_order_acquire);
    }

    /**
     * @brief 检查缓冲区是否已满
     * @return 满返回true，否则返回false
     */
    bool full() const {
        // 借鉴kfifo：检查下一个位置是否等于head
        size_t head = head_.load(std::memory_order_acquire);
        size_t tail = tail_.load(std::memory_order_acquire);
        return ((tail + 1) & (SIZE - 1)) == head;
    }

    /**
     * @brief 获取缓冲区容量
     * @return 缓冲区容量（SIZE - 1由于环形缓冲区设计）
     */
    constexpr size_t capacity() const {
        return SIZE - 1;
    }

    };

} // namespace canbus_sdk

#endif // LOCK_FREE_RING_BUFFER_H