#pragma once

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

namespace leju::mujoco_sim {

class DepthPublishCadence {
 public:
  using Clock = std::chrono::steady_clock;

  explicit DepthPublishCadence(Clock::duration period) : period_(period) {}

  bool tryReserve(Clock::time_point now = Clock::now()) {
    if (now < next_publish_) return false;
    next_publish_ = now + period_;
    return true;
  }

 private:
  Clock::duration period_;
  Clock::time_point next_publish_{};
};

class PersistentTaskGroup {
 public:
  using Task = std::function<void(size_t)>;

  explicit PersistentTaskGroup(size_t worker_count)
      : worker_count_(worker_count) {
    if (worker_count_ == 0) {
      throw std::invalid_argument("PersistentTaskGroup requires workers");
    }
    workers_.reserve(worker_count_);
    for (size_t index = 0; index < worker_count_; ++index) {
      workers_.emplace_back(&PersistentTaskGroup::workerLoop, this, index);
    }
  }

  ~PersistentTaskGroup() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping_ = true;
    }
    start_cv_.notify_all();
    for (auto& worker : workers_) {
      if (worker.joinable()) worker.join();
    }
  }

  PersistentTaskGroup(const PersistentTaskGroup&) = delete;
  PersistentTaskGroup& operator=(const PersistentTaskGroup&) = delete;

  void dispatch(Task task) {
    std::unique_lock<std::mutex> lock(mutex_);
    done_cv_.wait(lock, [this] { return remaining_ == 0; });
    task_ = std::move(task);
    remaining_ = worker_count_;
    ++generation_;
    lock.unlock();
    start_cv_.notify_all();
  }

  void wait() {
    std::unique_lock<std::mutex> lock(mutex_);
    done_cv_.wait(lock, [this] { return remaining_ == 0; });
  }

  size_t workerCount() const { return worker_count_; }

 private:
  void workerLoop(size_t worker_index) {
    uint64_t observed_generation = 0;
    for (;;) {
      Task task;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        start_cv_.wait(lock, [this, observed_generation] {
          return stopping_ || generation_ != observed_generation;
        });
        if (stopping_) return;
        observed_generation = generation_;
        task = task_;
      }

      task(worker_index);

      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (--remaining_ == 0) done_cv_.notify_all();
      }
    }
  }

  const size_t worker_count_;
  std::vector<std::thread> workers_;
  mutable std::mutex mutex_;
  std::condition_variable start_cv_;
  std::condition_variable done_cv_;
  Task task_;
  size_t remaining_ = 0;
  uint64_t generation_ = 0;
  bool stopping_ = false;
};

}  // namespace leju::mujoco_sim
