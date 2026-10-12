#include <atomic>
#include <chrono>
#include <mutex>
#include <set>
#include <thread>

#include <gtest/gtest.h>

#include "leju-mujoco-sim/depth_camera_scheduler.h"

namespace leju::mujoco_sim {
namespace {

TEST(DepthPublishCadence, ReservesOnlyOneFramePerPeriod) {
  using Clock = std::chrono::steady_clock;
  DepthPublishCadence cadence(std::chrono::milliseconds(33));
  const Clock::time_point start{};

  EXPECT_TRUE(cadence.tryReserve(start));
  EXPECT_FALSE(cadence.tryReserve(start));
  EXPECT_FALSE(cadence.tryReserve(start + std::chrono::milliseconds(32)));
  EXPECT_TRUE(cadence.tryReserve(start + std::chrono::milliseconds(33)));
}

TEST(PersistentTaskGroup, ReusesSleepingWorkersAcrossFrames) {
  PersistentTaskGroup workers(4);
  std::mutex ids_mutex;
  std::set<std::thread::id> first_ids;
  std::set<std::thread::id> second_ids;
  std::atomic<int> calls{0};

  workers.dispatch([&](size_t) {
    std::lock_guard<std::mutex> lock(ids_mutex);
    first_ids.insert(std::this_thread::get_id());
    ++calls;
  });
  workers.wait();

  workers.dispatch([&](size_t) {
    std::lock_guard<std::mutex> lock(ids_mutex);
    second_ids.insert(std::this_thread::get_id());
    ++calls;
  });
  workers.wait();

  EXPECT_EQ(calls.load(), 8);
  EXPECT_EQ(first_ids.size(), 4u);
  EXPECT_EQ(second_ids, first_ids);
  EXPECT_EQ(workers.workerCount(), 4u);
}

}  // namespace
}  // namespace leju::mujoco_sim
