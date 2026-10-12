// LiveHeftReferenceProvider 缓冲/重采样单元测试
// - 100→50Hz 重采样正确性（线性插值）
// - 四元数 slerp 符号连续
// - [-8,+16] 窗口 + 游标延迟
// - 预热门槛（覆盖不足不 ready）
// - 丢包保持（最后一致窗口）
// - 乱序/重复拒绝
// - 校准 epoch 重置

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>

#include "leju-rl-controller/motion/live_heft_reference_provider.h"

namespace leju {
namespace {

double SteadyNow() {
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

LiveHeftReferenceProvider::Config MakeConfig() {
  LiveHeftReferenceProvider::Config cfg;
  cfg.num_joints = 21;
  cfg.policy_dt = 0.02;
  cfg.cursor_delay = 0.32;
  cfg.history_margin = 0.18;
  cfg.fresh_timeout = 0.15;
  cfg.hold_timeout = 0.5;
  cfg.buffer_capacity_sec = 2.0;
  return cfg;
}

array_t ConstJpos(double v) {
  array_t j(21);
  j.setConstant(v);
  return j;
}

// 注入一段 100Hz 的线性运动流：pos.x = t, jpos = t, quat 绕 z 匀速旋转
void InjectLinearStream(LiveHeftReferenceProvider& p, double t_start,
                        double t_end, double dt = 0.01,
                        uint32_t epoch = 1, uint64_t seq_start = 1) {
  uint64_t seq = seq_start;
  for (double t = t_start; t <= t_end + 1e-9; t += dt) {
    const double rel = t - t_start;
    Eigen::Quaterniond q(Eigen::AngleAxisd(0.1 * rel, Eigen::Vector3d::UnitZ()));
    p.injectFrame(t, /*valid=*/true, /*calibrated=*/true, epoch, seq++,
                  Eigen::Vector3d(rel, 0.0, 0.7), q, ConstJpos(rel));
  }
}

const std::vector<int> kHeftOffsets = {0, 1, 2, 3, 4, 6, 8, 10, 12, 16,
                                       -1, -2, -4, -8};

TEST(LiveHeftReferenceProvider, NotReadyBeforeCoverage) {
  LiveHeftReferenceProvider p(MakeConfig());
  const double now = SteadyNow();
  // 只有 0.2s 数据：覆盖需求为 0.32+0.18=0.5s
  InjectLinearStream(p, now - 0.2, now);
  EXPECT_FALSE(p.ready());
}

TEST(LiveHeftReferenceProvider, ReadyAfterCoverageAndFresh) {
  LiveHeftReferenceProvider p(MakeConfig());
  const double now = SteadyNow();
  InjectLinearStream(p, now - 0.8, now);
  EXPECT_TRUE(p.ready());
}

TEST(LiveHeftReferenceProvider, NotReadyWhenUncalibrated) {
  LiveHeftReferenceProvider p(MakeConfig());
  const double now = SteadyNow();
  uint64_t seq = 1;
  for (double t = now - 0.8; t <= now; t += 0.01) {
    p.injectFrame(t, true, /*calibrated=*/false, 1, seq++,
                  Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(),
                  ConstJpos(0.0));
  }
  EXPECT_FALSE(p.ready());
}

TEST(LiveHeftReferenceProvider, ResamplingLinearInterpolation) {
  LiveHeftReferenceProvider p(MakeConfig());
  const double now = SteadyNow();
  const double t0 = now - 1.0;
  InjectLinearStream(p, t0, now);
  ASSERT_TRUE(p.ready());

  std::vector<HeftRefFrame> window;
  ASSERT_TRUE(p.sampleWindow(kHeftOffsets, window));
  ASSERT_EQ(window.size(), kHeftOffsets.size());

  // 游标 = newest - 0.32；offset o 的期望 pos.x = (cursor + o*0.02) - t0
  const double cursor_rel = (now - 0.32) - t0;
  for (size_t i = 0; i < kHeftOffsets.size(); ++i) {
    const double expect = cursor_rel + kHeftOffsets[i] * 0.02;
    EXPECT_NEAR(window[i].pos.x(), expect, 1e-6)
        << "offset " << kHeftOffsets[i];
    EXPECT_NEAR(window[i].jpos[0], expect, 1e-6);
    // 四元数角度与时间线性一致
    const double angle =
        2.0 * std::atan2(window[i].quat.vec().norm(), std::abs(window[i].quat.w()));
    EXPECT_NEAR(angle, 0.1 * expect, 1e-4);
  }
}

TEST(LiveHeftReferenceProvider, SlerpSignContinuity) {
  LiveHeftReferenceProvider p(MakeConfig());
  const double now = SteadyNow();
  // 两帧四元数符号相反但表示同一姿态附近：slerp 不得跳变
  uint64_t seq = 1;
  const double t0 = now - 0.8;
  for (double t = t0; t <= now + 1e-9; t += 0.01) {
    Eigen::Quaterniond q(Eigen::AngleAxisd(0.05, Eigen::Vector3d::UnitZ()));
    if (seq % 2 == 0) {
      q.coeffs() = -q.coeffs();  // 隔帧翻转符号
    }
    p.injectFrame(t, true, true, 1, seq++, Eigen::Vector3d::Zero(), q,
                  ConstJpos(0.0));
  }
  std::vector<HeftRefFrame> window;
  ASSERT_TRUE(p.sampleWindow(kHeftOffsets, window));
  for (const auto& f : window) {
    // 姿态角必须保持 ~0.05 rad，无中间跳变
    const double angle =
        2.0 * std::atan2(f.quat.vec().norm(), std::abs(f.quat.w()));
    EXPECT_NEAR(angle, 0.05, 1e-6);
  }
}

TEST(LiveHeftReferenceProvider, RejectsOutOfOrderAndDuplicate) {
  LiveHeftReferenceProvider p(MakeConfig());
  const double now = SteadyNow();
  p.injectFrame(now - 0.02, true, true, 1, 10, Eigen::Vector3d::Zero(),
                Eigen::Quaterniond::Identity(), ConstJpos(1.0));
  // 重复 seq / 旧 seq 应被拒
  p.injectFrame(now - 0.01, true, true, 1, 10, Eigen::Vector3d::Zero(),
                Eigen::Quaterniond::Identity(), ConstJpos(2.0));
  p.injectFrame(now, true, true, 1, 9, Eigen::Vector3d::Zero(),
                Eigen::Quaterniond::Identity(), ConstJpos(3.0));
  EXPECT_EQ(p.bufferedFrames(), 1u);
}

TEST(LiveHeftReferenceProvider, RejectsNonFiniteAndBadQuat) {
  LiveHeftReferenceProvider p(MakeConfig());
  const double now = SteadyNow();
  p.injectFrame(now, true, true, 1, 1,
                Eigen::Vector3d(std::nan(""), 0, 0),
                Eigen::Quaterniond::Identity(), ConstJpos(0.0));
  Eigen::Quaterniond bad(2.0, 0.0, 0.0, 0.0);  // 未归一化
  p.injectFrame(now, true, true, 1, 2, Eigen::Vector3d::Zero(), bad,
                ConstJpos(0.0));
  EXPECT_EQ(p.bufferedFrames(), 0u);
}

TEST(LiveHeftReferenceProvider, EpochChangeResetsBuffer) {
  LiveHeftReferenceProvider p(MakeConfig());
  const double now = SteadyNow();
  InjectLinearStream(p, now - 0.8, now, 0.01, /*epoch=*/1);
  EXPECT_TRUE(p.ready());
  const size_t before = p.bufferedFrames();
  EXPECT_GT(before, 50u);

  // 新 epoch 的第一帧：缓冲清空并重新计数
  p.injectFrame(now + 0.01, true, true, /*epoch=*/2, /*seq=*/1,
                Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(),
                ConstJpos(0.0));
  EXPECT_EQ(p.bufferedFrames(), 1u);
  EXPECT_FALSE(p.ready());  // 覆盖不足，需重新预热
}

TEST(LiveHeftReferenceProvider, HoldsLastWindowOnPacketLoss) {
  LiveHeftReferenceProvider p(MakeConfig());
  const double now = SteadyNow();
  // 数据结束于 0.3s 前（超过 fresh_timeout=0.15）→ 不新鲜
  InjectLinearStream(p, now - 1.3, now - 0.3);
  EXPECT_FALSE(p.ready());

  // 先在新鲜期内取一次窗口（模拟激活时曾经取过）
  // 直接调用 sampleWindow：即使不新鲜，第一次也会用现有缓冲插值
  std::vector<HeftRefFrame> w1, w2;
  ASSERT_TRUE(p.sampleWindow(kHeftOffsets, w1));
  // 再次采样：应返回保持的窗口（相同内容）
  ASSERT_TRUE(p.sampleWindow(kHeftOffsets, w2));
  ASSERT_EQ(w1.size(), w2.size());
  for (size_t i = 0; i < w1.size(); ++i) {
    EXPECT_NEAR(w1[i].pos.x(), w2[i].pos.x(), 1e-12);
    EXPECT_NEAR(w1[i].jpos[0], w2[i].jpos[0], 1e-12);
  }
}

TEST(LiveHeftReferenceProvider, DegradedAfterHoldTimeout) {
  LiveHeftReferenceProvider p(MakeConfig());
  const double now = SteadyNow();
  // 最后一帧在 1.0s 前：> fresh_timeout + hold_timeout
  InjectLinearStream(p, now - 2.0, now - 1.0);
  EXPECT_TRUE(p.degraded());
  EXPECT_FALSE(p.ready());
}

TEST(LiveHeftReferenceProvider, NotDegradedBeforeAnyData) {
  LiveHeftReferenceProvider p(MakeConfig());
  EXPECT_FALSE(p.degraded());
}

TEST(LiveHeftReferenceProvider, ClampsToNewestForFutureBeyondBuffer) {
  auto cfg = MakeConfig();
  cfg.cursor_delay = 0.0;  // 强制游标在最新帧：所有 future 偏移越界
  LiveHeftReferenceProvider p(cfg);
  const double now = SteadyNow();
  const double t0 = now - 1.0;
  InjectLinearStream(p, t0, now);

  std::vector<HeftRefFrame> window;
  ASSERT_TRUE(p.sampleWindow({0, 1, 16}, window));
  const double newest_rel = now - t0;
  // future 偏移全部夹取到最新帧
  EXPECT_NEAR(window[0].pos.x(), newest_rel, 1e-6);
  EXPECT_NEAR(window[1].pos.x(), newest_rel, 1e-6);
  EXPECT_NEAR(window[2].pos.x(), newest_rel, 1e-6);
}

TEST(LiveHeftReferenceProvider, FrozenHoldResumePrefillsBuffer) {
  LiveHeftReferenceProvider p(MakeConfig());
  const double now = SteadyNow();
  uint64_t seq = 1;
  for (int i = 0; i < 80; ++i) {
    const double t = now - 0.8 + 0.01 * static_cast<double>(i);
    p.injectFrame(t, true, true, 1, seq++, Eigen::Vector3d::Zero(),
                  Eigen::Quaterniond::Identity(), ConstJpos(0.0));
  }
  ASSERT_TRUE(p.ready());

  for (int i = 0; i < 15; ++i) {
    p.injectFrame(now + 0.01 * static_cast<double>(i + 1), true, true, 1,
                  seq++, Eigen::Vector3d::Zero(),
                  Eigen::Quaterniond::Identity(), ConstJpos(0.0));
  }

  p.injectFrame(now + 0.2, true, true, 1, seq++, Eigen::Vector3d::Zero(),
                Eigen::Quaterniond::Identity(), ConstJpos(0.5));
  EXPECT_TRUE(p.consumeReferenceDiscontinuity());
  EXPECT_FALSE(p.consumeReferenceDiscontinuity());

  std::vector<HeftRefFrame> window;
  ASSERT_TRUE(p.sampleWindow(kHeftOffsets, window));
  for (const auto& frame : window) {
    EXPECT_NEAR(frame.jpos[0], 0.5, 1e-9);
  }
}

}  // namespace
}  // namespace leju
