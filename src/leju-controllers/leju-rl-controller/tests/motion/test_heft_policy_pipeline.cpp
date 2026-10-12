// HEFT 端到端策略管线测试：
//   真实 deploy CSV + 真实 ONNX 策略，分别经 CsvHeftReferenceSource 与
//   LiveHeftReferenceProvider 采样 → 按 HEFT 观测数学构建 1368 维观测 →
//   真实 ONNX 推理 → 21 维 action。验证：
//   - 两条参考路径的观测逐元素一致
//   - 观测维度与策略输入形状匹配（1368）
//   - 策略输出为 21 维且数值有限
// 机器人历史状态（IMU/关节/action 缓冲）在两条路径下取相同合成零值，
// 因此观测差异只来自参考窗口——这正是 CSV/Live 需要保证的契约。

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include "leju-rl-controller/inference/model_factory.h"
#include "leju-rl-controller/motion/csv_heft_reference_source.h"
#include "leju-rl-controller/motion/live_heft_reference_provider.h"
#include "leju-rl-controller/motion/motion_trajectory.h"

namespace leju {
namespace {

#ifndef HEFT_CSV_PATH
#define HEFT_CSV_PATH "config/17/motion_data/roban_default_stand_10s_deploy.csv"
#endif
#ifndef HEFT_POLICY_PATH
#define HEFT_POLICY_PATH "config/17/policy/heft/policy_0805.onnx"
#endif

constexpr int kJoints = 21;
constexpr double kPolicyDt = 0.02;
const std::vector<int> kFutureSteps = {0, 1, 2, 3, 4, 6, 8, 10, 12, 16,
                                       -1, -2, -4, -8};
const std::vector<int> kHistoryTaps = {0, 1, 2, 3, 4, 8, 12, 16, 20};
constexpr int kPrevActionSteps = 8;
constexpr int kBootMax = 25;
constexpr double kGravityZ = -1.0;

Eigen::Vector3d QuatApplyInverse(const Eigen::Quaterniond& q,
                                 const Eigen::Vector3d& v) {
  return q.conjugate() * v;
}

Eigen::Quaterniond QuatInFrame(const Eigen::Quaterniond& frame,
                               const Eigen::Quaterniond& quat_w) {
  return frame.conjugate() * quat_w;
}

array_t QuatToRot6d(const Eigen::Quaterniond& q) {
  const Eigen::Matrix3d R = q.normalized().toRotationMatrix();
  array_t out(6);
  out << R(0, 0), R(1, 0), R(2, 0), R(0, 1), R(1, 1), R(2, 1);
  return out;
}

// 严格复刻 HeftRLController::computeObservation() 的观测数学。
// 机器人历史状态取合成零值（两条路径相同），只让参考窗口产生差异。
// 返回观测维度并写入 obs。
int BuildObservation(const std::vector<HeftRefFrame>& frames,
                     array_t& obs) {
  const int S = static_cast<int>(frames.size());
  const int H = static_cast<int>(kHistoryTaps.size());

  // 观测维度公式与 HeftRLController::expectedObsDim() 一致
  const int dim =
      1 + 3 * (S - 1) + 6 * S + 2 * kJoints * S + S + 3 * S +
      2 * 3 * H + 2 * kJoints * H + kJoints * kPrevActionSteps;
  obs.resize(dim);
  obs.setZero();

  int offset = 0;
  auto put = [&](const array_t& seg) {
    obs.segment(offset, seg.size()) = seg;
    offset += static_cast<int>(seg.size());
  };

  // boot indicator（首 tick = 1.0）
  obs[offset++] = 1.0;

  // 参考相对根位移（不含 i=0，共 S-1 个）
  for (int i = 1; i < S; ++i) {
    const Eigen::Vector3d diff =
        QuatApplyInverse(frames[0].quat, frames[i].pos - frames[0].pos);
    put(array_t(diff.array()));
  }
  // 参考根姿态 rot6d（机器人根姿态取单位四元数 → rq_aligned=identity）
  for (int i = 0; i < S; ++i) {
    put(QuatToRot6d(QuatInFrame(Eigen::Quaterniond::Identity(), frames[i].quat)));
  }
  // 参考关节位置
  for (int i = 0; i < S; ++i) {
    put(frames[i].jpos);
  }
  // 参考关节位置 - 当前关节位置（当前取 0）
  for (int i = 0; i < S; ++i) {
    put(array_t(frames[i].jpos));
  }
  // 参考根高度
  for (int i = 0; i < S; ++i) {
    obs[offset++] = frames[i].pos[2];
  }
  // 参考系下重力方向
  for (int i = 0; i < S; ++i) {
    const Eigen::Vector3d g =
        QuatApplyInverse(frames[i].quat, Eigen::Vector3d(0., 0., kGravityZ));
    put(array_t(g.array()));
  }

  // 机器人历史（合成零值）
  const int angvel_hist = 3 * H;
  const int projg_hist = 3 * H;
  const int jpos_hist = kJoints * H;
  const int jvel_hist = kJoints * H;
  const int act_hist = kJoints * kPrevActionSteps;
  offset += angvel_hist + projg_hist + jpos_hist + jvel_hist + act_hist;

  EXPECT_EQ(offset, static_cast<int>(obs.size()));
  return offset;
}

TEST(HeftPolicyPipeline, CsvAndLiveProduceIdenticalPolicyActions) {
  // ---------- 加载真实 deploy CSV ----------
  MotionTrajectory motion;
  ASSERT_TRUE(motion.load(HEFT_CSV_PATH)) << "deploy CSV load failed";
  const int kFrames = motion.getNumFrames();
  ASSERT_GT(kFrames, 20) << "CSV too short";

  CsvHeftReferenceSource csv_source([&motion]() { return &motion; });

  // ---------- Live 路径：按 20ms 栅格注入相同帧 ----------
  LiveHeftReferenceProvider::Config cfg;
  cfg.num_joints = kJoints;
  cfg.policy_dt = kPolicyDt;
  cfg.cursor_delay = 16 * kPolicyDt;
  cfg.history_margin = 8 * kPolicyDt + 0.02;
  cfg.fresh_timeout = 1e9;  // 合成时间戳：关闭新鲜度门槛
  cfg.hold_timeout = 1e9;
  cfg.buffer_capacity_sec = 1e9;
  LiveHeftReferenceProvider live(cfg);
  for (int k = 0; k < kFrames; ++k) {
    live.injectFrame(k * kPolicyDt, true, true, 1, k + 1,
                     motion.getFrameBodyPos(k), motion.getFrameBodyQuat(k),
                     motion.getFrameJointPos(k));
  }

  // 游标对齐：CSV 前进到 newest-16 帧；Live 游标自动为 newest-0.32s
  const int cursor_frame = kFrames - 1 - 16;
  while (motion.getCurrentFrame() < cursor_frame) motion.next();
  ASSERT_EQ(motion.getCurrentFrame(), cursor_frame);

  // ---------- 采样窗口 ----------
  std::vector<HeftRefFrame> w_csv, w_live;
  ASSERT_TRUE(csv_source.sampleWindow(kFutureSteps, w_csv));
  ASSERT_TRUE(live.sampleWindow(kFutureSteps, w_live));
  ASSERT_EQ(w_csv.size(), w_live.size());
  ASSERT_EQ(w_csv.size(), kFutureSteps.size());

  // ---------- 构建观测 ----------
  array_t obs_csv, obs_live;
  const int dim = BuildObservation(w_csv, obs_csv);
  ASSERT_EQ(dim, 1368) << "HEFT observation dim must be 1368";
  BuildObservation(w_live, obs_live);

  for (int i = 0; i < dim; ++i) {
    EXPECT_NEAR(obs_live[i], obs_csv[i], 1e-9)
        << "obs mismatch at " << i << " (CSV vs Live)";
  }

  // ---------- 加载真实策略 ----------
  auto model = ModelFactory::create("onnxruntime");
  ASSERT_NE(model, nullptr);
  model->setOutputName("action");
  ASSERT_TRUE(model->load(HEFT_POLICY_PATH)) << "policy load failed";
  ASSERT_TRUE(model->isLoaded()) << "output 'action' not found";

  // ---------- 两条路径各跑一次推理 ----------
  std::vector<float> in_csv(obs_csv.data(), obs_csv.data() + dim);
  std::vector<float> in_live(obs_live.data(), obs_live.data() + dim);
  const auto act_csv = model->forward(in_csv);
  const auto act_live = model->forward(in_live);

  ASSERT_EQ(act_csv.size(), 21u) << "HEFT action dim must be 21";
  ASSERT_EQ(act_live.size(), 21u);

  for (size_t i = 0; i < act_csv.size(); ++i) {
    EXPECT_TRUE(std::isfinite(act_csv[i])) << "action " << i << " not finite";
    EXPECT_NEAR(act_live[i], act_csv[i], 1e-6)
        << "action " << i << " differs CSV vs Live";
  }
}

}  // namespace
}  // namespace leju
