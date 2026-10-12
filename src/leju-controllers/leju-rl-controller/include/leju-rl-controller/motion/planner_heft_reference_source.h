#pragma once

#include <atomic>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "leju-rl-controller/motion/flow_planner.h"
#include "leju-rl-controller/motion/heft_reference_source.h"
#include "leju-rl-controller/motion/motion_trajectory.h"

namespace leju {

/// CSV 提供 m_cmd，闭环 flow 规划作为 student 参考（deploy ``heft_planner``）。
///
/// 规划在独立线程跑（默认 2 ORT 线程），policy tick 只读最新 buffer，
/// 不阻塞 1kHz。自恢复：每次 replan 用当前机器人 ``p_tilde``。
class PlannerHeftReferenceSource final : public HeftReferenceSource {
 public:
  using MotionGetter = std::function<MotionTrajectory*()>;

  struct Config {
    FlowPlanner::Config flow;
    int exec_steps = 2;
    int command_offset = 150;
    int history_pad = 8;
    int num_joints = 21;
    double policy_dt = 0.02;
  };

  PlannerHeftReferenceSource(Config config, MotionGetter getter);
  ~PlannerHeftReferenceSource() override;

  PlannerHeftReferenceSource(const PlannerHeftReferenceSource&) = delete;
  PlannerHeftReferenceSource& operator=(const PlannerHeftReferenceSource&) =
      delete;

  bool load();

  /// Policy tick 前写入当前机器人姿态（IMU + HEFT 绝对关节）。
  void setRobotState(const Eigen::Quaterniond& quat, const array_t& jpos);

  bool ready() const override;
  bool sampleWindow(const std::vector<int>& offsets,
                    std::vector<HeftRefFrame>& out) override;
  void onPolicyTick() override;
  void onActivate() override;
  Eigen::Quaterniond currentRootQuat() const override;
  double progressFrame() const override;

 private:
  struct Job {
    std::vector<float> p_tilde;
    std::vector<float> m_cmd;
    std::vector<float> warm;
    std::vector<Eigen::Vector3d> root_fill;
  };

  void threadLoop();
  void seedFromMotion();
  void writePlan(const FlowPlanner::Plan& plan);
  bool buildJob(Job& job);
  HeftRefFrame clampFrame(int frame) const;

  Config config_;
  MotionGetter getter_;
  FlowPlanner planner_;

  mutable std::mutex mu_;
  std::vector<HeftRefFrame> motion_;
  int cursor_ = 0;
  int steps_since_replan_ = 0;
  Eigen::Quaterniond robot_quat_ = Eigen::Quaterniond::Identity();
  array_t robot_jpos_;
  bool have_robot_ = false;
  bool loaded_ = false;

  std::thread thread_;
  std::condition_variable cv_;
  std::atomic<bool> stop_{false};
  std::atomic<bool> busy_{false};
  bool job_pending_ = false;
  Job pending_;
};

}  // namespace leju
