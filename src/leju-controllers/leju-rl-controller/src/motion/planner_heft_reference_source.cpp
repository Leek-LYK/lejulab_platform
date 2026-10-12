#include "leju-rl-controller/motion/planner_heft_reference_source.h"

#include <algorithm>
#include <cmath>

#include "leju-rl-controller/rl_log.h"

namespace leju {
namespace {

int clampi(int v, int lo, int hi) {
  return std::max(lo, std::min(v, hi));
}

}  // namespace

PlannerHeftReferenceSource::PlannerHeftReferenceSource(Config config,
                                                       MotionGetter getter)
    : config_(std::move(config)), getter_(std::move(getter)) {
  robot_jpos_ = array_t::Zero(config_.num_joints);
}

PlannerHeftReferenceSource::~PlannerHeftReferenceSource() {
  stop_ = true;
  cv_.notify_all();
  if (thread_.joinable()) {
    thread_.join();
  }
}

bool PlannerHeftReferenceSource::load() {
  if (!planner_.load(config_.flow)) {
    return false;
  }
  loaded_ = true;
  stop_ = false;
  thread_ = std::thread([this] { threadLoop(); });
  seedFromMotion();
  return true;
}

bool PlannerHeftReferenceSource::ready() const {
  std::lock_guard<std::mutex> lock(mu_);
  return loaded_ && !motion_.empty();
}

void PlannerHeftReferenceSource::setRobotState(const Eigen::Quaterniond& quat,
                                               const array_t& jpos) {
  std::lock_guard<std::mutex> lock(mu_);
  robot_quat_ = quat;
  robot_jpos_ = jpos;
  have_robot_ = true;
}

void PlannerHeftReferenceSource::onActivate() {
  {
    std::lock_guard<std::mutex> lock(mu_);
    job_pending_ = false;
    steps_since_replan_ = config_.exec_steps;
  }
  seedFromMotion();
}

void PlannerHeftReferenceSource::seedFromMotion() {
  HeftRefFrame f;
  f.jpos = array_t::Zero(config_.num_joints);
  f.jvel = array_t::Zero(config_.num_joints);
  if (MotionTrajectory* m = getter_()) {
    if (m->isLoaded()) {
      f.pos = m->getFrameBodyPos(0);
      f.quat = m->getFrameBodyQuat(0);
      f.jpos = m->getFrameJointPos(0);
      f.jvel = m->getFrameJointVel(0);
    }
  }
  std::lock_guard<std::mutex> lock(mu_);
  motion_.assign(1, f);
  cursor_ = 0;
}

HeftRefFrame PlannerHeftReferenceSource::clampFrame(int frame) const {
  if (motion_.empty()) {
    HeftRefFrame z;
    z.jpos = array_t::Zero(config_.num_joints);
    z.jvel = array_t::Zero(config_.num_joints);
    return z;
  }
  const int i = clampi(frame, 0, static_cast<int>(motion_.size()) - 1);
  return motion_[static_cast<size_t>(i)];
}

bool PlannerHeftReferenceSource::sampleWindow(const std::vector<int>& offsets,
                                              std::vector<HeftRefFrame>& out) {
  std::lock_guard<std::mutex> lock(mu_);
  if (motion_.empty()) {
    return false;
  }
  out.resize(offsets.size());
  for (size_t i = 0; i < offsets.size(); ++i) {
    out[i] = clampFrame(cursor_ + offsets[i]);
  }
  return true;
}

Eigen::Quaterniond PlannerHeftReferenceSource::currentRootQuat() const {
  std::lock_guard<std::mutex> lock(mu_);
  if (motion_.empty()) {
    return Eigen::Quaterniond::Identity();
  }
  return clampFrame(cursor_).quat;
}

double PlannerHeftReferenceSource::progressFrame() const {
  std::lock_guard<std::mutex> lock(mu_);
  return static_cast<double>(cursor_);
}

bool PlannerHeftReferenceSource::buildJob(Job& job) {
  MotionTrajectory* motion = getter_();
  if (!motion || !motion->isLoaded() || !have_robot_) {
    return false;
  }
  const int Tclip = motion->getNumFrames();
  const int t = motion->getCurrentFrame();
  const int cmd_f = clampi(t + config_.command_offset, 0, Tclip - 1);
  const int warm_f =
      clampi(t + planner_.horizonFrames(), 0, cmd_f);

  const Eigen::VectorXd q_robot = robot_jpos_.matrix();
  job.p_tilde = planner_.packState(Eigen::Vector3d::Zero(), robot_quat_, q_robot);
  job.m_cmd = planner_.packState(motion->getFrameBodyPos(cmd_f),
                                 motion->getFrameBodyQuat(cmd_f),
                                 motion->getFrameJointPos(cmd_f).matrix());
  job.warm = planner_.packState(motion->getFrameBodyPos(warm_f),
                                motion->getFrameBodyQuat(warm_f),
                                motion->getFrameJointPos(warm_f).matrix());

  const float horizon_s =
      static_cast<float>(planner_.horizonFrames()) / config_.flow.memmap_fps;
  const int Td = std::max(
      2, static_cast<int>(std::lround(horizon_s * config_.flow.control_hz)) + 1);
  job.root_fill.resize(static_cast<size_t>(Td));
  const int t1 = clampi(t + std::max(planner_.horizonFrames(), 1), 0, Tclip - 1);
  for (int i = 0; i < Td; ++i) {
    const double u = (Td == 1) ? 0.0 : static_cast<double>(i) / (Td - 1);
    const double fi = t + u * (t1 - t);
    const int i0 = clampi(static_cast<int>(std::floor(fi)), 0, Tclip - 1);
    const int i1 = clampi(i0 + 1, 0, Tclip - 1);
    const double a = fi - i0;
    job.root_fill[static_cast<size_t>(i)] =
        (1.0 - a) * motion->getFrameBodyPos(i0) + a * motion->getFrameBodyPos(i1);
  }
  return true;
}

void PlannerHeftReferenceSource::writePlan(const FlowPlanner::Plan& plan) {
  const int T = static_cast<int>(plan.jpos.size());
  if (T <= 1) {
    return;
  }
  const int pad = std::max(0, config_.history_pad);
  std::vector<HeftRefFrame> past;
  {
    // Keep recent history from the previous student buffer.
    const int t = cursor_;
    const int start = std::max(0, t - pad);
    for (int i = start; i < t && i < static_cast<int>(motion_.size()); ++i) {
      past.push_back(motion_[static_cast<size_t>(i)]);
    }
  }
  while (static_cast<int>(past.size()) < pad && !past.empty()) {
    past.insert(past.begin(), past.front());
  }
  while (static_cast<int>(past.size()) < pad && T > 1) {
    HeftRefFrame f;
    f.pos = plan.pos[1];
    f.quat = plan.quat[1];
    f.jpos = plan.jpos[1].array();
    f.jvel = array_t::Zero(config_.num_joints);
    past.push_back(f);
  }

  std::vector<HeftRefFrame> live;
  live.reserve(static_cast<size_t>(T - 1));
  for (int i = 1; i < T; ++i) {
    HeftRefFrame f;
    f.pos = plan.pos[static_cast<size_t>(i)];
    f.quat = plan.quat[static_cast<size_t>(i)];
    f.jpos = plan.jpos[static_cast<size_t>(i)].array();
    if (i + 1 < T) {
      f.jvel = ((plan.jpos[static_cast<size_t>(i + 1)] -
                 plan.jpos[static_cast<size_t>(i)]) /
                config_.policy_dt)
                   .array();
    } else if (i > 0) {
      f.jvel = ((plan.jpos[static_cast<size_t>(i)] -
                 plan.jpos[static_cast<size_t>(i - 1)]) /
                config_.policy_dt)
                   .array();
    } else {
      f.jvel = array_t::Zero(config_.num_joints);
    }
    live.push_back(std::move(f));
  }

  motion_.clear();
  motion_.insert(motion_.end(), past.begin(), past.end());
  motion_.insert(motion_.end(), live.begin(), live.end());
  cursor_ = static_cast<int>(past.size());
}

void PlannerHeftReferenceSource::onPolicyTick() {
  Job job;
  bool submit = false;
  {
    std::lock_guard<std::mutex> lock(mu_);
    cursor_ = clampi(cursor_ + 1, 0, std::max(0, static_cast<int>(motion_.size()) - 1));
    ++steps_since_replan_;
    if (steps_since_replan_ >= config_.exec_steps && !busy_ && !job_pending_) {
      if (buildJob(job)) {
        pending_ = std::move(job);
        job_pending_ = true;
        steps_since_replan_ = 0;
        submit = true;
      }
    }
  }
  if (submit) {
    cv_.notify_one();
  }
}

void PlannerHeftReferenceSource::threadLoop() {
  while (!stop_) {
    Job job;
    {
      std::unique_lock<std::mutex> lock(mu_);
      cv_.wait(lock, [this] { return stop_ || job_pending_; });
      if (stop_) {
        break;
      }
      job = std::move(pending_);
      job_pending_ = false;
      busy_ = true;
    }
    FlowPlanner::Plan plan;
    const bool ok =
        planner_.plan(job.p_tilde, job.m_cmd, job.warm, job.root_fill, plan);
    {
      std::lock_guard<std::mutex> lock(mu_);
      if (ok) {
        writePlan(plan);
      } else {
        RL_LOG_WARNING("HEFT planner replan failed; keeping last buffer");
      }
      busy_ = false;
    }
  }
}

}  // namespace leju
