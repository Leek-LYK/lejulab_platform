#include "trajectory_publisher.h"

#include <chrono>
#include <iostream>
#include <thread>

namespace leju {
namespace vr {
namespace tact_player {
namespace {
constexpr double kPublishHz = 100.0;
}

TrajectoryPublisher::TrajectoryPublisher(VRBaseAPI* api,
                                         const BezierInterpolator* interpolator,
                                         std::atomic<bool>* running_flag)
    : api_(api),
      interpolator_(interpolator),
      running_flag_(running_flag) {}

bool TrajectoryPublisher::Run() {
  if (api_ == nullptr || interpolator_ == nullptr || running_flag_ == nullptr) {
    std::cerr << "invalid publisher dependencies" << std::endl;
    return false;
  }
  const auto period = std::chrono::duration<double>(1.0 / kPublishHz);

  while (running_flag_->load()) {
    const double total = interpolator_->Duration();
    const auto start_time = std::chrono::steady_clock::now();

    while (running_flag_->load()) {
      const auto now = std::chrono::steady_clock::now();
      const double elapsed = std::chrono::duration<double>(now - start_time).count();
      if (elapsed > total) {
        break;
      }

      const auto sample = interpolator_->Evaluate(elapsed);
      JointTrajectoryPoint cmd;
      cmd.q = sample.q;
      cmd.v = sample.v;
      cmd.acc = sample.acc;
      if (!api_->publishArmJointCmd(cmd)) {
        std::cerr << "publishArmJointCmd failed" << std::endl;
      }

      // Publish waist command if available
      // 注意：Roban 腰部关节方向与 tact 文件定义相反，需要取反
      if (sample.has_waist) {
        JointTrajectoryPoint waist_cmd;
        waist_cmd.q = {-sample.waist_q};
        waist_cmd.v = {-sample.waist_v};
        waist_cmd.acc = {-sample.waist_acc};
        if (!api_->publishWaistJointCmd(waist_cmd)) {
          std::cerr << "publishWaistJointCmd failed" << std::endl;
        }
      }

      const auto cycle_end = std::chrono::steady_clock::now();
      const auto elapsed_cycle = cycle_end - now;
      if (elapsed_cycle < period) {
        std::this_thread::sleep_for(period - elapsed_cycle);
      }
    }

    if (!running_flag_->load()) {
      break;
    }

    // ensure last sample is sent at trajectory end.
    const auto end_sample = interpolator_->Evaluate(total);
    JointTrajectoryPoint end_cmd;
    end_cmd.q = end_sample.q;
    end_cmd.v = end_sample.v;
    end_cmd.acc = end_sample.acc;
    api_->publishArmJointCmd(end_cmd);

    // Publish final waist command if available
    if (end_sample.has_waist) {
      JointTrajectoryPoint waist_end_cmd;
      waist_end_cmd.q = {-end_sample.waist_q};
      waist_end_cmd.v = {-end_sample.waist_v};
      waist_end_cmd.acc = {-end_sample.waist_acc};
      api_->publishWaistJointCmd(waist_end_cmd);
    }
    break;
  }

  return true;
}

}  // namespace tact_player
}  // namespace vr
}  // namespace leju
