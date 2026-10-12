#include "bezier_interpolator.h"

#include <algorithm>
#include <cmath>

namespace leju {
namespace vr {
namespace tact_player {
namespace {

constexpr double kKeyframeToSec = 0.01;
constexpr double kEps = 1e-9;
constexpr double kPi = 3.14159265358979323846;

}  // namespace

double BezierInterpolator::DegToRad(double deg) {
  return deg * kPi / 180.0;
}

bool BezierInterpolator::Build(const TactAction& action,
                               const InterpolateOptions& options,
                               std::string* error_message) {
  if (error_message == nullptr) {
    return false;
  }
  if (options.arm_dof == 0) {
    *error_message = "arm_dof must be > 0";
    return false;
  }
  if (options.speed_scale <= 0.0) {
    *error_message = "speed_scale must be > 0";
    return false;
  }
  if (action.frames.size() < 2) {
    *error_message = "tact frames must contain at least 2 keyframes";
    return false;
  }

  arm_dof_ = options.arm_dof;
  first_sec_ = action.first_sec;
  finish_sec_ = action.finish_sec;
  speed_scale_ = options.speed_scale;
  has_waist_ = options.enable_waist && action.has_waist;

  tracks_.assign(arm_dof_, {});
  waist_track_.clear();

  for (std::size_t joint = 0; joint < arm_dof_; ++joint) {
    auto& track = tracks_[joint];
    for (std::size_t i = 1; i < action.frames.size(); ++i) {
      const auto& prev = action.frames[i - 1];
      const auto& curr = action.frames[i];

      const double t0 = prev.keyframe * kKeyframeToSec;
      const double t1 = curr.keyframe * kKeyframeToSec;
      if (t1 - t0 <= kEps) {
        continue;
      }

      CubicBezierSegment seg;
      seg.t0 = t0;
      seg.t1 = t1;

      const double prev_deg = prev.servos_deg[joint];
      const double curr_deg = curr.servos_deg[joint];
      const auto& prev_right_cp = prev.attributes[joint].cp[1];
      const auto& curr_left_cp = curr.attributes[joint].cp[0];

      seg.y0 = DegToRad(prev_deg);
      seg.y1 = DegToRad(prev_deg + prev_right_cp[1]);
      seg.y2 = DegToRad(curr_deg + curr_left_cp[1]);
      seg.y3 = DegToRad(curr_deg);
      track.push_back(seg);
    }

    if (track.empty()) {
      *error_message = "joint " + std::to_string(joint) + " has no valid segment";
      return false;
    }
  }

  // Build waist track if enabled
  if (has_waist_) {
    for (std::size_t i = 1; i < action.frames.size(); ++i) {
      const auto& prev = action.frames[i - 1];
      const auto& curr = action.frames[i];

      if (!prev.has_waist || !curr.has_waist) {
        continue;
      }

      const double t0 = prev.keyframe * kKeyframeToSec;
      const double t1 = curr.keyframe * kKeyframeToSec;
      if (t1 - t0 <= kEps) {
        continue;
      }

      CubicBezierSegment seg;
      seg.t0 = t0;
      seg.t1 = t1;

      const double prev_deg = prev.waist_deg;
      const double curr_deg = curr.waist_deg;
      const auto& prev_right_cp = prev.waist_attribute.cp[1];
      const auto& curr_left_cp = curr.waist_attribute.cp[0];

      seg.y0 = DegToRad(prev_deg);
      seg.y1 = DegToRad(prev_deg + prev_right_cp[1]);
      seg.y2 = DegToRad(curr_deg + curr_left_cp[1]);
      seg.y3 = DegToRad(curr_deg);
      waist_track_.push_back(seg);
    }

    if (waist_track_.empty()) {
      // No valid waist segments, disable waist
      has_waist_ = false;
    }
  }

  if (finish_sec_ <= first_sec_) {
    finish_sec_ = action.frames.back().keyframe * kKeyframeToSec;
  }
  finish_sec_ = std::max(finish_sec_, tracks_.front().back().t1);
  first_sec_ = std::max(first_sec_, tracks_.front().front().t0);

  if (finish_sec_ - first_sec_ <= kEps) {
    *error_message = "invalid play range, finish <= first";
    return false;
  }

  duration_sec_ = (finish_sec_ - first_sec_) / speed_scale_;
  return true;
}

double BezierInterpolator::EvalPosition(const CubicBezierSegment& seg, double u) {
  const double one_minus_u = 1.0 - u;
  return one_minus_u * one_minus_u * one_minus_u * seg.y0 +
         3.0 * one_minus_u * one_minus_u * u * seg.y1 +
         3.0 * one_minus_u * u * u * seg.y2 +
         u * u * u * seg.y3;
}

double BezierInterpolator::EvalVelocity(const CubicBezierSegment& seg, double u) {
  const double dt = seg.t1 - seg.t0;
  const double one_minus_u = 1.0 - u;
  const double dy_du = 3.0 * one_minus_u * one_minus_u * (seg.y1 - seg.y0) +
                       6.0 * one_minus_u * u * (seg.y2 - seg.y1) +
                       3.0 * u * u * (seg.y3 - seg.y2);
  return dy_du / dt;
}

double BezierInterpolator::EvalAcceleration(const CubicBezierSegment& seg, double u) {
  const double dt = seg.t1 - seg.t0;
  const double d2y_du2 =
      6.0 * (1.0 - u) * (seg.y2 - 2.0 * seg.y1 + seg.y0) +
      6.0 * u * (seg.y3 - 2.0 * seg.y2 + seg.y1);
  return d2y_du2 / (dt * dt);
}

JointTrajectorySample BezierInterpolator::Evaluate(double elapsed_sec) const {
  JointTrajectorySample out;
  out.q.assign(arm_dof_, 0.0);
  out.v.assign(arm_dof_, 0.0);
  out.acc.assign(arm_dof_, 0.0);

  const double clamped_elapsed = std::clamp(elapsed_sec, 0.0, duration_sec_);
  const double query_time = std::clamp(first_sec_ + clamped_elapsed * speed_scale_, first_sec_, finish_sec_);

  for (std::size_t joint = 0; joint < arm_dof_; ++joint) {
    const auto& track = tracks_[joint];
    auto it = std::find_if(track.begin(), track.end(), [query_time](const CubicBezierSegment& seg) {
      return query_time <= seg.t1;
    });

    if (it == track.end()) {
      it = std::prev(track.end());
    }

    const auto& seg = *it;
    const double dt = seg.t1 - seg.t0;
    if (dt <= kEps) {
      out.q[joint] = seg.y3;
      out.v[joint] = 0.0;
      out.acc[joint] = 0.0;
      continue;
    }

    const double u = std::clamp((query_time - seg.t0) / dt, 0.0, 1.0);
    out.q[joint] = EvalPosition(seg, u);
    out.v[joint] = EvalVelocity(seg, u);
    out.acc[joint] = EvalAcceleration(seg, u);
  }

  // Evaluate waist if enabled
  if (has_waist_ && !waist_track_.empty()) {
    out.has_waist = true;

    auto it = std::find_if(waist_track_.begin(), waist_track_.end(),
                           [query_time](const CubicBezierSegment& seg) {
                             return query_time <= seg.t1;
                           });

    if (it == waist_track_.end()) {
      it = std::prev(waist_track_.end());
    }

    const auto& seg = *it;
    const double dt = seg.t1 - seg.t0;
    if (dt <= kEps) {
      out.waist_q = seg.y3;
      out.waist_v = 0.0;
      out.waist_acc = 0.0;
    } else {
      const double u = std::clamp((query_time - seg.t0) / dt, 0.0, 1.0);
      out.waist_q = EvalPosition(seg, u);
      out.waist_v = EvalVelocity(seg, u);
      out.waist_acc = EvalAcceleration(seg, u);
    }
  }

  return out;
}

}  // namespace tact_player
}  // namespace vr
}  // namespace leju
