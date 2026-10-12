#include "leju-rl-controller/motion/flow_planner.h"

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <random>

#include <nlohmann/json.hpp>

#include "leju-rl-controller/rl_log.h"

#ifdef USE_ONNXRUNTIME
#include <onnxruntime_cxx_api.h>
#endif

namespace leju {
namespace {

void quatToRot6d(const Eigen::Quaterniond& q, float* out6) {
  const Eigen::Matrix3d R = q.normalized().toRotationMatrix();
  out6[0] = static_cast<float>(R(0, 0));
  out6[1] = static_cast<float>(R(1, 0));
  out6[2] = static_cast<float>(R(2, 0));
  out6[3] = static_cast<float>(R(0, 1));
  out6[4] = static_cast<float>(R(1, 1));
  out6[5] = static_cast<float>(R(2, 1));
}

Eigen::Matrix3f rot6dToMatrix(const float* d6) {
  Eigen::Vector3f a1(d6[0], d6[1], d6[2]);
  Eigen::Vector3f a2(d6[3], d6[4], d6[5]);
  Eigen::Vector3f b1 = a1.normalized();
  Eigen::Vector3f b2 = (a2 - b1 * b1.dot(a2)).normalized();
  Eigen::Vector3f b3 = b1.cross(b2);
  Eigen::Matrix3f R;
  R.col(0) = b1;
  R.col(1) = b2;
  R.col(2) = b3;
  return R;
}

void matrixToRot6d(const Eigen::Matrix3f& R, float* out6) {
  out6[0] = R(0, 0);
  out6[1] = R(1, 0);
  out6[2] = R(2, 0);
  out6[3] = R(0, 1);
  out6[4] = R(1, 1);
  out6[5] = R(2, 1);
}

Eigen::Quaterniond matrixToQuat(const Eigen::Matrix3f& Rf) {
  Eigen::Quaternionf q(Rf);
  q.normalize();
  return Eigen::Quaterniond(q.cast<double>());
}

void makePinned(int root_dim, int joint_dim, std::vector<float>& token) {
  token.assign(root_dim + 6 + joint_dim, 0.f);
  token[root_dim + 0] = 1.f;
  token[root_dim + 4] = 1.f;
}

void stateToResidual(const float* anchor, const float* target, int root_dim,
                     int joint_dim, float* residual) {
  const int r6 = root_dim;
  const int q0 = root_dim + 6;
  for (int i = 0; i < root_dim; ++i) {
    residual[i] = target[i] - anchor[i];
  }
  const Eigen::Matrix3f R0 = rot6dToMatrix(anchor + r6);
  const Eigen::Matrix3f Rk = rot6dToMatrix(target + r6);
  const Eigen::Matrix3f Rrel = Rk * R0.transpose();
  matrixToRot6d(Rrel, residual + r6);
  for (int i = 0; i < joint_dim; ++i) {
    residual[q0 + i] = target[q0 + i] - anchor[q0 + i];
  }
}

void residualToState(const float* anchor, const float* residual, int root_dim,
                     int joint_dim, float* state) {
  const int r6 = root_dim;
  const int q0 = root_dim + 6;
  for (int i = 0; i < root_dim; ++i) {
    state[i] = anchor[i] + residual[i];
  }
  const Eigen::Matrix3f R0 = rot6dToMatrix(anchor + r6);
  const Eigen::Matrix3f Rrel = rot6dToMatrix(residual + r6);
  matrixToRot6d(Rrel * R0, state + r6);
  for (int i = 0; i < joint_dim; ++i) {
    state[q0 + i] = anchor[q0 + i] + residual[q0 + i];
  }
}

void normalizeKD(const std::vector<float>& mean, const std::vector<float>& stdv,
                 int K, int D, std::vector<float>& x) {
  const bool per_k = static_cast<int>(mean.size()) == K * D;
  for (int k = 0; k < K; ++k) {
    for (int d = 0; d < D; ++d) {
      const float m = per_k ? mean[k * D + d] : mean[d];
      const float s = per_k ? stdv[k * D + d] : stdv[d];
      x[k * D + d] = (x[k * D + d] - m) / s;
    }
  }
}

void denormalizeKD(const std::vector<float>& mean, const std::vector<float>& stdv,
                   int K, int D, std::vector<float>& x) {
  const bool per_k = static_cast<int>(mean.size()) == K * D;
  for (int k = 0; k < K; ++k) {
    for (int d = 0; d < D; ++d) {
      const float m = per_k ? mean[k * D + d] : mean[d];
      const float s = per_k ? stdv[k * D + d] : stdv[d];
      x[k * D + d] = x[k * D + d] * s + m;
    }
  }
}

void normalizeToken(const std::vector<float>& mean, const std::vector<float>& stdv,
                    int K, int D, int token, const std::vector<float>& raw,
                    std::vector<float>& out) {
  const bool per_k = static_cast<int>(mean.size()) == K * D;
  out.resize(D);
  for (int d = 0; d < D; ++d) {
    const float m = per_k ? mean[token * D + d] : mean[d];
    const float s = per_k ? stdv[token * D + d] : stdv[d];
    out[d] = (raw[d] - m) / s;
  }
}

std::vector<float> readFlat(const nlohmann::json& j) {
  std::vector<float> out;
  if (j.is_number()) {
    out.push_back(j.get<float>());
    return out;
  }
  if (!j.is_array() || j.empty()) {
    return out;
  }
  if (j.front().is_array()) {
    for (const auto& row : j) {
      for (const auto& v : row) {
        out.push_back(v.get<float>());
      }
    }
  } else {
    for (const auto& v : j) {
      out.push_back(v.get<float>());
    }
  }
  return out;
}

std::string siblingMeta(const std::string& onnx) {
  const std::string a = onnx + ".json";
  std::ifstream fa(a);
  if (fa.good()) {
    return a;
  }
  const auto dot = onnx.rfind(".onnx");
  if (dot != std::string::npos) {
    const std::string b = onnx.substr(0, dot) + ".json";
    std::ifstream fb(b);
    if (fb.good()) {
      return b;
    }
  }
  return a;
}

}  // namespace

struct FlowPlanner::Impl {
#ifdef USE_ONNXRUNTIME
  std::unique_ptr<Ort::Env> env;
  std::unique_ptr<Ort::Session> session;
  std::unique_ptr<Ort::SessionOptions> opts;
  std::unique_ptr<Ort::MemoryInfo> mem;
  std::vector<std::string> in_names;
  std::string out_name;
#endif
  std::vector<float> mean;
  std::vector<float> stdv;
  std::mt19937 rng{std::random_device{}()};
};

FlowPlanner::FlowPlanner() : impl_(std::make_unique<Impl>()) {}
FlowPlanner::~FlowPlanner() = default;

bool FlowPlanner::load(const Config& cfg) {
#ifndef USE_ONNXRUNTIME
  RL_LOG_FAILURE("FlowPlanner requires ONNX Runtime");
  return false;
#else
  loaded_ = false;
  ode_steps_ = std::max(1, cfg.ode_steps);
  t_start_ = cfg.t_start;
  control_hz_ = cfg.control_hz;
  memmap_fps_ = cfg.memmap_fps;

  const std::string meta_path = siblingMeta(cfg.onnx_path);
  std::ifstream in(meta_path);
  if (!in) {
    RL_LOG_FAILURE("Flow ONNX meta not found: %s", meta_path.c_str());
    return false;
  }
  nlohmann::json meta;
  try {
    in >> meta;
  } catch (const std::exception& e) {
    RL_LOG_FAILURE("Flow meta parse failed: %s", e.what());
    return false;
  }
  const auto& mc = meta["model_config"];
  num_keyframes_ = mc.value("num_keyframes", 8);
  residual_dim_ = mc.value("residual_dim", 27);
  state_dim_ = mc.value("state_dim", residual_dim_);
  joint_dim_ = mc.value("joint_dim", 21);
  root_dim_ = mc.value("root_dim", 0);
  if (cfg.horizon_frames > 0) {
    horizon_frames_ = cfg.horizon_frames;
  } else if (root_dim_ == 0 && num_keyframes_ == 8) {
    horizon_frames_ = 10;
  } else {
    horizon_frames_ = 20;
  }
  impl_->mean = readFlat(meta["normalizer"]["mean"]);
  impl_->stdv = readFlat(meta["normalizer"]["std"]);
  for (auto& s : impl_->stdv) {
    if (s < 1e-6f) {
      s = 1e-6f;
    }
  }
  const int expect = residual_dim_;
  const int k_d = num_keyframes_ * residual_dim_;
  if (static_cast<int>(impl_->mean.size()) != expect &&
      static_cast<int>(impl_->mean.size()) != k_d) {
    RL_LOG_FAILURE("Flow normalizer mean size %zu, expected %d or %d",
                   impl_->mean.size(), expect, k_d);
    return false;
  }

  try {
    impl_->env = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "heft_flow");
    impl_->opts = std::make_unique<Ort::SessionOptions>();
    impl_->opts->SetIntraOpNumThreads(std::max(1, cfg.intra_op_threads));
    impl_->opts->SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
    impl_->mem = std::make_unique<Ort::MemoryInfo>(
        Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault));
    impl_->session = std::make_unique<Ort::Session>(*impl_->env, cfg.onnx_path.c_str(),
                                                    *impl_->opts);
    Ort::AllocatorWithDefaultOptions alloc;
    impl_->in_names.clear();
    for (size_t i = 0; i < impl_->session->GetInputCount(); ++i) {
      auto n = impl_->session->GetInputNameAllocated(i, alloc);
      impl_->in_names.emplace_back(n.get());
    }
    auto on = impl_->session->GetOutputNameAllocated(0, alloc);
    impl_->out_name = on.get();
  } catch (const std::exception& e) {
    RL_LOG_FAILURE("Flow ONNX load failed: %s", e.what());
    return false;
  }

  loaded_ = true;
  RL_LOG_SUCCESS(
      "Flow planner loaded: %s K=%d D=%d root_dim=%d H=%d ode=%d threads=%d",
      cfg.onnx_path.c_str(), num_keyframes_, residual_dim_, root_dim_,
      horizon_frames_, ode_steps_, cfg.intra_op_threads);
  return true;
#endif
}

std::vector<float> FlowPlanner::packState(const Eigen::Vector3d& pos,
                                          const Eigen::Quaterniond& quat,
                                          const Eigen::VectorXd& q) const {
  std::vector<float> s(state_dim_, 0.f);
  int off = 0;
  if (root_dim_ == 3) {
    s[0] = static_cast<float>(pos.x());
    s[1] = static_cast<float>(pos.y());
    s[2] = static_cast<float>(pos.z());
    off = 3;
  } else if (root_dim_ == 1) {
    s[0] = static_cast<float>(pos.z());
    off = 1;
  }
  quatToRot6d(quat, s.data() + off);
  off += 6;
  const int n = std::min(joint_dim_, static_cast<int>(q.size()));
  for (int i = 0; i < n; ++i) {
    s[off + i] = static_cast<float>(q[i]);
  }
  return s;
}

bool FlowPlanner::plan(const std::vector<float>& p_tilde,
                       const std::vector<float>& m_cmd,
                       const std::vector<float>& warm_start,
                       const std::vector<Eigen::Vector3d>& root_fill,
                       Plan& out) {
#ifndef USE_ONNXRUNTIME
  (void)p_tilde;
  (void)m_cmd;
  (void)warm_start;
  (void)root_fill;
  (void)out;
  return false;
#else
  if (!loaded_ || !impl_->session) {
    return false;
  }
  if (static_cast<int>(p_tilde.size()) != state_dim_ ||
      static_cast<int>(m_cmd.size()) != state_dim_ ||
      static_cast<int>(warm_start.size()) != state_dim_) {
    RL_LOG_FAILURE("Flow plan state_dim mismatch");
    return false;
  }

  const int K = num_keyframes_;
  const int D = residual_dim_;
  std::vector<float> endpoint(D, 0.f);
  stateToResidual(p_tilde.data(), warm_start.data(), root_dim_, joint_dim_,
                  endpoint.data());

  std::vector<float> r0;
  makePinned(root_dim_, joint_dim_, r0);
  std::vector<float> prior(K * D, 0.f);
  for (int k = 0; k < K; ++k) {
    const float a = (K <= 1) ? 0.f : static_cast<float>(k) / static_cast<float>(K - 1);
    for (int d = 0; d < D; ++d) {
      prior[k * D + d] = r0[d] + a * (endpoint[d] - r0[d]);
    }
  }
  for (int d = 0; d < D; ++d) {
    prior[d] = r0[d];
  }

  std::vector<float> x = prior;
  normalizeKD(impl_->mean, impl_->stdv, K, D, x);
  std::vector<float> r0n;
  normalizeToken(impl_->mean, impl_->stdv, K, D, 0, r0, r0n);

  std::normal_distribution<float> n01(0.f, 1.f);
  std::vector<float> eps(K * D);
  for (float& v : eps) {
    v = n01(impl_->rng);
  }

  const float ts = t_start_;
  for (int i = 0; i < K * D; ++i) {
    x[i] = (1.f - ts) * x[i] + ts * eps[i];
  }
  for (int d = 0; d < D; ++d) {
    x[d] = (1.f - ts) * r0n[d] + ts * eps[d];
  }

  if (ts > 0.f) {
    std::vector<float> flow_time(1, 0.f);
    std::vector<float> vel(K * D);
    std::vector<float> p = p_tilde;
    std::vector<float> m = m_cmd;
    const std::vector<int64_t> xs_dims{1, K, D};
    const std::vector<int64_t> t_dims{1};
    const std::vector<int64_t> st_dims{1, state_dim_};
    const std::vector<int64_t> vel_dims{1, K, D};

    for (int step = 0; step < ode_steps_; ++step) {
      const float t_cur =
          ts + (0.f - ts) * static_cast<float>(step) / static_cast<float>(ode_steps_);
      const float t_next =
          ts + (0.f - ts) * static_cast<float>(step + 1) / static_cast<float>(ode_steps_);
      flow_time[0] = t_cur;

      Ort::Value xs_t = Ort::Value::CreateTensor<float>(
          *impl_->mem, x.data(), x.size(), xs_dims.data(), xs_dims.size());
      Ort::Value ft_t = Ort::Value::CreateTensor<float>(
          *impl_->mem, flow_time.data(), 1, t_dims.data(), t_dims.size());
      Ort::Value pt_t = Ort::Value::CreateTensor<float>(
          *impl_->mem, p.data(), p.size(), st_dims.data(), st_dims.size());
      Ort::Value mc_t = Ort::Value::CreateTensor<float>(
          *impl_->mem, m.data(), m.size(), st_dims.data(), st_dims.size());
      Ort::Value vel_t = Ort::Value::CreateTensor<float>(
          *impl_->mem, vel.data(), vel.size(), vel_dims.data(), vel_dims.size());

      std::vector<Ort::Value> feeds;
      std::vector<const char*> feed_names;
      feeds.reserve(impl_->in_names.size());
      feed_names.reserve(impl_->in_names.size());
      for (const auto& n : impl_->in_names) {
        feed_names.push_back(n.c_str());
        if (n == "x_s") {
          feeds.push_back(std::move(xs_t));
        } else if (n == "flow_time") {
          feeds.push_back(std::move(ft_t));
        } else if (n == "p_tilde") {
          feeds.push_back(std::move(pt_t));
        } else if (n == "m_cmd") {
          feeds.push_back(std::move(mc_t));
        } else {
          RL_LOG_FAILURE("Unknown flow ONNX input '%s'", n.c_str());
          return false;
        }
      }
      if (feeds.size() != impl_->in_names.size()) {
        RL_LOG_FAILURE("Flow ONNX input names mismatch");
        return false;
      }
      const char* out_n = impl_->out_name.c_str();
      Ort::RunOptions ro;
      impl_->session->Run(ro, feed_names.data(), feeds.data(), feeds.size(), &out_n,
                          &vel_t, 1);

      const float dt = t_next - t_cur;
      for (int i = 0; i < K * D; ++i) {
        x[i] += dt * vel[i];
      }
      for (int d = 0; d < D; ++d) {
        x[d] = (1.f - t_next) * r0n[d] + t_next * eps[d];
      }
    }
  }

  denormalizeKD(impl_->mean, impl_->stdv, K, D, x);
  for (int d = 0; d < D; ++d) {
    x[d] = r0[d];
  }

  std::vector<float> keys(K * state_dim_);
  for (int k = 0; k < K; ++k) {
    residualToState(p_tilde.data(), x.data() + k * D, root_dim_, joint_dim_,
                    keys.data() + k * state_dim_);
  }

  const float horizon_s = static_cast<float>(horizon_frames_) / memmap_fps_;
  const int T = std::max(2, static_cast<int>(std::lround(horizon_s * control_hz_)) + 1);
  if (root_dim_ < 3 && static_cast<int>(root_fill.size()) != T) {
    RL_LOG_FAILURE("Flow root_fill size %zu != dense T %d", root_fill.size(), T);
    return false;
  }

  out.pos.resize(T);
  out.quat.resize(T);
  out.jpos.resize(T);
  for (int t = 0; t < T; ++t) {
    const float u = (T == 1) ? 0.f : static_cast<float>(t) / static_cast<float>(T - 1);
    const float xk = u * static_cast<float>(K - 1);
    const int i0 = std::min(K - 2, std::max(0, static_cast<int>(std::floor(xk))));
    const float a = xk - static_cast<float>(i0);
    const int i1 = i0 + 1;

    Eigen::Quaterniond q0 = matrixToQuat(rot6dToMatrix(keys.data() + i0 * state_dim_ + root_dim_));
    Eigen::Quaterniond q1 = matrixToQuat(rot6dToMatrix(keys.data() + i1 * state_dim_ + root_dim_));
    if (q0.dot(q1) < 0.0) {
      q1.coeffs() *= -1.0;
    }
    out.quat[t] = q0.slerp(a, q1);

    out.jpos[t].resize(joint_dim_);
    for (int j = 0; j < joint_dim_; ++j) {
      const float v0 = keys[i0 * state_dim_ + root_dim_ + 6 + j];
      const float v1 = keys[i1 * state_dim_ + root_dim_ + 6 + j];
      out.jpos[t][j] = (1.f - a) * v0 + a * v1;
    }

    if (root_dim_ == 3) {
      Eigen::Vector3d p0, p1;
      for (int j = 0; j < 3; ++j) {
        p0[j] = keys[i0 * state_dim_ + j];
        p1[j] = keys[i1 * state_dim_ + j];
      }
      out.pos[t] = (1.0 - a) * p0 + a * p1;
    } else {
      out.pos[t] = root_fill[t];
      if (root_dim_ == 1) {
        const float z0 = keys[i0 * state_dim_ + 0];
        const float z1 = keys[i1 * state_dim_ + 0];
        out.pos[t].z() = (1.f - a) * z0 + a * z1;
      }
    }
  }
  return true;
#endif
}

}  // namespace leju
