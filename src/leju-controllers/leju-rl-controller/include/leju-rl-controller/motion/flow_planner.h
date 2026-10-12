#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace leju {

/// Flow-matching velocity field (ONNX) + Euler ODE + densify.
/// Port of infer_student_flow.py / sample_one_window (CPU ORT).
class FlowPlanner {
 public:
  struct Config {
    std::string onnx_path;
    int ode_steps = 5;
    float t_start = 0.7f;
    float control_hz = 50.f;
    int horizon_frames = -1;  ///< <0: auto (10 if root_dim=0 && K=8, else 20)
    int intra_op_threads = 2;
    float memmap_fps = 50.f;
  };

  struct Plan {
    std::vector<Eigen::Vector3d> pos;
    std::vector<Eigen::Quaterniond> quat;
    std::vector<Eigen::VectorXd> jpos;
  };

  FlowPlanner();
  ~FlowPlanner();

  FlowPlanner(const FlowPlanner&) = delete;
  FlowPlanner& operator=(const FlowPlanner&) = delete;

  bool load(const Config& cfg);
  bool loaded() const { return loaded_; }

  int rootDim() const { return root_dim_; }
  int jointDim() const { return joint_dim_; }
  int stateDim() const { return state_dim_; }
  int numKeyframes() const { return num_keyframes_; }
  int horizonFrames() const { return horizon_frames_; }

  std::vector<float> packState(const Eigen::Vector3d& pos,
                               const Eigen::Quaterniond& quat,
                               const Eigen::VectorXd& q) const;

  /// One receding-horizon sample. ``root_fill`` used when root_dim < 3
  /// (absolute XY from CSV); length must match densified T.
  bool plan(const std::vector<float>& p_tilde,
            const std::vector<float>& m_cmd,
            const std::vector<float>& warm_start,
            const std::vector<Eigen::Vector3d>& root_fill,
            Plan& out);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
  bool loaded_ = false;
  int root_dim_ = 0;
  int joint_dim_ = 21;
  int state_dim_ = 27;
  int residual_dim_ = 27;
  int num_keyframes_ = 8;
  int horizon_frames_ = 10;
  int ode_steps_ = 5;
  float t_start_ = 0.7f;
  float control_hz_ = 50.f;
  float memmap_fps_ = 50.f;
};

}  // namespace leju
