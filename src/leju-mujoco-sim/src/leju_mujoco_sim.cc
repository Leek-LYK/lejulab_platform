// Copyright 2021 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <fstream>
#include <memory>
#include <mutex>
#include <condition_variable>
#include <new>
#include <string>
#include <thread>
#include <vector>
#include <cmath>
#include <algorithm>
#include <array>
#include <random>
#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>

#include <mujoco/mujoco.h>
#include <mujoco/mjthread.h>
#include "simulate/glfw_adapter.h"
#include "simulate/simulate.h"
#include "simulate/array_safety.h"
#include "leju-mujoco-sim/depth_camera_scheduler.h"

#include "lejusdk-lowlevel/leju_sdk.h"
#include "lejusdk-lowlevel/robot_transmission/robot_transmission.h"
#include "kuavo_solver/ankle_solver.h"
#include "lejusdk-utils/time_utils.hpp"
#include <dds/dds.hpp>
#include <lejusdk-dds-idl/Float64Types.hpp>
#include <lejusdk-topic-pubsub/topic_publisher.hpp>
#include <lejusdk-topic-pubsub/topic_subscriber.hpp>
#ifdef LEJU_HAS_OPENCV
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#endif

namespace {

using leju::mujoco_sim::DepthPublishCadence;
using leju::mujoco_sim::PersistentTaskGroup;

std::string DefaultDepthWalkConfigPath() {
  const char* version = std::getenv("ROBOT_VERSION");
  const std::string ver = (version && version[0] != '\0') ? version : "17";
  return "src/leju-controllers/leju-rl-controller/config/" + ver +
         "/controllers/config_depth_walk.yaml";
}

double LoadDepthNoiseStdM(const std::string& yaml_path, double fallback) {
  try {
    const auto node =
        YAML::LoadFile(yaml_path)["HumanoidRobotCfg"]["depth"]["noise_std_m"];
    if (!node || node.IsNull()) {
      std::cerr << "[depth] noise_std_m missing in " << yaml_path
                << ", using " << fallback << " m" << std::endl;
      return fallback;
    }
    const double value = std::max(0.0, node.as<double>());
    std::cout << "[depth] noise_std_m=" << value << " m from " << yaml_path
              << std::endl;
    return value;
  } catch (const std::exception& e) {
    std::cerr << "[depth] failed to read " << yaml_path << ": " << e.what()
              << ", using noise_std_m=" << fallback << " m" << std::endl;
    return fallback;
  }
}

// MuJoCo ray-based depth publisher.  It deliberately uses the same small
// internal DDS frame as the controller, so headless simulation and a real
// camera can share the exact observation path without ROS or OpenGL.
class DepthDdsPublisher {
 public:
  static constexpr int kWidth = 64;
  static constexpr int kHeight = 36;
  static constexpr double kMaxDepth = 2.5;
  static constexpr double kMinDepth = 0.17;
  // 与训练端 PinholeCameraPatternCfg 一致（focal=2.12, 424x240 源分辨率 fx≈230.455 fy≈230.452）。
  // HFOV≈85.2°, VFOV≈55.0°；不使用 MJCF camera 的 fovy=65 直接推导。
  static constexpr double kFocalLength = 2.12;
  // static constexpr double kHorizontalAperture = 2.12 * 424.0 / 230.455;
  // static constexpr double kVerticalAperture = 2.12* 240.0 / 230.452;
  static constexpr double kHorizontalAperture = 4.24;
  static constexpr double kVerticalAperture = 2.4480;
  static constexpr int kRayThreads = 16;

  explicit DepthDdsPublisher(double frequency_hz, bool dump_depth = false,
                             bool show_depth = false, double noise_std_m = 0.05)
      : participant_(0),
        publisher_(participant_, "rt/depth_camera/frame_meters_36x64"),
        tasks_(kRayThreads),
        cadence_(std::chrono::duration_cast<DepthPublishCadence::Clock::duration>(
            std::chrono::duration<double>(
                1.0 / std::max(1.0, frequency_hz)))),
        ray_workers_(kRayThreads),
        frame_(kWidth * kHeight, kMaxDepth),
        blurred_(kWidth * kHeight, kMaxDepth),
        noise_std_m_(std::max(0.0, noise_std_m)),
        dump_depth_(dump_depth),
        show_depth_(show_depth),
        worker_(&DepthDdsPublisher::workerLoop, this) {
    const double radians_to_degrees = 180.0 / std::acos(-1.0);
    const double horizontal_fov =
        2.0 * std::atan(kHorizontalAperture / (2.0 * kFocalLength)) *
        radians_to_degrees;
    const double vertical_fov =
        2.0 * std::atan(kVerticalAperture / (2.0 * kFocalLength)) *
        radians_to_degrees;
    std::cout << "[depth] camera: focal=" << kFocalLength
              << " aperture=" << kHorizontalAperture << "x"
              << kVerticalAperture << " FOV=" << horizontal_fov << "x"
              << vertical_fov << " deg resolution=" << kWidth << "x"
              << kHeight << std::endl;
  }

  ~DepthDdsPublisher() {
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      stop_worker_ = true;
    }
    queue_cv_.notify_one();
    if (worker_.joinable()) worker_.join();
    clearRayModel();
    for (mjData*& snapshot : snapshots_) {
      if (snapshot) mj_deleteData(snapshot);
      snapshot = nullptr;
    }
  }

  // Reserve the 30 Hz slot in the physics thread before copying. This avoids
  // repeatedly cloning the same frame while the camera worker is waking up.
  // Two owned mjData slots are allocated once and then alternated between the
  // physics producer and ray-casting consumer.
  bool captureIfDue(const mjModel* model, const mjData* source) {
    if (!cadence_.tryReserve()) return false;
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (!ensureSnapshotsLocked(model)) return false;
    const int slot = pending_slot_ >= 0
        ? pending_slot_
        : (processing_slot_ == 0 ? 1 : 0);
    mj_copyData(snapshots_[slot], model, source);
    pending_slot_ = slot;
    queue_cv_.notify_one();
    return true;
  }

  void prepareForModelReload() {
    std::unique_lock<std::mutex> lock(queue_mutex_);
    pending_slot_ = -1;
    idle_cv_.wait(lock, [this] { return processing_slot_ < 0; });
    clearSnapshotsLocked();
    clearRayModel();
  }

 private:
  bool ensureSnapshotsLocked(const mjModel* model) {
    if (snapshot_model_ == model && snapshots_[0] && snapshots_[1]) {
      return true;
    }
    if (processing_slot_ >= 0) return false;
    clearSnapshotsLocked();
    snapshot_model_ = model;
    snapshots_[0] = mj_makeData(model);
    snapshots_[1] = mj_makeData(model);
    if (!snapshots_[0] || !snapshots_[1]) {
      clearSnapshotsLocked();
      return false;
    }
    return true;
  }

  void clearSnapshotsLocked() {
    for (mjData*& snapshot : snapshots_) {
      if (snapshot) mj_deleteData(snapshot);
      snapshot = nullptr;
    }
    snapshot_model_ = nullptr;
    pending_slot_ = -1;
  }

  bool ensureRayModel(const mjModel* source_model, const mjData* source_data) {
    if (ray_source_model_ == source_model && ray_model_ && ray_data_) return true;
    clearRayModel();
    ray_model_ = mj_copyModel(nullptr, source_model);
    if (!ray_model_) return false;
    ray_data_ = mj_makeData(ray_model_);
    if (!ray_data_) {
      clearRayModel();
      return false;
    }
    for (int i = 0; i < ray_model_->ngeom; ++i) {
      if (ray_model_->geom_type[i] == mjGEOM_MESH) ray_model_->geom_group[i] = 2;
    }
    ray_source_model_ = source_model;
    mju_copy(ray_data_->qpos, source_data->qpos, source_model->nq);
    mju_copy(ray_data_->qvel, source_data->qvel, source_model->nv);
    mj_forward(ray_model_, ray_data_);
    return true;
  }

  void clearRayModel() {
    if (ray_data_) mj_deleteData(ray_data_);
    if (ray_model_) mj_deleteModel(ray_model_);
    ray_data_ = nullptr;
    ray_model_ = nullptr;
    ray_source_model_ = nullptr;
    ray_directions_model_ = nullptr;
    ray_camera_id_ = -1;
    ray_directions_.clear();
  }

  void workerLoop() {
    for (;;) {
      const mjModel* model = nullptr;
      mjData* data = nullptr;
      int slot = -1;
      {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        queue_cv_.wait(lock,
                       [this] { return stop_worker_ || pending_slot_ >= 0; });
        if (stop_worker_ && pending_slot_ < 0) return;
        slot = pending_slot_;
        pending_slot_ = -1;
        processing_slot_ = slot;
        model = snapshot_model_;
        data = snapshots_[slot];
      }
      publishNow(model, data);
      {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        processing_slot_ = -1;
      }
      idle_cv_.notify_all();
    }
  }

  void publishNow(const mjModel* model, const mjData* data) {
    // MuJoCo 3.2 has a severe mesh-ray traversal regression on S17. Keep the
    // simulation model intact for rendering, but ray-cast against a private
    // clone whose visual mesh geoms are outside the source {0,1} ray mask.
    if (!ensureRayModel(model, data)) return;
    const mjModel* ray_model = ray_model_;
    mjData* ray_data = ray_data_;
    const int camera_id = mj_name2id(ray_model, mjOBJ_CAMERA, "camera_depth_frame");
    if (camera_id < 0) return;
    ensureRayDirections(ray_model, camera_id);
    mju_copy(ray_data->qpos, data->qpos, model->nq);
    mju_copy(ray_data->qvel, data->qvel, model->nv);
    if (model->na > 0) mju_copy(ray_data->act, data->act, model->na);
    ray_data->time = data->time;
    mj_forward(ray_model, ray_data);
    const mjtNum* origin = ray_data->cam_xpos + 3 * camera_id;
    const mjtNum* rotation = ray_data->cam_xmat + 9 * camera_id;
    std::array<mjtNum, 3> origin_copy{origin[0], origin[1], origin[2]};
    std::array<mjtNum, 9> rotation_copy{};
    std::copy(rotation, rotation + 9, rotation_copy.begin());
    std::fill(frame_.begin(), frame_.end(), kMaxDepth);
    RayTaskData main_task{this, ray_model, ray_data, origin_copy, rotation_copy, &frame_,
                          0, 0};
    const int chunk = static_cast<int>(ray_directions_.size()) /
                      (kRayThreads + 1);
    main_task.end = chunk;
    for (int i = 0; i < kRayThreads; ++i) {
      tasks_[i] = RayTaskData{this, ray_model, ray_data, origin_copy, rotation_copy,
                              &frame_, (i + 1) * chunk,
                              (i + 2) * chunk};
      if (i == kRayThreads - 1) tasks_[i].end = ray_directions_.size();
    }
    ray_workers_.dispatch(
        [this](size_t index) { computeRange(tasks_[index]); });
    computeRange(main_task);
    ray_workers_.wait();

    // Match the training-side range-dependent depth noise: perturb each
    // depth pixel in meters before the camera blur and DDS publication.
    // Sigma comes from HumanoidRobotCfg.depth.noise_std_m in
    // config_depth_walk.yaml (0 disables noise).
    if (noise_std_m_ > 0.0) {
      std::normal_distribution<double> depth_noise(0.0, noise_std_m_);
      for (double& value : frame_) {
        if (value >= kMinDepth && value <= kMaxDepth) {
          value = std::clamp(value + depth_noise(rng_), kMinDepth, kMaxDepth);
        }
      }
    }

    // Source RayCasterCamera applies the same 3x3 Gaussian blur before the
    // history buffer.  Keep it on the meter-valued frame; the controller then
    // performs the final /2.5 normalization.
    blurred_ = frame_;
    auto& blurred = blurred_;
    constexpr double kernel[3][3] = {
        {1.0 / 16.0, 2.0 / 16.0, 1.0 / 16.0},
        {2.0 / 16.0, 4.0 / 16.0, 2.0 / 16.0},
        {1.0 / 16.0, 2.0 / 16.0, 1.0 / 16.0}};
    for (int y = 0; y < kHeight; ++y) {
      for (int x = 0; x < kWidth; ++x) {
        double value = 0.0;
        for (int ky = -1; ky <= 1; ++ky) {
          for (int kx = -1; kx <= 1; ++kx) {
            const int sy = std::clamp(y + ky, 0, kHeight - 1);
            const int sx = std::clamp(x + kx, 0, kWidth - 1);
            value += kernel[ky + 1][kx + 1] * frame_[sy * kWidth + sx];
          }
        }
        blurred[y * kWidth + x] = value;
      }
    }
    int32_t sec = 0;
    uint32_t nanosec = 0;
    leju::common::GetUnixTimestampSecNsec(sec, nanosec);
    publisher_.publish(leju::msgs::Float64Array(sec, nanosec, blurred));
    if (show_depth_) {
#ifdef LEJU_HAS_OPENCV
      const auto [min_it, max_it] =
          std::minmax_element(blurred.begin(), blurred.end());
      const double min_value = *min_it;
      const double max_value = *max_it;
      const double auto_span = std::max(1e-6, max_value - min_value);
      cv::Mat fixed_gray(kHeight, kWidth, CV_8UC1);
      cv::Mat auto_gray(kHeight, kWidth, CV_8UC1);
      for (int i = 0; i < kWidth * kHeight; ++i) {
        const double value = blurred[static_cast<size_t>(i)];
        fixed_gray.data[i] = static_cast<uint8_t>(std::clamp(
            255.0 * (1.0 - blurred[static_cast<size_t>(i)] / kMaxDepth),
            0.0, 255.0));
        auto_gray.data[i] = static_cast<uint8_t>(std::clamp(
            255.0 * (max_value - value) / auto_span, 0.0, 255.0));
      }
      cv::Mat auto_color;
      cv::applyColorMap(auto_gray, auto_color, cv::COLORMAP_TURBO);

      // Never draw diagnostics onto the policy-sized 64x36 frame: doing so
      // obscures a large fraction of the useful image. The left panel keeps
      // the source image_raw grayscale semantics; the right panel is an RViz-like,
      // contrast-enhanced visualization only and is never published to DDS.
      cv::Mat fixed_display;
      cv::Mat auto_display;
      cv::Mat fixed_color;
      cv::cvtColor(fixed_gray, fixed_color, cv::COLOR_GRAY2BGR);
      cv::resize(fixed_color, fixed_display, cv::Size(640, 360), 0.0, 0.0,
                 cv::INTER_NEAREST);
      cv::resize(auto_color, auto_display, cv::Size(640, 360), 0.0, 0.0,
                 cv::INTER_CUBIC);
      cv::putText(fixed_display, "policy pixels: fixed 0-2.5m",
                  cv::Point(12, 26), cv::FONT_HERSHEY_SIMPLEX, 0.58,
                  cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
      cv::putText(auto_display, "preview only: auto contrast",
                  cv::Point(12, 26), cv::FONT_HERSHEY_SIMPLEX, 0.58,
                  cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
      const std::string stats =
          "64x36 min=" + std::to_string(min_value) +
          " max=" + std::to_string(max_value);
      cv::putText(auto_display, stats, cv::Point(12, 348),
                  cv::FONT_HERSHEY_SIMPLEX, 0.52,
                  cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
      cv::Mat preview;
      cv::hconcat(fixed_display, auto_display, preview);
      cv::namedWindow("DDS Head Depth", cv::WINDOW_NORMAL | cv::WINDOW_GUI_EXPANDED);
      cv::resizeWindow("DDS Head Depth", 1280, 360);
      cv::imshow("DDS Head Depth", preview);
      cv::waitKey(1);
#else
      static bool warned = false;
      if (!warned) {
        std::cerr << "[depth] --show-depth requested but OpenCV highgui is unavailable" << std::endl;
        warned = true;
      }
#endif
    }
    if (dump_depth_) {
      double min_value = kMaxDepth;
      double max_value = 0.0;
      double sum = 0.0;
      for (double value : blurred) {
        min_value = std::min(min_value, value);
        max_value = std::max(max_value, value);
        sum += value;
      }
      std::ofstream out("/tmp/leju_depth_camera.pgm", std::ios::trunc);
      if (out) {
        out << "P2\n" << kWidth << " " << kHeight << "\n255\n";
        for (double value : blurred) {
          const int gray = static_cast<int>(std::clamp(
              255.0 * (1.0 - value / kMaxDepth), 0.0, 255.0));
          out << gray << '\n';
        }
      }
      static int stats_count = 0;
      if ((stats_count++ % 30) == 0) {
        std::cerr << "[depth] min=" << min_value << " max=" << max_value
                  << " mean=" << (sum / blurred.size())
                  << " file=/tmp/leju_depth_camera.pgm" << std::endl;
      }
    }
  }

 public:
  struct RayTaskData {
    DepthDdsPublisher* owner;
    const mjModel* model;
    const mjData* data;
    std::array<mjtNum, 3> origin;
    std::array<mjtNum, 9> rotation;
    std::vector<double>* frame;
    int start;
    int end;
  };

  static void* rayTask(void* args) {
    auto* task = static_cast<RayTaskData*>(args);
    task->owner->computeRange(*task);
    return nullptr;
  }

  void ensureRayDirections(const mjModel* model, int camera_id) {
    if (model == ray_directions_model_ && camera_id == ray_camera_id_) return;
    ray_directions_model_ = model;
    ray_camera_id_ = camera_id;
    ray_directions_.resize(kWidth * kHeight);
    const double pixel_size_x = kHorizontalAperture / kWidth;
    const double pixel_size_y = kVerticalAperture / kHeight;
    for (int y = 0; y < kHeight; ++y) {
      for (int x = 0; x < kWidth; ++x) {
        mjtNum ray[3] = {
            (x + 0.5 - kWidth / 2.0) * pixel_size_x / kFocalLength,
            (kHeight / 2.0 - y - 0.5) * pixel_size_y / kFocalLength, -1.0};
        mju_normalize3(ray);
        ray_directions_[y * kWidth + x] = {ray[0], ray[1], ray[2]};
      }
    }
  }

  void computeRange(const RayTaskData& task) {
    const int body_exclude = task.model->cam_bodyid[ray_camera_id_];
    const mjtByte geomgroup[mjNGROUP] = {1, 1, 0, 0, 0, 0};
    for (int i = task.start; i < task.end; ++i) {
      const auto& ray = ray_directions_[i];
      mjtNum world_ray[3] = {
          task.rotation[0] * ray[0] + task.rotation[1] * ray[1] + task.rotation[2] * ray[2],
          task.rotation[3] * ray[0] + task.rotation[4] * ray[1] + task.rotation[5] * ray[2],
          task.rotation[6] * ray[0] + task.rotation[7] * ray[1] + task.rotation[8] * ray[2]};
      int geom_id = -1;
      const mjtNum distance = mj_ray(task.model, task.data, task.origin.data(),
                                     world_ray, geomgroup, 1, body_exclude,
                                     &geom_id);
      // mj_ray receives a unit direction here, so its return value is the
      // Euclidean range along the ray. The ROS RayCasterCamera converts that
      // range to optical-axis (Z) depth with
      //   z = range * focal / ||(pixel_x,pixel_y,focal)||
      // and the unit ray has -ray[2] equal to that same factor. Dividing by
      // -ray[2] stretches the image toward the borders and does not match
      // waist_camera/depth/image_raw.
      double range = kMaxDepth;
      if (distance >= 0.0 && std::isfinite(distance)) {
        range = std::clamp(static_cast<double>(distance), kMinDepth, kMaxDepth);
      }
      double z_depth = kMaxDepth;
      if (ray[2] < -1e-9) z_depth = range * (-ray[2]);
      (*task.frame)[i] = std::clamp(z_depth, kMinDepth, kMaxDepth);
    }
  }

  dds::domain::DomainParticipant participant_;
  leju::dds_common::TopicPublisher<leju::msgs::Float64Array> publisher_;
  std::vector<RayTaskData> tasks_;
  DepthPublishCadence cadence_;
  PersistentTaskGroup ray_workers_;
  std::vector<double> frame_;
  std::vector<double> blurred_;
  double noise_std_m_ = 0.05;
  std::mt19937 rng_{std::random_device{}()};
  mjModel* ray_model_ = nullptr;
  const mjModel* ray_source_model_ = nullptr;
  mjData* ray_data_ = nullptr;
  const mjModel* ray_directions_model_ = nullptr;
  int ray_camera_id_ = -1;
  std::vector<std::array<mjtNum, 3>> ray_directions_;
  bool dump_depth_ = false;
  bool show_depth_ = false;
  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::condition_variable idle_cv_;
  const mjModel* snapshot_model_ = nullptr;
  std::array<mjData*, 2> snapshots_{nullptr, nullptr};
  int pending_slot_ = -1;
  int processing_slot_ = -1;
  bool stop_worker_ = false;
  std::thread worker_;
};

}  // namespace

class LejuMujocoHelper {

 public:
  LejuMujocoHelper() = default;

  ~LejuMujocoHelper() {
    freeGhostResources();
  }

  LejuMujocoHelper(const LejuMujocoHelper&) = delete;
  LejuMujocoHelper& operator=(const LejuMujocoHelper&) = delete;

  void Init(const std::string& filename, bool run_mujoco_camera = true,
            double depth_frequency_hz = 30.0, bool dump_depth = false,
            bool show_depth = false, double depth_noise_std_m = 0.05) {
    // TODO process robottype and robotversion param from input args
    leju::RobotVersion robot_version = leju::RobotVersion::from_env();
    if (!leju::GlobalRobot::init_env(robot_version)) {
        std::cerr << "Call leju::GlobalRobot::init_env() failed." << std::endl;
        return; // TODO deal with this result
    }
    auto& robot = leju::GlobalRobot::getInstance();
    motor_count_ = robot.getMotorNumber();
    motor_names_ = robot.getMotorNames();
    transmission_ = std::make_unique<leju::transmission::RobotTransmission>(
      robot_version
    );
    // The S17 asset is calibrated for the nonlinear S2GEN_2 ankle tendon
    // geometry.  RobotTransmission predates S2GEN_2 and falls back to S2GEN
    // for all Roban2 versions, which changes the joint-to-motor Jacobian and
    // makes an otherwise valid policy command physically inconsistent in
    // simulation.  Keep the legacy path for every other robot version and
    // use the already shipped hardware solver only for S17.
    if (robot_version.major() == 1 && robot_version.minor() == 7) {
      ankle_solver_s17_ = std::make_unique<AnkleSolver>();
      ankle_solver_s17_->getconfig(ANKLE_SOLVER_TYPE_S2GEN_2);
      // Keep the legacy joint-space transmission for the standalone MuJoCo
      // policy path. The S17 MJCF exposes joint-space actuators; applying
      // the hardware S2GEN_2 conversion here changes the policy torque map
      // and destabilizes the simulator.
      use_s2gen2_s17_ = false;
      std::cout << "[mujoco] ROBOT_VERSION=17: using legacy joint transmission" << std::endl;
    }
    if (run_mujoco_camera) {
      try {
      depth_publisher_ = std::make_unique<DepthDdsPublisher>(
          depth_frequency_hz, dump_depth, show_depth, depth_noise_std_m);
      } catch (const std::exception& e) {
        std::cerr << "Depth DDS publisher disabled: " << e.what() << std::endl;
      }
    } else {
      std::cout << "[mujoco] Depth camera disabled by --no-mujoco-camera" << std::endl;
    }

    filename_ = filename;

    {
      std::lock_guard<std::mutex> lock(hand_cmd_mutex_);
      for (size_t i = 0; i < hand_position_.size(); ++i) {
        hand_position_[i] = (i == 0 || i == 6) ? 60.0 : 0.0;
      }
    }
    std::cout << "[mujoco] simulated ideal hand state enabled at 20Hz" << std::endl;

    setupSubscriptions();
    setupGhostSubscriptions();
  }

  void setSimulate(mujoco::Simulate* sim) { sim_ = sim; }

  void updateGhosts(mjModel* m, mujoco::Simulate& sim) {
    if (!m) {
      return;
    }
    ensureGhostResources(m, sim);

    std::vector<double> csv_pose;
    std::vector<double> flow_pose;
    bool csv_ok = false;
    bool flow_ok = false;
    bool dirty = false;
    {
      std::lock_guard<std::mutex> lock(ghost_mu_);
      const auto now = std::chrono::steady_clock::now();
      constexpr auto kStale = std::chrono::milliseconds(500);
      csv_ok = ghost_csv_.valid && (now - ghost_csv_.stamp) < kStale;
      flow_ok = ghost_flow_.valid && (now - ghost_flow_.stamp) < kStale;
      if (ghost_csv_.valid && !csv_ok) {
        ghost_csv_.valid = false;
        dirty = true;
      }
      if (ghost_flow_.valid && !flow_ok) {
        ghost_flow_.valid = false;
        dirty = true;
      }
      dirty = dirty || ghost_dirty_;
      ghost_dirty_ = false;
      if (csv_ok) {
        csv_pose = ghost_csv_.pose;
      }
      if (flow_ok) {
        flow_pose = ghost_flow_.pose;
      }
    }

    if (!dirty && user_scn_.ngeom > 0) {
      return;
    }
    if (!dirty && !csv_ok && !flow_ok && user_scn_.ngeom == 0) {
      return;
    }

    const auto now = std::chrono::steady_clock::now();
    if (user_scn_.ngeom > 0 &&
        now - last_ghost_rebuild_ < std::chrono::milliseconds(100)) {
      return;
    }
    last_ghost_rebuild_ = now;

    user_scn_.ngeom = 0;
    static constexpr float kGhostCsvRgba[4] = {0.95f, 0.85f, 0.1f, 0.45f};
    static constexpr float kGhostFlowRgba[4] = {0.0f, 0.9f, 0.25f, 0.5f};
    if (csv_ok && ghost_csv_data_) {
      writeGhostQpos(m, ghost_csv_data_, csv_pose);
      mj_kinematics(m, ghost_csv_data_);
      addGhostGeoms(m, ghost_csv_data_, kGhostCsvRgba);
    }
    if (flow_ok && ghost_flow_data_) {
      writeGhostQpos(m, ghost_flow_data_, flow_pose);
      mj_kinematics(m, ghost_flow_data_);
      addGhostGeoms(m, ghost_flow_data_, kGhostFlowRgba);
    }
  }

  void publishSimData(mjModel* m, mjData* d, bool publish_depth = true) {
    auto& robot = leju::GlobalRobot::getInstance();
    
    int pos_addr = m->sensor_adr[mj_name2id(m, mjOBJ_SENSOR, "BodyPos")];
    int ori_addr = m->sensor_adr[mj_name2id(m, mjOBJ_SENSOR, "BodyQuat")];
    int gyro_addr = m->sensor_adr[mj_name2id(m, mjOBJ_SENSOR, "BodyGyro")];
    int acc_addr = m->sensor_adr[mj_name2id(m, mjOBJ_SENSOR, "BodyAcc")];
    mjtNum* pos = d->sensordata + pos_addr;
    mjtNum* ori = d->sensordata + ori_addr;
    mjtNum* angVel = d->sensordata + gyro_addr;
    mjtNum* acc = d->sensordata + acc_addr;

    leju::RobotState state(motor_count_);
    const double publish_timestamp = leju::common::GetUnixTimestampS();

    state.timestamp = publish_timestamp;
    for (size_t i=0; i < motor_count_; i++) { // TODO use joint names
      state.q[i] = d->qpos[7 + i];
      state.v[i] = d->qvel[6 + i];
      state.tau[i] = d->qfrc_actuator[6 + i];
    }
    robot.publishRobotState(state);
    leju::ImuData imu_data;
    imu_data.acc = {acc[0], acc[1], acc[2]};
    imu_data.gyro = {angVel[0], angVel[1], angVel[2]};
    imu_data.quat = {ori[0], ori[1], ori[2], ori[3]};
    mjtNum ori_inv[4];
    mju_negQuat(ori_inv, ori);
    mjtNum gravity[3] = {0., 0., -9.81};
    mjtNum gravity_body[3];
    mju_rotVecQuat(gravity_body, gravity, ori_inv);
    mjtNum acc_free[3];
    mju_add3(acc_free, acc, gravity_body);
    imu_data.free_acc = {acc_free[0], acc_free[1], acc_free[2]};
    imu_data.timestamp = publish_timestamp;
    robot.publishImuData(imu_data);
    leju::HardwareState hw_state = leju::HardwareState::READY_OK;
    robot.publishHardwareState(hw_state);

    const auto steady_now = std::chrono::steady_clock::now();
    if (steady_now - last_hand_state_publish_ >= std::chrono::milliseconds(50)) {
      leju::HandState hand_state;
      {
        std::lock_guard<std::mutex> lock(hand_cmd_mutex_);
        hand_state.position.assign(hand_position_.begin(), hand_position_.end());
      }
      hand_state.timestamp = publish_timestamp;
      hand_state.left_valid = true;
      hand_state.right_valid = true;
      hand_state.left_sample_age_ms = 0;
      hand_state.right_sample_age_ms = 0;
      robot.publishHandState(hand_state);
      last_hand_state_publish_ = steady_now;
    }
    // Depth frames are submitted from PhysicsLoop after cloning mjData. Do
    // not ray-cast synchronously from this state publication path.
    (void)publish_depth;
  }

  bool captureDepthIfDue(const mjModel* model, const mjData* data) {
    return depth_publisher_ &&
           depth_publisher_->captureIfDue(model, data);
  }

  void prepareDepthForModelReload() {
    if (depth_publisher_) depth_publisher_->prepareForModelReload();
  }

  leju::RobotCmd getRobotCmd() {
    std::lock_guard<std::mutex> lock(robot_cmd_mutex_);
    return robot_cmd_;
  }

  // TODO move this out
  void calcSimctrl(mjModel* m, mjData* d, bool pause) {
    leju::RobotCmd cmd(motor_count_);
    {
      std::lock_guard<std::mutex> lock(robot_cmd_mutex_);
      cmd = robot_cmd_;
    }

    if (pause) {
      for (int i=0; i<motor_count_; i++) {
        d->qpos[7+i] = cmd.q[i];
      }
      return;
    }

    std::vector<double> joint_pos(motor_count_);
    std::vector<double> joint_vel(motor_count_);
    std::vector<double> joint_pos_target(motor_count_);
    std::vector<double> joint_vel_target(motor_count_);
    std::vector<double> joint_torque_target(motor_count_);
    std::vector<double> joint_torque_demand(motor_count_);

    std::vector<double> motor_pos(motor_count_);
    std::vector<double> motor_vel(motor_count_);
    std::vector<double> motor_pos_target(motor_count_);
    std::vector<double> motor_vel_target(motor_count_);
    std::vector<double> motor_torque_target(motor_count_);
    std::vector<double> motor_ctrl_mode_cmd(motor_count_);
    std::vector<double> motor_kp(motor_count_);
    std::vector<double> motor_kd(motor_count_);
    std::vector<double> motor_torque_demand(motor_count_);

    for (int i = 0; i < motor_count_; i++) {
      joint_pos[i] = d->qpos[i + 7];
      joint_vel[i] = d->qvel[i + 6];
      joint_pos_target[i] = cmd.q[i];
      joint_vel_target[i] = cmd.v[i];
      joint_torque_target[i] = cmd.tau[i];
      motor_ctrl_mode_cmd[i] = cmd.modes[i];
      motor_kp[i] = cmd.kp[i];
      motor_kd[i] = cmd.kd[i];
    }

    if (use_s2gen2_s17_) {
      // The MJCF still exposes 21 joint-space actuators.  Convert only the
      // first 12 leg joints to motor space for the PD/current calculation;
      // waist, arms and head remain direct joints.
      const Eigen::Map<const Eigen::VectorXd> q_leg(joint_pos.data(), 12);
      const Eigen::Map<const Eigen::VectorXd> v_leg(joint_vel.data(), 12);
      const Eigen::Map<const Eigen::VectorXd> q_target_leg(joint_pos_target.data(), 12);
      const Eigen::Map<const Eigen::VectorXd> v_target_leg(joint_vel_target.data(), 12);
      const Eigen::Map<const Eigen::VectorXd> tau_leg(joint_torque_target.data(), 12);
      const Eigen::VectorXd motor_q_leg = ankle_solver_s17_->joint_to_motor_position(q_leg);
      const Eigen::VectorXd motor_v_leg = ankle_solver_s17_->joint_to_motor_velocity(
          q_leg, motor_q_leg, v_leg);
      const Eigen::VectorXd motor_q_target_leg = ankle_solver_s17_->joint_to_motor_position(q_target_leg);
      const Eigen::VectorXd motor_v_target_leg = ankle_solver_s17_->joint_to_motor_velocity(
          q_leg, motor_q_leg, v_target_leg);
      const Eigen::VectorXd motor_tau_leg = ankle_solver_s17_->joint_to_motor_current(
          q_leg, motor_q_leg, tau_leg);
      for (int i = 0; i < 12; ++i) {
        motor_pos[i] = motor_q_leg[i];
        motor_vel[i] = motor_v_leg[i];
        motor_pos_target[i] = motor_q_target_leg[i];
        motor_vel_target[i] = motor_v_target_leg[i];
        motor_torque_target[i] = motor_tau_leg[i];
      }
      for (int i = 12; i < motor_count_; ++i) {
        motor_pos[i] = joint_pos[i];
        motor_vel[i] = joint_vel[i];
        motor_pos_target[i] = joint_pos_target[i];
        motor_vel_target[i] = joint_vel_target[i];
        motor_torque_target[i] = joint_torque_target[i];
      }
    } else {
      motor_pos = transmission_->joint_to_motor_position(joint_pos);
      motor_vel = transmission_->joint_to_motor_velocity(
          joint_pos, motor_pos, joint_vel);
      motor_pos_target = transmission_->joint_to_motor_position(joint_pos_target);
      motor_vel_target = transmission_->joint_to_motor_velocity(
          joint_pos, motor_pos, joint_vel_target);
      motor_torque_target = transmission_->joint_to_motor_torque(
          joint_pos, motor_pos, joint_torque_target);
    }
    
    for (int i=0; i<motor_count_; i++) {
      if (motor_ctrl_mode_cmd[i] == 2) {
        motor_torque_demand[i] = motor_kp[i]*(motor_pos_target[i] - motor_pos[i])
                                 + motor_kd[i]*(motor_vel_target[i] - motor_vel[i])
                                 + motor_torque_target[i];
      } else if (motor_ctrl_mode_cmd[i] == 1) {
        motor_torque_demand[i] = motor_kd[i]*(motor_vel_target[i] - motor_vel[i])
                                 + motor_torque_target[i];
      } else {
        motor_torque_demand[i] = motor_torque_target[i];
      }
    }
    if (use_s2gen2_s17_) {
      const Eigen::Map<const Eigen::VectorXd> q_leg(joint_pos.data(), 12);
      const Eigen::Map<const Eigen::VectorXd> motor_q_leg(motor_pos.data(), 12);
      const Eigen::Map<const Eigen::VectorXd> motor_tau_leg(motor_torque_demand.data(), 12);
      const Eigen::VectorXd joint_tau_leg = ankle_solver_s17_->motor_to_joint_torque(
          q_leg, motor_q_leg, motor_tau_leg);
      for (int i = 0; i < 12; ++i) joint_torque_demand[i] = joint_tau_leg[i];
      for (int i = 12; i < motor_count_; ++i) joint_torque_demand[i] = motor_torque_demand[i];
    } else {
      joint_torque_demand = transmission_->motor_to_joint_torque(
          joint_pos, motor_pos, motor_torque_demand);
    }

    // Mode 3 is reserved for the depth controller's source-compatible
    // MuJoCo path.  The ROS simulator writes joint-space commands directly:
    // ff_tau + kp*(0-q) + kd*(0-v).  Do this after the normal transmission
    // path so S2GEN2 is not applied a second time to those joints.
    for (int i = 0; i < motor_count_; ++i) {
      if (motor_ctrl_mode_cmd[i] == 3) {
        joint_torque_demand[i] =
            joint_torque_target[i] - motor_kp[i] * joint_pos[i]
            - motor_kd[i] * joint_vel[i];
      }
    }

    for (int i=0; i<motor_count_; i++) {
      d->ctrl[i] = joint_torque_demand[i];
    
    }

    // bool motor_pos_have_nan = false;
    // std::vector<int> nan_ids;
    // for (int i=0; i<motor_count_; i++) {
    //   motor_pos_have_nan |= (std::isnan(motor_pos[i]) | std::isinf(motor_pos[i]));
    //   nan_ids.emplace_back(i);
    // }
    // if (motor_pos_have_nan) {
    //   std::cout << "[WARN] motor_pos_have_nan, ids: [";
    //   for (const auto& id: nan_ids) {
    //     std::cout << id << ", ";
    //   }
    //   std::cout << "]." << std::endl;
    // }
  }

  std::string get_filename() const {
    return filename_;
  }

 private:
  void setupSubscriptions() {
    auto& robot = leju::GlobalRobot::getInstance();
    robot.subscribeRobotCmd([this](const leju::RobotCmdConstPtr& robot_cmd) {
      this->jointCmdCallback(robot_cmd);
    });
    robot.subscribeHandCmd([this](const leju::HandCmdConstPtr& hand_cmd) {
      this->handCmdCallback(hand_cmd);
    });
    robot.subscribeStopRobot([this](const leju::StringDataConstPtr& stop_robot) {
      this->stopRobotCallback(stop_robot);
    });
  }

  using GhostSub = leju::dds_common::TopicSubscriber<leju::msgs::Float64Array>;

  void setupGhostSubscriptions() {
    try {
      ghost_participant_ = dds::domain::DomainParticipant(0);
      dds::sub::qos::DataReaderQos qos;
      qos << dds::core::policy::Reliability::BestEffort();
      ghost_csv_sub_ = std::make_unique<GhostSub>(
          ghost_participant_, "/rl_controller/heft/ghost_csv",
          [this](const leju::msgs::Float64Array& msg) { onGhostMsg(msg, false); },
          nullptr, qos);
      ghost_flow_sub_ = std::make_unique<GhostSub>(
          ghost_participant_, "/rl_controller/heft/ghost_flow",
          [this](const leju::msgs::Float64Array& msg) { onGhostMsg(msg, true); },
          nullptr, qos);
      std::cout << "[mujoco] HEFT ghosts: yellow=CSV, green=flow/student ref"
                << std::endl;
    } catch (const std::exception& e) {
      std::cerr << "[mujoco] HEFT ghost overlay disabled: " << e.what()
                << std::endl;
    }
  }

  void onGhostMsg(const leju::msgs::Float64Array& msg, bool is_flow) {
    std::lock_guard<std::mutex> lock(ghost_mu_);
    GhostPose& slot = is_flow ? ghost_flow_ : ghost_csv_;
    slot.pose = msg.data();
    slot.stamp = std::chrono::steady_clock::now();
    slot.valid = slot.pose.size() >= 7;
    ghost_dirty_ = true;
  }

  void writeGhostQpos(const mjModel* model, mjData* data,
                      const std::vector<double>& pose) {
    if (!model || !data || pose.size() < 7) {
      return;
    }
    data->qpos[0] = pose[0];
    data->qpos[1] = pose[1];
    data->qpos[2] = pose[2];
    data->qpos[3] = pose[3];
    data->qpos[4] = pose[4];
    data->qpos[5] = pose[5];
    data->qpos[6] = pose[6];
    const int nj = std::min(model->nq - 7, static_cast<int>(pose.size()) - 7);
    for (int i = 0; i < nj; ++i) {
      data->qpos[7 + i] = pose[static_cast<size_t>(7 + i)];
    }
    if (data->qvel) {
      mju_zero(data->qvel, model->nv);
    }
  }

  void addGhostGeoms(const mjModel* model, mjData* data, const float rgba[4]) {
    const int start = user_scn_.ngeom;
    mjv_addGeoms(model, data, &ghost_opt_, &ghost_pert_, mjCAT_DYNAMIC,
                 &user_scn_);
    for (int i = start; i < user_scn_.ngeom; ++i) {
      mjvGeom& geom = user_scn_.geoms[i];
      geom.rgba[0] = rgba[0];
      geom.rgba[1] = rgba[1];
      geom.rgba[2] = rgba[2];
      geom.rgba[3] = rgba[3];
      geom.objtype = mjOBJ_UNKNOWN;
      geom.objid = -1;
    }
  }

  void ensureGhostResources(mjModel* m, mujoco::Simulate& sim) {
    if (ghost_model_ == m && ghost_csv_data_ && ghost_flow_data_ &&
        user_scn_ready_) {
      if (sim.user_scn != &user_scn_) {
        sim.user_scn = &user_scn_;
      }
      return;
    }
    freeGhostResources();
    ghost_model_ = m;
    ghost_csv_data_ = mj_makeData(m);
    ghost_flow_data_ = mj_makeData(m);
    mjv_defaultOption(&ghost_opt_);
    for (int i = 0; i < mjNGROUP; ++i) {
      ghost_opt_.geomgroup[i] = 0;
    }
    ghost_opt_.geomgroup[0] = 1;
    mjv_defaultPerturb(&ghost_pert_);
    mjv_defaultScene(&user_scn_);
    mjv_makeScene(m, &user_scn_, 512);
    user_scn_ready_ = true;
    sim.user_scn = &user_scn_;
    {
      std::lock_guard<std::mutex> lock(ghost_mu_);
      ghost_dirty_ = true;
    }
  }

  void freeGhostResources() {
    if (sim_ && sim_->user_scn == &user_scn_) {
      sim_->user_scn = nullptr;
    }
    if (ghost_csv_data_) {
      mj_deleteData(ghost_csv_data_);
      ghost_csv_data_ = nullptr;
    }
    if (ghost_flow_data_) {
      mj_deleteData(ghost_flow_data_);
      ghost_flow_data_ = nullptr;
    }
    if (user_scn_ready_) {
      mjv_freeScene(&user_scn_);
      user_scn_ready_ = false;
    }
    ghost_model_ = nullptr;
  }

  void jointCmdCallback(const leju::RobotCmdConstPtr& robot_cmd) {
    std::lock_guard<std::mutex> lock(robot_cmd_mutex_);
    robot_cmd_ = *robot_cmd;
  }

  void handCmdCallback(const leju::HandCmdConstPtr& hand_cmd) {
    if (!hand_cmd || !hand_cmd->isValid()) {
      return;
    }
    std::lock_guard<std::mutex> lock(hand_cmd_mutex_);
    std::copy(hand_cmd->position.begin(), hand_cmd->position.end(),
              hand_position_.begin());
  }

  void stopRobotCallback(const leju::StringDataConstPtr& stop_robot) {
    (void)stop_robot;
    std::cout << "[LejuMujocoHelper] Stop command received, exiting simulation..." << std::endl;
    if (sim_ != nullptr) {
      sim_->exitrequest.store(1);
    }
  }

  std::string filename_;

  size_t motor_count_;
  std::vector<std::string> motor_names_;
  std::unique_ptr<leju::transmission::RobotTransmission> transmission_;
  std::unique_ptr<AnkleSolver> ankle_solver_s17_;
  bool use_s2gen2_s17_ = false;
  mujoco::Simulate* sim_ = nullptr;

  std::mutex robot_cmd_mutex_;
  leju::RobotCmd robot_cmd_;
  std::mutex hand_cmd_mutex_;
  std::array<double, 12> hand_position_{};
  std::chrono::steady_clock::time_point last_hand_state_publish_{};
  std::unique_ptr<DepthDdsPublisher> depth_publisher_;

  struct GhostPose {
    std::vector<double> pose;
    std::chrono::steady_clock::time_point stamp{};
    bool valid = false;
  };
  dds::domain::DomainParticipant ghost_participant_{dds::core::null};
  std::unique_ptr<GhostSub> ghost_csv_sub_;
  std::unique_ptr<GhostSub> ghost_flow_sub_;
  std::mutex ghost_mu_;
  GhostPose ghost_csv_;
  GhostPose ghost_flow_;
  bool ghost_dirty_ = false;
  std::chrono::steady_clock::time_point last_ghost_rebuild_{};
  mjModel* ghost_model_ = nullptr;
  mjData* ghost_csv_data_ = nullptr;
  mjData* ghost_flow_data_ = nullptr;
  mjvOption ghost_opt_{};
  mjvPerturb ghost_pert_{};
  mjvScene user_scn_{};
  bool user_scn_ready_ = false;
};

extern "C" {
#if defined(_WIN32) || defined(__CYGWIN__)
  #include <windows.h>
#else
  #if defined(__APPLE__)
    #include <mach-o/dyld.h>
  #endif
  #include <sys/errno.h>
  #include <unistd.h>
#endif
}

namespace {
namespace mj = ::mujoco;
namespace mju = ::mujoco::sample_util;

const int kErrorLength = 1024;          // load error string length

// model and data
mjModel* m = nullptr;
mjData* d = nullptr;
std::string initial_keyframe = "home";

LejuMujocoHelper* leju_helper = nullptr;

bool ResetToInitialKeyframe(mjModel* model, mjData* data) {
  const int key_id = mj_name2id(model, mjOBJ_KEY, initial_keyframe.c_str());
  if (key_id < 0) {
    std::fprintf(stderr, "MuJoCo keyframe '%s' not found\n",
                 initial_keyframe.c_str());
    return false;
  }
  mj_resetDataKeyframe(model, data, key_id);
  return true;
}

using Seconds = std::chrono::duration<double>;

// FREE 相机 + 每帧加 body 位移增量：机器人移动时画面跟随，鼠标平移
// 产生的 lookat 偏移作为持续偏移保留，不会被覆盖。
int g_track_body_id = -1;
mjtNum g_track_body_prev_pos[3] = {0, 0, 0};

void ConfigureRobotTrackingCamera(mjModel* model, mjData* data, mj::Simulate* sim) {
  if (!model || !data || !sim) {
    return;
  }

  int track_body = mj_name2id(model, mjOBJ_BODY, "base_link");
  if (track_body < 0) {
    g_track_body_id = -1;
    return;
  }
  g_track_body_id = track_body;
  mju_copy3(g_track_body_prev_pos, data->xpos + 3 * track_body);

  sim->camera = 0;
  sim->cam.type = mjCAMERA_FREE;
  sim->cam.trackbodyid = -1;
  sim->cam.fixedcamid = -1;
  mju_copy3(sim->cam.lookat, data->xpos + 3 * track_body);
}

void UpdateRobotTrackingCamera(mjData* data, mj::Simulate* sim) {
  if (g_track_body_id < 0 || !data || !sim) {
    return;
  }
  const mjtNum* body_pos = data->xpos + 3 * g_track_body_id;
  mjtNum delta[3];
  mju_sub3(delta, body_pos, g_track_body_prev_pos);
  mju_addTo3(sim->cam.lookat, delta);
  mju_copy3(g_track_body_prev_pos, body_pos);
}

mjModel* LoadModel(const char* file, mj::Simulate& sim) {
  // this copy is needed so that the mju::strlen call below compiles
  char filename[mj::Simulate::kMaxFilenameLength];
  mju::strcpy_arr(filename, file);

  // make sure filename is not empty
  if (!filename[0]) {
    return nullptr;
  }

  // load and compile
  char loadError[kErrorLength] = "";
  mjModel* mnew = 0;
  auto load_start = mj::Simulate::Clock::now();
  if (mju::strlen_arr(filename)>4 &&
      !std::strncmp(filename + mju::strlen_arr(filename) - 4, ".mjb",
                    mju::sizeof_arr(filename) - mju::strlen_arr(filename)+4)) {
    mnew = mj_loadModel(filename, nullptr);
    if (!mnew) {
      mju::strcpy_arr(loadError, "could not load binary model");
    }
  } else {
    mnew = mj_loadXML(filename, nullptr, loadError, kErrorLength);

    // remove trailing newline character from loadError
    if (loadError[0]) {
      int error_length = mju::strlen_arr(loadError);
      if (loadError[error_length-1] == '\n') {
        loadError[error_length-1] = '\0';
      }
    }
  }
  auto load_interval = mj::Simulate::Clock::now() - load_start;
  double load_seconds = Seconds(load_interval).count();

  // if no error and load took more than 1/4 seconds, report load time
  if (!loadError[0] && load_seconds > 0.25) {
    mju::sprintf_arr(loadError, "Model loaded in %.2g seconds", load_seconds);
  }

  mju::strcpy_arr(sim.load_error, loadError);

  if (!mnew) {
    std::printf("%s\n", loadError);
    return nullptr;
  }

  sim.run = 0;
  // compiler warning: print and pause
  if (loadError[0]) {
    // mj_forward() below will print the warning message
    std::printf("Model compiled, but simulation warning (paused):\n  %s\n", loadError);
  }

  return mnew;
}

// simulate in background thread (while rendering in main thread)
void PhysicsLoop(mj::Simulate& sim) {
  auto t0 = std::chrono::steady_clock::now();
  
  // run until asked to exit
  while (!sim.exitrequest.load()) {
    if (sim.droploadrequest.load()) {
      sim.LoadMessage(sim.dropfilename);
      mjModel* mnew = LoadModel(sim.dropfilename, sim);
      sim.droploadrequest.store(false);

      mjData* dnew = nullptr;
      if (mnew) {
        dnew = mj_makeData(mnew);
        if (dnew) {
          if (!ResetToInitialKeyframe(mnew, dnew)) {
            std::exit(EXIT_FAILURE);
          }
        }
      }
      if (dnew) {
        sim.Load(mnew, dnew, sim.dropfilename);

        // lock the sim mutex
        const std::unique_lock<std::recursive_mutex> lock(sim.mtx);

        leju_helper->prepareDepthForModelReload();
        mj_deleteData(d);
        mj_deleteModel(m);

        m = mnew;
        d = dnew;
        mj_forward(m, d);
        ConfigureRobotTrackingCamera(m, d, &sim);

      } else {
        sim.LoadMessageClear();
      }
    }

    if (sim.uiloadrequest.load()) {
      sim.uiloadrequest.fetch_sub(1);
      sim.LoadMessage(sim.filename);
      mjModel* mnew = LoadModel(sim.filename, sim);
      mjData* dnew = nullptr;
      if (mnew) {
        dnew = mj_makeData(mnew);
        if (dnew) {
          if (!ResetToInitialKeyframe(mnew, dnew)) {
            std::exit(EXIT_FAILURE);
          }
        }
      }
      if (dnew) {
        sim.Load(mnew, dnew, sim.filename);

        // lock the sim mutex
        const std::unique_lock<std::recursive_mutex> lock(sim.mtx);

        leju_helper->prepareDepthForModelReload();
        mj_deleteData(d);
        mj_deleteModel(m);

        m = mnew;
        d = dnew;
        mj_forward(m, d);
        ConfigureRobotTrackingCamera(m, d, &sim);

      } else {
        sim.LoadMessageClear();
      }
    }

    // While paused, mj_forward is only needed for slider/UI edits.  Running it
    // at the physics rate can monopolize sim.mtx on the S17 model and starve
    // the renderer, leaving a blank unresponsive window.  Keep the normal
    // physics cadence when running and use a modest UI refresh cadence while
    // paused.
    const auto loop_period = sim.run
        ? std::chrono::milliseconds(std::max(1, static_cast<int>(m->opt.timestep * 1000)))
        : std::chrono::milliseconds(20);
    t0 += loop_period;
    std::this_thread::sleep_until(t0);

    {
      // lock the sim mutex
      const std::unique_lock<std::recursive_mutex> lock(sim.mtx);

      // run only if model is present
      if (m) {
        // running
        if (sim.run) {
          bool stepped = false;

          // inject noise
          sim.InjectNoise();

          // call mj_step
          sim.speed_changed = false;
          leju_helper->calcSimctrl(m, d, false);
          mj_step(m, d);
          stepped = true; // TODO implement controller-sync mode

          // save current state to history buffer
          if (stepped) {
            sim.AddToHistory();
          }
        }

        // paused
        else {
          // run mj_forward, to update rendering and joint sliders
          leju_helper->calcSimctrl(m, d, true);
          mj_forward(m, d);
          sim.speed_changed = true;
        }

        UpdateRobotTrackingCamera(d, &sim);
        // Rendering must remain responsive while paused.  Ray-casting the
        // 64x36 depth frame is intentionally skipped until the simulation is
        // running; otherwise the physics thread can hold sim.mtx long enough
        // to starve the GUI event/render loop.
        // Keep DDS state publication inside the simulation lock, but never do
        // the expensive depth ray-casting there.  Copy the current state and
        // process the copy after releasing sim.mtx so rendering remains
        // responsive while the robot is running.
        leju_helper->publishSimData(m, d, false);
        if (sim.run) leju_helper->captureDepthIfDue(m, d);
      }
    }  // release sim.mtx before ghost rebuild (kinematics + capsule geoms)
    if (m) {
      leju_helper->updateGhosts(m, sim);
    }
  }
}
}  // namespace

void PhysicsThread(mj::Simulate* sim, const char* filename) {
  // request loadmodel if file given (otherwise drag-and-drop)
  if (filename != nullptr) {
    sim->LoadMessage(filename);
    m = LoadModel(filename, *sim);
    if (m) {
      // lock the sim mutex
      const std::unique_lock<std::recursive_mutex> lock(sim->mtx);

      d = mj_makeData(m);
      if (d) {
        if (!ResetToInitialKeyframe(m, d)) {
          std::exit(EXIT_FAILURE);
        }
      }
    }
    if (d) {
      sim->Load(m, d, filename);

      // lock the sim mutex
      const std::unique_lock<std::recursive_mutex> lock(sim->mtx);

      mj_forward(m, d);
      ConfigureRobotTrackingCamera(m, d, sim);

    } else {
      sim->LoadMessageClear();
    }
  }
  
  PhysicsLoop(*sim);

  // delete everything we allocated
  leju_helper->prepareDepthForModelReload();
  mj_deleteData(d);
  mj_deleteModel(m);
}

//------------------------------------------ main --------------------------------------------------

// machinery for replacing command line error by a macOS dialog box when running under Rosetta
#if defined(__APPLE__) && defined(__AVX__)
extern void DisplayErrorDialogBox(const char* title, const char* msg);
static const char* rosetta_error_msg = nullptr;
__attribute__((used, visibility("default"))) extern "C" void _mj_rosettaError(const char* msg) {
  rosetta_error_msg = msg;
}
#endif

// headless physics loop (no OpenGL, no GLFW)
void HeadlessPhysicsLoop(const char* filename) {
  char loadError[kErrorLength] = "";
  m = mj_loadXML(filename, nullptr, loadError, kErrorLength);
  if (!m) {
    std::fprintf(stderr, "Failed to load model: %s\n", loadError);
    return;
  }
  d = mj_makeData(m);
  if (!d) {
    std::fprintf(stderr, "Failed to create mjData\n");
    mj_deleteModel(m);
    return;
  }

  if (!ResetToInitialKeyframe(m, d)) {
    mj_deleteData(d);
    mj_deleteModel(m);
    d = nullptr;
    m = nullptr;
    std::exit(EXIT_FAILURE);
  }
  mj_forward(m, d);

  std::printf("[headless] Model loaded: %s\n", filename);
  std::printf("[headless] Timestep: %.4f s, running physics + DDS loop...\n", m->opt.timestep);

  auto t0 = std::chrono::steady_clock::now();
  const auto dt = std::chrono::microseconds(static_cast<int>(m->opt.timestep * 1e6));

  while (true) {
    leju_helper->calcSimctrl(m, d, false);
    mj_step(m, d);
    leju_helper->publishSimData(m, d);
    leju_helper->captureDepthIfDue(m, d);

    t0 += dt;
    std::this_thread::sleep_until(t0);
  }

  leju_helper->prepareDepthForModelReload();
  mj_deleteData(d);
  mj_deleteModel(m);
}

// run event loop
int main(int argc, char** argv) {

  // display an error if running on macOS under Rosetta 2
#if defined(__APPLE__) && defined(__AVX__)
  if (rosetta_error_msg) {
    DisplayErrorDialogBox("Rosetta 2 is not supported", rosetta_error_msg);
    std::exit(1);
  }
#endif

  // print version, check compatibility
  std::printf("MuJoCo version %s\n", mj_versionString());
  if (mjVERSION_HEADER!=mj_version()) {
    mju_error("Headers and library have different versions");
  }

  // parse arguments
  bool headless = false;
  const char* xml_filename = nullptr;
  const char* depth_config_path = nullptr;
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--headless") == 0) {
      headless = true;
    } else if (std::strcmp(argv[i], "--no-mujoco-camera") == 0 ||
               std::strcmp(argv[i], "--run-mujoco-camera") == 0 ||
               std::strncmp(argv[i], "--run-mujoco-camera=", 20) == 0) {
      // Parsed below after the positional XML path is identified.
    } else if (std::strcmp(argv[i], "--depth-frequency") == 0 && i + 1 < argc) {
      ++i;  // value parsed below
    } else if (std::strcmp(argv[i], "--depth-config") == 0 && i + 1 < argc) {
      depth_config_path = argv[++i];
    } else if (std::strncmp(argv[i], "--depth-config=", 15) == 0) {
      depth_config_path = argv[i] + 15;
    } else if (std::strcmp(argv[i], "--dump-depth") == 0) {
      // Parsed below.
    } else if (std::strcmp(argv[i], "--show-depth") == 0) {
      // Parsed below.
    } else if (std::strcmp(argv[i], "--initial-keyframe") == 0 && i + 1 < argc) {
      initial_keyframe = argv[++i];
    } else if (std::strncmp(argv[i], "--initial-keyframe=", 19) == 0) {
      initial_keyframe = argv[i] + 19;
    } else if (!xml_filename) {
      xml_filename = argv[i];
    }
  }

  bool run_mujoco_camera = true;
  double depth_frequency_hz = 30.0;
  bool dump_depth = false;
  bool show_depth = false;
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--no-mujoco-camera") == 0) {
      run_mujoco_camera = false;
    } else if (std::strcmp(argv[i], "--run-mujoco-camera") == 0) {
      run_mujoco_camera = true;
    } else if (std::strncmp(argv[i], "--run-mujoco-camera=", 20) == 0) {
      run_mujoco_camera = std::strcmp(argv[i] + 20, "false") != 0;
    } else if (std::strcmp(argv[i], "--depth-frequency") == 0 && i + 1 < argc) {
      depth_frequency_hz = std::max(1.0, std::atof(argv[++i]));
    } else if (std::strcmp(argv[i], "--dump-depth") == 0) {
      dump_depth = true;
    } else if (std::strcmp(argv[i], "--show-depth") == 0) {
      show_depth = true;
    } else if (std::strcmp(argv[i], "--depth-config") == 0 && i + 1 < argc) {
      ++i;
    }
  }

  if (!xml_filename) {
    std::cerr << "Usage: " << argv[0]
              << " [--headless] [--run-mujoco-camera|--no-mujoco-camera]"
              << " [--depth-frequency Hz] [--depth-config YAML]"
              << " [--dump-depth] [--show-depth]"
              << " [--initial-keyframe NAME] <xml_file_path>" << std::endl;
    return 1;
  }
  std::printf("[mujoco] Initial keyframe: %s\n", initial_keyframe.c_str());

  const std::string depth_yaml =
      (depth_config_path && depth_config_path[0] != '\0')
          ? std::string(depth_config_path)
          : DefaultDepthWalkConfigPath();
  const double depth_noise_std_m = LoadDepthNoiseStdM(depth_yaml, 0.05);

  LejuMujocoHelper helper;
  leju_helper = &helper;
  leju_helper->Init(xml_filename, run_mujoco_camera, depth_frequency_hz,
                    dump_depth, show_depth, depth_noise_std_m);

  if (headless) {
    std::printf("[headless] Running without rendering (no OpenGL required)\n");
    HeadlessPhysicsLoop(xml_filename);
    return 0;
  }

  // GUI mode
  mjvCamera cam;
  mjv_defaultCamera(&cam);

  mjvOption opt;
  mjv_defaultOption(&opt);

  mjvPerturb pert;
  mjv_defaultPerturb(&pert);

  auto sim = std::make_unique<mj::Simulate>(
      std::make_unique<mj::GlfwAdapter>(),
      &cam, &opt, &pert, /* is_passive = */ false
  );
  leju_helper->setSimulate(sim.get());

  // start physics thread
  char name[256];
  mju::strcpy_arr(name, leju_helper->get_filename().c_str());
  std::thread physicsthreadhandle(&PhysicsThread, sim.get(), name);

  // start simulation UI loop (blocking call)
  sim->RenderLoop();
  physicsthreadhandle.join();

  return 0;
}
