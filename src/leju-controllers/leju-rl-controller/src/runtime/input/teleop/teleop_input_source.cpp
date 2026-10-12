/**
 * @file teleop_input_source.cpp
 * @brief 遥操作输入源实现
 */

#include "leju-rl-controller/runtime/input/teleop/teleop_input_source.h"

#include <algorithm>

#include "leju-rl-controller/runtime/input/teleop/joy_teleop_adapter.h"
#include "leju-rl-controller/runtime/input/teleop/pico_teleop_adapter.h"
#include "leju-rl-controller/runtime/input/teleop/quest_teleop_adapter.h"
#include "leju-rl-controller/runtime/input/teleop/teleop_binding_config.h"
#include "leju-rl-controller/runtime/input/trigger_buffer.h"
#include "leju-rl-controller/rl_log.h"
#include "lejusdk-utils/robot_version.hpp"

namespace leju {
namespace runtime {

// ============================================================================
// 构造函数/析构函数
// ============================================================================

TeleopInputSource::TeleopInputSource(const RobotVersion& version,
                                     TriggerBuffer* trigger_buffer)
    : trigger_buffer_(trigger_buffer),
      joy_adapter_(std::make_unique<JoyTeleopAdapter>(trigger_buffer)),
      quest_adapter_(std::make_unique<QuestTeleopAdapter>(version, trigger_buffer)),
      pico_adapter_(std::make_unique<PicoTeleopAdapter>(trigger_buffer)) {
  RL_LOGI("TeleopInputSource: Created Joy, Quest and Pico adapters");
}

TeleopInputSource::~TeleopInputSource() {
  shutdown();
}

// ============================================================================
// 生命周期管理
// ============================================================================

bool TeleopInputSource::initialize() {
  if (initialized_.load()) {
    return true;
  }

  bool any_initialized = false;

  // 初始化 Joy 适配器（需要主动订阅 SDK）
  if (joy_adapter_ && !joy_adapter_->isInitialized()) {
    if (joy_adapter_->initialize()) {
      any_initialized = true;
      RL_LOGI("TeleopInputSource: Initialized Joy adapter");
    } else {
      RL_LOGW("TeleopInputSource: Failed to initialize Joy adapter");
    }
  }

  // 初始化 Quest 适配器（主动订阅 Quest 数据）
  if (quest_adapter_ && !quest_adapter_->isInitialized()) {
    if (quest_adapter_->initialize()) {
      any_initialized = true;
      RL_LOGI("TeleopInputSource: Initialized Quest adapter");
    } else {
      RL_LOGW("TeleopInputSource: Failed to initialize Quest adapter");
    }
  }

  // 初始化 Pico 适配器（订阅 DDS /rt/pico/joysticks）
  if (pico_adapter_ && !pico_adapter_->isInitialized()) {
    if (pico_adapter_->initialize()) {
      any_initialized = true;
      RL_LOGI("TeleopInputSource: Initialized Pico adapter");
    } else {
      RL_LOGW("TeleopInputSource: Failed to initialize Pico adapter");
    }
  }

  if (any_initialized) {
    initialized_ = true;
    RL_LOGI("TeleopInputSource: Initialization complete");
  }

  return any_initialized;
}

void TeleopInputSource::shutdown() {
  if (!initialized_.load()) {
    return;
  }

  // 关闭 Joy 适配器
  if (joy_adapter_) {
    joy_adapter_->shutdown();
    RL_LOGI("TeleopInputSource: Shutdown Joy adapter");
  }

  // 关闭 Quest 适配器
  if (quest_adapter_) {
    quest_adapter_->shutdown();
    RL_LOGI("TeleopInputSource: Shutdown Quest adapter");
  }

  // 关闭 Pico 适配器
  if (pico_adapter_) {
    pico_adapter_->shutdown();
    RL_LOGI("TeleopInputSource: Shutdown Pico adapter");
  }

  initialized_ = false;
  RL_LOGI("TeleopInputSource: Shutdown complete");
}

// ============================================================================
// InputSource 接口实现
// ============================================================================

CommandBuffer::Snapshot TeleopInputSource::getSnapshot() const {
  const auto has_active_cmd = [](const CommandBuffer::Snapshot& snapshot) {
    return snapshot.cmd_vel.valid &&
           !snapshot.cmd_vel.isNearZero(VelocityDeadzone::kLinearX,
                                        VelocityDeadzone::kLinearY,
                                        VelocityDeadzone::kAngularZ);
  };

  // 蹲起状态：当 External 正处于下蹲/姿态模式（cmd_stance=1）时，遥操作的行走命令
  // 不应接管，否则会在 mergeAllCmdVel 里用 mode=0 的行走命令覆盖 External 的
  // 下蹲高度指令（External 优先级 kExternal=2 最低，遍历顺序在后）。
  const bool stance_active =
      stance_active_query_ ? stance_active_query_() : false;

  // cmd_vel 与 hand/arm/... 分开合并：
  // 摇杆静止时仍要透出 Pico 的 hand_target，否则 ControlLogic 100Hz 重发拿不到目标。
  CommandBuffer::Snapshot out;
  bool have_cmd_vel = false;

  const auto merge_targets = [&out](const CommandBuffer::Snapshot& snapshot) {
    if (!out.arm_target.hasValue() && snapshot.arm_target.hasValue()) {
      out.arm_target = snapshot.arm_target;
    }
    if (!out.head_target.hasValue() && snapshot.head_target.hasValue()) {
      out.head_target = snapshot.head_target;
    }
    if (!out.waist_target.hasValue() && snapshot.waist_target.hasValue()) {
      out.waist_target = snapshot.waist_target;
    }
    if (!out.hand_target.hasValue() && snapshot.hand_target.hasValue()) {
      out.hand_target = snapshot.hand_target;
    }
  };

  // 按优先级遍历：Joy > Quest > Pico
  if (joy_adapter_) {
    auto snapshot = joy_adapter_->getSnapshot();
    merge_targets(snapshot);
    if (!have_cmd_vel && has_active_cmd(snapshot)) {
      out.cmd_vel = snapshot.cmd_vel;
      have_cmd_vel = true;
    }
  }

  if (quest_adapter_) {
    auto snapshot = quest_adapter_->getSnapshot();
    merge_targets(snapshot);
    // 由于下蹲走extern，防止下蹲时候抢占external的cmd_vel
    if (!have_cmd_vel && has_active_cmd(snapshot) && !stance_active) {
      out.cmd_vel = snapshot.cmd_vel;
      have_cmd_vel = true;
    }
  }

  if (pico_adapter_) {
    auto snapshot = pico_adapter_->getSnapshot();
    merge_targets(snapshot);
    if (!have_cmd_vel && has_active_cmd(snapshot)) {
      out.cmd_vel = snapshot.cmd_vel;
      have_cmd_vel = true;
    }
  }

  return out;
}

// ============================================================================
// 配置加载
// ============================================================================

bool TeleopInputSource::loadBindingConfig(const std::string& config_path) {
  // 先加载一次配置验证文件有效，避免重复打印错误
  TeleopBindingConfig config;
  if (!config.loadFromFile(config_path)) {
    return false;
  }

  // 将已加载的配置传给各个适配器
  if (joy_adapter_) {
    joy_adapter_->setBindingConfig(config);
    RL_LOGI("TeleopInputSource: Loaded binding config for Joy");
  }

  if (quest_adapter_) {
    quest_adapter_->setBindingConfig(config);
    RL_LOGI("TeleopInputSource: Loaded binding config for Quest");
  }

  if (pico_adapter_) {
    pico_adapter_->setBindingConfig(config);
    RL_LOGI("TeleopInputSource: Loaded binding config for Pico");
  }

  // 记录路径供热重载使用 (仅在加载成功后)
  {
    std::lock_guard<std::mutex> lk(path_mutex_);
    binding_config_path_ = config_path;
  }

  return true;
}

bool TeleopInputSource::mergeAmpHandTeleopFromControllerConfig(
    const std::string& config_path) {
  bool merged = false;
  if (joy_adapter_ &&
      joy_adapter_->mergeAmpHandTeleopFromControllerConfig(config_path)) {
    merged = true;
    RL_LOGI("TeleopInputSource: Merged amp_hand teleop config for Joy");
  }
  if (quest_adapter_ &&
      quest_adapter_->mergeAmpHandTeleopFromControllerConfig(config_path)) {
    merged = true;
    RL_LOGI("TeleopInputSource: Merged amp_hand teleop config for Quest");
  }
  return merged;
}

bool TeleopInputSource::reloadBindingConfig() {
  std::string path;
  {
    std::lock_guard<std::mutex> lk(path_mutex_);
    path = binding_config_path_;
  }

  if (path.empty()) {
    RL_LOGW("TeleopInputSource: reloadBindingConfig called but no config has been loaded yet");
    return false;
  }

  RL_LOGI("TeleopInputSource: Reloading teleop binding config from %s", path.c_str());
  return loadBindingConfig(path);
}

}  // namespace runtime
}  // namespace leju
