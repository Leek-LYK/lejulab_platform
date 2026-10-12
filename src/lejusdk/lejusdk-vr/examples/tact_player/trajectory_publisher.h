// Copyright 2024 Leju Robotics. All rights reserved.
//
// 轨迹发布器，将插值后的轨迹数据发布到机器人控制器。
// 以固定频率（100Hz）发布手臂和腰部关节指令。

#ifndef LEJUSDK_VR_EXAMPLES_TACT_PLAYER_TRAJECTORY_PUBLISHER_H_
#define LEJUSDK_VR_EXAMPLES_TACT_PLAYER_TRAJECTORY_PUBLISHER_H_

#include <atomic>

#include <lejusdk-vr/lejusdk_vr.h>

#include "bezier_interpolator.h"

namespace leju {
namespace vr {
namespace tact_player {

// 轨迹发布器，负责将贝塞尔插值结果发布到机器人。
class TrajectoryPublisher {
 public:
  // 构造函数。
  // @param api VR API 接口指针
  // @param interpolator 贝塞尔插值器指针
  // @param running_flag 运行标志，设为 false 时停止发布
  TrajectoryPublisher(VRBaseAPI* api,
                      const BezierInterpolator* interpolator,
                      std::atomic<bool>* running_flag);

  // 运行轨迹发布循环。
  // 以 100Hz 频率持续发布轨迹点，直到轨迹结束或 running_flag 被置为 false。
  // @return 成功返回 true
  bool Run();

 private:
  VRBaseAPI* api_;                          // VR API 接口
  const BezierInterpolator* interpolator_;  // 贝塞尔插值器
  std::atomic<bool>* running_flag_;         // 运行控制标志
};

}  // namespace tact_player
}  // namespace vr
}  // namespace leju

#endif  // LEJUSDK_VR_EXAMPLES_TACT_PLAYER_TRAJECTORY_PUBLISHER_H_
