/**
 * @file kuavo_vr_api.cpp
 * @brief Kuavo 机器人 VR 控制 API 实现
 */

#include "lejusdk-vr/vr_api/kuavo_vr_api.h"

namespace leju {
namespace vr {

KuavoVRAPI::KuavoVRAPI(const RobotVersion& version) : VRBaseAPI(version) {}

}  // namespace vr
}  // namespace leju
