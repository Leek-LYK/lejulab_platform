/**
 * @file vr_communication.cpp
 * @brief VR 通信单例实现
 */

#include "vr_communication.h"
#include "vr_dds_communicator.h"

namespace leju {
namespace vr {

IVRCommunication& VRCommunication::getInstance() {
  // Leaky singleton: intentionally never deleted to avoid static destruction
  // order issues when CycloneDDS is statically linked. The OS will reclaim
  // all memory when the process exits.
  static VRDdsCommunicator* instance = new VRDdsCommunicator();
  return *instance;
}

}  // namespace vr
}  // namespace leju
