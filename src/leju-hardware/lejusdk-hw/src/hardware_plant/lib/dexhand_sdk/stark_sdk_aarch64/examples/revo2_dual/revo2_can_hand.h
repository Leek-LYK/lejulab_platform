// Minimal DexHandBase implementation for Revo2 over classic CAN (v1.1.9 SDK).
//
// - Upper-layer API uses 0~100 positions / -100~+100 speeds (per DexHandBase contract).
// - Device uses 0~1000 / -1000~+1000. This class does the 10x scaling.
// - Transport is set up externally (see dual_hand_test.cpp): this class only
//   holds a Stark DeviceHandler and calls stark_* APIs with its slave_id.
//
// slave_id convention (matches reference kuavo-ros-control):
//   0x01 = left hand, 0x02 = right hand.

#pragma once

#include "dexhand_base.h"
#include "stark-sdk.h"

#include <memory>
#include <mutex>
#include <string>

namespace dexhand {

class Revo2CanHand : public DexHandBase {
 public:
  static std::shared_ptr<Revo2CanHand> Create(uint8_t slave_id, std::string bus_name);

  Revo2CanHand(uint8_t slave_id, std::string bus_name);
  ~Revo2CanHand() override;

  uint8_t slave_id() const { return slave_id_; }
  const std::string& bus_name() const { return bus_name_; }
  bool ok() const { return handle_ != nullptr; }

  // ---- DexHandBase: implemented ----
  DexHandFwType getDexHandFwType() override { return V2_STANDARD; }
  bool getDeviceInfo(DeviceInfo_t& info) override;
  void setFingerPositions(const UnsignedFingerArray& positions /*0-100*/) override;
  void setFingerSpeeds(const FingerArray& speeds /*-100~100*/) override;
  bool getFingerStatus(FingerStatus& status) override;

  // ---- DexHandBase: stubs (not needed for basic dual-hand bring-up) ----
  void setGripForce(GripForce) override {}
  GripForce getGripForce() override { return FORCE_LEVEL_NORMAL; }
  void setTurboModeEnabled(bool) override {}
  bool isTurboModeEnabled() override { return false; }
  TurboConfig_t getTurboConfig() override { return {0, 0}; }
  void runActionSequence(ActionSequenceId_t) override {}
  bool setActionSequence(ActionSequenceId_t, const ActionSeqDataTypeVec&) override { return false; }

 private:
  // Device-side range is 0~1000 (positions) and -1000~+1000 (speeds/currents).
  static constexpr uint16_t kUserPosMax     = 100;
  static constexpr uint16_t kDevicePosMax   = 1000;
  static constexpr int16_t  kUserSpeedMax   = 100;
  static constexpr int16_t  kDeviceSpeedMax = 1000;
  static constexpr int kScale = kDevicePosMax / kUserPosMax;  // 10x

  uint8_t slave_id_;
  std::string bus_name_;
  DeviceHandler* handle_ = nullptr;
  std::mutex mu_;
};

}  // namespace dexhand
