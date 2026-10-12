#include "revo2_can_hand.h"

#include <algorithm>
#include <cstring>
#include <iostream>

namespace dexhand {

std::shared_ptr<Revo2CanHand> Revo2CanHand::Create(uint8_t slave_id, std::string bus_name) {
  return std::make_shared<Revo2CanHand>(slave_id, std::move(bus_name));
}

Revo2CanHand::Revo2CanHand(uint8_t slave_id, std::string bus_name)
    : slave_id_(slave_id), bus_name_(std::move(bus_name)) {
  // v1.1.9: init_device_handler creates a per-hand handle. For CAN 2.0 the
  // master_id arg is unused by the SDK (transport is via our TX/RX callbacks).
  handle_ = init_device_handler(STARK_PROTOCOL_TYPE_CAN, 0);
  if (!handle_) {
    std::cerr << "[Revo2CanHand] init_device_handler failed for slave=0x"
              << std::hex << static_cast<int>(slave_id_) << std::dec << "\n";
    return;
  }
  std::cout << "[Revo2CanHand] slave=0x" << std::hex << static_cast<int>(slave_id_)
            << std::dec << " bus=" << bus_name_ << " ready\n";
}

Revo2CanHand::~Revo2CanHand() = default;
// v1.1.9 stark-sdk.h does not expose a public `close_device_handler` symbol
// for CAN handles; the SDK reclaims on process exit. Left intentionally blank.

bool Revo2CanHand::getDeviceInfo(DeviceInfo_t& out) {
  if (!handle_) return false;
  CDeviceInfo* info = stark_get_device_info(handle_, slave_id_);
  if (!info) return false;

  out.sku_type = static_cast<DexHandType>(info->sku_type);
  out.serial_number   = info->serial_number   ? info->serial_number   : "";
  out.firmware_version= info->firmware_version? info->firmware_version: "";

  free_device_info(info);
  return true;
}

void Revo2CanHand::setFingerPositions(const UnsignedFingerArray& pos_user) {
  if (!handle_) return;
  uint16_t pos_dev[6];
  for (size_t i = 0; i < 6; ++i) {
    uint16_t u = std::min<uint16_t>(pos_user[i], kUserPosMax);
    pos_dev[i] = static_cast<uint16_t>(u) * kScale;  // 0~100 → 0~1000
  }
  std::lock_guard<std::mutex> lk(mu_);
  stark_set_finger_positions(handle_, slave_id_, pos_dev, 6);
}

void Revo2CanHand::setFingerSpeeds(const FingerArray& speeds_user) {
  if (!handle_) return;
  int16_t speeds_dev[6];
  for (size_t i = 0; i < 6; ++i) {
    int16_t v = std::max<int16_t>(-kUserSpeedMax,
                 std::min<int16_t>(kUserSpeedMax, speeds_user[i]));
    speeds_dev[i] = static_cast<int16_t>(v * kScale);  // -100~100 → -1000~1000
  }
  std::lock_guard<std::mutex> lk(mu_);
  stark_set_finger_speeds(handle_, slave_id_, speeds_dev, 6);
}

bool Revo2CanHand::getFingerStatus(FingerStatus& out) {
  if (!handle_) return false;

  CMotorStatusData* data = nullptr;
  {
    std::lock_guard<std::mutex> lk(mu_);
    data = stark_get_motor_status(handle_, slave_id_);
  }
  if (!data) return false;

  for (int i = 0; i < 6; ++i) {
    // Device side: positions 0~1000, speeds -1000~1000. Downscale to user range.
    out.positions[i] = static_cast<uint16_t>(data->positions[i] / kScale);
    out.speeds[i]    = static_cast<int16_t>(data->speeds[i]   / kScale);
    out.currents[i]  = static_cast<int16_t>(data->currents[i]);  // keep raw (mA-ish)
    out.states[i]    = static_cast<uint16_t>(data->states[i]);
  }
  free_motor_status_data(data);
  return true;
}

}  // namespace dexhand
