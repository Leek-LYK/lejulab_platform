// Dual-hand Revo2 classic CAN bring-up.
//
// Goal: test left hand on bcan2 + right hand on bcan3 in ONE process.
//
// Architecture (mirrors kuavo-ros-control dual_revo2_can_customed.cpp but
// uses canbus_sdk SOCKETCAN backend instead of LEJU_CAN_A/B):
//
//   +----------------------+
//   | canbus_sdk           |  opens bcan2 + bcan3 (SocketCAN, classic 1Mbps),
//   | CanBusController     |  owns RX threads, dispatches by device matcher.
//   +----------+-----------+
//              |  on_rx_{left,right}()
//              v
//   +----------------------+      pop
//   | per-hand RX queue    |<--------+
//   +----------------------+         |
//                                    |
//   +------- Stark SDK (v1.1.9) -----|--------+
//   | stark_* APIs     <- handle_    |        |
//   |   TX callback  -> route by slave_id to  |
//   |                   canbus_sdk->sendMessage(bcan2|bcan3, ...)
//   |   RX callback  -> pop from matching hand's queue
//   +-----------------------------------------+
//              ^
//              |
//   +----------+-----------+
//   | Revo2CanHand         |  DexHandBase impl, 0~100 <-> 0~1000 scaling
//   +----------------------+

#include "revo2_can_hand.h"

#include "canbus_sdk/canbus_sdk.h"
#include "stark-sdk.h"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <csignal>
#include <iostream>
#include <mutex>
#include <queue>
#include <thread>

using namespace dexhand;
using namespace canbus_sdk;

// =========================================================================
// Configuration (matches kuavo canbus_device_config.yaml convention)
// =========================================================================
static constexpr uint8_t kLhandId = 0x01;
static constexpr uint8_t kRhandId = 0x02;
static constexpr const char* kLhandBus = "bcan2";
static constexpr const char* kRhandBus = "bcan3";

// =========================================================================
// Per-hand RX queue (Stark RX callback pops from here)
// =========================================================================
struct CanMessageQueue {
  std::queue<CanMessageFrame> q;
  std::mutex mu;
  std::condition_variable cv;

  void push(const CanMessageFrame& f) {
    {
      std::lock_guard<std::mutex> lk(mu);
      q.push(f);
    }
    cv.notify_one();
  }

  bool front(CanMessageFrame* out) {
    std::lock_guard<std::mutex> lk(mu);
    if (q.empty()) return false;
    *out = q.front();
    return true;
  }

  bool try_pop(CanMessageFrame* out, int timeout_ms = 200) {
    std::unique_lock<std::mutex> lk(mu);
    if (cv.wait_for(lk, std::chrono::milliseconds(timeout_ms),
                    [this] { return !q.empty(); })) {
      *out = q.front();
      q.pop();
      return true;
    }
    return false;
  }
};

static CanMessageQueue g_left_q;
static CanMessageQueue g_right_q;
static CanBusController* g_ctrl = nullptr;
static std::atomic<bool> g_running{true};

// =========================================================================
// canbus_sdk RX callbacks — enqueue raw frames to per-hand queues
// =========================================================================
static void on_rx_left(CanMessageFrame* frame, const CallbackContext*) {
  g_left_q.push(*frame);
  freeCanMessageFrame(frame);
}

static void on_rx_right(CanMessageFrame* frame, const CallbackContext*) {
  g_right_q.push(*frame);
  freeCanMessageFrame(frame);
}

static void on_tef(CanMessageFrame* frame, const CallbackContext*) {
  // TX-event feedback: ignore for now, just free.
  freeCanMessageFrame(frame);
}

// =========================================================================
// Stark SDK bridge callbacks (global; route by slave_id)
// =========================================================================
static int32_t stark_tx_cb(uint8_t slave_id, uint32_t can_id,
                           const uint8_t* data, uintptr_t data_len) {
  CanMessageFrame frame{};
  frame.id.SID = static_cast<uint32_t>(can_id);
  frame.ctrl.tx.dlc = payload_length_to_dlc(8);
  std::memcpy(frame.payload, data, data_len);

  if (slave_id == kLhandId) {
    g_ctrl->sendMessage(kLhandBus, frame);
  } else if (slave_id == kRhandId) {
    g_ctrl->sendMessage(kRhandBus, frame);
  } else {
    std::cerr << "[stark_tx] unknown slave_id=" << static_cast<int>(slave_id) << "\n";
    return -1;
  }
  return 0;
}

static int32_t stark_rx_cb(uint8_t slave_id, uint32_t /*expected_can_id*/,
                           uint8_t /*expected_frames*/,
                           uint32_t* can_id_out, uint8_t* data_out,
                           uintptr_t* data_len_out) {
  CanMessageQueue* q = nullptr;
  if (slave_id == kLhandId)       q = &g_left_q;
  else if (slave_id == kRhandId)  q = &g_right_q;
  else {
    std::cerr << "[stark_rx] unknown slave_id=" << static_cast<int>(slave_id) << "\n";
    return -1;
  }

  // Pop the first frame (blocking up to 200ms).
  CanMessageFrame first;
  if (!q->try_pop(&first)) return -1;

  constexpr int kMaxBuf = 8;
  CanMessageFrame frames[kMaxBuf];
  frames[0] = first;
  int got = 1;
  int total_dlc = first.ctrl.rx.dlc;

  // Multi-frame aggregation when cmd=0x0B (motor status burst).
  const uint32_t base_cid = first.id.SID;
  const uint8_t  base_cmd = static_cast<uint8_t>((base_cid >> 3) & 0x0F);
  if (base_cmd == 0x0B) {
    while (got < kMaxBuf) {
      CanMessageFrame peek;
      if (!q->front(&peek)) break;
      const uint32_t ncid = peek.id.SID;
      const uint8_t  ncmd = static_cast<uint8_t>((ncid >> 3) & 0x0F);
      if (ncid != base_cid || ncmd != base_cmd) break;
      CanMessageFrame nf;
      if (!q->try_pop(&nf, 50)) break;
      frames[got++] = nf;
      total_dlc += nf.ctrl.rx.dlc;
    }
  }

  // Concatenate payloads
  int idx = 0;
  for (int i = 0; i < got; ++i) {
    const int dlc = frames[i].ctrl.rx.dlc;
    for (int j = 0; j < dlc; ++j) data_out[idx++] = frames[i].payload[j];
  }
  *can_id_out = first.id.SID & 0x1FFFFFFF;
  *data_len_out = static_cast<uintptr_t>(total_dlc);
  return 0;
}

// =========================================================================
// Bus setup
// =========================================================================
static bool setup_can_buses() {
  g_ctrl = &CanBusController::getInstance();
  g_ctrl->init();

  CanBusBitrate bitrate{};
  bitrate.nbitrate   = 1000;  // 1 Mbps classic CAN arbitration
  bitrate.dbitrate   = 0;     // 0 => classic CAN (FD disabled)
  bitrate.nsamplepos = 75;
  bitrate.dsamplepos = 75;

  auto r1 = g_ctrl->openCanBus(kLhandBus, CanBusModelType::SOCKETCAN, bitrate);
  if (!r1.has_value()) {
    std::cerr << "[setup] openCanBus " << kLhandBus << " failed: "
              << errorToString(r1.error()) << "\n";
    return false;
  }
  auto r2 = g_ctrl->openCanBus(kRhandBus, CanBusModelType::SOCKETCAN, bitrate);
  if (!r2.has_value()) {
    std::cerr << "[setup] openCanBus " << kRhandBus << " failed: "
              << errorToString(r2.error()) << "\n";
    return false;
  }

  // Revo2 hand CAN ID layout (11-bit standard):
  //   bits[10:7] = device_id (1..14), bits[6:3] = cmd, bits[2:0] = channel
  //   full range: 0x080..0x77F (device_id shifted by 7)
  auto matcher = [](uint32_t can_id, uint32_t device_id) -> bool {
    constexpr uint32_t kMinId = 0x080, kMaxId = 0x77F;
    if (can_id < kMinId || can_id > kMaxId) return false;
    return ((can_id >> 7) & 0x0F) == device_id;
  };

  CallbackParams lcb{};
  lcb.msg_callback = on_rx_left;
  lcb.tef_callback = on_tef;
  auto rl = g_ctrl->registerDevice(
      DeviceInfo("revo2_Lhand", DeviceType::REVO2_HAND, kLhandId, matcher),
      kLhandBus, lcb);
  if (!rl.has_value()) {
    std::cerr << "[setup] registerDevice L failed: " << errorToString(rl.error()) << "\n";
    return false;
  }

  CallbackParams rcb{};
  rcb.msg_callback = on_rx_right;
  rcb.tef_callback = on_tef;
  auto rr = g_ctrl->registerDevice(
      DeviceInfo("revo2_Rhand", DeviceType::REVO2_HAND, kRhandId, matcher),
      kRhandBus, rcb);
  if (!rr.has_value()) {
    std::cerr << "[setup] registerDevice R failed: " << errorToString(rr.error()) << "\n";
    return false;
  }

  set_can_tx_callback(stark_tx_cb);
  set_can_rx_callback(stark_rx_cb);
  return true;
}

// =========================================================================
// Main
// =========================================================================
static void sig_handler(int) { g_running = false; }

int main(int /*argc*/, char** /*argv*/) {
  std::signal(SIGINT,  sig_handler);
  std::signal(SIGTERM, sig_handler);

  init_logging(LOG_LEVEL_INFO);

  if (!setup_can_buses()) {
    std::cerr << "[main] setup_can_buses FAILED\n";
    return 1;
  }
  std::cout << "[main] buses up: L=" << kLhandBus << " R=" << kRhandBus << "\n";

  auto left  = Revo2CanHand::Create(kLhandId, kLhandBus);
  auto right = Revo2CanHand::Create(kRhandId, kRhandBus);
  if (!left->ok() || !right->ok()) {
    std::cerr << "[main] handler creation FAILED\n";
    return 2;
  }

  // Handshake: device info with retry (3 attempts each, matches reference).
  auto handshake = [](const char* side, std::shared_ptr<Revo2CanHand>& h) {
    for (int i = 0; i < 3; ++i) {
      DeviceInfo_t info;
      if (h->getDeviceInfo(info)) {
        std::cout << "[" << side << "] connected:\n" << info;
        return true;
      }
      std::cerr << "[" << side << "] getDeviceInfo retry " << (i + 1) << "/3\n";
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::cerr << "[" << side << "] getDeviceInfo TIMED OUT\n";
    return false;
  };
  handshake("left",  left);
  handshake("right", right);

  // 0~100 position sweep on both hands at ~50 Hz.
  std::cout << "[main] starting 0-100 sweep (Ctrl-C to stop)\n";
  while (g_running) {
    for (uint16_t p = 0; g_running && p <= 100; p += 5) {
      UnsignedFingerArray pos;
      // Thumb (index 0,1) scaled smaller — safer during first bring-up.
      pos[0] = static_cast<uint16_t>(p * 2 / 5);
      pos[1] = static_cast<uint16_t>(p * 2 / 5);
      pos[2] = p; pos[3] = p; pos[4] = p; pos[5] = p;
      left->setFingerPositions(pos);
      right->setFingerPositions(pos);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    for (int p = 100; g_running && p >= 0; p -= 5) {
      UnsignedFingerArray pos;
      pos[0] = static_cast<uint16_t>(p * 2 / 5);
      pos[1] = static_cast<uint16_t>(p * 2 / 5);
      pos[2] = static_cast<uint16_t>(p);
      pos[3] = static_cast<uint16_t>(p);
      pos[4] = static_cast<uint16_t>(p);
      pos[5] = static_cast<uint16_t>(p);
      left->setFingerPositions(pos);
      right->setFingerPositions(pos);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }

  std::cout << "[main] stopping\n";
  return 0;
}
