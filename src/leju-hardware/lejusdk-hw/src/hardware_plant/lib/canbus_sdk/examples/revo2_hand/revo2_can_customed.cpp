/*
 * revo2_can_customed.cpp
 *
 * 单手Revo2灵巧手CAN通信示例程序
 *
 * 功能：
 * - 控制单个Revo2灵巧手
 * - 使用can0总线进行通信
 * - 支持手指位置控制和持续时间设置
 * - 实时显示电机状态信息
 * - 通过消息队列处理CAN通信数据
 *
 * Copyright (c) 2025 LejuRobot
 * 
 * ohh
 */

#include <stdio.h>
#include <unistd.h>
#include <signal.h>
#include <execinfo.h>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <chrono>

#include "stark-sdk.h"
#include "canbus_sdk/canbus_sdk.h"

// 调试打印宏
#define DEBUG_PRINT_CAN_DATA 0

#define LHAND_DEVICE_ID 0x01  /* 左手 */
#define RHADN_DEVICE_ID 0x02  /* 右手 */
#define HAND_DEVICE_ID LHAND_DEVICE_ID

// 声明函数
void setup_canfd_callbacks();
void get_device_info(DeviceHandler *handleint, uint8_t slave_id);
void can_receive_callback(canbus_sdk::CanMessageFrame* frame, const canbus_sdk::CallbackContext* context);
void can_tef_callback(canbus_sdk::CanMessageFrame* frame, const canbus_sdk::CallbackContext* context);
void print_motor_status(DeviceHandler *handle, uint8_t slave_id);

void handler(int sig)
{
  void *array[10];
  size_t size;

  // 获取堆栈帧
  size = backtrace(array, 10);

  // 打印所有堆栈帧到 stderr
  fprintf(stderr, "Error: signal %d:\n", sig);
  backtrace_symbols_fd(array, size, STDERR_FILENO);
  exit(1);
}

canbus_sdk::CanBusController *canbus_controller = nullptr;

// CAN消息队列结构
struct CanMessageQueue {
    std::queue<canbus_sdk::CanMessageFrame> queue;
    std::mutex mutex;
    std::condition_variable cv;

    void push(const canbus_sdk::CanMessageFrame& frame) {
        std::lock_guard<std::mutex> lock(mutex);
        queue.push(frame);
        cv.notify_one();
    }

      // 获取队列第一条消息（不弹出）
    bool front(canbus_sdk::CanMessageFrame* frame_out) {
        std::lock_guard<std::mutex> lock(mutex);
        if (!queue.empty()) {
            *frame_out = queue.front();
            return true;
        }
        return false;
    }

    // 弹出一条消息
    bool try_pop(canbus_sdk::CanMessageFrame* frame_out, int timeout_ms = 100) {
        std::unique_lock<std::mutex> lock(mutex);
        if (cv.wait_for(lock, std::chrono::milliseconds(timeout_ms), [this] { return !queue.empty(); })) {
            if (!queue.empty()) {
                *frame_out = queue.front();
                queue.pop();
                return true;
            }
        }
        return false;
    }
};

CanMessageQueue g_can_message_queue;

int main(int argc, char const *argv[])
{
  signal(SIGSEGV, handler); // Install our handler for SIGSEGV (segmentation fault)
  signal(SIGABRT, handler); // Install our handler for SIGABRT (abort signal)

  ////////////////////////////////////////////////////////////////////////////
  // 初始化 CANBUS_SDK !
  // 初始化CAN总线控制器
  using namespace canbus_sdk;
  canbus_controller = &CanBusController::getInstance();
  canbus_controller->init();

  // 配置CAN总线比特率
  CanBusBitrate bitrate = {
      .nbitrate = 1000,
      .dbitrate = 2000,    
      .nsamplepos = 75,   
      .dsamplepos = 75
  };

  // 打开CAN总线
  auto result = canbus_controller->openCanBus("can0", CanBusModelType::LEJU_CAN_A, bitrate);
  if (!result.has_value()) {
      printf("Failed to open CAN bus: %s\n", errorToString(result.error()));
      return -1;
  }

  auto Matcher = [](uint32_t can_id, uint32_t device_id) -> bool {
      ///////////////////////////////////////
      //  device_id : 4 bit 1~14
      //  cmd :       4 bit 1~15
      //  channel : 3 bit 0~7
      // 0001,0000,000 ~ 1110,1111,111 0X80 ~ 0x77F
      ///////////////////////////////////////
      constexpr uint32_t kHandMinCanId = 0X80, kHandMaxCanId = 0X77F;
      if (kHandMinCanId <= can_id && can_id <= kHandMaxCanId) {
        uint32_t id = (can_id >> 7) & 0x0F;  // 7-10位是Device ID
        return id == device_id;
      }
      return false;
  };

  canbus_sdk::CallbackParams callback_params{
      .msg_callback = can_receive_callback,
      .msg_cb_ctx = nullptr,
      .tef_callback = can_tef_callback,
      .tef_cb_ctx = nullptr
  };
  canbus_controller->registerDevice(canbus_sdk::DeviceInfo(
      "revo2_Rhand",
      canbus_sdk::DeviceType::REVO2_HAND,
      HAND_DEVICE_ID,
      Matcher
  ), "can0", callback_params);
  ///////////////////////////////////////////////////////////////////////////////

  setup_canfd_callbacks(); // 设置读写回调

  init_cfg(STARK_PROTOCOL_TYPE_CAN, LOG_LEVEL_INFO);
  
  useconds_t delay = 1000 * 1000 * 1;
  uint8_t slave_id_hand = HAND_DEVICE_ID; 
  auto handle = create_device_handler();
  /* !!! IMPORTANT !!! 必须要调用获取设备信息识别一下设备! */
  get_device_info(handle, slave_id_hand);
  /* !!! IMPORTANT !!! */

  uint16_t durations[6] = {30, 30, 30, 30, 30, 30};
  uint16_t positions_list[][6] = {
    {100, 100, 100, 100, 100, 100},
    {200, 100, 900, 100, 100, 100},
    {300, 100, 900, 900, 100, 100},
    {400, 100, 900, 900, 900, 100},
    {500, 100, 900, 900, 900, 900},
  };
  const int num_steps = sizeof(positions_list) / sizeof(positions_list[0]);

  for(int j = 0; j < 10; ++j) {
    for (int i = 0; i < num_steps; ++i) {
      stark_set_finger_positions_and_durations(handle, slave_id_hand,
                                             positions_list[i], durations, 6);
      print_motor_status(handle, slave_id_hand);
      usleep(delay);
    }
    for (int i = num_steps - 1; i >= 0; --i) {
      stark_set_finger_positions(handle, slave_id_hand, positions_list[i], 6);
      print_motor_status(handle, slave_id_hand);
      usleep(delay);
    }
  }

  return 0;
}

void setup_canfd_callbacks()
{
  // 需要由接入方实现，开启CAN FD设备，和以下读写回调
  set_can_tx_callback([](uint8_t slave_id,
                         uint32_t can_id,
                         const uint8_t *data,
                         uintptr_t data_len) -> int
                      {
#if DEBUG_PRINT_CAN_DATA
                        uint8_t device_id = (can_id >> 7) & 0x0F;  // 7-10位是Device ID
                        uint8_t cmd = (can_id >> 3) & 0x0F;        // 3-6位是Command
                        uint8_t channel = can_id & 0x07;           // 0-2位是Channel
                        printf("CAN TX: Slave ID: %d, CAN ID: 0x%x, cmd:0x%X, Data Length: %zu, Data: ", slave_id, can_id, cmd, data_len);
                        for (uintptr_t i = 0; i < data_len; ++i) {
                            printf("%02X ", data[i]);
                        }
                        printf("\n");
#endif
                        using namespace canbus_sdk;
                        CanMessageFrame can_frame{};
                        can_frame.id.SID = static_cast<uint32_t>(can_id);
                        can_frame.ctrl.tx.dlc = payload_length_to_dlc(8);
                        memcpy(can_frame.payload, data, data_len);
                        canbus_controller->sendMessage("can0", can_frame);
                        return 0; // Return 0 to indicate success
                      });
                  
  set_can_rx_callback([](uint8_t slave_id,
                         uint32_t *can_id_out,
                         uint8_t *data_out,
                         uintptr_t *data_len_out) -> int
                      {
                        // 从消息队列中读取数据
                        canbus_sdk::CanMessageFrame first_frame;
                        if (!g_can_message_queue.try_pop(&first_frame)) {
                          printf("Failed to read from CAN message queue\n");
                          return -1;
                        }
                          
                        constexpr int MAX_BUF_SIZE = 8;
                        canbus_sdk::CanMessageFrame received_frames[MAX_BUF_SIZE];
                        received_frames[0] = first_frame;
                        int frame_count = 1;

                        // 解析第一条消息的CAN ID和cmd
                        uint32_t base_can_id = received_frames[0].id.SID;
                        uint8_t base_cmd = (base_can_id >> 3) & 0x0F;

                        // 如果cmd是0x0B，继续获取后续连续的具有相同CAN ID和cmd的消息
                        if (base_cmd == 0x0B) {
                            while (frame_count < MAX_BUF_SIZE) {
                                canbus_sdk::CanMessageFrame peek_frame;
                                if (!g_can_message_queue.front(&peek_frame))
                                    break;

                                uint32_t next_can_id = peek_frame.id.SID;
                                uint8_t next_cmd = (next_can_id >> 3) & 0x0F;

                                // 检查是否具有相同的CAN ID和cmd
                                if (next_can_id != base_can_id || next_cmd != base_cmd)
                                    break;

                                // 弹出这条消息
                                canbus_sdk::CanMessageFrame next_frame;
                                if (!g_can_message_queue.try_pop(&next_frame))
                                    break;

                                received_frames[frame_count] = next_frame;
                                frame_count++;
                            }
                        }

                        // 复制数据
                        int idx = 0;
                        int total_dlc = 0;
                        for (int i = 0; i < frame_count; i++)
                        {
                          canbus_sdk::CanMessageFrame &frame = received_frames[i];
                          int can_dlc = frame.ctrl.rx.dlc;
                          for (int j = 0; j < can_dlc; j++)
                          {
                            data_out[idx++] = frame.payload[j];
                          }
                          total_dlc += can_dlc;
                        }

                        *can_id_out = received_frames[0].id.SID & 0x1FFFFFFF;
                        *data_len_out = total_dlc;
                        return 0;
                      });
}

void can_receive_callback(canbus_sdk::CanMessageFrame* frame, const canbus_sdk::CallbackContext* context)
{
#if DEBUG_PRINT_CAN_DATA
  printf("\033[32mCAN FD Received: ID: 0x%X, DLC: %d, Data: ", frame->id.SID, frame->ctrl.rx.dlc);
  for (int i = 0; i < frame->ctrl.rx.dlc; ++i)
  {
    printf("%02x ", frame->payload[i]);
  }
  printf("\033[0m\n");
#endif

  // 将接收到的消息存入队列，供stark-sdk使用
  g_can_message_queue.push(*frame);
  freeCanMessageFrame(frame);
}

void can_tef_callback(canbus_sdk::CanMessageFrame* frame, const canbus_sdk::CallbackContext* context)
{
  printf("\033[33mTEF Event - CAN transmission completed: ID: 0x%X, DLC: %d, Data: ", frame->id.SID, frame->ctrl.rx.dlc);
  for (int i = 0; i < frame->ctrl.rx.dlc; ++i)
  {
    printf("%02x ", frame->payload[i]);
  }
  printf("\033[0m\n");

  freeCanMessageFrame(frame);
}

void get_device_info(DeviceHandler *handle, uint8_t slave_id)
{
  DeviceInfo *info = stark_get_device_info(handle, slave_id);
  if (info != NULL)
  {
    printf("\033[32m"); // 绿色
    printf("Slave[%hhu] Serial Number: %s\n", slave_id, info->serial_number);
    printf("Slave[%hhu] Firmware Version: %s\n", slave_id, info->firmware_version);
    printf("Slave[%hhu] Hardware Type: %d\n", slave_id, info->hardware_type);
    printf("Slave[%hhu] sku_type: %d\n", slave_id, info->sku_type);
    printf("\033[0m"); // 重置颜色
    free_device_info(info);
  }
  else
  {
    printf("\033[31m"); // 红色
    printf("Failed to get device info for Slave[%hhu]\n", slave_id);
    printf("\033[0m"); // 重置颜色
  }
}

void print_motor_status(DeviceHandler *handle, uint8_t slave_id)
{
  MotorStatusData* data = stark_get_motor_status(handle, slave_id);
  if(data) {
    printf("\033[32m"); // 绿色
    printf("------------------\nMotor Status: ");
    printf("\npositions: ");
    for(int i = 0; i < 6; ++i)
      printf("%u ", data->positions[i]);
      printf("\nspeeds: ");
    for(int i = 0; i < 6; ++i)
      printf("%d ", data->speeds[i]);
    printf("\ncurrents: ");
    for(int i = 0; i < 6; ++i)
      printf("%d ", data->currents[i]);
    printf("\nstatus: ");
    for(int i = 0; i < 6; ++i)
      printf("%u ", static_cast<uint32_t>(data->states[i]));
    printf("\n");
    printf("\033[0m"); // 重置颜色
  }
  else {
    printf("\033[31m"); // 红色
    printf("Failed to get motor status\n");
    printf("\033[0m"); // 重置颜色
  }
  free_motor_status_data(data);
}
