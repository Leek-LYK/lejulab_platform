#ifndef CANFD_LEJU_H
#define CANFD_LEJU_H

#include <linux/can.h>
#include <vector>
#include <string>
#include <mutex>
#include <deque>
#include <termios.h>  // 串口支持

#define DEBUG_ENABLE 0
#define PRINTF_ENABLE 0

// 状态返回枚举
typedef enum {
    LEJU_ERROR_OK = 0,              // 成功
    LEJU_ERROR_BUSTIMEOUT = 1,      // 总线超时
    LEJU_ERROR_INIT = 2,            // 初始化失败
    LEJU_ERROR_SEND = 3,            // 发送失败
    LEJU_ERROR_RECEIVE = 4,         // 接收失败
    LEJU_ERROR_PARSE = 5,           // 解析失败
    LEJU_ERROR_INVALID_PARAM = 6,   // 无效参数
    LEJU_ERROR_UNKNOWN = 0xFF       // 未知错误
} LEJU_StatusTypeDef;

struct Recv_frame {
    uint32_t id;
    uint8_t dlc;
    std::vector<uint8_t> data;
};

class CANable {
public:
    explicit CANable(const std::string& channel_name = "canbus0", bool enable_canfd = false);
    ~CANable();

    bool Init();

    LEJU_StatusTypeDef Send(uint32_t id, const std::vector<uint8_t>& data);

    LEJU_StatusTypeDef Receive(Recv_frame& rx_frame);

    uint8_t GetError();

    void Close();


private:
    std::string can_channel;
    int serial_fd;
    bool canfd_enabled;
    std::mutex send_mutex;        // 发送互斥锁
    std::mutex recv_mutex;        // 接收互斥锁

    struct termios tty_old;

    std::string recv_buffer; 
    std::deque<Recv_frame> frame_queue;  // 帧队列缓存
    
    LEJU_StatusTypeDef batch_read_and_parse();
    bool CANfd_serial_init();
    std::string Frame_to_SLCAN(uint32_t id, const std::vector<uint8_t>& data);
    Recv_frame SLCAN_to_Frame(const std::string& slcan_str);
    uint8_t get_dlc_code(size_t data_len);
    size_t get_dlc_length(uint8_t dlc_code);
    void get_firmware_version(); 
    std::string get_DevicePath(const std::string& channel_name);
    
    std::string setBitrate(uint32_t bitrate);
    std::string setFDBitrate(uint32_t fd_bitrate);
};

#endif 
