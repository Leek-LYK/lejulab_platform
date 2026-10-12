#include "CANFD_leju.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <sstream>
#include <iomanip>
#include <poll.h>
#include <errno.h>
#include <time.h>


static inline int nanosleep_us(unsigned int us) {
    struct timespec req;
    req.tv_sec = us / 1000000u;
    req.tv_nsec = (long)((us % 1000000u) * 1000u);
    while (clock_nanosleep(CLOCK_MONOTONIC, 0, &req, &req) == EINTR);
    return 0;
}

CANable::CANable(const std::string& channel_name, bool enable_canfd) 
        : can_channel(channel_name), serial_fd(-1), canfd_enabled(enable_canfd) {}

CANable::~CANable() {
    Close();
}

// 获取CAN FD DLC编码
uint8_t CANable::get_dlc_code(size_t data_len) {
    if (data_len <= 8) return data_len;
    if (data_len <= 12) return 9;
    if (data_len <= 16) return 0xA;
    if (data_len <= 20) return 0xB;
    if (data_len <= 24) return 0xC;
    if (data_len <= 32) return 0xD;
    if (data_len <= 48) return 0xE;
    return 0xF; // 最大64字节
}

// 从DLC编码获取数据长度
size_t CANable::get_dlc_length(uint8_t dlc_code) {
    if (dlc_code <= 8) return dlc_code;
    switch (dlc_code) {
        case 9: return 12;
        case 0xA: return 16;
        case 0xB: return 20;
        case 0xC: return 24;
        case 0xD: return 32;
        case 0xE: return 48;
        case 0xF: return 64;
        default: return 8;
    }
}

// 编码CAN FD帧为SLCAN格式
std::string CANable::Frame_to_SLCAN(uint32_t id, const std::vector<uint8_t>& data) {
    // 添加数据长度检查
    if (data.size() > 64) {
        printf("[ERROR] 数据长度超出CAN FD最大限制(64字节): %zu\n", data.size());
        return "";
    }

    std::stringstream ss;

    bool is_fd = false;                                   // 是否使用CAN FD
    bool is_brs = false;                                 // 默认启用BRS（波特率切换）
    bool is_extended = (id > 0x7FF);                    // 是否为扩展帧
    bool is_remote = false;                             // 是否为远程帧

    // 帧类型选择逻辑
    char frame_type_tx;
    if (is_remote) {
        frame_type_tx = (is_extended ? 'R' : 'r');
    } else if(is_fd){
        if (is_brs) {
            frame_type_tx = (is_extended ? 'B' : 'b');  // CAN FD帧带BRS
        } else {
            frame_type_tx = (is_extended ? 'D' : 'd');
        }
    } else {
        frame_type_tx = (is_extended ? 'T' : 't');      // 标准CAN帧
    }

    ss << frame_type_tx; // 帧类型前缀

    // 根据ID范围选择格式
    if (is_extended) {
        // 扩展ID CAN FD: DIIIIIIIILDD...
        ss << std::setfill('0') << std::setw(8) << std::hex << std::uppercase << id;

        #if DEBUG_ENABLE
            printf("[DEBUG] 使用扩展ID: 0x%X\n", id);
        #endif
    } else {
        // 标准ID CAN FD: dIIILDD...
        ss << std::setfill('0') << std::setw(3) << std::hex << std::uppercase << id;

        #if DEBUG_ENABLE
            printf("[DEBUG] 使用标准ID: 0x%X\n", id);
        #endif
    }
    
    // DLC编码
    uint8_t dlc_code;
    if (frame_type_tx == 't' || frame_type_tx == 'T' || 
        frame_type_tx == 'r' || frame_type_tx == 'R') {
        // 标准帧：DLC直接等于数据长度
        dlc_code = (data.size() > 8) ? 8 : data.size();
    } else {
        // CAN FD帧：使用DLC编码
        dlc_code = get_dlc_code(data.size());
    }
 
    // 输出DLC编码
    if (dlc_code <= 9) {
        ss << static_cast<char>('0' + dlc_code);
    } else {
        ss << static_cast<char>('A' + dlc_code - 10);
    }

    // 数据字节
    for (const auto& byte : data) {
        ss << std::setfill('0') << std::setw(2) << std::hex << std::uppercase << static_cast<int>(byte);
    }
    
    ss << "\r";
    return ss.str();
}

// 解码SLCAN格式为CAN FD帧
Recv_frame CANable::SLCAN_to_Frame(const std::string& slcan_str) {
    Recv_frame frame = {0};
    
    if (slcan_str.length() < 5) return frame;
    
    char frame_type = slcan_str[0];
    std::string data_str = slcan_str.substr(1);
    
    try {
        if (frame_type == 't' || frame_type == 'T' || 
            frame_type == 'd' || frame_type == 'D' || 
            frame_type == 'b' || frame_type == 'B' || 
            frame_type == 'r' || frame_type == 'R') {
            // CAN FD帧解析
            int id_len = (frame_type == 't' || frame_type == 'd' || frame_type == 'b' || frame_type == 'r') ? 3 : 8;
            frame.id = std::stoul(data_str.substr(0, id_len), nullptr, 16);
            
            // 解析DLC编码
            char dlc_char = data_str[id_len];
            uint8_t dlc_code;
            if (dlc_char >= '0' && dlc_char <= '9') {
                dlc_code = dlc_char - '0';
            } else if (dlc_char >= 'A' && dlc_char <= 'F') {
                dlc_code = dlc_char - 'A' + 10;
            } else {
                return frame; // 无效DLC
            }
            
            // 标准帧和FD帧的DLC处理
            if (frame_type == 't' || frame_type == 'T' || 
                frame_type == 'r' || frame_type == 'R') {
                frame.dlc = dlc_code;  // 标准帧DLC直接=数据长度
            } else {
                frame.dlc = get_dlc_length(dlc_code);
            }
            
            // 解析数据字节
            for (int i = 0; i < frame.dlc; i++) {
                size_t byte_pos = id_len + 1 + i * 2;
                if (byte_pos + 1 < data_str.length()) {
                    std::string byte_str = data_str.substr(byte_pos, 2);
                    frame.data.push_back(std::stoul(byte_str, nullptr, 16));
                }
            }
        }
    } catch (const std::exception& e) {
        printf("[ERROR] 解析CAN FD帧失败: %s\n", e.what());
        frame = {0};
    }
    
    return frame;
}


LEJU_StatusTypeDef CANable::Receive(Recv_frame& rx_frame) {
    rx_frame = {0};
    
    if (!canfd_enabled || serial_fd < 0) {
        return LEJU_ERROR_INIT;
    }
    
    // 使用单一锁保护接收操作
    std::lock_guard<std::mutex> lock(recv_mutex);
    
    // 如果队列为空，尝试批量读取和解析
    if (frame_queue.empty()) {
        LEJU_StatusTypeDef status = batch_read_and_parse();
        if (status != LEJU_ERROR_OK) {
            return status;
        }
    }
    
    // 从队列中取帧
    if (!frame_queue.empty()) {
        rx_frame = frame_queue.front();
        frame_queue.pop_front();
        
        #if PRINTF_ENABLE
            printf("\033[1;32m[%s][RX-FD] 已接收 ID=0x%X LEN=%d Data=\033[0m", can_channel.c_str(), rx_frame.id, rx_frame.dlc);
            for (const auto& byte : rx_frame.data) {
                printf("%02X ", byte);
            }
            printf("\n");
        #endif
        
        return LEJU_ERROR_OK;
    }
    
    return LEJU_ERROR_BUSTIMEOUT;
}

LEJU_StatusTypeDef CANable::batch_read_and_parse() {
    // 检查串口是否有数据
    struct pollfd pfd;
    pfd.fd = serial_fd;
    pfd.events = POLLIN;
    pfd.revents = 0;
    int poll_result = poll(&pfd, 1, 1);  // 1ms 超时，减少空轮询
    if (poll_result < 0) {
        return LEJU_ERROR_UNKNOWN;
    }
    if (poll_result == 0 || !(pfd.revents & POLLIN)) {
        return LEJU_ERROR_BUSTIMEOUT;
    }
    
    // 批量读取串口数据
    int bytes_avail_before = 0;
    ioctl(serial_fd, FIONREAD, &bytes_avail_before);
    
    char buffer[2048] = {0};
    int total_read = 0;
    
    // 循环读取直到串口缓存清空或达到上限
    while (bytes_avail_before > 0 && total_read < 1500) {
        int nbytes = read(serial_fd, buffer + total_read, sizeof(buffer) - total_read - 1);
        if (nbytes <= 0) break;
        
        total_read += nbytes;
        ioctl(serial_fd, FIONREAD, &bytes_avail_before);
    }
    
    if (total_read <= 0) {
        return LEJU_ERROR_BUSTIMEOUT;
    }
    
    buffer[total_read] = '\0';
    recv_buffer.append(buffer, total_read);
    
    #if DEBUG_ENABLE
        printf("\033[1;36m[%s][RX-FD][DBG] 批量读取: %d字节, recv_buffer: %zu字节\033[0m\n",
               can_channel.c_str(), total_read, recv_buffer.size());
    #endif
    
    // 多帧解析：循环解析所有完整帧
    size_t frames_parsed = 0;
    while (true) {
        size_t pos = recv_buffer.find('\r');
        if (pos == std::string::npos) {
            break;  // 无完整帧
        }
        
        std::string frame_str = recv_buffer.substr(0, pos);
        recv_buffer.erase(0, pos + 1);
        
        Recv_frame parsed_frame = SLCAN_to_Frame(frame_str);
        if (parsed_frame.id != 0 || !parsed_frame.data.empty()) {
            frame_queue.push_back(parsed_frame);
            frames_parsed++;
        }
    }
    
    // 防止缓冲区过长
    if (recv_buffer.length() > 2048) {
        recv_buffer.clear();
        printf("\033[1;31m[WARNING] 接收缓冲区过长，已清空\033[0m\n");
    }
    
    #if DEBUG_ENABLE
        if (frames_parsed > 0) {
            printf("\033[1;36m[%s][RX-FD][DBG] 批量解析完成: %zu帧\033[0m\n",
                   can_channel.c_str(), frames_parsed);
        }
    #endif
    
    // 返回解析结果
    return (frames_parsed > 0) ? LEJU_ERROR_OK : LEJU_ERROR_BUSTIMEOUT;
}


LEJU_StatusTypeDef CANable::Send(uint32_t id, const std::vector<uint8_t>& data) {
    if (!canfd_enabled || serial_fd < 0) {
        return LEJU_ERROR_INIT;
    }
    
    if (data.size() > 64) {
        return LEJU_ERROR_INVALID_PARAM;
    }
    
    std::string slcan_frame = Frame_to_SLCAN(id, data);
    if (slcan_frame.empty()) {
        return LEJU_ERROR_PARSE;
    }

    #if DEBUG_ENABLE
        printf("\033[1;34m[%s][TX-FD] 发送的原始帧: %s\n\033[0m", can_channel.c_str(), slcan_frame.c_str());
    #endif
    
    // 循环直至整帧写完，处理EAGAIN/EINTR并等待可写
    std::lock_guard<std::mutex> lock(send_mutex);
    const char* buf = slcan_frame.c_str();
    size_t total_len = slcan_frame.length();
    size_t sent = 0;

    while (sent < total_len) {
        ssize_t n = write(serial_fd, buf + sent, total_len - sent);
        if (n > 0) {
            sent += static_cast<size_t>(n);
            continue;
        }
        if (n == 0) {
            // 等待可写后重试
            struct pollfd pfd; pfd.fd = serial_fd; pfd.events = POLLOUT; pfd.revents = 0;
            (void)poll(&pfd, 1, 5);
            continue;
        }
        // n < 0
        if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
            struct pollfd pfd; pfd.fd = serial_fd; pfd.events = POLLOUT; pfd.revents = 0;
            (void)poll(&pfd, 1, 5);
            continue;
        }
        perror("[ERROR] CAN FD 发送失败");
        return LEJU_ERROR_SEND;
    }
    
    #if PRINTF_ENABLE
        printf("\033[1;34m[%s][TX-FD] 已发送 ID=0x%X LEN=%d Data=\033[0m", can_channel.c_str(), id, (int)data.size());
        for (const auto& byte : data) {
            printf("%02X ", byte);
        }
        printf("\n");
    #endif
 
    nanosleep_us(90);

    return LEJU_ERROR_OK;
}


void CANable::get_firmware_version() {
    if (serial_fd < 0) return;

    if (write(serial_fd, "V\r", 2) < 0) {
        perror("[ERROR] 发送版本查询命令失败");
        return;
    }

    usleep(50000);  // 等待响应
    
    char ver[128] = {0};  // 初始化为0
    ssize_t nbytes = read(serial_fd, ver, sizeof(ver) - 1);
    if (nbytes > 0) {
        ver[nbytes] = '\0';  // 确保字符串终止

        printf("\033[1;33m固件版本: %s\033[0m\n", ver);
    }  else {
        perror("[ERROR] 读取固件版本失败");
    }
}

// 获取CAN错误寄存器状态
uint8_t CANable::GetError() {
    if (serial_fd < 0) 
    {
        printf("[ERROR] CAN 未初始化\n");
        return 0xFF;
    }

    write(serial_fd, "E\r", 2);
    usleep(50000);
    
    char response[64] = {0};
    int nbytes = read(serial_fd, response, sizeof(response) - 1);
    
    if (nbytes > 0) {
        response[nbytes] = '\0';
        
        // 解析错误寄存器值 (格式: CANable Error Register: XX)
        uint32_t error_reg = 0;
        if (sscanf(response, "CANable Error Register: %X", &error_reg) == 1) {
            #if PRINTF_ENABLE
                printf("\033[1;31m[WARNING] CANable Error Register: %02X\033[0m\n", (uint8_t)error_reg);
            #endif
            
            return (uint8_t)error_reg;
        }else{
            printf("[ERROR] 解析错误寄存器值失败\n");
        }
    }
    
    return 0xFF;
}


std::string CANable::get_DevicePath(const std::string& channel_name) {
    if (channel_name == "canbus0") {
        printf("\033[1;33m选择[ 左手 ]CAN通道: can0，对应设备: /dev/canable1\033[0m\n");
        return "/dev/canable1";
    }
    if (channel_name == "canbus1") {
        printf("\033[1;33m选择[ 右手 ]CAN通道: can1，对应设备: /dev/canable2\033[0m\n");
        return "/dev/canable2";
    }

    if (channel_name.find("/dev/") == 0) {
        return channel_name;  // 直接传入设备路径
    }
    
    printf("[WARNING] 未识别的通道名称 '%s'，使用默认路径 /dev/canable1\n", channel_name.c_str());
    return "/dev/canable1";
}


void CANable::Close() {
    if (serial_fd >= 0) {
        // 关闭CAN通道
        std::string close_cmd = "C\r";
        write(serial_fd, close_cmd.c_str(), close_cmd.length());
        usleep(100000);
        
        // 恢复串口设置
        tcsetattr(serial_fd, TCSANOW, &tty_old);
        close(serial_fd);
        serial_fd = -1;
    }
    
    {
        std::lock_guard<std::mutex> lock(recv_mutex);
        recv_buffer.clear();
        frame_queue.clear();
    }
}

// 初始化CAN FD串口
bool CANable::CANfd_serial_init() {
    std::string device = get_DevicePath(can_channel);
    
    serial_fd = open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_fd < 0) {
        perror("打开串口设备失败");
        return false;
    }
    
    // 保存原始串口设置
    tcgetattr(serial_fd, &tty_old);
    
    struct termios tty_new;
    memset(&tty_new, 0, sizeof(tty_new));
    
    // 配置串口参数
    tty_new.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
    tty_new.c_iflag = IGNPAR;
    tty_new.c_oflag = 0;
    tty_new.c_lflag = 0;
    tty_new.c_cc[VTIME] = 0;  // 非阻塞
    tty_new.c_cc[VMIN] = 0;   // 非阻塞
    
    tcflush(serial_fd, TCIFLUSH);
    tcsetattr(serial_fd, TCSANOW, &tty_new);
    
    std::string cmd;
  
    // 关闭CAN通道（防止之前未正常关闭）
    cmd = "C\r";
    write(serial_fd, cmd.c_str(), cmd.length());
    usleep(50000);

    // 设置标准CAN波特率
    cmd = setBitrate(1000000);
    write(serial_fd, cmd.c_str(), cmd.length());
    usleep(50000);
    
    // 如果启用CAN FD，设置数据波特率
    cmd = setFDBitrate(2000000);
    write(serial_fd, cmd.c_str(), cmd.length());
    usleep(50000);
    
    // 打开CAN通道
    cmd = "O\r";
    write(serial_fd, cmd.c_str(), cmd.length());
    usleep(100000);
    
    printf("\033[1;33mLeju_CANFD初始化完成，设备路径: %s\033[0m\n", device.c_str());

    get_firmware_version();

    return true;
}


bool CANable::Init() {
    // 每次初始化都清理对应设备的旧 slcand 进程
    std::string device = get_DevicePath(can_channel);
    std::string cmd = "pkill -f 'slcand.*" + device + "' 2>/dev/null";
    system(cmd.c_str());
    usleep(50000);  // 等待50ms

    // 修改：如果启用CAN FD，使用串口直接通信
    if (canfd_enabled) {
        return CANfd_serial_init();
    }

    return true;
}

std::string CANable::setBitrate(uint32_t bitrate) {
    switch (bitrate) {
        case 10000:   return "S0\r";
        case 20000:   return "S1\r";
        case 50000:   return "S2\r";
        case 100000:  return "S3\r";
        case 125000:  return "S4\r";
        case 250000:  return "S5\r";
        case 500000:  return "S6\r";
        case 750000:  return "S7\r";
        case 800000:  return "S9\r";
        case 1000000: return "S8\r";
        default:
            printf("[WARNING] 不支持的波特率 %u，使用默认值 1000000\n", bitrate);
            return "S8\r";
    }
}

std::string CANable::setFDBitrate(uint32_t fd_bitrate) {
    switch (fd_bitrate) {
        case 2000000: return "Y2\r";
        case 5000000: return "Y5\r";
        default:
            printf("[WARNING] 不支持的CAN FD波特率 %u，使用默认值 2000000\n", fd_bitrate);
            return "Y2\r";
    }
}