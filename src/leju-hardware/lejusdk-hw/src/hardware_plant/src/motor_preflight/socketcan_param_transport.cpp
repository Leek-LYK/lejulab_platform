#include "motor_preflight/socketcan_param_transport.h"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cstdio>

namespace motor_preflight {

namespace {
// 只关心参数协议帧: CAN ID ∈ [0x600, 0x6FF]
constexpr canid_t kParamFilterId = 0x600;
constexpr canid_t kParamFilterMask = 0x700;
}  // namespace

SocketCanParamTransport::SocketCanParamTransport(int timeout_ms, int retries)
    : timeout_ms_(timeout_ms), retries_(retries) {}

SocketCanParamTransport::~SocketCanParamTransport() { Close(); }

bool SocketCanParamTransport::Open(const std::string& ifname, bool use_fd) {
    Close();

    fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd_ < 0) {
        std::fprintf(stderr, "[MotorPreflight] %s: 创建 CAN socket 失败: %s\n", ifname.c_str(),
                     strerror(errno));
        return false;
    }

    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, ifname.c_str(), IFNAMSIZ - 1);
    if (ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) {
        std::fprintf(stderr, "[MotorPreflight] CAN 接口 %s 不存在: %s\n", ifname.c_str(),
                     strerror(errno));
        Close();
        return false;
    }

    use_fd_ = use_fd;
    if (use_fd) {
        int canfd_on = 1;
        if (setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &canfd_on, sizeof(canfd_on)) != 0) {
            std::fprintf(stderr,
                         "[MotorPreflight] %s: 开启 CAN FD 失败, 回退经典 CAN: %s\n",
                         ifname.c_str(), strerror(errno));
            use_fd_ = false;
        }
    }

    // 不收自己发出的帧
    int recv_own = 0;
    setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &recv_own, sizeof(recv_own));

    // 只收参数协议响应帧
    struct can_filter filter;
    filter.can_id = kParamFilterId;
    filter.can_mask = kParamFilterMask;
    setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter));

    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(fd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        std::fprintf(stderr, "[MotorPreflight] %s: bind 失败: %s\n", ifname.c_str(),
                     strerror(errno));
        Close();
        return false;
    }

    return true;
}

void SocketCanParamTransport::Close() {
    if (fd_ >= 0) {
        close(fd_);
        fd_ = -1;
    }
}

bool SocketCanParamTransport::SendFrame(uint8_t motor_id, const std::array<uint8_t, 8>& req) {
    const canid_t can_id = kParamCanIdBase + motor_id;
    if (use_fd_) {
        struct canfd_frame frame;
        memset(&frame, 0, sizeof(frame));
        frame.can_id = can_id;
        frame.len = 8;
        frame.flags = CANFD_BRS;  // 对应 cansend '##1'
        memcpy(frame.data, req.data(), 8);
        return write(fd_, &frame, sizeof(frame)) == static_cast<ssize_t>(sizeof(frame));
    }
    struct can_frame frame;
    memset(&frame, 0, sizeof(frame));
    frame.can_id = can_id;
    frame.can_dlc = 8;
    memcpy(frame.data, req.data(), 8);
    return write(fd_, &frame, sizeof(frame)) == static_cast<ssize_t>(sizeof(frame));
}

void SocketCanParamTransport::Drain() {
    // can_frame 与 canfd_frame 的 can_id/len/data 字段偏移兼容, 统一按 canfd_frame 读
    struct canfd_frame frame;
    struct pollfd pfd = {fd_, POLLIN, 0};
    while (poll(&pfd, 1, 0) > 0) {
        if (read(fd_, &frame, sizeof(frame)) <= 0) {
            break;
        }
    }
}

bool SocketCanParamTransport::Transact(uint8_t motor_id, const std::array<uint8_t, 8>& req,
                                       std::array<uint8_t, 8>* resp) {
    if (fd_ < 0 || resp == nullptr) {
        return false;
    }
    const canid_t expect_id = kParamCanIdBase + motor_id;

    for (int attempt = 0; attempt <= retries_; ++attempt) {
        Drain();
        if (!SendFrame(motor_id, req)) {
            continue;
        }

        auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms_);
        while (true) {
            auto remain = std::chrono::duration_cast<std::chrono::milliseconds>(
                              deadline - std::chrono::steady_clock::now())
                              .count();
            if (remain <= 0) {
                break;  // 本次尝试超时
            }
            struct pollfd pfd = {fd_, POLLIN, 0};
            const int pret = poll(&pfd, 1, static_cast<int>(remain));
            if (pret <= 0) {
                break;
            }
            struct canfd_frame frame;
            const ssize_t n = read(fd_, &frame, sizeof(frame));
            if (n <= 0) {
                break;
            }
            // 响应: 同 CAN ID, 数据 8 字节, 首字节非 0x67 (非请求帧)
            if ((frame.can_id & CAN_EFF_MASK) != expect_id || frame.len < 8 ||
                frame.data[0] == 0x67) {
                continue;
            }
            memcpy(resp->data(), frame.data, 8);
            return true;
        }
    }
    return false;
}

bool SocketCanParamTransport::SendOnly(uint8_t motor_id, const std::array<uint8_t, 8>& req) {
    if (fd_ < 0) {
        return false;
    }
    return SendFrame(motor_id, req);
}

}  // namespace motor_preflight
