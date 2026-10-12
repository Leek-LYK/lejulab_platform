#include "socketcan_wrapper.h"
#include "canbus_sdk/result.h"
#include "canbus_sdk/canbus_log.h"
#include "canbus_sdk/canbus_sdk.h"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <cerrno>
#include <cstdlib>
#include <mutex>
#include <string>

#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <fcntl.h>
#include <poll.h>

namespace canbus_sdk {

// ===== 诊断: TX 丢帧统计日志 (所有 bus 共享同一文件) =====
// 默认编译关闭, 避免 TX 路径做同步(行缓冲)文件 IO; 开启: cmake -DLEJU_ENABLE_DIAG_FILELOG=ON
#ifdef LEJU_ENABLE_DIAG_FILELOG
namespace {
// 惰性初始化, 单次 "w" 打开 (覆盖旧内容); 失败时为 nullptr, 后续打印全部跳过.
FILE* getTxDropLogFile() {
    static std::once_flag init_flag;
    static FILE* logf = nullptr;
    std::call_once(init_flag, []() {
        logf = fopen("/tmp/socketcan_tx_drops.log", "w");
        if (logf) setvbuf(logf, nullptr, _IOLBF, 0);  // 行缓冲, 避免因缓冲丢失末尾
    });
    return logf;
}
std::mutex g_tx_drop_log_mu;  // 保护 fprintf (多 bus 并发)
} // namespace
#endif

// ===== Inner 结构体：封装 SocketCAN 文件描述符 =====
struct SocketCanWrapper::Inner {
    int socket_fd;       ///< CAN 原始套接字
    bool canfd_enabled;  ///< 是否启用了 CAN FD

    // ===== TX 统计 (诊断用, relaxed atomic, 每帧几 ns 开销) =====
    std::atomic<uint64_t> tx_total{0};     // 所有 write 尝试
    std::atomic<uint64_t> tx_eagain{0};    // EAGAIN/EWOULDBLOCK — SocketCAN TX 队列满
    std::atomic<uint64_t> tx_enobufs{0};   // ENOBUFS — 内核无 skb buffer
    std::atomic<uint64_t> tx_hw_err{0};    // 其他硬件失败
    std::atomic<int64_t>  last_stats_ns{0};   // 上次打印 stats 的 steady_clock ns

    Inner() : socket_fd(-1), canfd_enabled(false) {}

    void reset() {
        if (socket_fd >= 0) {
            close(socket_fd);
            socket_fd = -1;
        }
        canfd_enabled = false;
    }
};

// ===== 构造函数和析构函数 =====

SocketCanWrapper::SocketCanWrapper(const std::string& canbus_name,
                                   CanBusModelType model_type,
                                   const CanBusBitrate& bitrate)
    : CanBusBase(canbus_name, model_type, bitrate),
      inner_(new Inner()) {
    LOG_D("SocketCanWrapper constructor called for interface: %s", canbus_name.c_str());
}

SocketCanWrapper::~SocketCanWrapper() {
    LOG_D("Destroying SocketCanWrapper for interface: %s", canbus_name_.c_str());
    if (inner_) {
        inner_->reset();
    }
}

// ===== 初始化 =====

// CAN 接口由 init 脚本 (/etc/init.d/can_add_server.sh) 配置
// 程序不再执行 ip link set 命令, 避免破坏 bitrate/txqueuelen/IRQ affinity 等设置

Result<bool> SocketCanWrapper::init() {
    LOG_I("[SocketCAN] Initializing interface: %s", canbus_name_.c_str());

    // CAN 接口已由 init 脚本配置, 程序直接使用

    // 1. 创建 CAN 原始套接字
    inner_->socket_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (inner_->socket_fd < 0) {
        LOG_E("[SocketCAN] Failed to create socket: %s", strerror(errno));
        return Result<bool>::error(static_cast<ErrorType>(CanBusError::ERROR_HARDWARE_FAILURE));
    }

    // 2. 查找接口索引
    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, canbus_name_.c_str(), IFNAMSIZ - 1);
    if (ioctl(inner_->socket_fd, SIOCGIFINDEX, &ifr) < 0) {
        LOG_E("[SocketCAN] Interface '%s' not found: %s", canbus_name_.c_str(), strerror(errno));
        inner_->reset();
        return Result<bool>::error(static_cast<ErrorType>(CanBusError::ERROR_BUS_NOT_FOUND));
    }

    // 3. 仅在配置为 CAN FD 时启用 CAN_RAW_FD_FRAMES
    //    判断条件与 configureSocketCanInterface() 一致: dbitrate > 0 且 dbitrate != nbitrate
    bool use_fd = (bitrate_.dbitrate > 0 && bitrate_.dbitrate != bitrate_.nbitrate);
    if (use_fd) {
        int canfd_on = 1;
        if (setsockopt(inner_->socket_fd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES,
                       &canfd_on, sizeof(canfd_on)) == 0) {
            inner_->canfd_enabled = true;
            LOG_I("[SocketCAN] CAN FD enabled on %s", canbus_name_.c_str());
        } else {
            inner_->canfd_enabled = false;
            LOG_W("[SocketCAN] CAN FD requested but not available on %s, falling back to classic CAN",
                  canbus_name_.c_str());
        }
    } else {
        inner_->canfd_enabled = false;
        LOG_I("[SocketCAN] Classic CAN mode on %s (nbitrate=%d, dbitrate=%d)",
              canbus_name_.c_str(), bitrate_.nbitrate, bitrate_.dbitrate);
    }

    // 4. 禁止接收自己发出的帧 (避免广播命令帧 0x10/0x20 被自己收到)
    int recv_own = 0;
    setsockopt(inner_->socket_fd, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS,
               &recv_own, sizeof(recv_own));

    // 5. 增大 socket 发送缓冲区 (高频控制防止 "No buffer space available")
    int sndbuf = 1048576;  // 1MB
    setsockopt(inner_->socket_fd, SOL_SOCKET, SO_SNDBUF,
               &sndbuf, sizeof(sndbuf));

    // 6. 设置接收过滤器（接收所有帧）
    struct can_filter filter;
    filter.can_id = 0;
    filter.can_mask = 0;  // 匹配所有 ID
    setsockopt(inner_->socket_fd, SOL_CAN_RAW, CAN_RAW_FILTER,
               &filter, sizeof(filter));

    // 7. 设置 socket 为非阻塞模式
    //    写端非阻塞 — write() 立即返回，EAGAIN 时丢帧（电机控制下一周期覆盖）
    //    读端非阻塞 — recv 用 MSG_DONTWAIT 已实现，O_NONBLOCK 统一保证
    int flags = fcntl(inner_->socket_fd, F_GETFL, 0);
    if (flags < 0 || fcntl(inner_->socket_fd, F_SETFL, flags | O_NONBLOCK) < 0) {
        LOG_W("[SocketCAN] Failed to set non-blocking mode on %s: %s",
              canbus_name_.c_str(), strerror(errno));
    } else {
        LOG_I("[SocketCAN] Non-blocking mode enabled on %s", canbus_name_.c_str());
    }

    // 8. 绑定到 CAN 接口
    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(inner_->socket_fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        LOG_E("[SocketCAN] Failed to bind to %s: %s", canbus_name_.c_str(), strerror(errno));
        inner_->reset();
        return Result<bool>::error(static_cast<ErrorType>(CanBusError::ERROR_HARDWARE_FAILURE));
    }

    LOG_I("[SocketCAN] Interface %s initialized successfully (FD=%s)",
          canbus_name_.c_str(), inner_->canfd_enabled ? "yes" : "no");
    LOG_I("[SocketCAN] Bitrate - Nominal: %d kbps, Data: %d kbps",
          bitrate_.nbitrate, bitrate_.dbitrate);

    return Result<bool>::ok(true);
}

// ===== 发送消息 =====

Result<ErrorType> SocketCanWrapper::writeCanMessage(const CanMessageFrame& frame) {
    if (!inner_ || inner_->socket_fd < 0) {
        LOG_E("[SocketCAN] Socket not initialized");
        return Result<ErrorType>::error(static_cast<ErrorType>(CanBusError::ERROR_UNINITIALIZED));
    }

    // ===== 诊断计数: 总 write 尝试 (relaxed, 几 ns 开销) =====
    inner_->tx_total.fetch_add(1, std::memory_order_relaxed);

    // ===== 诊断: 每 2 秒打印一次 TX 统计 (函数尾部和提前返回点都调用) =====
    // 默认编译关闭, 调用点保留为空操作 (原子计数器仍累加, 几 ns 开销).
    auto maybe_print_stats = [this]() {
#ifdef LEJU_ENABLE_DIAG_FILELOG
        int64_t now_ns = std::chrono::steady_clock::now().time_since_epoch().count();
        int64_t last = inner_->last_stats_ns.load(std::memory_order_relaxed);
        if (now_ns - last < 2000000000LL) return;
        if (!inner_->last_stats_ns.compare_exchange_strong(last, now_ns,
                                                            std::memory_order_relaxed)) {
            return;
        }
        uint64_t total   = inner_->tx_total.load(std::memory_order_relaxed);
        uint64_t eagain  = inner_->tx_eagain.load(std::memory_order_relaxed);
        uint64_t enobufs = inner_->tx_enobufs.load(std::memory_order_relaxed);
        uint64_t hw_err  = inner_->tx_hw_err.load(std::memory_order_relaxed);
        uint64_t drops = eagain + enobufs + hw_err;
        double drop_pct = total > 0
                          ? 100.0 * static_cast<double>(drops) / static_cast<double>(total)
                          : 0.0;

        if (FILE* logf = getTxDropLogFile()) {
            std::lock_guard<std::mutex> lk(g_tx_drop_log_mu);
            fprintf(logf,
                    "[%-6s] total=%-10lu eagain=%-8lu enobufs=%-8lu hw_err=%-6lu drop=%.4f%%\n",
                    canbus_name_.c_str(),
                    (unsigned long)total,
                    (unsigned long)eagain,
                    (unsigned long)enobufs,
                    (unsigned long)hw_err,
                    drop_pct);
        }
#else
        (void)this;
#endif
    };

    uint32_t can_id = frame.id.SID;
    if (frame.id.EID) {
        can_id |= (frame.id.EID << 11) | CAN_EFF_FLAG;
    }

    uint8_t payload_length = dlc_to_payload_length(frame.ctrl.tx.dlc);
    ssize_t nbytes;

    if (inner_->canfd_enabled && payload_length > CAN_MAX_DLEN) {
        // CAN FD 帧
        struct canfd_frame fd_frame;
        memset(&fd_frame, 0, sizeof(fd_frame));
        fd_frame.can_id = can_id;
        fd_frame.len = payload_length;
        fd_frame.flags = CANFD_BRS;  // Bit Rate Switch
        memcpy(fd_frame.data, frame.payload,
               payload_length < CANFD_MAX_DLEN ? payload_length : CANFD_MAX_DLEN);

        nbytes = write(inner_->socket_fd, &fd_frame, sizeof(fd_frame));
        if (nbytes != sizeof(fd_frame)) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                inner_->tx_eagain.fetch_add(1, std::memory_order_relaxed);
                maybe_print_stats();
                return Result<ErrorType>::ok(static_cast<ErrorType>(CanBusError::SUCCESS));
            }
            if (errno == ENOBUFS) {
                inner_->tx_enobufs.fetch_add(1, std::memory_order_relaxed);
                maybe_print_stats();
                return Result<ErrorType>::ok(static_cast<ErrorType>(CanBusError::SUCCESS));
            }
            inner_->tx_hw_err.fetch_add(1, std::memory_order_relaxed);
            LOG_W("[SocketCAN] Send CAN FD frame failed: %s", strerror(errno));
            return Result<ErrorType>::error(static_cast<ErrorType>(CanBusError::ERROR_HARDWARE_FAILURE));
        }
    } else {
        // payload <= 8 字节: 经典 CAN 帧
        struct can_frame classic_frame;
        memset(&classic_frame, 0, sizeof(classic_frame));
        classic_frame.can_id = can_id;
        classic_frame.can_dlc = payload_length < CAN_MAX_DLEN ? payload_length : CAN_MAX_DLEN;
        memcpy(classic_frame.data, frame.payload, classic_frame.can_dlc);

        nbytes = write(inner_->socket_fd, &classic_frame, sizeof(classic_frame));
        if (nbytes != sizeof(classic_frame)) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                inner_->tx_eagain.fetch_add(1, std::memory_order_relaxed);
                maybe_print_stats();
                return Result<ErrorType>::ok(static_cast<ErrorType>(CanBusError::SUCCESS));
            }
            if (errno == ENOBUFS) {
                inner_->tx_enobufs.fetch_add(1, std::memory_order_relaxed);
                maybe_print_stats();
                return Result<ErrorType>::ok(static_cast<ErrorType>(CanBusError::SUCCESS));
            }
            inner_->tx_hw_err.fetch_add(1, std::memory_order_relaxed);
            LOG_W("[SocketCAN] Send classic frame failed: %s", strerror(errno));
            return Result<ErrorType>::error(static_cast<ErrorType>(CanBusError::ERROR_HARDWARE_FAILURE));
        }
    }

    // 触发 TEF 回调（发送完成通知）
    CanMessageFrame* tef_frame = createCanMessageFrame();
    if (tef_frame) {
        *tef_frame = frame;
        if (tef_callback_) {
            tef_callback_(tef_frame, tef_context_);
        } else {
            freeCanMessageFrame(tef_frame);
        }
    }

    maybe_print_stats();
    return Result<ErrorType>::ok(static_cast<ErrorType>(CanBusError::SUCCESS));
}

// ===== 设置回调函数 =====

void SocketCanWrapper::setMessageReceiveCallback(MessageCallback callback,
                                                  const CallbackContext* context) {
    callback_ = callback;
    context_ = context;
    LOG_D("[SocketCAN] Message receive callback set");
}

void SocketCanWrapper::setTefEventCallback(TefEeventCallback callback,
                                           const CallbackContext* context) {
    tef_callback_ = callback;
    tef_context_ = context;
    LOG_D("[SocketCAN] TEF event callback set");
}

// ===== 接收消息 =====

Result<ErrorType> SocketCanWrapper::receivedCanMessages() {
    if (!inner_ || inner_->socket_fd < 0) {
        LOG_E("[SocketCAN] Socket not initialized");
        return Result<ErrorType>::error(static_cast<ErrorType>(CanBusError::ERROR_UNINITIALIZED));
    }

    int message_count = 0;
    constexpr int MAX_MESSAGES_PER_POLL = 100;

    for (int i = 0; i < MAX_MESSAGES_PER_POLL; i++) {
        // 使用 canfd_frame 接收（兼容经典 CAN 和 CAN FD）
        struct canfd_frame fd_frame;
        ssize_t nbytes = recv(inner_->socket_fd, &fd_frame, sizeof(fd_frame), MSG_DONTWAIT);

        if (nbytes < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                break;  // 没有更多消息
            }
            LOG_E("[SocketCAN] Read error: %s", strerror(errno));
            return Result<ErrorType>::error(static_cast<ErrorType>(CanBusError::ERROR_HARDWARE_FAILURE));
        }

        if (nbytes < (ssize_t)sizeof(struct can_frame)) {
            LOG_W("[SocketCAN] Incomplete frame received (%zd bytes)", nbytes);
            continue;
        }

        message_count++;

        // 转换为 SDK 格式
        CanMessageFrame* frame = createCanMessageFrame();
        if (!frame) {
            LOG_E("[SocketCAN] Failed to allocate CanMessageFrame");
            continue;
        }

        // 提取 CAN ID
        uint32_t raw_id = fd_frame.can_id;
        if (raw_id & CAN_EFF_FLAG) {
            // 扩展帧（29 位 ID）
            uint32_t full_id = raw_id & CAN_EFF_MASK;
            frame->id.SID = full_id & 0x7FF;
            frame->id.EID = (full_id >> 11) & 0x3FFFF;
        } else {
            // 标准帧（11 位 ID）
            frame->id.SID = raw_id & CAN_SFF_MASK;
            frame->id.EID = 0;
        }
        frame->id.SID11 = 0;
        frame->id.unimplemented1 = 0;

        // 设置 DLC
        uint8_t dlc_code = payload_length_to_dlc(fd_frame.len);
        frame->ctrl.rx.dlc = dlc_code;

        // 复制 payload
        size_t copy_len = fd_frame.len < sizeof(frame->payload) ? fd_frame.len : sizeof(frame->payload);
        memcpy(frame->payload, fd_frame.data, copy_len);
        if (copy_len < sizeof(frame->payload)) {
            memset(&frame->payload[copy_len], 0, sizeof(frame->payload) - copy_len);
        }

        // 调用回调
        if (callback_) {
            callback_(frame, context_);
        } else {
            freeCanMessageFrame(frame);
        }
    }

    return Result<ErrorType>::ok(static_cast<ErrorType>(message_count));
}

} // namespace canbus_sdk
