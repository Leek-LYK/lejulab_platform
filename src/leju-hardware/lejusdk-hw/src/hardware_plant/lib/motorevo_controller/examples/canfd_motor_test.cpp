/**
 * CAN FD 电机控制测试程序
 *
 * 支持正弦波跟踪、阶跃响应、正弦扫频和力矩扫频四种测试模式。
 * 支持多 CAN 总线同时测试。
 *
 * 用法:
 *   sudo ./canfd_motor_test [选项]
 *
 * 选项:
 *   --can <接口名>    CAN 接口，逗号分隔多总线 (默认 bcan0，例: bcan0,bcan1)
 *   --motors <数量>   每条总线电机数量，逗号分隔 (默认 7，例: 7,6)
 *   --kp <值>         kp (默认 150)
 *   --kd <值>         kd (默认 2)
 *   --amp <度>        振幅/阶跃幅度 (默认 15)
 *   --freq <Hz>       正弦频率 (默认 0.5, 仅 sine 模式)
 *   --mode <sine|step|chirp|torque-chirp|torque-step> 测试模式 (默认 sine)
 *   --torque-amp <Nm>  力矩幅值 (默认 1.0, 仅 torque-chirp/torque-step)
 *   --pos-limit <rad>  位置安全限制 (默认 0.5, 仅 torque-chirp/torque-step)
 *   --step-hold <秒>  阶跃保持时间 (默认 3)
 *   --steps <次数>    阶跃重复次数 (默认 5)
 *   --duration <秒>   测试总时长 (默认 100, 仅 sine 模式)
 *   --inertia <J>     反映惯量 kg·m² (默认 0, 启用加速度前馈)
 *   --hz <频率>       控制频率 (默认 500)
 *   --log <file.csv>  CSV 日志 (也可用 --csv)
 *   --cali            校准模式 (仅校准零点，不做控制测试)
 *   --ratio <值>      减速比 (默认 36)
 *   --max-vel <rad/s> 输出轴最大速度 (默认 15)
 *   --rated-vel <rad/s> 输出轴额定速度 (默认 9)
 */

#include "motorevo/motor_ctrl.h"
#include "motorevo/motor_def.h"
#include "canbus_sdk/canbus_sdk.h"

#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <thread>
#include <chrono>
#include <cmath>
#include <sstream>
#include <iomanip>
#include <atomic>
#include <csignal>
#include <cstdlib>
#include <string>
#include <algorithm>
#include <numeric>
#include <memory>
#include <time.h>

// ---- 工具函数 ----
std::vector<std::string> splitComma(const std::string& s) {
    std::vector<std::string> parts;
    std::istringstream iss(s);
    std::string token;
    while (std::getline(iss, token, ','))
        if (!token.empty()) parts.push_back(token);
    return parts;
}

// ---- 每条总线的信息 ----
struct BusInfo {
    std::string can_name;
    int num_motors;
    std::shared_ptr<motorevo::RevoMotorControl> motor_control;
    std::vector<motorevo::MotorId> motor_ids;
    std::map<motorevo::MotorId, double> zero_positions;
    std::map<motorevo::MotorId, double> last_act_positions;
    std::map<motorevo::MotorId, int> stale_counter;
};

class ControlLoop {
public:
    ControlLoop(std::vector<std::shared_ptr<motorevo::RevoMotorControl>> mcs, int freq_hz)
        : mcs_(mcs), running_(true), period_ns_(1000000000L / freq_hz),
          thread_([this]() {
              struct timespec next_time;
              clock_gettime(CLOCK_MONOTONIC, &next_time);
              while (running_) {
                  next_time.tv_nsec += period_ns_;
                  if (next_time.tv_nsec >= 1000000000L) {
                      next_time.tv_sec++;
                      next_time.tv_nsec -= 1000000000L;
                  }
                  for (auto& mc : mcs_) mc->write();
                  clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);
              }
          }) {}
    ~ControlLoop() { running_ = false; if (thread_.joinable()) thread_.join(); }
    void stop() { running_ = false; }
private:
    std::vector<std::shared_ptr<motorevo::RevoMotorControl>> mcs_;
    std::atomic<bool> running_;
    long period_ns_;
    std::thread thread_;
};

std::atomic<bool> g_running(true);
ControlLoop* g_control_loop = nullptr;

void signalHandler(int) {
    g_running = false;
    if (g_control_loop) g_control_loop->stop();
}

// ---- 阶跃响应指标 ----
struct StepMetrics {
    double rise_time_ms;      // 10% -> 90% 上升时间
    double overshoot_pct;     // 超调量 %
    double settling_time_ms;  // ±2% 稳态误差带
    double steady_state_err_deg; // 稳态误差
};

StepMetrics computeStepMetrics(const std::vector<double>& times,
                                const std::vector<double>& positions,
                                double start_pos, double target_pos) {
    StepMetrics m = {0, 0, 0, 0};
    if (times.empty()) return m;

    double step_size = target_pos - start_pos;
    if (std::fabs(step_size) < 1e-6) return m;

    double pos_10 = start_pos + 0.1 * step_size;
    double pos_90 = start_pos + 0.9 * step_size;
    double t_10 = -1, t_90 = -1;
    double peak_overshoot = 0;
    double settle_band = std::fabs(step_size) * 0.02;  // ±2%
    double t_settle = 0;

    for (size_t i = 0; i < times.size(); ++i) {
        double pos = positions[i];
        double normalized = (pos - start_pos) / step_size;  // 0->1

        // 上升时间
        if (t_10 < 0) {
            if ((step_size > 0 && pos >= pos_10) || (step_size < 0 && pos <= pos_10))
                t_10 = times[i];
        }
        if (t_90 < 0) {
            if ((step_size > 0 && pos >= pos_90) || (step_size < 0 && pos <= pos_90))
                t_90 = times[i];
        }

        // 超调
        double overshoot = (normalized - 1.0);
        if (overshoot > peak_overshoot) peak_overshoot = overshoot;

        // 稳定时间: 最后一次离开 ±2% 稳态带的时刻
        if (std::fabs(pos - target_pos) > settle_band) {
            t_settle = times[i];
        }
    }

    m.rise_time_ms = (t_10 >= 0 && t_90 >= 0) ? (t_90 - t_10) * 1000 : -1;
    m.overshoot_pct = peak_overshoot * 100;
    m.settling_time_ms = t_settle * 1000;

    // 稳态误差: 最后 20% 数据的平均
    int tail_start = static_cast<int>(times.size() * 0.8);
    double sum = 0;
    int count = 0;
    for (size_t i = tail_start; i < times.size(); ++i) {
        sum += (positions[i] - target_pos);
        count++;
    }
    m.steady_state_err_deg = (count > 0) ? (sum / count) * 180.0 / M_PI : 0;

    return m;
}

int main(int argc, char* argv[]) {
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    // ---- 参数 ----
    std::string can_arg = "bcan0";
    std::string motors_arg = "7";
    std::string log_file;
    std::string mode = "sine";
    double param_kp = 150.0;
    double param_kd = 2.0;
    double amplitude_degrees = 15.0;
    double frequency = 0.5;
    double duration = 100.0;
    double step_hold = 3.0;
    int step_count = 5;
    int ctrl_freq = 500;
    long ctrl_period_ns = 1000000000L / ctrl_freq;  // 在解析参数后更新
    double inertia_J = 0.0;   // 反映惯量 (kg·m²), >0 时启用加速度前馈
    bool cali_mode = false;
    // chirp 模式参数
    double chirp_f_start = 0.1;
    double chirp_f_end = 10.0;
    int chirp_steps = 15;
    int chirp_cycles = 5;  // 每个频率的周期数
    // 电机本体参数
    double motor_ratio = 36.0;       // 减速比
    double motor_c2t = 1.14;         // 电流-力矩常数 (N·m/A)
    double motor_max_vel = 15.0;     // 输出轴最大速度 (rad/s)
    double motor_rated_vel = 9.0;    // 输出轴额定速度 (rad/s)
    // torque-chirp 模式参数
    double torque_amp = 1.0;           // 力矩幅值 (Nm)
    double torque_pos_limit = 0.5;     // 位置安全限制 (rad), ~28.6°
    std::string ids_arg;               // --id 指定电机 ID 列表 (如 "4" 或 "3,5")
    std::string torque_steps_arg;      // --torque-steps 自定义力矩序列 (如 "0,1,2,0")

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--can" && i+1 < argc) can_arg = argv[++i];
        else if ((arg == "--log" || arg == "--csv") && i+1 < argc) log_file = argv[++i];
        else if (arg == "--kp" && i+1 < argc) param_kp = std::stod(argv[++i]);
        else if (arg == "--kd" && i+1 < argc) param_kd = std::stod(argv[++i]);
        else if (arg == "--amp" && i+1 < argc) amplitude_degrees = std::stod(argv[++i]);
        else if (arg == "--freq" && i+1 < argc) frequency = std::stod(argv[++i]);
        else if (arg == "--mode" && i+1 < argc) mode = argv[++i];
        else if (arg == "--step-hold" && i+1 < argc) step_hold = std::stod(argv[++i]);
        else if (arg == "--steps" && i+1 < argc) step_count = std::stoi(argv[++i]);
        else if (arg == "--duration" && i+1 < argc) duration = std::stod(argv[++i]);
        else if (arg == "--motors" && i+1 < argc) motors_arg = argv[++i];
        else if (arg == "--hz" && i+1 < argc) ctrl_freq = std::stoi(argv[++i]);
        else if (arg == "--f-start" && i+1 < argc) chirp_f_start = std::stod(argv[++i]);
        else if (arg == "--f-end" && i+1 < argc) chirp_f_end = std::stod(argv[++i]);
        else if (arg == "--f-steps" && i+1 < argc) chirp_steps = std::stoi(argv[++i]);
        else if (arg == "--cycles" && i+1 < argc) chirp_cycles = std::stoi(argv[++i]);
        else if (arg == "--inertia" && i+1 < argc) inertia_J = std::stod(argv[++i]);
        else if (arg == "--cali") cali_mode = true;
        else if (arg == "--ratio" && i+1 < argc) motor_ratio = std::stod(argv[++i]);
        else if (arg == "--max-vel" && i+1 < argc) motor_max_vel = std::stod(argv[++i]);
        else if (arg == "--rated-vel" && i+1 < argc) motor_rated_vel = std::stod(argv[++i]);
        else if (arg == "--torque-amp" && i+1 < argc) torque_amp = std::stod(argv[++i]);
        else if (arg == "--pos-limit" && i+1 < argc) torque_pos_limit = std::stod(argv[++i]);
        else if (arg == "--id" && i+1 < argc) ids_arg = argv[++i];
        else if (arg == "--torque-steps" && i+1 < argc) torque_steps_arg = argv[++i];
    }

    ctrl_period_ns = 1000000000L / ctrl_freq;  // 更新（可能被 --hz 修改）

    // ---- 解析多总线参数 ----
    std::vector<std::string> can_names = splitComma(can_arg);
    std::vector<int> motor_counts;
    {
        auto strs = splitComma(motors_arg);
        for (auto& s : strs) motor_counts.push_back(std::stoi(s));
    }
    if (motor_counts.empty()) motor_counts.push_back(7);
    while (motor_counts.size() < can_names.size())
        motor_counts.push_back(motor_counts.back());

    const double amplitude = amplitude_degrees * M_PI / 180.0;
    const bool multi_bus = can_names.size() > 1;

    int total_motors = 0;
    for (size_t i = 0; i < can_names.size(); ++i) total_motors += motor_counts[i];

    // ---- 打印配置 ----
    std::cout << "\033[32m=== CAN FD 电机控制测试 ===\033[0m\n"
              << "模式: " << mode << ", 总线: ";
    for (size_t i = 0; i < can_names.size(); ++i) {
        if (i > 0) std::cout << ", ";
        std::cout << can_names[i] << "(" << motor_counts[i] << "电机)";
    }
    std::cout << ", 总电机数: " << total_motors
              << ", 控制频率: " << ctrl_freq << " Hz"
              << ", kp=" << param_kp << ", kd=" << param_kd << "\n";
    if (mode == "step") {
        std::cout << "阶跃幅度: " << amplitude_degrees << "°, 保持: " << step_hold
                  << "s, 重复: " << step_count << " 次\n";
    } else if (mode == "chirp") {
        std::cout << "振幅: " << amplitude_degrees << "°, 频率: "
                  << chirp_f_start << " -> " << chirp_f_end << " Hz (" << chirp_steps << " 步)"
                  << ", 每频率 " << chirp_cycles << " 个周期\n";
    } else if (mode == "torque-chirp") {
        std::cout << "力矩幅值: " << torque_amp << " Nm, 位置安全限制: "
                  << torque_pos_limit << " rad (" << torque_pos_limit * 180/M_PI << "°)\n";
        std::cout << "频率: " << chirp_f_start << " -> " << chirp_f_end
                  << " Hz (" << chirp_steps << " 步), 每频率 " << chirp_cycles << " 个周期\n";
    } else if (mode == "torque-step") {
        std::cout << "力矩幅值: " << torque_amp << " Nm, 位置安全限制: "
                  << torque_pos_limit << " rad (" << torque_pos_limit * 180/M_PI << "°)\n";
        std::cout << "保持: " << step_hold << "s, 重复: " << step_count << " 次 (±交替)\n";
    } else {
        std::cout << "振幅: " << amplitude_degrees << "°, 频率: " << frequency << " Hz\n";
    }
    if (inertia_J > 0) {
        std::cout << "\033[33m加速度前馈: J=" << inertia_J << " kg·m²\033[0m\n";
    }

    // ---- 电机参数与速度安全 ----
    // 正弦运动 v_max = amplitude * 2πf
    // f_rated = motor_rated_vel / (amplitude * 2π)
    // f_max   = motor_max_vel   / (amplitude * 2π)
    double f_rated = motor_rated_vel / (amplitude * 2.0 * M_PI);
    double f_max   = motor_max_vel   / (amplitude * 2.0 * M_PI);
    std::cout << "电机参数: 减速比=" << motor_ratio
              << ", 最大速度=" << motor_max_vel << " rad/s"
              << ", 额定速度=" << motor_rated_vel << " rad/s\n";
    std::cout << "速度限制: 振幅=" << amplitude_degrees << "° 时,"
              << " 额定频率上限=" << std::fixed << std::setprecision(2) << f_rated << " Hz"
              << ", 最大频率上限=" << std::setprecision(2) << f_max << " Hz\n";

    // KD 范围警告 (CAN FD 协议 KD 最大值为 5.0)
    if (param_kd > 5.0) {
        std::cout << "\033[31m警告: KD=" << param_kd << " 超过 CAN FD 协议上限 5.0，将被硬件钳位到 5.0\033[0m\n";
    }

    // torque-chirp 提示: 力矩范围 [-30, 30] Nm
    if (mode == "torque-chirp" && std::fabs(torque_amp) > 30.0) {
        std::cout << "\033[31m警告: 力矩幅值 " << torque_amp << " Nm 超过 CANFD 协议上限 ±30 Nm，将被硬件钳位\033[0m\n";
        torque_amp = std::min(torque_amp, 30.0);
    }

    // chirp 模式自动钳位频率上限
    if (mode == "chirp" && chirp_f_end > f_max) {
        std::cout << "\033[33m警告: chirp 频率上限 " << std::setprecision(2) << chirp_f_end
                  << " Hz 超过电机最大速度限制 " << f_max << " Hz，已自动钳位\033[0m\n";
        chirp_f_end = f_max;
    }
    // sine 模式频率超限警告
    if (mode == "sine" && frequency > f_max) {
        std::cout << "\033[31m警告: 正弦频率 " << frequency << " Hz 超过电机最大速度限制 "
                  << std::setprecision(2) << f_max << " Hz！电机可能过载\033[0m\n";
    } else if (mode == "sine" && frequency > f_rated) {
        std::cout << "\033[33m注意: 正弦频率 " << frequency << " Hz 超过额定速度限制 "
                  << std::setprecision(2) << f_rated << " Hz (仍在最大速度范围内)\033[0m\n";
    }

    std::cout << "按 Ctrl+C 退出\n" << std::endl;

    // ---- 初始化 CAN ----
    auto& canbus_controller = canbus_sdk::CanBusController::getInstance();
    canbus_controller.init();

    canbus_sdk::CanBusBitrate bitrate = {
        .nbitrate = 1000000, .dbitrate = 5000000,
        .nsamplepos = 80, .dsamplepos = 75
    };

    for (auto& name : can_names) {
        auto result = canbus_controller.openCanBus(name, canbus_sdk::CanBusModelType::SOCKETCAN, bitrate);
        if (!result.has_value()) {
            std::cerr << "\033[31m" << name << " CAN 总线打开失败\033[0m\n";
            return -1;
        }
    }

    // ---- 创建各总线的电机控制 ----
    std::vector<BusInfo> buses;
    for (size_t bi = 0; bi < can_names.size(); ++bi) {
        BusInfo bus;
        bus.can_name = can_names[bi];
        bus.num_motors = motor_counts[bi];

        if (!ids_arg.empty() && bi == 0) {
            // --id 指定的电机 ID 列表 (仅对第一条总线生效)
            auto id_strs = splitComma(ids_arg);
            for (auto& s : id_strs)
                bus.motor_ids.push_back(static_cast<motorevo::MotorId>(std::stoi(s)));
            bus.num_motors = static_cast<int>(bus.motor_ids.size());
        } else {
            for (int m = 1; m <= bus.num_motors; ++m)
                bus.motor_ids.push_back(static_cast<motorevo::MotorId>(m));
        }

        bus.motor_control = std::make_shared<motorevo::RevoMotorControl>(
            bus.can_name, motorevo::MotorProtocol::CANFD_BROADCAST);

        buses.push_back(std::move(bus));
    }

    // 创建控制循环 (在 init 之前启动，与原单总线行为一致)
    std::vector<std::shared_ptr<motorevo::RevoMotorControl>> all_mcs;
    for (auto& bus : buses) all_mcs.push_back(bus.motor_control);
    g_control_loop = new ControlLoop(all_mcs, ctrl_freq);

    // 初始化各总线电机
    for (size_t bi = 0; bi < buses.size(); ++bi) {
        auto& bus = buses[bi];

        std::vector<motorevo::RevoMotorConfig_t> motor_configs;
        for (auto mid : bus.motor_ids) {
            std::ostringstream ns;
            ns << "motor_" << static_cast<int>(mid);
            motor_configs.push_back({
                .id = mid,
                .name = ns.str(),
                .negtive = false, .ignore = false, .zero_offset = 0.0f,
                .default_params = {
                    .vel = 0, .kp_pos = static_cast<float>(param_kp),
                    .kd_pos = static_cast<float>(param_kd),
                    .tor = 0, .kp_vel = 0, .kd_vel = 0, .ki_vel = 0
                }
            });
        }

        if (!bus.motor_control->init(motor_configs, cali_mode, true)) {
            std::cerr << "\033[31m" << bus.can_name << " 电机初始化失败\033[0m\n";
            delete g_control_loop; g_control_loop = nullptr;
            for (auto& b : buses) b.motor_control.reset();
            return -1;
        }
    }

    // ---- 校准模式 ----
    if (cali_mode) {
        // init() 标定模式下不自动标定，需显式调用 calibrateAllMotorsAndHold()
        for (auto& bus : buses) {
            if (!bus.motor_control->calibrateAllMotorsAndHold()) {
                std::cerr << "\033[31m" << bus.can_name << " 标定失败\033[0m\n";
            }
        }
        std::cout << "\033[32m校准完成\033[0m\n";
        for (auto& bus : buses) {
            if (multi_bus) std::cout << "[" << bus.can_name << "]:\n";
            auto pos = bus.motor_control->getPositions();
            auto offsets = bus.motor_control->getZeroOffsets();
            for (auto& [id, p] : pos) {
                float offset = offsets.count(id) ? offsets[id] : 0.0f;
                std::cout << "  电机 0x" << std::hex << static_cast<int>(id) << std::dec
                          << ": 位置=" << std::fixed << std::setprecision(4) << p << " rad"
                          << ", zero_offset=" << offset << " rad\n";
            }
        }
        delete g_control_loop; g_control_loop = nullptr;
        for (auto& bus : buses) {
            bus.motor_control->disableAll();
            bus.motor_control.reset();
        }
        canbus_controller.closeAllCanBuses();
        std::cout << "\033[32m校准测试完成\033[0m\n";
        return 0;
    }

    std::cout << "\033[32m电机初始化成功 (" << total_motors << " 个电机, kp="
              << param_kp << ", kd=" << param_kd << ")\033[0m\n";

    // ---- 读零位 ----
    {
        std::cout << "当前位置:\n";
        for (auto& bus : buses) {
            if (multi_bus) std::cout << "[" << bus.can_name << "]:\n";
            auto pos = bus.motor_control->getPositions();
            for (auto mid : bus.motor_ids) {
                double p = pos.count(mid) ? static_cast<double>(pos[mid]) : 0.0;
                std::cout << "  M" << static_cast<int>(mid) << ": "
                          << std::fixed << std::setprecision(4) << p << " rad ("
                          << std::setprecision(1) << p * 180.0 / M_PI << "°)\n";
            }
        }
    }

    for (auto& bus : buses) {
        auto pos = bus.motor_control->getPositions();
        for (auto mid : bus.motor_ids)
            bus.zero_positions[mid] = pos.count(mid) ? static_cast<double>(pos[mid]) : 0.0;
    }

    // 初始化通信丢失检测
    for (auto& bus : buses) {
        for (auto mid : bus.motor_ids) {
            bus.last_act_positions[mid] = 0.0;
            bus.stale_counter[mid] = 0;
        }
    }

    // ---- CSV ----
    std::ofstream csv_log;
    if (!log_file.empty()) {
        csv_log.open(log_file);
        if (csv_log.is_open()) {
            csv_log << "time_s";
            for (size_t bi = 0; bi < buses.size(); ++bi) {
                for (auto mid : buses[bi].motor_ids) {
                    std::string prefix = multi_bus ?
                        "B" + std::to_string(bi) + "M" + std::to_string(static_cast<int>(mid)) :
                        "M" + std::to_string(static_cast<int>(mid));
                    if (mode == "torque-chirp" || mode == "torque-step") {
                        csv_log << "," << prefix << "_torque_cmd"
                                << "," << prefix << "_act_rad"
                                << "," << prefix << "_vel"
                                << "," << prefix << "_act_torque"
                                << "," << prefix << "_fault";
                    } else {
                        csv_log << "," << prefix << "_tgt_rad"
                                << "," << prefix << "_act_rad"
                                << "," << prefix << "_err_deg"
                                << "," << prefix << "_vel"
                                << "," << prefix << "_tor"
                                << "," << prefix << "_fault";
                    }
                }
            }
            csv_log << "\n";
            std::cout << "\033[32mCSV: " << log_file << "\033[0m\n";
        }
    }

    // ================================================================
    //  SINE 模式
    // ================================================================
    if (mode == "sine") {
        std::cout << "\n\033[36m开始正弦波测试 (" << total_motors << " 个电机)...\033[0m\n";

        auto start_time = std::chrono::steady_clock::now();
        int total_iters = static_cast<int>(duration * ctrl_freq);

        // CSV 门控: 检测反馈开始变化后重置正弦起点并开始记录
        bool csv_active = false;
        // 用每条总线第一个电机的初始 act 判断反馈是否活了
        std::vector<double> feedback_init(buses.size());
        for (size_t bi = 0; bi < buses.size(); ++bi) {
            auto pos = buses[bi].motor_control->getPositions();
            auto mid0 = buses[bi].motor_ids[0];
            feedback_init[bi] = pos.count(mid0) ? static_cast<double>(pos[mid0]) : 0.0;
        }

        struct timespec sine_ts;
        clock_gettime(CLOCK_MONOTONIC, &sine_ts);

        for (int i = 0; i < total_iters && g_running; ++i) {
            auto now = std::chrono::steady_clock::now();
            double t = std::chrono::duration<double>(now - start_time).count();

            double omega = 2 * M_PI * frequency;
            double sine_val = amplitude * std::sin(omega * t);
            double vel_ff = amplitude * omega * std::cos(omega * t);
            // 钳位速度前馈到电机最大速度
            vel_ff = std::max(-motor_max_vel, std::min(motor_max_vel, vel_ff));
            // 加速度前馈: τ_ff = J * θ̈ = -J * A * ω² * sin(ωt)
            double torque_ff = -inertia_J * amplitude * omega * omega * std::sin(omega * t);

            // 发送命令到所有总线
            for (auto& bus : buses) {
                std::map<motorevo::MotorId, motorevo::RevoMotorCmd_t> cmd;
                for (auto mid : bus.motor_ids) {
                    double target = bus.zero_positions[mid] + sine_val;
                    cmd[mid] = {.pos = target, .vel = vel_ff, .torque = torque_ff, .kp = param_kp, .kd = param_kd};
                }
                bus.motor_control->setTargetPositions(cmd);
            }

            // 检测反馈是否活了 (每条总线首个电机的 act 与初始值不同)
            if (!csv_active) {
                bool all_alive = true;
                for (size_t bi = 0; bi < buses.size(); ++bi) {
                    auto pos = buses[bi].motor_control->getPositions();
                    auto mid0 = buses[bi].motor_ids[0];
                    double act = pos.count(mid0) ? static_cast<double>(pos[mid0]) : 0.0;
                    if (act == feedback_init[bi])
                        all_alive = false;
                }
                if (all_alive) {
                    csv_active = true;
                    // 重置正弦时间原点: sin(0)=0, 电机也在零位, 消除启动相位偏移
                    std::cout << "\033[33m反馈就绪 (延迟 " << std::fixed << std::setprecision(1)
                              << t << "s)，重置正弦起点\033[0m\n";
                    start_time = now;
                    i = 0;  // 重置迭代计数，保证完整 duration
                    // 重新读零位 (反馈已活跃，位置准确)
                    for (auto& bus : buses) {
                        auto pos = bus.motor_control->getPositions();
                        for (auto mid : bus.motor_ids)
                            bus.zero_positions[mid] = pos.count(mid) ? static_cast<double>(pos[mid]) : 0.0;
                    }
                    continue;  // 从 t=0 的正弦重新开始
                }
            }

            // CSV 记录 (仅反馈活跃后)
            if (csv_active && csv_log.is_open()) {
                csv_log << std::fixed << std::setprecision(4) << t;
                for (auto& bus : buses) {
                    auto positions = bus.motor_control->getPositions();
                    auto velocities = bus.motor_control->getVelocities();
                    auto torques = bus.motor_control->getTorques();
                    auto faults = bus.motor_control->getFaultCodes();
                    for (auto mid : bus.motor_ids) {
                        double tgt = bus.zero_positions[mid] + sine_val;
                        double act = positions.count(mid) ? static_cast<double>(positions[mid]) : 0.0;
                        double err_deg = (tgt - act) * 180.0 / M_PI;
                        int fault = faults.count(mid) ? static_cast<int>(faults[mid]) : 0;
                        csv_log << "," << tgt << "," << act
                                << "," << std::setprecision(3) << err_deg
                                << "," << (velocities.count(mid) ? velocities[mid] : 0.0f)
                                << "," << (torques.count(mid) ? torques[mid] : 0.0f)
                                << "," << fault;
                    }
                }
                csv_log << "\n";
            }

            // 终端 10Hz
            if (i % (ctrl_freq / 10) == 0) {
                std::cout << "\r" << std::fixed << std::setprecision(1) << t << "s ";
                for (auto& bus : buses) {
                    if (multi_bus) std::cout << "[" << bus.can_name << "] ";
                    auto positions = bus.motor_control->getPositions();
                    for (auto mid : bus.motor_ids) {
                        double act = positions.count(mid) ? static_cast<double>(positions[mid]) : 0.0;
                        double err_deg = (bus.zero_positions[mid] + sine_val - act) * 180.0 / M_PI;
                        std::cout << "M" << static_cast<int>(mid) << ":"
                                  << std::setw(6) << std::setprecision(2) << err_deg << "° ";
                    }
                }
                // 检查所有总线电机故障码
                for (auto& bus : buses) {
                    auto faults = bus.motor_control->getFaultCodes();
                    for (auto mid : bus.motor_ids) {
                        if (faults.count(mid) && faults[mid] != motorevo::MotorErrCode::NO_FAULT) {
                            std::cout << "\n\033[31m";
                            if (multi_bus) std::cout << "[" << bus.can_name << "] ";
                            std::cout << "M" << static_cast<int>(mid)
                                      << " 故障: " << motorevo::MotorErrCode2string(faults[mid])
                                      << "\033[0m";
                        }
                    }
                }
                // 检查通信丢失
                for (auto& bus : buses) {
                    auto positions = bus.motor_control->getPositions();
                    for (auto mid : bus.motor_ids) {
                        double act = positions.count(mid) ? static_cast<double>(positions[mid]) : 0.0;
                        if (std::fabs(act - bus.last_act_positions[mid]) < 1e-6) {
                            bus.stale_counter[mid]++;
                        } else {
                            bus.stale_counter[mid] = 0;
                        }
                        bus.last_act_positions[mid] = act;
                        if (bus.stale_counter[mid] == 50) {
                            std::cout << "\n\033[31m";
                            if (multi_bus) std::cout << "[" << bus.can_name << "] ";
                            std::cout << "M" << static_cast<int>(mid)
                                      << " 通信丢失: 反馈位置已停止更新 5s\033[0m";
                        }
                    }
                }
                // 检查电机状态
                for (auto& bus : buses) {
                    auto states = bus.motor_control->getMotorStates();
                    for (auto mid : bus.motor_ids) {
                        if (states.count(mid) && states[mid] != motorevo::MotorState::MotorSate) {
                            std::cout << "\n\033[31m";
                            if (multi_bus) std::cout << "[" << bus.can_name << "] ";
                            std::cout << "M" << static_cast<int>(mid)
                                      << " 异常: 非运行状态\033[0m";
                        }
                    }
                }
                std::cout << std::flush;
            }

            sine_ts.tv_nsec += ctrl_period_ns;
            if (sine_ts.tv_nsec >= 1000000000L) { sine_ts.tv_sec++; sine_ts.tv_nsec -= 1000000000L; }
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &sine_ts, NULL);
        }
        std::cout << "\n\033[32m正弦波测试完成\033[0m\n";
    }
    // ================================================================
    //  STEP 模式
    // ================================================================
    else if (mode == "step") {
        std::cout << "\n\033[36m开始阶跃响应测试 (" << total_motors << " 个电机)...\033[0m\n";

        int hold_iters = static_cast<int>(step_hold * ctrl_freq);
        std::vector<StepMetrics> all_metrics;  // 以第一条总线第一个电机为代表

        auto global_start = std::chrono::steady_clock::now();
        auto& bus0 = buses[0];
        auto mid0 = bus0.motor_ids[0];  // 代表电机用于指标计算

        for (int s = 0; s < step_count && g_running; ++s) {
            double offset_from = (s % 2 == 0) ? 0.0 : amplitude;
            double offset_to   = (s % 2 == 0) ? amplitude : 0.0;

            std::cout << "\n\033[33m阶跃 " << (s+1) << "/" << step_count << ": "
                      << std::fixed << std::setprecision(1)
                      << offset_from * 180 / M_PI << "° -> "
                      << offset_to * 180 / M_PI << "°\033[0m\n";

            // 确保在起始位置稳定 1s
            {
                for (auto& bus : buses) {
                    std::map<motorevo::MotorId, motorevo::RevoMotorCmd_t> cmd;
                    for (auto mid : bus.motor_ids)
                        cmd[mid] = {.pos = bus.zero_positions[mid] + offset_from, .vel = 0.0, .torque = 0.0, .kp = param_kp, .kd = param_kd};
                    bus.motor_control->setTargetPositions(cmd);
                }
                struct timespec settle_ts;
                clock_gettime(CLOCK_MONOTONIC, &settle_ts);
                for (int t = 0; t < ctrl_freq * 1 && g_running; ++t) {
                    settle_ts.tv_nsec += ctrl_period_ns;
                    if (settle_ts.tv_nsec >= 1000000000L) { settle_ts.tv_sec++; settle_ts.tv_nsec -= 1000000000L; }
                    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &settle_ts, NULL);
                }
            }
            if (!g_running) break;

            // 读实际起始位置 (代表电机)
            double actual_start = 0.0;
            {
                auto pos = bus0.motor_control->getPositions();
                actual_start = pos.count(mid0) ? static_cast<double>(pos[mid0]) : bus0.zero_positions[mid0] + offset_from;
            }

            // 施加阶跃
            std::vector<double> rec_times, rec_positions;
            auto step_start = std::chrono::steady_clock::now();
            double to_pos0 = bus0.zero_positions[mid0] + offset_to;

            struct timespec hold_ts;
            clock_gettime(CLOCK_MONOTONIC, &hold_ts);
            for (int t = 0; t < hold_iters && g_running; ++t) {
                auto now = std::chrono::steady_clock::now();
                double elapsed = std::chrono::duration<double>(now - step_start).count();
                double global_t = std::chrono::duration<double>(now - global_start).count();

                // 发送命令到所有总线
                for (auto& bus : buses) {
                    std::map<motorevo::MotorId, motorevo::RevoMotorCmd_t> cmd;
                    for (auto mid : bus.motor_ids)
                        cmd[mid] = {.pos = bus.zero_positions[mid] + offset_to, .vel = 0.0, .torque = 0.0, .kp = param_kp, .kd = param_kd};
                    bus.motor_control->setTargetPositions(cmd);
                }

                // 代表电机数据 (用于指标计算)
                auto pos0 = bus0.motor_control->getPositions();
                double act0 = pos0.count(mid0) ? static_cast<double>(pos0[mid0]) : 0.0;
                rec_times.push_back(elapsed);
                rec_positions.push_back(act0);

                // CSV
                if (csv_log.is_open()) {
                    csv_log << std::fixed << std::setprecision(4) << global_t;
                    for (auto& bus : buses) {
                        auto positions = bus.motor_control->getPositions();
                        auto velocities = bus.motor_control->getVelocities();
                        auto torques = bus.motor_control->getTorques();
                        auto step_faults = bus.motor_control->getFaultCodes();
                        for (auto mid : bus.motor_ids) {
                            double tgt = bus.zero_positions[mid] + offset_to;
                            double act = positions.count(mid) ? static_cast<double>(positions[mid]) : 0.0;
                            double err_deg = (tgt - act) * 180.0 / M_PI;
                            int fault = step_faults.count(mid) ? static_cast<int>(step_faults[mid]) : 0;
                            csv_log << "," << tgt << "," << act
                                    << "," << std::setprecision(3) << err_deg
                                    << "," << (velocities.count(mid) ? velocities[mid] : 0.0f)
                                    << "," << (torques.count(mid) ? torques[mid] : 0.0f)
                                    << "," << fault;
                        }
                    }
                    csv_log << "\n";
                }

                // 终端 10Hz
                if (t % (ctrl_freq / 10) == 0) {
                    std::cout << "\r  " << std::fixed << std::setprecision(3) << elapsed << "s ";
                    for (auto& bus : buses) {
                        if (multi_bus) std::cout << "[" << bus.can_name << "] ";
                        auto positions = bus.motor_control->getPositions();
                        for (auto mid : bus.motor_ids) {
                            double act = positions.count(mid) ? static_cast<double>(positions[mid]) : 0.0;
                            double err_deg = (bus.zero_positions[mid] + offset_to - act) * 180.0 / M_PI;
                            std::cout << "M" << static_cast<int>(mid) << ":" << std::setw(6)
                                      << std::setprecision(2) << err_deg << "° ";
                        }
                    }
                    std::cout << std::flush;
                }

                hold_ts.tv_nsec += ctrl_period_ns;
                if (hold_ts.tv_nsec >= 1000000000L) { hold_ts.tv_sec++; hold_ts.tv_nsec -= 1000000000L; }
                clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &hold_ts, NULL);
            }

            // 计算指标 (以代表电机为准)
            StepMetrics m = computeStepMetrics(rec_times, rec_positions, actual_start, to_pos0);
            all_metrics.push_back(m);

            std::cout << "\n  M1 上升时间: " << std::fixed << std::setprecision(1) << m.rise_time_ms << " ms"
                      << "  超调: " << std::setprecision(1) << m.overshoot_pct << "%"
                      << "  稳定时间: " << std::setprecision(1) << m.settling_time_ms << " ms"
                      << "  稳态误差: " << std::setprecision(3) << m.steady_state_err_deg << "°\n";
        }

        // ---- 汇总 ----
        if (!all_metrics.empty()) {
            std::cout << "\n\033[32m=== 阶跃响应汇总 ===\033[0m\n"
                      << "kp=" << param_kp << " kd=" << param_kd
                      << " 幅度=" << amplitude_degrees << "°\n\n";

            std::cout << std::right
                      << std::setw(5) << "次数"
                      << std::setw(14) << "上升时间ms"
                      << std::setw(12) << "超调%"
                      << std::setw(14) << "稳定时间ms"
                      << std::setw(14) << "稳态误差°"
                      << "\n" << std::string(59, '=') << "\n";

            double sum_rise = 0, sum_os = 0, sum_settle = 0, sum_sse = 0;
            int valid = 0;
            for (size_t i = 0; i < all_metrics.size(); ++i) {
                const auto& m = all_metrics[i];
                std::cout << std::fixed
                          << std::setw(5) << (i+1)
                          << std::setw(14) << std::setprecision(1) << m.rise_time_ms
                          << std::setw(12) << std::setprecision(1) << m.overshoot_pct
                          << std::setw(14) << std::setprecision(1) << m.settling_time_ms
                          << std::setw(14) << std::setprecision(3) << m.steady_state_err_deg
                          << "\n";
                if (m.rise_time_ms >= 0) {
                    sum_rise += m.rise_time_ms;
                    sum_os += m.overshoot_pct;
                    sum_settle += m.settling_time_ms;
                    sum_sse += std::fabs(m.steady_state_err_deg);
                    valid++;
                }
            }
            std::cout << std::string(59, '-') << "\n";
            if (valid > 0) {
                std::cout << std::fixed
                          << std::setw(5) << "平均"
                          << std::setw(14) << std::setprecision(1) << sum_rise / valid
                          << std::setw(12) << std::setprecision(1) << sum_os / valid
                          << std::setw(14) << std::setprecision(1) << sum_settle / valid
                          << std::setw(14) << std::setprecision(3) << sum_sse / valid
                          << "\n";
            }
            std::cout << std::string(59, '=') << "\n";
        }
    }
    // ================================================================
    //  CHIRP 模式 — 正弦扫频, 一次测试得到频率响应
    // ================================================================
    else if (mode == "chirp") {
        std::cout << "\n\033[36m开始正弦扫频测试...\033[0m\n";

        // 对数均匀频率列表
        std::vector<double> freq_list;
        for (int fi = 0; fi < chirp_steps; ++fi) {
            double ratio = static_cast<double>(fi) / std::max(chirp_steps - 1, 1);
            double f = chirp_f_start * std::pow(chirp_f_end / chirp_f_start, ratio);
            freq_list.push_back(f);
        }

        // 频率响应结果
        struct FreqResponse {
            double freq;
            double gain_db;       // 20*log10(实际幅值/目标幅值)
            double phase_lag_deg; // 相位滞后 (度)
            double rms_err_deg;
            double max_err_deg;
        };
        std::vector<FreqResponse> freq_responses;

        auto global_start = std::chrono::steady_clock::now();
        auto& bus0 = buses[0];
        auto mid0 = bus0.motor_ids[0];  // 代表电机

        for (size_t fi = 0; fi < freq_list.size() && g_running; ++fi) {
            double f = freq_list[fi];
            double period = 1.0 / f;
            double test_duration = period * chirp_cycles;
            // 跳过瞬态: 至少 0.5s 或 1 个周期
            double skip_duration = std::max(0.5, period);
            // 总时长取整到完整周期数，确保 sin() 结束在零点，避免切换超调
            int total_cycles = static_cast<int>(std::ceil((test_duration + skip_duration) * f));
            double total_time = static_cast<double>(total_cycles) / f;
            int total_iters = static_cast<int>(total_time * ctrl_freq);

            // 当前频率下的峰值速度
            double peak_vel = amplitude * 2.0 * M_PI * f;
            const char* vel_warn = (peak_vel > motor_max_vel) ? " \033[31m>MAX!\033[0m" :
                                   (peak_vel > motor_rated_vel) ? " \033[33m>额定\033[0m" : "";

            std::cout << "\033[33m[" << (fi+1) << "/" << freq_list.size() << "] "
                      << std::fixed << std::setprecision(2) << f << " Hz"
                      << " (v_peak=" << std::setprecision(1) << peak_vel << " rad/s" << vel_warn
                      << "\033[33m, " << chirp_cycles << " 周期, " << std::setprecision(1)
                      << test_duration + skip_duration << "s)\033[0m" << std::flush;

            // 直接开始正弦波, 无 hold (sin(0)=0, 从零位平滑起步)
            std::vector<double> rec_t, rec_tgt, rec_act;
            double max_err = 0, sum_sq_err = 0;
            int n_stat = 0;

            auto seg_start = std::chrono::steady_clock::now();
            struct timespec chirp_ts;
            clock_gettime(CLOCK_MONOTONIC, &chirp_ts);

            for (int it = 0; it < total_iters && g_running; ++it) {
                auto now = std::chrono::steady_clock::now();
                double t = std::chrono::duration<double>(now - seg_start).count();
                double gt = std::chrono::duration<double>(now - global_start).count();

                double omega_f = 2.0 * M_PI * f;
                double sine_val = amplitude * std::sin(omega_f * t);
                double vel_ff = amplitude * omega_f * std::cos(omega_f * t);
                // 钳位速度前馈到电机最大速度
                vel_ff = std::max(-motor_max_vel, std::min(motor_max_vel, vel_ff));
                double torque_ff = -inertia_J * amplitude * omega_f * omega_f * std::sin(omega_f * t);

                // 发送命令到所有总线
                for (auto& bus : buses) {
                    std::map<motorevo::MotorId, motorevo::RevoMotorCmd_t> cmd;
                    for (auto mid : bus.motor_ids)
                        cmd[mid] = {.pos = bus.zero_positions[mid] + sine_val, .vel = vel_ff, .torque = torque_ff, .kp = param_kp, .kd = param_kd};
                    bus.motor_control->setTargetPositions(cmd);
                }

                // 代表电机数据
                auto pos0 = bus0.motor_control->getPositions();
                double target0 = bus0.zero_positions[mid0] + sine_val;
                double act0 = pos0.count(mid0) ? static_cast<double>(pos0[mid0]) : 0.0;

                // CSV
                if (csv_log.is_open()) {
                    csv_log << std::fixed << std::setprecision(4) << gt;
                    for (auto& bus : buses) {
                        auto positions = bus.motor_control->getPositions();
                        auto vels = bus.motor_control->getVelocities();
                        auto torques = bus.motor_control->getTorques();
                        auto chirp_faults = bus.motor_control->getFaultCodes();
                        for (auto mid : bus.motor_ids) {
                            double tgt = bus.zero_positions[mid] + sine_val;
                            double act = positions.count(mid) ? static_cast<double>(positions[mid]) : 0.0;
                            double err_deg = (tgt - act) * 180.0 / M_PI;
                            int fault = chirp_faults.count(mid) ? static_cast<int>(chirp_faults[mid]) : 0;
                            csv_log << "," << tgt << "," << act
                                    << "," << std::setprecision(3) << err_deg
                                    << "," << (vels.count(mid) ? vels[mid] : 0.0f)
                                    << "," << (torques.count(mid) ? torques[mid] : 0.0f)
                                    << "," << fault;
                        }
                    }
                    csv_log << "\n";
                }

                // 统计 (跳过瞬态, 以代表电机为准)
                if (t >= skip_duration) {
                    rec_t.push_back(t);
                    rec_tgt.push_back(target0);
                    rec_act.push_back(act0);

                    double err = std::fabs(target0 - act0) * 180.0 / M_PI;
                    if (err > max_err) max_err = err;
                    sum_sq_err += err * err;
                    n_stat++;
                }

                chirp_ts.tv_nsec += ctrl_period_ns;
                if (chirp_ts.tv_nsec >= 1000000000L) { chirp_ts.tv_sec++; chirp_ts.tv_nsec -= 1000000000L; }
                clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &chirp_ts, NULL);
            }

            if (n_stat == 0 || !g_running) {
                if (g_running) std::cout << "  (无数据)\n";
                continue;
            }

            // ---- 计算增益和相位 ----
            double act_peak_pos = -1e9, act_peak_neg = 1e9;
            for (double v : rec_act) {
                double dv = v - bus0.zero_positions[mid0];
                if (dv > act_peak_pos) act_peak_pos = dv;
                if (dv < act_peak_neg) act_peak_neg = dv;
            }
            double actual_amp = (act_peak_pos - act_peak_neg) / 2.0;
            double gain_ratio = actual_amp / amplitude;
            double gain_db = 20.0 * std::log10(std::max(gain_ratio, 1e-6));

            // 相位: 通过上升过零点估算
            std::vector<double> tgt_zc_times, act_zc_times;
            for (size_t i = 1; i < rec_t.size(); ++i) {
                double tv0 = rec_tgt[i-1] - bus0.zero_positions[mid0], tv1 = rec_tgt[i] - bus0.zero_positions[mid0];
                if (tv0 < 0 && tv1 >= 0) {
                    double tc = rec_t[i-1] + (-tv0) * (rec_t[i] - rec_t[i-1]) / (tv1 - tv0);
                    tgt_zc_times.push_back(tc);
                }
                double av0 = rec_act[i-1] - bus0.zero_positions[mid0], av1 = rec_act[i] - bus0.zero_positions[mid0];
                if (av0 < 0 && av1 >= 0) {
                    double tc = rec_t[i-1] + (-av0) * (rec_t[i] - rec_t[i-1]) / (av1 - av0);
                    act_zc_times.push_back(tc);
                }
            }

            double phase_lag_deg = 0;
            if (!tgt_zc_times.empty() && !act_zc_times.empty()) {
                std::vector<double> delays;
                for (double tz : tgt_zc_times) {
                    double best = 1e9;
                    for (double az : act_zc_times) {
                        double d = az - tz;
                        if (d > -period/2 && d < period/2 && std::fabs(d) < std::fabs(best))
                            best = d;
                    }
                    if (std::fabs(best) < period) delays.push_back(best);
                }
                if (!delays.empty()) {
                    double avg_delay = 0;
                    for (double d : delays) avg_delay += d;
                    avg_delay /= delays.size();
                    phase_lag_deg = avg_delay * f * 360.0;
                }
            }

            double rms_err = std::sqrt(sum_sq_err / n_stat);

            FreqResponse fr = {f, gain_db, phase_lag_deg, rms_err, max_err};
            freq_responses.push_back(fr);

            std::cout << "  增益=" << std::fixed << std::setprecision(2) << gain_db << "dB"
                      << " 相位=" << std::setprecision(1) << phase_lag_deg << "°"
                      << " RMS=" << std::setprecision(3) << rms_err << "°\n";

            // ---- 频率切换间: 回零位等待平稳 ----
            if (fi + 1 < freq_list.size() && g_running) {
                const double settle_threshold_rad = 0.5 * M_PI / 180.0;  // 0.5°
                const double settle_hold = 0.2;   // 持续 200ms 内误差均 < 阈值
                const double settle_timeout = 3.0; // 最长等待 3s
                double settle_ok_start = -1.0;

                auto settle_begin = std::chrono::steady_clock::now();
                struct timespec settle_ts;
                clock_gettime(CLOCK_MONOTONIC, &settle_ts);
                while (g_running) {
                    auto now = std::chrono::steady_clock::now();
                    double elapsed = std::chrono::duration<double>(now - settle_begin).count();

                    // 发送零位命令
                    for (auto& bus : buses) {
                        std::map<motorevo::MotorId, motorevo::RevoMotorCmd_t> cmd;
                        for (auto mid : bus.motor_ids)
                            cmd[mid] = {.pos = bus.zero_positions[mid], .vel = 0.0, .torque = 0.0, .kp = param_kp, .kd = param_kd};
                        bus.motor_control->setTargetPositions(cmd);
                    }

                    // CSV 记录 (settling 期间也记录)
                    if (csv_log.is_open()) {
                        double gt = std::chrono::duration<double>(now - global_start).count();
                        csv_log << std::fixed << std::setprecision(4) << gt;
                        for (auto& bus : buses) {
                            auto positions = bus.motor_control->getPositions();
                            auto vels = bus.motor_control->getVelocities();
                            auto torques = bus.motor_control->getTorques();
                            auto sfaults = bus.motor_control->getFaultCodes();
                            for (auto mid : bus.motor_ids) {
                                double tgt = bus.zero_positions[mid];
                                double act = positions.count(mid) ? static_cast<double>(positions[mid]) : 0.0;
                                double err_deg = (tgt - act) * 180.0 / M_PI;
                                int fault = sfaults.count(mid) ? static_cast<int>(sfaults[mid]) : 0;
                                csv_log << "," << tgt << "," << act
                                        << "," << std::setprecision(3) << err_deg
                                        << "," << (vels.count(mid) ? vels[mid] : 0.0f)
                                        << "," << (torques.count(mid) ? torques[mid] : 0.0f)
                                        << "," << fault;
                            }
                        }
                        csv_log << "\n";
                    }

                    // 检查所有电机是否平稳
                    bool all_settled = true;
                    for (auto& bus : buses) {
                        auto positions = bus.motor_control->getPositions();
                        for (auto mid : bus.motor_ids) {
                            double act = positions.count(mid) ? static_cast<double>(positions[mid]) : 0.0;
                            if (std::fabs(act - bus.zero_positions[mid]) > settle_threshold_rad) {
                                all_settled = false;
                                break;
                            }
                        }
                        if (!all_settled) break;
                    }

                    if (all_settled) {
                        if (settle_ok_start < 0) settle_ok_start = elapsed;
                        if (elapsed - settle_ok_start >= settle_hold) {
                            std::cout << "  平稳 (" << std::fixed << std::setprecision(0)
                                      << elapsed * 1000 << "ms)\n";
                            break;
                        }
                    } else {
                        settle_ok_start = -1.0;
                    }

                    if (elapsed >= settle_timeout) {
                        std::cout << "  \033[33m平稳超时 (" << std::setprecision(0)
                                  << settle_timeout * 1000 << "ms)\033[0m\n";
                        break;
                    }

                    settle_ts.tv_nsec += ctrl_period_ns;
                    if (settle_ts.tv_nsec >= 1000000000L) { settle_ts.tv_sec++; settle_ts.tv_nsec -= 1000000000L; }
                    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &settle_ts, NULL);
                }
            }
        }

        // ---- 频率响应汇总 (Bode 图数据) ----
        std::cout << "\n\033[32m======================================\033[0m\n"
                  << "\033[32m  频率响应 (Bode 图)  \033[0m\n"
                  << "\033[32m======================================\033[0m\n"
                  << "kp=" << param_kp << " kd=" << param_kd
                  << " amp=" << amplitude_degrees << "°\n\n";

        std::cout << std::right
                  << std::setw(10) << "频率(Hz)"
                  << std::setw(12) << "增益(dB)"
                  << std::setw(12) << "相位(°)"
                  << std::setw(12) << "RMS误差°"
                  << std::setw(12) << "最大误差°"
                  << std::setw(12) << "幅值比"
                  << "\n" << std::string(70, '=') << "\n";

        for (const auto& fr : freq_responses) {
            double ratio = std::pow(10.0, fr.gain_db / 20.0);
            std::cout << std::fixed
                      << std::setw(10) << std::setprecision(2) << fr.freq
                      << std::setw(12) << std::setprecision(2) << fr.gain_db
                      << std::setw(12) << std::setprecision(1) << fr.phase_lag_deg
                      << std::setw(12) << std::setprecision(3) << fr.rms_err_deg
                      << std::setw(12) << std::setprecision(3) << fr.max_err_deg
                      << std::setw(12) << std::setprecision(3) << ratio
                      << "\n";
        }
        std::cout << std::string(70, '=') << "\n";

        // 找 -3dB 带宽
        double bw_3db = -1;
        for (const auto& fr : freq_responses) {
            if (fr.gain_db < -3.0) {
                bw_3db = fr.freq;
                break;
            }
        }
        if (bw_3db > 0) {
            std::cout << "\n\033[33m-3dB 带宽: ~" << std::setprecision(2) << bw_3db << " Hz\033[0m\n";
        } else {
            std::cout << "\n\033[33m-3dB 带宽: > " << std::setprecision(1) << chirp_f_end << " Hz (未达到)\033[0m\n";
        }

        // 找 -45° 相位频率
        double f_45 = -1;
        for (const auto& fr : freq_responses) {
            if (fr.phase_lag_deg < -45.0) {
                f_45 = fr.freq;
                break;
            }
        }
        if (f_45 > 0) {
            std::cout << "\033[33m-45° 相位频率: ~" << std::setprecision(2) << f_45 << " Hz\033[0m\n";
        }
    }
    // ================================================================
    //  TORQUE-STEP 模式 — 力矩阶跃, kp=0 kd=0, 观察速度响应
    // ================================================================
    else if (mode == "torque-step") {
        // 构建力矩序列: --torque-steps 自定义 或 ±amp 交替
        std::vector<double> torque_sequence;
        if (!torque_steps_arg.empty()) {
            auto strs = splitComma(torque_steps_arg);
            for (auto& s : strs) torque_sequence.push_back(std::stod(s));
            step_count = static_cast<int>(torque_sequence.size());
        } else {
            for (int s = 0; s < step_count; ++s)
                torque_sequence.push_back((s % 2 == 0) ? torque_amp : -torque_amp);
        }

        std::cout << "\n\033[36m开始力矩阶跃测试 (kp=0, kd=0), 序列:";
        for (auto t : torque_sequence) std::cout << " " << t;
        std::cout << " Nm, 每阶段 " << step_hold << "s\033[0m\n";

        auto global_start = std::chrono::steady_clock::now();
        int hold_iters = static_cast<int>(step_hold * ctrl_freq);

        for (int s = 0; s < step_count && g_running; ++s) {
            double torque_cmd = torque_sequence[s];

            std::cout << "\n\033[33m阶跃 " << (s+1) << "/" << step_count << ": τ="
                      << std::fixed << std::setprecision(2) << torque_cmd << " Nm, 保持 "
                      << step_hold << "s\033[0m\n";

            auto step_start = std::chrono::steady_clock::now();
            struct timespec ts;
            clock_gettime(CLOCK_MONOTONIC, &ts);
            bool aborted = false;

            for (int it = 0; it < hold_iters && g_running; ++it) {
                auto now = std::chrono::steady_clock::now();
                double t = std::chrono::duration<double>(now - step_start).count();
                double gt = std::chrono::duration<double>(now - global_start).count();

                // 发送力矩命令: kp=0, kd=0
                for (auto& bus : buses) {
                    std::map<motorevo::MotorId, motorevo::RevoMotorCmd_t> cmd;
                    for (auto mid : bus.motor_ids) {
                        cmd[mid] = {.pos = bus.zero_positions[mid], .vel = 0.0,
                                    .torque = torque_cmd, .kp = 0.0, .kd = 0.0};
                    }
                    bus.motor_control->setTargetPositions(cmd);
                }

                // 安全检查: 位置偏移超限
                for (auto& bus : buses) {
                    auto positions = bus.motor_control->getPositions();
                    for (auto mid : bus.motor_ids) {
                        double act = positions.count(mid) ? static_cast<double>(positions[mid]) : 0.0;
                        double offset = std::fabs(act - bus.zero_positions[mid]);
                        if (offset > torque_pos_limit) {
                            std::cout << "\n\033[31m安全中止: ";
                            if (multi_bus) std::cout << "[" << bus.can_name << "] ";
                            std::cout << "M" << static_cast<int>(mid)
                                      << " 位置偏移 " << std::fixed << std::setprecision(2)
                                      << offset << " rad (" << offset * 180.0 / M_PI << "°)"
                                      << " 超过限制 " << torque_pos_limit << " rad\033[0m\n";
                            aborted = true;
                            break;
                        }
                    }
                    if (aborted) break;
                }
                if (aborted) break;

                // CSV 记录
                if (csv_log.is_open()) {
                    csv_log << std::fixed << std::setprecision(4) << gt;
                    for (auto& bus : buses) {
                        auto positions = bus.motor_control->getPositions();
                        auto vels = bus.motor_control->getVelocities();
                        auto torques = bus.motor_control->getTorques();
                        auto ts_faults = bus.motor_control->getFaultCodes();
                        for (auto mid : bus.motor_ids) {
                            double act = positions.count(mid) ? static_cast<double>(positions[mid]) : 0.0;
                            int fault = ts_faults.count(mid) ? static_cast<int>(ts_faults[mid]) : 0;
                            csv_log << "," << torque_cmd
                                    << "," << act
                                    << "," << (vels.count(mid) ? vels[mid] : 0.0f)
                                    << "," << (torques.count(mid) ? torques[mid] : 0.0f)
                                    << "," << fault;
                        }
                    }
                    csv_log << "\n";
                }

                // 终端 10Hz
                if (it % (ctrl_freq / 10) == 0) {
                    std::cout << "\r  " << std::fixed << std::setprecision(2) << t << "s "
                              << "τ=" << torque_cmd << "Nm ";
                    for (auto& bus : buses) {
                        if (multi_bus) std::cout << "[" << bus.can_name << "] ";
                        auto positions = bus.motor_control->getPositions();
                        auto velocities = bus.motor_control->getVelocities();
                        for (auto mid : bus.motor_ids) {
                            double act = positions.count(mid) ? static_cast<double>(positions[mid]) : 0.0;
                            double offset_deg = (act - bus.zero_positions[mid]) * 180.0 / M_PI;
                            double vel = velocities.count(mid) ? static_cast<double>(velocities[mid]) : 0.0;
                            std::cout << "M" << static_cast<int>(mid) << ":"
                                      << std::setw(6) << std::setprecision(1) << offset_deg << "°"
                                      << std::setw(6) << std::setprecision(2) << vel << "r/s ";
                        }
                    }
                    std::cout << std::flush;
                }

                ts.tv_nsec += ctrl_period_ns;
                if (ts.tv_nsec >= 1000000000L) { ts.tv_sec++; ts.tv_nsec -= 1000000000L; }
                clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);
            }

            if (aborted) {
                std::cout << "  \033[31m中止\033[0m\n";
            } else {
                std::cout << "\n  完成\n";
            }

            // ---- 回零位 (位置模式) ----
            if (g_running) {
                const double settle_threshold_rad = 0.5 * M_PI / 180.0;
                const double settle_hold_time = 0.2;
                const double settle_timeout = 3.0;
                double settle_ok_start = -1.0;

                auto settle_begin = std::chrono::steady_clock::now();
                struct timespec settle_ts;
                clock_gettime(CLOCK_MONOTONIC, &settle_ts);
                while (g_running) {
                    auto now = std::chrono::steady_clock::now();
                    double elapsed = std::chrono::duration<double>(now - settle_begin).count();

                    for (auto& bus : buses) {
                        std::map<motorevo::MotorId, motorevo::RevoMotorCmd_t> cmd;
                        for (auto mid : bus.motor_ids)
                            cmd[mid] = {.pos = bus.zero_positions[mid], .vel = 0.0, .torque = 0.0, .kp = param_kp, .kd = param_kd};
                        bus.motor_control->setTargetPositions(cmd);
                    }

                    bool all_settled = true;
                    for (auto& bus : buses) {
                        auto positions = bus.motor_control->getPositions();
                        for (auto mid : bus.motor_ids) {
                            double act = positions.count(mid) ? static_cast<double>(positions[mid]) : 0.0;
                            if (std::fabs(act - bus.zero_positions[mid]) > settle_threshold_rad) {
                                all_settled = false;
                                break;
                            }
                        }
                        if (!all_settled) break;
                    }

                    if (all_settled) {
                        if (settle_ok_start < 0) settle_ok_start = elapsed;
                        if (elapsed - settle_ok_start >= settle_hold_time) {
                            std::cout << "  回零 (" << std::fixed << std::setprecision(0)
                                      << elapsed * 1000 << "ms)\n";
                            break;
                        }
                    } else {
                        settle_ok_start = -1.0;
                    }

                    if (elapsed >= settle_timeout) {
                        std::cout << "  \033[33m回零超时 (" << std::setprecision(0)
                                  << settle_timeout * 1000 << "ms)\033[0m\n";
                        break;
                    }

                    settle_ts.tv_nsec += ctrl_period_ns;
                    if (settle_ts.tv_nsec >= 1000000000L) { settle_ts.tv_sec++; settle_ts.tv_nsec -= 1000000000L; }
                    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &settle_ts, NULL);
                }
            }
        }

        std::cout << "\n\033[32m力矩阶跃测试完成\033[0m\n";
    }
    // ================================================================
    //  TORQUE-CHIRP 模式 — 纯力矩扫频, kp=0 kd=0, torque=A*sin(ωt)
    // ================================================================
    else if (mode == "torque-chirp") {
        std::cout << "\n\033[36m开始力矩扫频测试 (kp=0, kd=0, torque=" << torque_amp << " Nm)...\033[0m\n";

        // 对数均匀频率列表 (与 chirp 模式相同)
        std::vector<double> freq_list;
        for (int fi = 0; fi < chirp_steps; ++fi) {
            double ratio = static_cast<double>(fi) / std::max(chirp_steps - 1, 1);
            double f = chirp_f_start * std::pow(chirp_f_end / chirp_f_start, ratio);
            freq_list.push_back(f);
        }

        auto global_start = std::chrono::steady_clock::now();

        for (size_t fi = 0; fi < freq_list.size() && g_running; ++fi) {
            double f = freq_list[fi];
            double period = 1.0 / f;
            double test_duration = period * chirp_cycles;
            int total_iters = static_cast<int>(test_duration * ctrl_freq);

            std::cout << "\033[33m[" << (fi+1) << "/" << freq_list.size() << "] "
                      << std::fixed << std::setprecision(2) << f << " Hz"
                      << " (" << chirp_cycles << " 周期, " << std::setprecision(1)
                      << test_duration << "s)\033[0m" << std::flush;

            auto seg_start = std::chrono::steady_clock::now();
            struct timespec tc_ts;
            clock_gettime(CLOCK_MONOTONIC, &tc_ts);
            bool aborted = false;

            for (int it = 0; it < total_iters && g_running; ++it) {
                auto now = std::chrono::steady_clock::now();
                double t = std::chrono::duration<double>(now - seg_start).count();
                double gt = std::chrono::duration<double>(now - global_start).count();

                double omega_f = 2.0 * M_PI * f;
                double torque_cmd = torque_amp * std::sin(omega_f * t);

                // 发送命令: kp=0, kd=0, pos=零位, vel=0, torque=正弦
                for (auto& bus : buses) {
                    std::map<motorevo::MotorId, motorevo::RevoMotorCmd_t> cmd;
                    for (auto mid : bus.motor_ids) {
                        cmd[mid] = {.pos = bus.zero_positions[mid], .vel = 0.0,
                                    .torque = torque_cmd, .kp = 0.0, .kd = 0.0};
                    }
                    bus.motor_control->setTargetPositions(cmd);
                }

                // 安全检查: 位置偏移超限
                for (auto& bus : buses) {
                    auto positions = bus.motor_control->getPositions();
                    for (auto mid : bus.motor_ids) {
                        double act = positions.count(mid) ? static_cast<double>(positions[mid]) : 0.0;
                        double offset = std::fabs(act - bus.zero_positions[mid]);
                        if (offset > torque_pos_limit) {
                            std::cout << "\n\033[31m安全中止: ";
                            if (multi_bus) std::cout << "[" << bus.can_name << "] ";
                            std::cout << "M" << static_cast<int>(mid)
                                      << " 位置偏移 " << std::fixed << std::setprecision(2)
                                      << offset << " rad (" << offset * 180.0 / M_PI << "°)"
                                      << " 超过限制 " << torque_pos_limit << " rad\033[0m\n";
                            aborted = true;
                            break;
                        }
                    }
                    if (aborted) break;
                }
                if (aborted) break;

                // CSV 记录
                if (csv_log.is_open()) {
                    csv_log << std::fixed << std::setprecision(4) << gt;
                    for (auto& bus : buses) {
                        auto positions = bus.motor_control->getPositions();
                        auto vels = bus.motor_control->getVelocities();
                        auto torques = bus.motor_control->getTorques();
                        auto tc_faults = bus.motor_control->getFaultCodes();
                        for (auto mid : bus.motor_ids) {
                            double act = positions.count(mid) ? static_cast<double>(positions[mid]) : 0.0;
                            int fault = tc_faults.count(mid) ? static_cast<int>(tc_faults[mid]) : 0;
                            csv_log << "," << torque_cmd
                                    << "," << act
                                    << "," << (vels.count(mid) ? vels[mid] : 0.0f)
                                    << "," << (torques.count(mid) ? torques[mid] : 0.0f)
                                    << "," << fault;
                        }
                    }
                    csv_log << "\n";
                }

                // 终端 10Hz
                if (it % (ctrl_freq / 10) == 0) {
                    std::cout << "\r  " << std::fixed << std::setprecision(2) << t << "s "
                              << "τ=" << std::setprecision(2) << torque_cmd << "Nm ";
                    for (auto& bus : buses) {
                        if (multi_bus) std::cout << "[" << bus.can_name << "] ";
                        auto positions = bus.motor_control->getPositions();
                        for (auto mid : bus.motor_ids) {
                            double act = positions.count(mid) ? static_cast<double>(positions[mid]) : 0.0;
                            double offset_deg = (act - bus.zero_positions[mid]) * 180.0 / M_PI;
                            std::cout << "M" << static_cast<int>(mid) << ":"
                                      << std::setw(6) << std::setprecision(1) << offset_deg << "° ";
                        }
                    }
                    std::cout << std::flush;
                }

                tc_ts.tv_nsec += ctrl_period_ns;
                if (tc_ts.tv_nsec >= 1000000000L) { tc_ts.tv_sec++; tc_ts.tv_nsec -= 1000000000L; }
                clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &tc_ts, NULL);
            }

            if (aborted) {
                std::cout << "  \033[31m中止\033[0m\n";
            } else {
                std::cout << "  完成\n";
            }

            // ---- 频率切换间: 回零位等待平稳 (用位置模式) ----
            if (fi + 1 < freq_list.size() && g_running) {
                const double settle_threshold_rad = 0.5 * M_PI / 180.0;  // 0.5°
                const double settle_hold = 0.2;   // 持续 200ms 内误差均 < 阈值
                const double settle_timeout = 3.0; // 最长等待 3s
                double settle_ok_start = -1.0;

                auto settle_begin = std::chrono::steady_clock::now();
                struct timespec settle_ts;
                clock_gettime(CLOCK_MONOTONIC, &settle_ts);
                while (g_running) {
                    auto now = std::chrono::steady_clock::now();
                    double elapsed = std::chrono::duration<double>(now - settle_begin).count();

                    // 切回位置模式回零
                    for (auto& bus : buses) {
                        std::map<motorevo::MotorId, motorevo::RevoMotorCmd_t> cmd;
                        for (auto mid : bus.motor_ids)
                            cmd[mid] = {.pos = bus.zero_positions[mid], .vel = 0.0, .torque = 0.0, .kp = param_kp, .kd = param_kd};
                        bus.motor_control->setTargetPositions(cmd);
                    }

                    // 检查所有电机是否平稳
                    bool all_settled = true;
                    for (auto& bus : buses) {
                        auto positions = bus.motor_control->getPositions();
                        for (auto mid : bus.motor_ids) {
                            double act = positions.count(mid) ? static_cast<double>(positions[mid]) : 0.0;
                            if (std::fabs(act - bus.zero_positions[mid]) > settle_threshold_rad) {
                                all_settled = false;
                                break;
                            }
                        }
                        if (!all_settled) break;
                    }

                    if (all_settled) {
                        if (settle_ok_start < 0) settle_ok_start = elapsed;
                        if (elapsed - settle_ok_start >= settle_hold) {
                            std::cout << "  回零 (" << std::fixed << std::setprecision(0)
                                      << elapsed * 1000 << "ms)\n";
                            break;
                        }
                    } else {
                        settle_ok_start = -1.0;
                    }

                    if (elapsed >= settle_timeout) {
                        std::cout << "  \033[33m回零超时 (" << std::setprecision(0)
                                  << settle_timeout * 1000 << "ms)\033[0m\n";
                        break;
                    }

                    settle_ts.tv_nsec += ctrl_period_ns;
                    if (settle_ts.tv_nsec >= 1000000000L) { settle_ts.tv_sec++; settle_ts.tv_nsec -= 1000000000L; }
                    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &settle_ts, NULL);
                }
            }
        }

        std::cout << "\n\033[32m力矩扫频测试完成\033[0m\n";
    }
    else {
        std::cerr << "\033[31m未知模式: " << mode << " (可选: sine, step, chirp, torque-step, torque-chirp)\033[0m\n";
    }

    // ---- 清理 ----
    if (csv_log.is_open()) {
        csv_log.close();
        std::cout << "\033[32mCSV 已保存: " << log_file << "\033[0m\n";
    }

    // 回零位
    {
        for (auto& bus : buses) {
            std::map<motorevo::MotorId, motorevo::RevoMotorCmd_t> cmd;
            for (auto mid : bus.motor_ids)
                cmd[mid] = {.pos = bus.zero_positions[mid], .vel = 0.0, .torque = 0.0, .kp = param_kp, .kd = param_kd};
            bus.motor_control->setTargetPositions(cmd);
        }
        struct timespec zero_ts;
        clock_gettime(CLOCK_MONOTONIC, &zero_ts);
        for (int t = 0; t < ctrl_freq * 2; ++t) {
            zero_ts.tv_nsec += ctrl_period_ns;
            if (zero_ts.tv_nsec >= 1000000000L) { zero_ts.tv_sec++; zero_ts.tv_nsec -= 1000000000L; }
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &zero_ts, NULL);
        }
    }

    delete g_control_loop;
    g_control_loop = nullptr;
    for (auto& bus : buses) bus.motor_control.reset();
    std::this_thread::sleep_for(std::chrono::seconds(1));

    std::cout << "\033[32m测试完成\033[0m\n";
    return 0;
}
