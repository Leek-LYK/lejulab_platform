/**
 * CAN FD 电机参数自动调优测试
 *
 * 自动遍历 kp × kd × 振幅 的参数组合，在固定频率下测试正弦跟踪性能，
 * 找出 RMS 误差最小的最优参数组合。
 *
 * 用法:
 *   sudo ./canfd_sine_sweep_test [选项]
 *
 * 选项:
 *   --can <接口名>          CAN 接口 (默认 bcan0)
 *   --id <电机ID>           电机 ID (默认 0x01)
 *   --freq <Hz>             测试频率 (默认 0.5)
 *   --kps <k1,k2,...>       kp 列表 (默认 50,100,150,200,250,300)
 *   --kds <k1,k2,...>       kd 列表 (默认 1,2,3,4)
 *   --amps <a1,a2,...>      振幅列表 度 (默认 5,10,15,20)
 *   --duration <秒>         每组测试时长 (默认 5)
 *   --settle <秒>           切换稳定时间 (默认 2)
 *   --log <文件.csv>        保存 CSV 原始数据
 *   --ctrl-freq <Hz>        控制频率 (默认 500)
 *
 * 示例:
 *   # 默认参数 (96 种组合, 约 11 分钟)
 *   sudo ./canfd_sine_sweep_test --can bcan0 --log sweep.csv
 *
 *   # 缩小搜索范围
 *   sudo ./canfd_sine_sweep_test --kps 100,150,200 --kds 2,3 --amps 10,15 --log sweep.csv
 *
 *   # 测试高频跟踪
 *   sudo ./canfd_sine_sweep_test --freq 2.0 --kps 200,300,400 --kds 2,3,4 --amps 5,10 --log sweep.csv
 */

#include "motorevo/motor_ctrl.h"
#include "motorevo/motor_def.h"
#include "canbus_sdk/canbus_sdk.h"

#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <string>
#include <sstream>
#include <thread>
#include <chrono>
#include <cmath>
#include <csignal>
#include <atomic>
#include <iomanip>
#include <algorithm>
#include <time.h>

// ---- 控制循环线程 (clock_nanosleep TIMER_ABSTIME 绝对定时) ----
class ControlLoop {
public:
    explicit ControlLoop(std::shared_ptr<motorevo::RevoMotorControl> mc, int freq_hz)
        : mc_(mc), running_(true), period_ns_(1000000000L / freq_hz),
          thread_([this]() {
              struct timespec next_time;
              clock_gettime(CLOCK_MONOTONIC, &next_time);
              while (running_) {
                  next_time.tv_nsec += period_ns_;
                  if (next_time.tv_nsec >= 1000000000L) {
                      next_time.tv_sec++;
                      next_time.tv_nsec -= 1000000000L;
                  }
                  mc_->write();
                  clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);
              }
          }) {}
    ~ControlLoop() { running_ = false; if (thread_.joinable()) thread_.join(); }
    void stop() { running_ = false; }
private:
    std::shared_ptr<motorevo::RevoMotorControl> mc_;
    std::atomic<bool> running_;
    long period_ns_;
    std::thread thread_;
};

static std::atomic<bool> g_running(true);
static ControlLoop* g_loop = nullptr;
static void sigHandler(int) { g_running = false; if (g_loop) g_loop->stop(); }

static std::vector<double> parseList(const std::string& s) {
    std::vector<double> v;
    std::istringstream ss(s);
    std::string tok;
    while (std::getline(ss, tok, ',')) v.push_back(std::stod(tok));
    return v;
}

// 单次测试结果
struct TestResult {
    double kp, kd, amp_deg, freq_hz;
    double rms_err_deg;
    double max_err_deg;
    double phase_lag_ms;   // 通过过零点估算
    int samples;
};

// 估算相位差 (ms): 通过目标和实际信号的上升过零点
static double estimatePhaseLag(const std::vector<double>& times,
                                const std::vector<double>& targets,
                                const std::vector<double>& actuals,
                                double zero_pos) {
    // 找上升过零点
    auto findZeroCrossings = [](const std::vector<double>& ts,
                                 const std::vector<double>& vals,
                                 double offset) -> std::vector<double> {
        std::vector<double> crossings;
        for (size_t i = 1; i < vals.size(); ++i) {
            double v0 = vals[i-1] - offset;
            double v1 = vals[i] - offset;
            if (v0 < 0 && v1 >= 0) {  // rising zero crossing
                double tc = ts[i-1] + (-v0) * (ts[i] - ts[i-1]) / (v1 - v0);
                crossings.push_back(tc);
            }
        }
        return crossings;
    };

    auto tgt_zc = findZeroCrossings(times, targets, zero_pos);
    auto act_zc = findZeroCrossings(times, actuals, zero_pos);

    if (tgt_zc.empty() || act_zc.empty()) return 0.0;

    // 对每个目标过零点，找最近的实际过零点
    std::vector<double> delays;
    for (double tz : tgt_zc) {
        double best = 1e9;
        for (double az : act_zc) {
            double d = az - tz;
            if (d > -0.5 && d < 0.5 && std::fabs(d) < std::fabs(best)) {
                best = d;
            }
        }
        if (std::fabs(best) < 1e8) delays.push_back(best);
    }

    if (delays.empty()) return 0.0;
    double sum = 0;
    for (double d : delays) sum += d;
    return (sum / delays.size()) * 1000.0;  // ms
}

int main(int argc, char* argv[]) {
    std::signal(SIGINT, sigHandler);
    std::signal(SIGTERM, sigHandler);

    // ---- 默认参数 ----
    std::string can_name = "bcan0";
    uint8_t motor_id = 0x01;
    double freq = 0.5;
    std::vector<double> kps = {50, 100, 150, 200, 250, 300};
    std::vector<double> kds = {1, 2, 3, 4};
    std::vector<double> amps = {5, 10, 15, 20};
    double duration_s = 5.0;
    double settle_s = 2.0;
    std::string log_file;
    int ctrl_freq = 500;

    // ---- 解析参数 ----
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "-h" || a == "--help") {
            std::cerr << "用法: sudo " << argv[0] << " [选项]\n"
                << "  --can <名>         CAN 接口 (默认 bcan0)\n"
                << "  --id <ID>          电机 ID (默认 0x01)\n"
                << "  --freq <Hz>        测试频率 (默认 0.5)\n"
                << "  --kps <k1,k2,...>  kp 列表 (默认 50,100,150,200,250,300)\n"
                << "  --kds <k1,k2,...>  kd 列表 (默认 1,2,3,4)\n"
                << "  --amps <a1,a2,...> 振幅列表 度 (默认 5,10,15,20)\n"
                << "  --duration <秒>    每组时长 (默认 5)\n"
                << "  --settle <秒>      稳定时间 (默认 2)\n"
                << "  --log <file.csv>   CSV 原始数据\n"
                << "  --ctrl-freq <Hz>   控制频率 (默认 500)\n";
            return 0;
        }
        else if (a == "--can"       && i+1 < argc) can_name   = argv[++i];
        else if (a == "--id"        && i+1 < argc) motor_id   = static_cast<uint8_t>(std::stoul(argv[++i], nullptr, 0));
        else if (a == "--freq"      && i+1 < argc) freq       = std::stod(argv[++i]);
        else if (a == "--kps"       && i+1 < argc) kps        = parseList(argv[++i]);
        else if (a == "--kds"       && i+1 < argc) kds        = parseList(argv[++i]);
        else if (a == "--amps"      && i+1 < argc) amps       = parseList(argv[++i]);
        else if (a == "--duration"  && i+1 < argc) duration_s = std::stod(argv[++i]);
        else if (a == "--settle"    && i+1 < argc) settle_s   = std::stod(argv[++i]);
        else if (a == "--log"       && i+1 < argc) log_file   = argv[++i];
        else if (a == "--ctrl-freq" && i+1 < argc) ctrl_freq  = std::stoi(argv[++i]);
    }

    int total_combos = static_cast<int>(kps.size() * kds.size() * amps.size());
    double est_time = total_combos * (duration_s + settle_s);

    // ---- 打印配置 ----
    std::cout << "\033[32m=== CAN FD 参数自动调优 ===\033[0m\n"
              << "CAN: " << can_name << ", ID: 0x" << std::hex << (int)motor_id << std::dec
              << ", 频率: " << freq << " Hz, 控制: " << ctrl_freq << " Hz\n"
              << "kp 列表 (" << kps.size() << "):";
    for (double v : kps) std::cout << " " << v;
    std::cout << "\nkd 列表 (" << kds.size() << "):";
    for (double v : kds) std::cout << " " << v;
    std::cout << "\n振幅列表 (" << amps.size() << "):";
    for (double v : amps) std::cout << " " << v << "°";
    std::cout << "\n总组合数: " << total_combos
              << ", 预计时间: " << std::fixed << std::setprecision(0) << est_time / 60 << " 分钟"
              << "\n每组: " << duration_s << "s 测试 + " << settle_s << "s 稳定"
              << "\n按 Ctrl+C 退出\n" << std::endl;

    // ---- 初始化 CAN 总线 ----
    auto& canbus_ctrl = canbus_sdk::CanBusController::getInstance();
    canbus_ctrl.init();

    canbus_sdk::CanBusBitrate bitrate = {
        .nbitrate = 1000000, .dbitrate = 5000000,
        .nsamplepos = 80, .dsamplepos = 75
    };
    auto result = canbus_ctrl.openCanBus(can_name, canbus_sdk::CanBusModelType::SOCKETCAN, bitrate);
    if (!result.has_value()) {
        std::cerr << "\033[31mCAN 总线打开失败\033[0m\n";
        return -1;
    }

    // ---- 配置电机 ----
    motorevo::RevoMotorConfig_t cfg = {
        .id = static_cast<motorevo::MotorId>(motor_id),
        .name = "test_motor",
        .negtive = false, .ignore = false, .zero_offset = 0.0f,
        .default_params = {.vel = 0, .kp_pos = 100, .kd_pos = 2, .tor = 0, .kp_vel = 0, .kd_vel = 0, .ki_vel = 0}
    };

    auto mc = std::make_shared<motorevo::RevoMotorControl>(can_name, motorevo::MotorProtocol::CANFD_BROADCAST);
    g_loop = new ControlLoop(mc, ctrl_freq);

    if (!mc->init({cfg}, false, true)) {
        std::cerr << "\033[31m电机初始化失败\033[0m\n";
        delete g_loop; g_loop = nullptr;
        return -1;
    }
    std::cout << "\033[32m电机初始化成功\033[0m\n";

    auto mid = static_cast<motorevo::MotorId>(motor_id);

    // ---- CSV 日志 ----
    std::ofstream csv;
    if (!log_file.empty()) {
        csv.open(log_file);
        if (csv.is_open()) {
            csv << "combo,kp,kd,amp_deg,freq_hz,time_s,target_rad,actual_rad,error_deg,velocity,torque\n";
        }
    }

    // ---- 稳定 2s ----
    const long ctrl_period_ns = 1000000000L / ctrl_freq;
    {
        auto pos = mc->getPositions();
        double hp = pos.count(mid) ? static_cast<double>(pos[mid]) : 0.0;
        std::map<motorevo::MotorId, motorevo::RevoMotorCmd_t> cmd;
        cmd[mid] = {.pos = hp, .vel = 0.0, .torque = 0.0, .kp = 100.0, .kd = 2.0};
        mc->setTargetPositions(cmd);
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        for (int t = 0; t < ctrl_freq * 2 && g_running; ++t) {
            ts.tv_nsec += ctrl_period_ns;
            if (ts.tv_nsec >= 1000000000L) { ts.tv_sec++; ts.tv_nsec -= 1000000000L; }
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);
        }
    }

    double zero_pos = 0.0;
    {
        auto pos = mc->getPositions();
        zero_pos = pos.count(mid) ? static_cast<double>(pos[mid]) : 0.0;
    }
    std::cout << "零位: " << std::fixed << std::setprecision(2)
              << zero_pos * 180 / M_PI << "°\n" << std::endl;

    // ============================================================
    //  网格搜索主循环
    // ============================================================
    std::vector<TestResult> results;
    int combo_idx = 0;

    for (double amp_deg : amps) {
        for (double kp : kps) {
            for (double kd : kds) {
                if (!g_running) break;

                combo_idx++;
                double amp_rad = amp_deg * M_PI / 180.0;

                std::cout << "\033[36m[" << combo_idx << "/" << total_combos << "] "
                          << "kp=" << kp << " kd=" << kd << " amp=" << amp_deg << "°"
                          << " freq=" << freq << "Hz\033[0m" << std::flush;

                // -- 稳定: 回零位 --
                {
                    std::map<motorevo::MotorId, motorevo::RevoMotorCmd_t> cmd;
                    cmd[mid] = {.pos = zero_pos, .vel = 0.0, .torque = 0.0, .kp = kp, .kd = kd};
                    mc->setTargetPositions(cmd);
                    struct timespec ts;
                    clock_gettime(CLOCK_MONOTONIC, &ts);
                    for (int t = 0; t < static_cast<int>(ctrl_freq * settle_s) && g_running; ++t) {
                        ts.tv_nsec += ctrl_period_ns;
                        if (ts.tv_nsec >= 1000000000L) { ts.tv_sec++; ts.tv_nsec -= 1000000000L; }
                        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);
                    }
                }
                if (!g_running) break;

                // -- 正弦波测试 --
                auto t_start = std::chrono::steady_clock::now();
                double max_err = 0.0, sum_sq_err = 0.0;
                int n_samples = 0;

                // 记录用于相位估算
                std::vector<double> rec_times, rec_targets, rec_actuals;

                int total_iters = static_cast<int>(duration_s * ctrl_freq);
                int log_interval = ctrl_freq / 500;  // 最多 500Hz 记录
                if (log_interval < 1) log_interval = 1;
                int log_counter = 0;

                struct timespec sine_ts;
                clock_gettime(CLOCK_MONOTONIC, &sine_ts);

                for (int it = 0; it < total_iters && g_running; ++it) {
                    auto now = std::chrono::steady_clock::now();
                    double t = std::chrono::duration<double>(now - t_start).count();

                    double target = zero_pos + amp_rad * std::sin(2.0 * M_PI * freq * t);
                    double vel_ff = amp_rad * 2.0 * M_PI * freq * std::cos(2.0 * M_PI * freq * t);

                    std::map<motorevo::MotorId, motorevo::RevoMotorCmd_t> cmd;
                    cmd[mid] = {.pos = target, .vel = vel_ff, .torque = 0.0, .kp = kp, .kd = kd};
                    mc->setTargetPositions(cmd);

                    // 采样统计 (跳过前 0.5s 瞬态)
                    if (++log_counter >= log_interval) {
                        log_counter = 0;

                        auto positions = mc->getPositions();
                        double act = positions.count(mid) ? static_cast<double>(positions[mid]) : 0.0;
                        double err_deg = (target - act) * 180.0 / M_PI;

                        if (t > 0.5) {
                            double ae = std::fabs(err_deg);
                            if (ae > max_err) max_err = ae;
                            sum_sq_err += err_deg * err_deg;
                            n_samples++;

                            rec_times.push_back(t);
                            rec_targets.push_back(target);
                            rec_actuals.push_back(act);
                        }

                        // CSV 原始数据
                        if (csv.is_open()) {
                            auto torques = mc->getTorques();
                            auto vels = mc->getVelocities();
                            csv << combo_idx
                                << "," << kp << "," << kd << "," << amp_deg << "," << freq
                                << "," << std::fixed << std::setprecision(4) << t
                                << "," << target << "," << act
                                << "," << std::setprecision(3) << err_deg
                                << "," << (vels.count(mid) ? vels[mid] : 0.0f)
                                << "," << (torques.count(mid) ? torques[mid] : 0.0f)
                                << "\n";
                        }
                    }

                    sine_ts.tv_nsec += ctrl_period_ns;
                    if (sine_ts.tv_nsec >= 1000000000L) { sine_ts.tv_sec++; sine_ts.tv_nsec -= 1000000000L; }
                    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &sine_ts, NULL);
                }

                // -- 统计 --
                double rms_err = (n_samples > 0) ? std::sqrt(sum_sq_err / n_samples) : 999.0;
                double phase_ms = estimatePhaseLag(rec_times, rec_targets, rec_actuals, zero_pos);

                TestResult tr = {kp, kd, amp_deg, freq, rms_err, max_err, phase_ms, n_samples};
                results.push_back(tr);

                std::cout << "  -> RMS=" << std::fixed << std::setprecision(3) << rms_err
                          << "° Max=" << max_err << "° Lag=" << std::setprecision(1) << phase_ms << "ms\n";
            }
            if (!g_running) break;
        }
        if (!g_running) break;
    }

    // ============================================================
    //  结果排序 + 汇总
    // ============================================================
    // 按 RMS 误差排序
    std::vector<size_t> sorted_idx(results.size());
    for (size_t i = 0; i < sorted_idx.size(); ++i) sorted_idx[i] = i;
    std::sort(sorted_idx.begin(), sorted_idx.end(),
              [&](size_t a, size_t b) { return results[a].rms_err_deg < results[b].rms_err_deg; });

    std::cout << "\n\033[32m========================================\033[0m\n"
              << "\033[32m  参数自动调优结果 (按 RMS 误差排序)\033[0m\n"
              << "\033[32m========================================\033[0m\n"
              << "测试频率: " << freq << " Hz, 控制频率: " << ctrl_freq << " Hz\n\n";

    std::cout << std::right
              << std::setw(5)  << "排名"
              << std::setw(8)  << "kp"
              << std::setw(8)  << "kd"
              << std::setw(8)  << "amp°"
              << std::setw(12) << "RMS误差°"
              << std::setw(12) << "最大误差°"
              << std::setw(12) << "相位lag_ms"
              << std::endl;
    std::cout << std::string(65, '=') << std::endl;

    int show_count = std::min(static_cast<int>(results.size()), 20);
    for (int rank = 0; rank < show_count; ++rank) {
        const auto& r = results[sorted_idx[rank]];
        bool is_best = (rank == 0);
        if (is_best) std::cout << "\033[33m";
        std::cout << std::fixed
                  << std::setw(5) << (rank + 1)
                  << std::setw(8) << std::setprecision(0) << r.kp
                  << std::setw(8) << std::setprecision(1) << r.kd
                  << std::setw(8) << std::setprecision(0) << r.amp_deg
                  << std::setw(12) << std::setprecision(3) << r.rms_err_deg
                  << std::setw(12) << std::setprecision(3) << r.max_err_deg
                  << std::setw(12) << std::setprecision(1) << r.phase_lag_ms;
        if (is_best) std::cout << "  <-- BEST\033[0m";
        std::cout << std::endl;
    }
    std::cout << std::string(65, '=') << std::endl;

    // 最优参数
    if (!results.empty()) {
        const auto& best = results[sorted_idx[0]];
        std::cout << "\n\033[32m最优参数:\033[0m\n"
                  << "  kp = " << best.kp << "\n"
                  << "  kd = " << best.kd << "\n"
                  << "  振幅 = " << best.amp_deg << "°\n"
                  << "  RMS 误差 = " << std::setprecision(3) << best.rms_err_deg << "°\n"
                  << "  最大误差 = " << best.max_err_deg << "°\n"
                  << "  相位滞后 = " << std::setprecision(1) << best.phase_lag_ms << " ms\n";

        // 输出推荐命令
        std::cout << "\n\033[36m推荐验证命令:\033[0m\n"
                  << "  sudo ./canfd_motor_test --can " << can_name
                  << " --kp " << best.kp << " --kd " << best.kd
                  << " --amp " << best.amp_deg << " --freq " << freq
                  << " --log verify.csv\n";
    }

    // ---- 回零位 ----
    std::cout << "\n回到零位..." << std::endl;
    {
        std::map<motorevo::MotorId, motorevo::RevoMotorCmd_t> cmd;
        cmd[mid] = {.pos = zero_pos, .vel = 0.0, .torque = 0.0, .kp = 100.0, .kd = 2.0};
        mc->setTargetPositions(cmd);
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        for (int t = 0; t < ctrl_freq * 2; ++t) {
            ts.tv_nsec += ctrl_period_ns;
            if (ts.tv_nsec >= 1000000000L) { ts.tv_sec++; ts.tv_nsec -= 1000000000L; }
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);
        }
    }

    if (csv.is_open()) {
        csv.close();
        std::cout << "\033[32mCSV 已保存: " << log_file << "\033[0m\n";
    }

    delete g_loop;
    g_loop = nullptr;
    mc.reset();
    std::this_thread::sleep_for(std::chrono::seconds(1));

    std::cout << "\033[32m调优完成\033[0m\n";
    return 0;
}
