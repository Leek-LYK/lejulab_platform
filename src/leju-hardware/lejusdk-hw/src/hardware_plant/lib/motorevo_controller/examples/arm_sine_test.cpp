/**
 * 手臂电机正弦波跟踪测试程序
 *
 * 使用 MotorevoActuator 控制 bcan2/bcan3 上的手臂和头部电机，
 * 以正弦曲线驱动各关节并实时打印跟踪误差。
 *
 * 用法:
 *   sudo ./arm_sine_test <config.yaml> [选项]
 *
 * 选项:
 *   --amp <度>        正弦振幅 (默认 10°)
 *   --freq <Hz>       正弦频率 (默认 0.3 Hz)
 *   --duration <秒>   测试时长 (默认 30s，0=无限)
 *   --log <文件.csv>  保存 CSV 日志
 *   --no-head         不控制头部电机
 */

#include "motorevo/motorevo_actuator.h"
#include "motorevo/motor_def.h"
#include "canbus_sdk/canbus_sdk.h"

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <thread>
#include <chrono>
#include <cmath>
#include <csignal>
#include <atomic>
#include <iomanip>
#include <time.h>

static std::atomic<bool> g_running(true);

static void signalHandler(int) {
    g_running = false;
}

static void printUsage(const char* prog) {
    std::cerr << "用法: sudo " << prog << " <config.yaml> [选项]\n"
              << "选项:\n"
              << "  --amp <度>        正弦振幅 (默认 10)\n"
              << "  --freq <Hz>       正弦频率 (默认 0.3)\n"
              << "  --duration <秒>   测试时长 (默认 30, 0=无限)\n"
              << "  --log <file.csv>  保存 CSV 日志\n"
              << "  --no-head         不控制头部电机\n";
}

int main(int argc, char* argv[]) {
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    // ---- 解析参数 ----
    if (argc < 2) {
        printUsage(argv[0]);
        return 1;
    }
    std::string config_file = argv[1];
    double amp_deg = 10.0;
    double freq_hz = 0.3;
    double duration_s = 30.0;
    std::string log_file;
    bool control_head = true;

    for (int i = 2; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--amp" && i + 1 < argc) amp_deg = std::stod(argv[++i]);
        else if (arg == "--freq" && i + 1 < argc) freq_hz = std::stod(argv[++i]);
        else if (arg == "--duration" && i + 1 < argc) duration_s = std::stod(argv[++i]);
        else if (arg == "--log" && i + 1 < argc) log_file = argv[++i];
        else if (arg == "--no-head") control_head = false;
    }

    double amp_rad = amp_deg * M_PI / 180.0;
    int ctrl_freq = 250;
    int display_hz = 5;
    int display_interval = ctrl_freq / display_hz;

    std::cout << "\033[32m=== 手臂电机正弦波跟踪测试 ===\033[0m\n"
              << "配置文件: " << config_file << "\n"
              << "振幅: " << amp_deg << "°, 频率: " << freq_hz << " Hz, 时长: "
              << (duration_s > 0 ? std::to_string((int)duration_s) + "s" : "无限") << "\n"
              << "控制头部: " << (control_head ? "是" : "否") << "\n"
              << "按 Ctrl+C 退出\n" << std::endl;

    // ---- 初始化 MotorevoActuator ----
    motorevo::MotorevoActuator actuator(config_file, std::set<std::string>{}, ctrl_freq);

    std::cout << "正在初始化执行器..." << std::endl;
    int ret = actuator.init();
    if (ret != 0) {
        std::cerr << "\033[31m初始化失败 (code=" << ret << ")\033[0m" << std::endl;
        return -1;
    }

    int num_motors = static_cast<int>(actuator.getPositions().size());

    if (num_motors == 0) {
        std::cerr << "\033[31m未发现电机，请检查配置和接线\033[0m" << std::endl;
        actuator.reset();
        return -1;
    }

    // ---- 立即锁定当前位置，防止控制线程用默认 pos=0 驱动电机 ----
    auto init_positions = actuator.getPositions();
    {
        std::vector<motorevo::RevoMotorCmd_t> lock_cmd(num_motors);
        for (int i = 0; i < num_motors; ++i) {
            lock_cmd[i].pos = init_positions[i];
            lock_cmd[i].vel = 0.0;
            lock_cmd[i].torque = 0.0;
            bool is_head = (i >= num_motors - 2);
            lock_cmd[i].kp = is_head ? 10.0 : 20.0;
            lock_cmd[i].kd = 1.0;
        }
        actuator.setTargets(lock_cmd);
    }

    std::cout << "初始化成功，发现 \033[33m" << num_motors << "\033[0m 个电机\n" << std::endl;
    std::cout << "初始位置 (度):" << std::endl;
    for (int i = 0; i < num_motors; ++i) {
        double deg = init_positions[i] * 180.0 / M_PI;
        std::cout << "  电机[" << i << "] = " << std::fixed << std::setprecision(2) << deg << "°" << std::endl;
    }
    std::cout << std::endl;

    // ---- 设置 kp/kd ----
    {
        std::vector<int> indices;
        std::vector<double> kp_vec, kd_vec;
        for (int i = 0; i < num_motors; ++i) {
            indices.push_back(i);
            bool is_head = (i >= num_motors - 2) && control_head;
            kp_vec.push_back(is_head ? 10.0 : 20.0);
            kd_vec.push_back(is_head ? 1.0 : 1.0);
        }
        actuator.set_joint_gains(indices, kp_vec, kd_vec);
        std::cout << "已设置 kp/kd 增益" << std::endl;
    }

    // ---- CSV 日志 ----
    std::ofstream csv;
    if (!log_file.empty()) {
        csv.open(log_file);
        if (csv.is_open()) {
            csv << "time_s";
            for (int i = 0; i < num_motors; ++i) {
                csv << ",M" << i << "_cmd_deg,M" << i << "_act_deg,M" << i << "_err_deg,M" << i << "_tor";
            }
            csv << "\n";
            std::cout << "CSV 日志: " << log_file << std::endl;
        }
    }

    // ---- 正弦波控制循环 ----
    std::cout << "\n\033[36m开始正弦波控制...\033[0m\n" << std::endl;

    // 打印表头
    std::cout << std::left << std::setw(8) << "时间";
    for (int i = 0; i < num_motors; ++i) {
        std::string hdr = "M" + std::to_string(i) + "(目标/实际/误差)";
        std::cout << std::setw(28) << hdr;
    }
    std::cout << std::endl;
    std::cout << std::string(8 + 28 * num_motors, '-') << std::endl;

    const long ctrl_period_ns = 1000000000L / ctrl_freq;
    auto t_start = std::chrono::steady_clock::now();
    int counter = 0;
    struct timespec loop_ts;
    clock_gettime(CLOCK_MONOTONIC, &loop_ts);

    while (g_running) {
        auto now = std::chrono::steady_clock::now();
        double t = std::chrono::duration<double>(now - t_start).count();

        if (duration_s > 0 && t >= duration_s) break;

        // 计算目标
        double sine_val = amp_rad * std::sin(2.0 * M_PI * freq_hz * t);

        std::vector<motorevo::RevoMotorCmd_t> cmd(num_motors);
        for (int i = 0; i < num_motors; ++i) {
            bool is_head = (i >= num_motors - 2);

            // 头部电机: 如果 --no-head 则保持初始位置
            if (is_head && !control_head) {
                cmd[i].pos = init_positions[i];
            } else if (i == 2) {
                // 肘部关节(index 2): 限位约束, 在 [10°, 20°] 范围运动
                constexpr double elbow_center = 15.0 * M_PI / 180.0;
                constexpr double elbow_amp    =  5.0 * M_PI / 180.0;
                double elbow_sine = elbow_amp * std::sin(2.0 * M_PI * freq_hz * t);
                cmd[i].pos = init_positions[i] + elbow_center + elbow_sine;
            } else {
                cmd[i].pos = init_positions[i] + sine_val;
            }
            cmd[i].vel = 0.0;
            cmd[i].torque = 0.0;
            cmd[i].kp = is_head ? 10.0 : 20.0;
            cmd[i].kd = 1.0;
        }

        actuator.setTargets(cmd);

        // 显示 + 日志 (5Hz)
        if (++counter >= display_interval) {
            counter = 0;

            auto act_pos = actuator.getPositions();
            auto act_tor = actuator.getTorques();

            // CSV
            if (csv.is_open()) {
                csv << std::fixed << std::setprecision(4) << t;
                for (int i = 0; i < num_motors; ++i) {
                    double cmd_deg = cmd[i].pos * 180.0 / M_PI;
                    double act_deg = (i < (int)act_pos.size()) ? act_pos[i] * 180.0 / M_PI : 0.0;
                    double err_deg = cmd_deg - act_deg;
                    double tor = (i < (int)act_tor.size()) ? act_tor[i] : 0.0;
                    csv << "," << std::setprecision(2) << cmd_deg
                        << "," << act_deg << "," << err_deg
                        << "," << std::setprecision(3) << tor;
                }
                csv << "\n";
            }

            // 终端
            std::cout << "\r" << std::fixed << std::setprecision(1) << std::setw(6) << t << "s ";
            for (int i = 0; i < num_motors; ++i) {
                double cmd_deg = cmd[i].pos * 180.0 / M_PI;
                double act_deg = (i < (int)act_pos.size()) ? act_pos[i] * 180.0 / M_PI : 0.0;
                double err_deg = cmd_deg - act_deg;

                std::cout << std::fixed << std::setprecision(1)
                          << std::setw(6) << cmd_deg << "/"
                          << std::setw(6) << act_deg << "/"
                          << std::setprecision(2) << std::setw(6) << err_deg << "° ";
            }
            std::cout << std::flush;
        }

        loop_ts.tv_nsec += ctrl_period_ns;
        if (loop_ts.tv_nsec >= 1000000000L) { loop_ts.tv_sec++; loop_ts.tv_nsec -= 1000000000L; }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &loop_ts, NULL);
    }

    // ---- 停止: 回到初始位置 ----
    std::cout << "\n\n\033[33m正在回到初始位置...\033[0m" << std::endl;
    {
        std::vector<motorevo::RevoMotorCmd_t> stop_cmd(num_motors);
        for (int i = 0; i < num_motors; ++i) {
            stop_cmd[i].pos = init_positions[i];
            stop_cmd[i].vel = 0.0;
            stop_cmd[i].torque = 0.0;
            stop_cmd[i].kp = (i >= num_motors - 2) ? 10.0 : 20.0;
            stop_cmd[i].kd = 1.0;
        }
        struct timespec stop_ts;
        clock_gettime(CLOCK_MONOTONIC, &stop_ts);
        for (int t = 0; t < ctrl_freq * 2; ++t) {
            actuator.setTargets(stop_cmd);
            stop_ts.tv_nsec += ctrl_period_ns;
            if (stop_ts.tv_nsec >= 1000000000L) { stop_ts.tv_sec++; stop_ts.tv_nsec -= 1000000000L; }
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &stop_ts, NULL);
        }
    }

    // ---- 清理 ----
    if (csv.is_open()) {
        csv.close();
        std::cout << "CSV 已保存: " << log_file << std::endl;
    }

    actuator.reset();
    canbus_sdk::CanBusController::getInstance().closeAllCanBuses();

    std::cout << "\033[32m测试完成\033[0m" << std::endl;
    return 0;
}
