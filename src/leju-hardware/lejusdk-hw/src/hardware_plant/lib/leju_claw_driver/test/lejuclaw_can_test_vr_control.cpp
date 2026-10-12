#include "lejuclaw_can_customed.h"

#include <csignal>
#include <iostream>
#include <thread>
#include <chrono>
#include <vector>
#include <cmath>

using namespace lejuclaw_can;

// ========== 配置参数 ==========
const double MIN_POSITION = 0.0;            // 最小目标位置 (%)
const double MAX_POSITION = 100.0;          // 最大目标位置 (%)
const double TIME_MIN_TO_MAX = 2.0;         // 从最小到最大的时间 (秒)
const double TIME_MAX_TO_MIN = 2.0;         // 从最大到最小的时间 (秒)
const int CONTROL_FREQUENCY_HZ = 100;       // 控制频率 (Hz)
const double CONTROL_INTERVAL_MS = 1000.0 / CONTROL_FREQUENCY_HZ;  // 控制间隔 (ms)
// ==============================

static volatile std::sig_atomic_t g_stop = 0;

void signal_handler(int sig) {
    (void)sig;
    g_stop = 1;
    std::cout << "\n[lejuclaw_can_test_vr] 接收到中断信号，正在退出..." << std::endl;
}

static void print_vector(const char* name, const std::vector<double>& v) {
    std::cout << name << ": [";
    for (size_t i = 0; i < v.size(); ++i) {
        std::cout << v[i];
        if (i + 1 < v.size()) std::cout << ", ";
    }
    std::cout << "]\n";
}

// 根据时间计算目标位置（线性插值）
double calculate_target_position(double elapsed_time, double min_pos, double max_pos, 
                                 double time_min_to_max, double time_max_to_min) {
    double total_cycle_time = time_min_to_max + time_max_to_min;
    double cycle_time = std::fmod(elapsed_time, total_cycle_time);
    
    if (cycle_time <= time_min_to_max) {
        // 从最小到最大：线性插值
        double t = cycle_time / time_min_to_max;
        return min_pos + (max_pos - min_pos) * t;
    } else {
        // 从最大到最小：线性插值
        double t = (cycle_time - time_min_to_max) / time_max_to_min;
        return max_pos - (max_pos - min_pos) * t;
    }
}

int main(int argc, char** argv) {
    (void)argc; (void)argv;
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    LeJuClawCan can;
    std::cout << "[lejuclaw_can_test_vr] initializing..." << std::endl;
    if (can.initialize() != 0) {
        std::cerr << "[lejuclaw_can_test_vr] initialize failed" << std::endl;
        return -1;
    }

    // 读取一次当前状态
    auto pos = can.get_positions();
    auto vel = can.get_velocity();
    auto tor = can.get_torque();
    print_vector("positions(%)", pos);
    print_vector("velocity(rad/s)", vel);
    print_vector("torque", tor);

    if (pos.empty()) {
        std::cerr << "[lejuclaw_can_test_vr] no motors detected" << std::endl;
        return 0;
    }

    // 显示配置信息
    std::cout << "\n[lejuclaw_can_test_vr] ========== 配置参数 ==========" << std::endl;
    std::cout << "  最小目标位置: " << MIN_POSITION << "%" << std::endl;
    std::cout << "  最大目标位置: " << MAX_POSITION << "%" << std::endl;
    std::cout << "  最小→最大时间: " << TIME_MIN_TO_MAX << " 秒" << std::endl;
    std::cout << "  最大→最小时间: " << TIME_MAX_TO_MIN << " 秒" << std::endl;
    std::cout << "  控制频率: " << CONTROL_FREQUENCY_HZ << " Hz" << std::endl;
    std::cout << "  控制间隔: " << CONTROL_INTERVAL_MS << " ms" << std::endl;
    std::cout << "[lejuclaw_can_test_vr] ================================" << std::endl;

    // 将夹爪移动到起始位置（最小位置）- 使用非VR模式，等待完成
    std::vector<double> start_pos(pos.size(), MIN_POSITION);
    std::vector<double> zero(pos.size(), 0.0);
    std::cout << "\n[lejuclaw_can_test_vr] 移动到起始位置 " << MIN_POSITION << "% (非VR模式，等待完成)" << std::endl;
    can.move_paw(start_pos, zero, zero, false);  // 非VR模式，阻塞直到完成
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    print_vector("positions(%)", can.get_positions());

    // VR模式平滑运动控制循环
    std::cout << "\n[lejuclaw_can_test_vr] 开始VR模式平滑运动控制循环" << std::endl;
    std::cout << "[lejuclaw_can_test_vr] 按 Ctrl+C 退出程序" << std::endl;
    
    auto start_time = std::chrono::steady_clock::now();
    int cycle_count = 0;
    double last_target = MIN_POSITION;
    
    while (!g_stop) {
        // 计算经过的时间
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            current_time - start_time).count();
        double elapsed_seconds = elapsed / 1000.0;
        
        // 计算当前周期内的秒数
        double total_cycle_time = TIME_MIN_TO_MAX + TIME_MAX_TO_MIN;
        double cycle_time = std::fmod(elapsed_seconds, total_cycle_time);
        
        // 根据时间计算目标位置
        double target_pos = calculate_target_position(
            elapsed_seconds, MIN_POSITION, MAX_POSITION, 
            TIME_MIN_TO_MAX, TIME_MAX_TO_MIN);
        
        // 如果目标位置变化超过阈值，才更新（减少频繁打印）
        if (std::abs(target_pos - last_target) > 1.0 || cycle_count % 50 == 0) {
            std::cout << "[lejuclaw_can_test_vr] 总时间: " << elapsed_seconds << "s, "
                      << "周期内时间: " << cycle_time << "s, "
                      << "目标位置: " << target_pos << "%" << std::endl;
            last_target = target_pos;
        }
        
        // 发布目标位置（VR模式，60ms超时，不阻塞）
        std::vector<double> target(pos.size(), target_pos);
        can.move_paw(target, zero, zero, true);  // VR模式
        
        cycle_count++;
        
        // 按控制频率等待
        std::this_thread::sleep_for(
            std::chrono::milliseconds(static_cast<int>(CONTROL_INTERVAL_MS)));
        
        // 每1000次循环打印一次当前位置（约10秒）
        if (cycle_count % 1000 == 0) {
            print_vector("positions(%)", can.get_positions());
        }
    }

    // 读取 joint map 接口
    auto joint_map = can.get_joint_state();
    std::cout << "\n[lejuclaw_can_test_vr] joint_state size: " << joint_map.size() << std::endl;
    for (const auto& kv : joint_map) {
        std::cout << "  id=" << kv.first
                  << ", pos=" << kv.second.pos
                  << ", vel=" << kv.second.vel
                  << ", tor=" << kv.second.torque
                  << std::endl;
    }

    std::cout << "[lejuclaw_can_test_vr] done" << std::endl;

    can.close();
    return 0;
}


