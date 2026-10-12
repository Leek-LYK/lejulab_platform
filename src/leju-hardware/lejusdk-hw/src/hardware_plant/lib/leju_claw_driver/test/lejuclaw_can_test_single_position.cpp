#include "lejuclaw_can_customed.h"

#include <csignal>
#include <iostream>
#include <thread>
#include <chrono>
#include <vector>

using namespace lejuclaw_can;

// 目标位置配置
const double POSITION_1 = 0.0;                     // 第一个目标位置 (20%)
const double POSITION_2 = 100.0;                      // 第二个目标位置 (80%)
const double EXIT_POSITION = 50.0;                   // 退出位置 (50%)

// 全局状态变量
static volatile std::sig_atomic_t g_stop = 0;

void signal_handler(int sig) {
    (void)sig;
    g_stop = 1;
    std::cout << "[lejuclaw_can_test] 接收到中断信号，启动退出程序..." << std::endl;
}

static void print_vector(const char* name, const std::vector<double>& v) {
    std::cout << name << ": [";
    for (size_t i = 0; i < v.size(); ++i) {
        std::cout << v[i];
        if (i + 1 < v.size()) std::cout << ", ";
    }
    std::cout << "]\n";
}

int main(int argc, char** argv) {
    (void)argc; (void)argv;
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    LeJuClawCan can;
    std::cout << "[lejuclaw_can_test] initializing..." << std::endl;
    if (can.initialize() != 0) {
        std::cerr << "[lejuclaw_can_test] initialize failed" << std::endl;
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
        std::cerr << "[lejuclaw_can_test] no motors detected" << std::endl;
        return 0;
    }

    std::cout << "[lejuclaw_can_test] 系统初始化成功！" << std::endl;
    std::cout << "[lejuclaw_can_test] 配置信息:" << std::endl;
    std::cout << "  • 目标位置1: " << POSITION_1 << "%" << std::endl;
    std::cout << "  • 目标位置2: " << POSITION_2 << "%" << std::endl;
    std::cout << "  • 退出位置: " << EXIT_POSITION << "%" << std::endl;
    std::cout << "[lejuclaw_can_test] 按 Ctrl+C 退出程序（运动到退出位置 " << EXIT_POSITION << "%）" << std::endl;

    // 准备控制参数
    std::vector<double> zero(pos.size(), 0.0);
    
    // 当前目标位置
    std::vector<double> current_target(pos.size(), POSITION_1);
    bool moving_to_position_2 = true;
    int round_count = 0;  // 轮次计数

    std::cout << "\n[lejuclaw_can_test] 开始位置控制循环（非VR模式，等待完成）" << std::endl;

    while (true) {
        // 检查是否请求退出程序
        if (g_stop) {
            std::cout << "[lejuclaw_can_test] 执行退出程序..." << std::endl;
            
            // 运动到退出位置（50%）
            std::vector<double> exit_target(pos.size(), EXIT_POSITION);
            std::cout << "[lejuclaw_can_test] 发送退出位置命令: " << EXIT_POSITION << "%" << std::endl;
            
            // 持续等待直到夹爪真正到达目标位置
            int exit_attempts = 0;
            const int max_exit_attempts = 10;
            
            while (exit_attempts < max_exit_attempts) {
                auto exit_result = can.move_paw(exit_target, zero, zero, false);  // 非VR模式，等待完成
                
                if (exit_result == LeJuClawCan::PawMoveState::LEFT_REACHED_RIGHT_REACHED) {
                    std::cout << "[lejuclaw_can_test] 夹爪已安全到达退出位置: " << EXIT_POSITION << "%" << std::endl;
                    std::cout << "[lejuclaw_can_test] 正在关闭系统..." << std::endl;
                    can.close();
                    std::cout << "[lejuclaw_can_test] 系统已安全关闭" << std::endl;
                    return 0;
                }
                else if (exit_result == LeJuClawCan::PawMoveState::ERROR) {
                    std::cerr << "[lejuclaw_can_test] 退出位置命令执行失败！强制关闭系统..." << std::endl;
                    can.close();
                    return -1;
                }
                else {
                    // 夹爪还在运动中，继续等待
                    exit_attempts++;
                    std::cout << "[lejuclaw_can_test] 等待夹爪到达退出位置... (尝试 " << exit_attempts << "/" << max_exit_attempts << ")" << std::endl;
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
            }
            
            // 如果超过最大尝试次数仍未到达，强制停止
            std::cerr << "[lejuclaw_can_test] 退出超时！强制关闭系统..." << std::endl;
            can.close();
            return -1;
        }
        
        // 发送目标位置并等待夹爪稳定（非VR模式，阻塞直到完成）
        std::cout << "[lejuclaw_can_test] 发送位置命令: " << current_target[0] << "%" << std::endl;
        
        auto result = can.move_paw(current_target, zero, zero, false);  // 非VR模式，等待完成
        
        if (result == LeJuClawCan::PawMoveState::LEFT_REACHED_RIGHT_REACHED) {
            std::cout << "[lejuclaw_can_test] 夹爪已稳定到达目标位置: " << current_target[0] << "%" << std::endl;
            print_vector("positions(%)", can.get_positions());
            std::this_thread::sleep_for(std::chrono::milliseconds(500));  // 0.5秒
        }
        else if (result == LeJuClawCan::PawMoveState::ERROR) {
            std::cerr << "[lejuclaw_can_test] 位置命令执行失败！" << std::endl;
            continue;
        }
        
        // 切换目标位置
        if (moving_to_position_2) {
            current_target.assign(pos.size(), POSITION_2);
        }
        else {
            current_target.assign(pos.size(), POSITION_1);
            // 完成一轮（从位置1到位置2再回到位置1）
            round_count++;
            std::cout << "──────────────────────────────────────────" << std::endl;
            std::cout << "[lejuclaw_can_test] 第 " << round_count << " 轮完成！" << std::endl;
            std::cout << "──────────────────────────────────────────" << std::endl;
        }
        moving_to_position_2 = !moving_to_position_2;
    }

    can.close();
    return 0;
}


