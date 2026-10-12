#include "lejuclaw.h"
#include <cstdlib>
#include <thread>
#include <vector>
#include <iostream>
#include <experimental/filesystem>
#include <csignal>
#include <cmath>
#include <unistd.h>

namespace fs = std::experimental::filesystem;

// 颜色代码定义
#define COLOR_RESET   "\033[0m"
#define COLOR_GREEN   "\033[32m"
#define COLOR_YELLOW  "\033[33m"
#define COLOR_RED     "\033[31m"

LeJuClaw actuator;

// 目标位置配置
const double POSITION_1 = 0.0;                     // 第一个目标位置
const double POSITION_2 = 100.0;                   // 第二个目标位置
const double EXIT_POSITION = 50.0;                 // 退出位置

// 全局状态变量
bool emergency_stop_requested = false;

void signalHandler(int signum)
{
    std::cout << COLOR_YELLOW << "\n[!] 接收到中断信号 (" << signum << ")，启动退出程序..." << COLOR_RESET << std::endl;
    
    // 设置退出标志
    emergency_stop_requested = true;
    
    std::cout << COLOR_YELLOW << "[!] 夹爪将运动到退出位置 (" << EXIT_POSITION << "%)..." << COLOR_RESET << std::endl;
}

int main()
{
    // 注册信号处理器
    signal(SIGINT, signalHandler);   // Ctrl+C
    signal(SIGTERM, signalHandler);  // 终止信号
    
    bool init_bmapilib = true;
    if (0 != actuator.initialize(init_bmapilib))
    {
        std::cout << COLOR_RED << "[✗] 初始化失败！" << COLOR_RESET << std::endl;
        return -1;
    }
    std::cout << std::endl;
    std::cout << COLOR_GREEN << "╭────────────────────────────────────────────────────────────╮" << COLOR_RESET << std::endl;
    std::cout << COLOR_GREEN << "│                    LEJU 夹爪控制测试程序                   │" << COLOR_RESET << std::endl;
    std::cout << COLOR_GREEN << "╰────────────────────────────────────────────────────────────╯" << COLOR_RESET << std::endl;
    
    std::cout << COLOR_GREEN << "[✓] 系统初始化成功！" << COLOR_RESET << std::endl;
    std::cout << COLOR_YELLOW << "[!] 等待系统稳定..." << COLOR_RESET << std::endl;

    usleep(1000000);

    std::vector<double> velocity = {0.0, 0.0};
    std::vector<double> torque = {0.0, 0.0};
    
    // 当前目标位置
    std::vector<double> current_target = {POSITION_1, POSITION_1};
    bool moving_to_position_2 = true;
    int round_count = 0;  // 轮次计数
    
    std::cout << COLOR_GREEN << "\n[✓] 开始位置发送" << COLOR_RESET << std::endl;
    std::cout << COLOR_YELLOW << "配置信息:" << COLOR_RESET << std::endl;
    std::cout << "  • 目标位置1: " << COLOR_GREEN << POSITION_1 << COLOR_RESET << std::endl;
    std::cout << "  • 目标位置2: " << COLOR_GREEN << POSITION_2 << COLOR_RESET << std::endl;
    
    std::cout << COLOR_YELLOW << "\n[!] 按 Ctrl+C 退出程序（运动到退出位置 " << EXIT_POSITION << "%）" << COLOR_RESET << std::endl;
    
    while (1)
    {
        // 检查是否请求退出程序
        if (emergency_stop_requested)
        {
            std::cout << COLOR_RED << "\n[!] 执行退出程序..." << COLOR_RESET << std::endl;
            
            // 运动到退出位置（80%）
            std::vector<double> exit_target = {EXIT_POSITION, EXIT_POSITION};
            std::cout << COLOR_YELLOW << "[!] 发送退出位置命令: ｛" << exit_target[0] << ", " << exit_target[1] << "｝" << COLOR_RESET << std::endl;
            
            // 持续等待直到夹爪真正到达目标位置
            int exit_attempts = 0;
            const int max_exit_attempts = 10;
            
            while (exit_attempts < max_exit_attempts)
            {
                auto exit_result = actuator.move_paw(exit_target, velocity, torque);
                
                if (exit_result == LeJuClaw::PawMoveState::LEFT_REACHED_RIGHT_REACHED)
                {
                    std::cout << COLOR_GREEN << "[✓] 夹爪已安全到达退出位置: ｛" << exit_target[0] << ", " << exit_target[1] << "｝" << COLOR_RESET << std::endl;
                    
                    // 清零电流并关闭系统
                    std::cout << COLOR_YELLOW << "[!] 清零夹爪电流..." << COLOR_RESET << std::endl;
                    actuator.clear_all_torque();
                    usleep(200000);
                    
                    std::cout << COLOR_GREEN << "[✓] 夹爪电流已清零，正在关闭系统..." << COLOR_RESET << std::endl;
                    actuator.close();
                    std::cout << COLOR_GREEN << "[✓] 系统已安全关闭" << COLOR_RESET << std::endl;
                    return 0; // 直接返回，确保程序退出
                }
                else if (exit_result == LeJuClaw::PawMoveState::ERROR)
                {
                    std::cout << COLOR_RED << "[✗] 退出位置命令执行失败！强制清零电流..." << COLOR_RESET << std::endl;
                    actuator.clear_all_torque();
                    usleep(200000);
                    actuator.close();
                    return -1; // 直接返回错误码
                }
                else
                {
                    // 夹爪还在运动中，继续等待
                    exit_attempts++;
                    std::cout << COLOR_YELLOW << "[!] 等待夹爪到达退出位置... (尝试 " << exit_attempts << "/" << max_exit_attempts << ")" << COLOR_RESET << std::endl;
                    usleep(10000);
                }
            }
            
            // 如果超过最大尝试次数仍未到达，强制停止
            std::cout << COLOR_RED << "[✗] 退出超时！强制清零电流并关闭系统..." << COLOR_RESET << std::endl;
            actuator.clear_all_torque();
            usleep(200000);
            actuator.close();
            return -1;
        }
        
        // 发送目标位置并等待夹爪稳定
        std::cout << COLOR_YELLOW << "[!] 发送位置命令: ｛" << current_target[0] << ", " << current_target[1] << "｝" << COLOR_RESET << std::endl;
        
        // 在发送位置命令前检查信号状态
        if (signal(SIGINT, SIG_IGN) != SIG_IGN) {
            signal(SIGINT, signalHandler);
        }
        
        auto result = actuator.move_paw(current_target, velocity, torque);
        
        if (result == LeJuClaw::PawMoveState::LEFT_REACHED_RIGHT_REACHED)
        {
            std::cout << COLOR_GREEN << "[✓] 夹爪已稳定到达目标位置: ｛" << current_target[0] << ", " << current_target[1] << "｝" << COLOR_RESET << std::endl;
            usleep(500000);     // 0.5秒
        }
        else if (result == LeJuClaw::PawMoveState::ERROR)
        {
            std::cout << COLOR_RED << "[✗] 位置命令执行失败！" << COLOR_RESET << std::endl;
            continue;
        }
        
        // 切换目标位置
        if (moving_to_position_2)
        {
            current_target = {POSITION_2, POSITION_2};
        }
        else
        {
            current_target = {POSITION_1, POSITION_1};
            // 完成一轮（从位置1到位置2再回到位置1）
            round_count++;
            std::cout << COLOR_YELLOW << "──────────────────────────────────────────" << COLOR_RESET << std::endl;
            std::cout << COLOR_GREEN << "[✓] 第 " << round_count << " 轮完成！" << COLOR_RESET << std::endl;
            std::cout << COLOR_YELLOW << "──────────────────────────────────────────" << COLOR_RESET << std::endl;
        }
        moving_to_position_2 = !moving_to_position_2;
    }
    actuator.close();
    return 0;
}
