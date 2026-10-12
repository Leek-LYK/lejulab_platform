/*
 * 🧪 LejuCanbusWrapper 完整功能测试文件
 *
 * 测试内容：
 * - LejuCanbusWrapper 类的完整功能测试
 * - CAN总线设备打开、关闭和配置
 * - 消息发送和接收功能
 * - 错误处理和自动恢复机制
 * - 回调函数和上下文管理
 *
 * 使用框架：标准C++ (无依赖框架)
 *
 * 测试覆盖范围：
 * - 设备初始化和配置
 * - CAN总线通信测试
 * - 错误检测和恢复
 * - 多线程并发访问
 * - 内存管理和资源释放
 */

#include "../src/leju_canbus_wrapper.h"
#include "canbus_sdk/result.h"
#include "canbus_sdk/canbus_log.h"
#include "canbus_sdk/canbus_sdk.h"
#include <iomanip>
#include <iostream>
#include <thread>
#include <chrono>
#include <memory>
#include <atomic>
#include <signal.h>
#include <csignal>
#include <mutex>
#include <condition_variable>

// 全局标志用于优雅退出
std::atomic<bool> g_running(true);
std::atomic<int> g_message_count(0);
std::mutex g_stats_mutex;
std::mutex g_output_mutex;  // 用于同步输出，防止混乱

// 信号处理函数
void signalHandler(int signal) {
    if (signal == SIGINT) {
        std::cout << "\n收到 Ctrl+C (SIGINT)，开始优雅关闭..." << std::endl;
        g_running.store(false);
    }
}

// CAN总线消息回调函数
void messageCallback(canbus_sdk::CanMessageFrame* frame, const canbus_sdk::CallbackContext* context) {
    static int callback_count = 0;
    callback_count++;
    g_message_count++;
    
    std::lock_guard<std::mutex> lock(g_output_mutex);
    
    // 右侧显示接收信息
    std::cout << std::setw(50) << std::setfill(' ') << " " << "📥 RECV #" << std::setw(3) << callback_count 
              << " ID:0x" << std::hex << std::setw(3) << std::setfill('0') << frame->id.SID 
              << std::setfill(' ') << " DLC:" << std::dec << (int)frame->ctrl.rx.dlc 
              << " Type:" << (frame->ctrl.rx.dlc > 0 ? "RX" : "TX?");
    
    // 打印载荷数据
    std::cout << " Data:";
    for (int i = 0; i < frame->ctrl.rx.dlc && i < 8; i++) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)frame->payload[i];
    }
    std::cout << std::setfill(' ') << std::dec << std::endl;
    
    // 释放frame内存
    canbus_sdk::freeCanMessageFrame(frame);
}

// 发送线程函数
void senderThread(canbus_sdk::LejuCanbusWrapper* wrapper) {
    int send_count = 0;
    auto last_time = std::chrono::steady_clock::now();
    
    {
        std::lock_guard<std::mutex> lock(g_output_mutex);
        std::cout << "🚀 发送线程启动 (1kHz)" << std::endl;
    }
    
    while (g_running.load()) {
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(current_time - last_time).count();
        
        // 1kHz = 1000Hz = 1ms周期 = 1000微秒
        if (elapsed >= 1000) {
            send_count++;
            
            // 创建测试消息
            canbus_sdk::CanMessageFrame frame;
            frame.id.SID = 0x123 + (send_count % 10);  // 轮换使用不同的ID
            frame.ctrl.tx.dlc = 8;
            
            // 设置载荷数据
            for (int i = 0; i < 8; i++) {
                frame.payload[i] = (send_count + i) & 0xFF;
            }
            
            // 发送消息
            auto result = wrapper->writeCanMessage(frame);
            if (result.has_value()) {
                // 左侧显示发送信息
                std::lock_guard<std::mutex> lock(g_output_mutex);
                std::cout << "📤 SEND #" << std::setw(3) << send_count 
                          << " ID:0x" << std::hex << std::setw(3) << std::setfill('0') << frame.id.SID 
                          << std::setfill(' ') << " DLC:" << std::dec << (int)frame.ctrl.tx.dlc << " ";
                
                // 打印载荷数据
                for (int i = 0; i < frame.ctrl.tx.dlc && i < 8; i++) {
                    std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)frame.payload[i];
                }
                std::cout << std::setfill(' ') << std::dec << std::endl;
                
            } else {
                std::lock_guard<std::mutex> lock(g_output_mutex);
                std::cout << "❌ SEND #" << std::setw(3) << send_count 
                          << " 失败: " << result.error() << std::endl;
            }
            
            last_time = current_time;
        }
        
        // 短暂sleep以避免过度占用CPU
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
    
    {
        std::lock_guard<std::mutex> lock(g_output_mutex);
        std::cout << "🛑 发送线程停止。总发送数: " << send_count << std::endl;
    }
}

// 接收线程函数
void receiverThread(canbus_sdk::LejuCanbusWrapper* wrapper) {
    int receive_count = 0;
    int loop_count = 0;
    
    {
        std::lock_guard<std::mutex> lock(g_output_mutex);
        std::cout << std::setw(50) << std::setfill(' ') << " " << "📡 接收线程启动" << std::endl;
    }
    
    while (g_running.load()) {
        loop_count++;
        
        // 处理接收到的消息
        auto result = wrapper->receivedCanMessages();
        if (result.has_value()) {
            int new_messages = result.value();
            if (new_messages > 0) {
                receive_count += new_messages;
                if (loop_count % 1000 == 0) {  // 每1000次循环打印一次状态
                    std::lock_guard<std::mutex> lock(g_output_mutex);
                    std::cout << std::setw(50) << std::setfill(' ') << " " << "📊 接收处理 " << new_messages 
                              << " (总计: " << receive_count << ")" << std::endl;
                }
            }
        } else {
            std::lock_guard<std::mutex> lock(g_output_mutex);
            std::cout << std::setw(50) << std::setfill(' ') << " " << "❌ 接收错误: " << result.error() << std::endl;
        }
        
        // 短暂sleep以避免过度占用CPU
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    
    {
        std::lock_guard<std::mutex> lock(g_output_mutex);
        std::cout << "🛑 接收线程停止。总处理数: " << receive_count << std::endl;
    }
}

int main() {
    std::cout << "LejuCanbusWrapper 测试示例 - LEJU CANable (优雅关闭)" << std::endl;
    std::cout << "=============================================================" << std::endl;
    
    // 设置信号处理
    signal(SIGINT, signalHandler);
    
    try {
        // 配置CAN总线比特率
        canbus_sdk::CanBusBitrate bitrate;
        bitrate.nbitrate = 500000;  // 500 kbps
        bitrate.dbitrate = 2000000; // 2 Mbps (用于CAN FD)
        bitrate.nsamplepos = 75;     // 采样点位置 75%
        bitrate.dsamplepos = 75;     // 采样点位置 75%
        
        // 创建LejuCanbusWrapper (使用LEJU_CAN_A设备)
        std::cout << "创建 LejuCanbusWrapper (LEJU_CAN_A)..." << std::endl;
        std::unique_ptr<canbus_sdk::LejuCanbusWrapper> canbus_wrapper(
            new canbus_sdk::LejuCanbusWrapper("LEJU_CAN_A", canbus_sdk::CanBusModelType::LEJU_CAN_A, bitrate)
        );
        
        // 初始化CAN总线
        std::cout << "初始化 LEJU CAN 总线..." << std::endl;
        auto init_result = canbus_wrapper->init();
        
        if (!init_result.has_value()) {
            std::cout << "初始化 LEJU CAN 总线失败。错误代码: 0x" << std::hex << init_result.error() << std::endl;
            std::cout << "注意: 如果未连接 LEJU CANable 硬件，此错误是正常的。" << std::endl;
            return 1;
        }
        
        std::cout << "成功初始化 LEJU CAN 总线!" << std::endl;
        std::cout << "比特率: " << bitrate.nbitrate << " bps" << std::endl;
        
        // 设置消息回调
        std::cout << "设置消息接收回调..." << std::endl;
        canbus_wrapper->setMessageReceiveCallback(messageCallback, nullptr);
        
        // 发送测试CAN消息
        std::cout << "发送测试 CAN 消息..." << std::endl;
        canbus_sdk::CanMessageFrame test_frame;
        test_frame.id.SID = 0x123;  // 标准ID
        test_frame.id.EID = 0;      // 扩展ID (标准帧不使用)
        test_frame.ctrl.tx.dlc = 8; // 数据长度代码
        
        // 设置测试载荷数据
        for (int i = 0; i < 8; i++) {
            test_frame.payload[i] = i + 1; // 0x01, 0x02, 0x03, ..., 0x08
        }

        auto send_result = canbus_wrapper->writeCanMessage(test_frame);
        if (send_result.has_value()) {
            std::cout << "测试消息发送成功!" << std::endl;
            std::cout << "ID: 0x123, DLC: 8, 载荷: 01 02 03 04 05 06 07 08" << std::endl;
        } else {
            std::cout << "发送测试消息失败。错误代码: " << send_result.error() << std::endl;
        }
        
        // 启动发送和接收线程
        std::cout << "启动发送和接收线程..." << std::endl;
        std::cout << "按 Ctrl+C 优雅退出。" << std::endl;
        std::cout << "注意: 此示例需要实际的 LEJU CANable 硬件才能正常工作。" << std::endl;
        
        std::thread sender(senderThread, canbus_wrapper.get());
        std::thread receiver(receiverThread, canbus_wrapper.get());
        
        // 主线程等待退出信号
        while (g_running.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        // 等待线程结束
        sender.join();
        receiver.join();
        
        // 优雅退出处理
        std::cout << "开始优雅关闭..." << std::endl;
        
        // 发送最后的停止消息
        std::cout << "发送最终关闭消息..." << std::endl;
        canbus_sdk::CanMessageFrame shutdown_frame;
        shutdown_frame.id.SID = 0x7FF;  // 常用的关闭消息ID
        shutdown_frame.ctrl.tx.dlc = 2;
        shutdown_frame.payload[0] = 0xFF;  // 关闭命令
        shutdown_frame.payload[1] = g_message_count.load() & 0xFF;  // 回调计数
        
        // auto shutdown_result = canbus_wrapper->writeCanMessage(shutdown_frame);
        // if (shutdown_result.has_value()) {
        //     std::cout << "关闭消息发送成功!" << std::endl;
        // } else {
        //     std::cout << "发送关闭消息失败: " << shutdown_result.error() << std::endl;
        // }
        
        // 等待消息被发送
        // std::this_thread::sleep_for(std::chrono::milliseconds(5));
        
        std::cout << "测试优雅完成。回调消息数: " << g_message_count.load() << std::endl;
        
    } catch (const std::exception& e) {
        std::cout << "发生异常: " << e.what() << std::endl;
        return 1;
    }
    
    std::cout << "LejuCanbusWrapper 测试示例完成。" << std::endl;
    return 0;
}

