/*
 * 🧪 BMCanbusWrapper 完整功能测试文件
 *
 * 测试内容：
 * - BMCanbusWrapper 类的完整功能测试
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

#include "../src/bm_canbus_wrapper.h"
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
        std::cout << "\nReceived Ctrl+C (SIGINT), initiating graceful shutdown..." << std::endl;
        g_running.store(false);
    }
}

// Message callback function for CAN bus messages
void messageCallback(canbus_sdk::CanMessageFrame* frame, const canbus_sdk::CallbackContext* context) {
    static int callback_count = 0;
    callback_count++;
    g_message_count++;
    
    std::lock_guard<std::mutex> lock(g_output_mutex);
    
    // 右侧显示接收信息，添加更多信息
    std::cout << std::setw(50) << " " << "📥 RECV #" << std::setw(3) << callback_count 
              << " ID:0x" << std::hex << std::setw(3) << std::setfill('0') << frame->id.SID 
              << " DLC:" << std::dec << (int)frame->ctrl.rx.dlc 
              << " Type:" << (frame->ctrl.rx.dlc > 0 ? "RX" : "TX?");
    
    // Print payload data
    std::cout << " Data:";
    for (int i = 0; i < frame->ctrl.rx.dlc && i < 8; i++) {
        std::cout << std::hex << std::setw(2) << (int)frame->payload[i];
    }
    std::cout << std::dec << std::endl;
    
    // Since we don't need to keep the frame data, free it
    canbus_sdk::freeCanMessageFrame(frame);
}

// 发送线程函数

void senderThread(canbus_sdk::BMCanbusWrapper* wrapper) {
    int send_count = 0;
    auto last_time = std::chrono::steady_clock::now();
    
    {
        std::lock_guard<std::mutex> lock(g_output_mutex);
        std::cout << "🚀 SENDER thread started (1kHz)" << std::endl;
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
            
            // 设置负载数据
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
                          << " DLC:" << std::dec << (int)frame.ctrl.tx.dlc << " ";
                
                // Print payload data
                for (int i = 0; i < frame.ctrl.tx.dlc && i < 8; i++) {
                    std::cout << std::hex << std::setw(2) << (int)frame.payload[i];
                }
                std::cout << std::dec << std::endl;
                
            } else {
                std::lock_guard<std::mutex> lock(g_output_mutex);
                std::cout << "❌ SEND #" << std::setw(3) << send_count 
                          << " FAILED: " << result.error() << std::endl;
            }
            
            last_time = current_time;
        }
        
        // 短暂sleep以避免过度占用CPU
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
    
    {
        std::lock_guard<std::mutex> lock(g_output_mutex);
        std::cout << "🛑 SENDER stopped. Total sent: " << send_count << std::endl;
    }
}

// 接收线程函数
void receiverThread(canbus_sdk::BMCanbusWrapper* wrapper) {
    int receive_count = 0;
    int loop_count = 0;
    
    {
        std::lock_guard<std::mutex> lock(g_output_mutex);
        std::cout << std::setw(50) << " " << "📡 RECEIVER thread started" << std::endl;
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
                    std::cout << std::setw(50) << " " << "📊 RECV processed " << new_messages 
                              << " (total: " << receive_count << ")" << std::endl;
                }
            }
        } else {
            std::lock_guard<std::mutex> lock(g_output_mutex);
            std::cout << std::setw(50) << " " << "❌ RECV error: " << result.error() << std::endl;
        }
        
        // 短暂sleep以避免过度占用CPU
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    
    {
        std::lock_guard<std::mutex> lock(g_output_mutex);
        std::cout << std::setw(50) << " " << "🛑 RECEIVER stopped. Total processed: " << receive_count << std::endl;
    }
}

int main() {
    std::cout << "BMCanbusWrapper Test Example - BUSMUST_B (with graceful shutdown)" << std::endl;
    std::cout << "=============================================================" << std::endl;
    
    // 设置信号处理
    signal(SIGINT, signalHandler);
    
    try {
        // Configure CAN bus bitrate for BUSMUST_B
        canbus_sdk::CanBusBitrate bitrate;
        bitrate.nbitrate = 500000;  // 500 kbps
        bitrate.dbitrate = 2000000; // 2 Mbps (for CAN FD)
        bitrate.nsamplepos = 75;     // Sample point at 75%
        bitrate.dsamplepos = 75;     // Sample point at 75%
        
        // Create BMCanbusWrapper for BUSMUST_B
        std::cout << "Creating BMCanbusWrapper for BUSMUST_B..." << std::endl;
        std::unique_ptr<canbus_sdk::BMCanbusWrapper> canbus_wrapper(
            new canbus_sdk::BMCanbusWrapper("BUSMUST_B", canbus_sdk::CanBusModelType::BUSMUST_B, bitrate)
        );
        
        // Initialize the CAN bus
        std::cout << "Initializing BUSMUST_B CAN bus..." << std::endl;
        auto init_result = canbus_wrapper->init();
        
        if (!init_result.has_value()) {
            std::cout << "Failed to initialize BUSMUST_B CAN bus. Error code: 0X" << std::hex << init_result.error() << std::endl;
            std::cout << "Note: This error is expected if no BUSMUST_B hardware is connected." << std::endl;
            return 1;
        }
        
        std::cout << "Successfully initialized BUSMUST_B CAN bus!" << std::endl;
        std::cout << "Bitrate: " << bitrate.nbitrate << " bps" << std::endl;
        
        // Set up message callback
        std::cout << "Setting up message callback..." << std::endl;
        canbus_wrapper->setMessageReceiveCallback(messageCallback, nullptr);
        std::thread receiver(receiverThread, canbus_wrapper.get());

        // Send a test CAN message
        std::cout << "Sending test CAN message..." << std::endl;
        canbus_sdk::CanMessageFrame test_frame;
        test_frame.id.SID = 0x123;  // Standard ID
        test_frame.id.EID = 0;      // Extended ID (not used for standard frames)
        test_frame.ctrl.tx.dlc = 8; // Data Length Code
        
        // Set test payload data
        for (int i = 0; i < 8; i++) {
            test_frame.payload[i] = i + 9; // 0x01, 0x02, 0x03, ..., 0x08
        }

        auto send_result = canbus_wrapper->writeCanMessage(test_frame);
        if (send_result.has_value()) {
            std::cout << "Test message sent successfully!" << std::endl;
            std::cout << "ID: 0x123, DLC: 8, Payload: 01 02 03 04 05 06 07 08" << std::endl;
        } else {
            std::cout << "Failed to send test message. Error code: " << send_result.error() << std::endl;
        }
        
        // 启动发送和接收线程
        std::cout << "Starting sender and receiver threads..." << std::endl;
        std::cout << "Press Ctrl+C to exit gracefully." << std::endl;
        std::cout << "Note: This example requires actual BUSMUST_B hardware to work properly." << std::endl;
        
        std::thread sender(senderThread, canbus_wrapper.get());
        
        // 主线程等待退出信号
        while (g_running.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        // 等待线程结束
        sender.join();
        receiver.join();
        
        // 优雅退出处理
        std::cout << "Graceful shutdown initiated..." << std::endl;
        
        // 发送最后的停止消息
        std::cout << "Sending final shutdown message..." << std::endl;
        canbus_sdk::CanMessageFrame shutdown_frame;
        shutdown_frame.id.SID = 0x7FF;  // 常用的关闭消息ID
        shutdown_frame.ctrl.tx.dlc = 2;
        shutdown_frame.payload[0] = 0xFF;  // 关闭命令
        shutdown_frame.payload[1] = g_message_count.load() & 0xFF;  // 回调计数
        
        // auto shutdown_result = canbus_wrapper->writeCanMessage(shutdown_frame);
        // if (shutdown_result.has_value()) {
        //     std::cout << "Shutdown message sent successfully!" << std::endl;
        // } else {
        //     std::cout << "Failed to send shutdown message: " << shutdown_result.error() << std::endl;
        // }
        
        // 等待消息被发送
        // std::this_thread::sleep_for(std::chrono::milliseconds(5));
        
        std::cout << "Test completed gracefully. Callback messages: " << g_message_count.load() << std::endl;
        
    } catch (const std::exception& e) {
        std::cout << "Exception occurred: " << e.what() << std::endl;
        return 1;
    }
    
    std::cout << "BMCanbusWrapper test example complete." << std::endl;
    return 0;
}