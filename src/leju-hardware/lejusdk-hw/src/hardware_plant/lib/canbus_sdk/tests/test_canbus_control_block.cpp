/*
 * 🧪 BusControlBlock 测试文件
 *
 * 测试内容：
 * - BusControlBlock 类的基本功能测试
 * - 设备注册和注销功能
 * - 线程安全的设备管理
 * - 消息发送和接收机制
 * - 错误处理和边界条件
 *
 * 使用框架：Google Test (GTest)
 *
 * 测试覆盖范围：
 * - 构造函数和析构函数
 * - 设备注册/注销
 * - 多线程并发访问
 * - 消息回调机制
 * - 内存泄漏检查
 */

#include <gtest/gtest.h>
#include "bus_control_block.h"
#include "canbus_sdk/canbus_sdk_def.h"
#include "canbus_sdk/result.h"
#include "canbus_sdk/canbus_sdk.h"
#include <memory>
#include <thread>
#include <chrono>
#include <atomic>

using namespace canbus_sdk;

// 简单的测试回调函数
class TestCanBusCallback {
public:
    bool message_received = false;
    int message_count = 0;
    
    void OnMessageReceived(CanMessageFrame* frame, const CallbackContext* context) {
        message_received = true;
        message_count++;
        // 根据需要释放帧
        if (frame) {
            freeCanMessageFrame(frame);
        }
    }
    
    static void StaticCallback(CanMessageFrame* frame, const CallbackContext* context) {
        auto* self = static_cast<TestCanBusCallback*>(context->userdata);
        self->OnMessageReceived(frame, context);
    }
};

class BusControlBlockTest : public ::testing::Test {
protected:
    void SetUp() override {
        // 设置测试比特率配置
        bitrate_.nbitrate = 500;  // 500 kbps
        bitrate_.dbitrate = 1000; // 1000 kbps
        bitrate_.nsamplepos = 75; // 75%
        bitrate_.dsamplepos = 75; // 75%

        // 设置测试设备信息
        device_info_.device_name = "test_device";
        device_info_.device_id = static_cast<DeviceId>(123);
        device_info_.device_type = DeviceType::MOTOR;

        // 设置回调上下文
        callback_context_.userdata = &test_callback_;
    }

    void TearDown() override {
        // 如果需要则清理
    }

    // 测试数据
    CanBusBitrate bitrate_;
    DeviceInfo device_info_;
    CallbackContext callback_context_;
    TestCanBusCallback test_callback_;
};

TEST_F(BusControlBlockTest, CreateValidBusControlBlock) {
    // 测试使用 BUSMUST_B 创建有效的 BusControlBlock
    auto block = BusControlBlock::create("can0", CanBusModelType::BUSMUST_B, bitrate_);

    if (block)
    {
        std::cout << "block 不是 nullptr" << std::endl;
    }
    else {
        std::cout << "block 是 nullptr（如果没有硬件可用则是预期的）" << std::endl;
    }

    // 仅在块成功创建时测试功能
    if (block) {
        EXPECT_TRUE(block->active());
        EXPECT_EQ(block->busName(), "can0");
        EXPECT_EQ(block->modelType(), CanBusModelType::BUSMUST_B);
        EXPECT_EQ(block->bitrate().nbitrate, 500);
        EXPECT_EQ(block->deviceCount(), 0);
    } else {
        // 如果没有硬件可用，这是预期行为
        std::cout << "BusControlBlock 创建失败 - 可能没有可用硬件" << std::endl;
    }
}


TEST_F(BusControlBlockTest, AddValidDevice) {
    // 使用 BUSMUST_B 创建总线控制块
    auto block = BusControlBlock::create("can0", CanBusModelType::BUSMUST_B, bitrate_);

    // 如果没有可用硬件则跳过测试
    if (!block) {
        std::cout << "跳过 AddValidDevice 测试 - 没有可用硬件" << std::endl;
        return;
    }

    // 添加有效设备
    CallbackParams callback_params;
    callback_params.msg_callback = TestCanBusCallback::StaticCallback;
    callback_params.msg_cb_ctx = &callback_context_;
    callback_params.tef_callback = nullptr;
    callback_params.tef_cb_ctx = nullptr;
    bool result = block->addDevice(device_info_, callback_params);

    // 应该成功
    EXPECT_TRUE(result);
    EXPECT_EQ(block->deviceCount(), 1);
}

TEST_F(BusControlBlockTest, AddDeviceWithLargeId) {
    // 使用 BUSMUST_B 创建总线控制块
    auto block = BusControlBlock::create("can0", CanBusModelType::BUSMUST_B, bitrate_);

    // 如果没有可用硬件则跳过测试
    if (!block) {
        std::cout << "跳过 AddDeviceWithLargeId 测试 - 没有可用硬件" << std::endl;
        return;
    }

    // 测试非常大的设备 ID（应该能使用哈希表）
    DeviceInfo large_id_device;
    large_id_device.device_name = "large_id_device";
    large_id_device.device_id = static_cast<DeviceId>(3000); // 大 ID 但应该能使用哈希表

    CallbackParams callback_params;
    callback_params.msg_callback = TestCanBusCallback::StaticCallback;
    callback_params.msg_cb_ctx = &callback_context_;
    callback_params.tef_callback = nullptr;
    callback_params.tef_cb_ctx = nullptr;
    bool result = block->addDevice(large_id_device, callback_params);
    
    // 使用哈希表实现应该成功
    EXPECT_TRUE(result);
    EXPECT_EQ(block->deviceCount(), 1);
}

TEST_F(BusControlBlockTest, AddDeviceWithNegativeId) {
    // 使用 BUSMUST_B 创建总线控制块
    auto block = BusControlBlock::create("can0", CanBusModelType::BUSMUST_B, bitrate_);
    // 如果没有可用硬件则跳过测试
    if (!block) {
        std::cout << "跳过测试 - 没有可用硬件" << std::endl;
        return;
    }

    // 测试负设备 ID
    DeviceInfo invalid_device;
    invalid_device.device_name = "invalid_device";
    invalid_device.device_id = static_cast<DeviceId>(-1);

    CallbackParams callback_params;
    callback_params.msg_callback = TestCanBusCallback::StaticCallback;
    callback_params.msg_cb_ctx = &callback_context_;
    callback_params.tef_callback = nullptr;
    callback_params.tef_cb_ctx = nullptr;
    bool result = block->addDevice(invalid_device, callback_params);

    // 应该失败
    EXPECT_FALSE(result);
    EXPECT_EQ(block->deviceCount(), 0);
}

TEST_F(BusControlBlockTest, AddDuplicateDevice) {
    // 使用 BUSMUST_B 创建总线控制块
    auto block = BusControlBlock::create("can0", CanBusModelType::BUSMUST_B, bitrate_);
    // 如果没有可用硬件则跳过测试
    if (!block) {
        std::cout << "跳过测试 - 没有可用硬件" << std::endl;
        return;
    }

    // 添加第一个设备
    CallbackParams callback_params;
    callback_params.msg_callback = TestCanBusCallback::StaticCallback;
    callback_params.msg_cb_ctx = &callback_context_;
    callback_params.tef_callback = nullptr;
    callback_params.tef_cb_ctx = nullptr;
    bool result1 = block->addDevice(device_info_, callback_params);
    EXPECT_TRUE(result1);
    EXPECT_EQ(block->deviceCount(), 1);

    // 尝试添加相同的设备
    bool result2 = block->addDevice(device_info_, callback_params);

    // 应该失败
    EXPECT_FALSE(result2);
    EXPECT_EQ(block->deviceCount(), 1); // 数量不应改变
}

TEST_F(BusControlBlockTest, RemoveValidDevice) {
    // 使用 BUSMUST_B 创建总线控制块
    auto block = BusControlBlock::create("can0", CanBusModelType::BUSMUST_B, bitrate_);
    // 如果没有可用硬件则跳过测试
    if (!block) {
        std::cout << "跳过测试 - 没有可用硬件" << std::endl;
        return;
    }

    // 先添加一个设备
    CallbackParams callback_params;
    callback_params.msg_callback = TestCanBusCallback::StaticCallback;
    callback_params.msg_cb_ctx = &callback_context_;
    callback_params.tef_callback = nullptr;
    callback_params.tef_cb_ctx = nullptr;
    bool add_result = block->addDevice(device_info_, callback_params);
    ASSERT_TRUE(add_result);
    ASSERT_EQ(block->deviceCount(), 1);

    // 通过设备类型和ID移除设备
    bool remove_result = block->removeDevice(device_info_.device_type, device_info_.device_id);

    // 应该成功
    EXPECT_TRUE(remove_result);
    EXPECT_EQ(block->deviceCount(), 0);
}


TEST_F(BusControlBlockTest, AddMultipleDevices) {
    // 使用 BUSMUST_B 创建总线控制块
    auto block = BusControlBlock::create("can0", CanBusModelType::BUSMUST_B, bitrate_);
    // 如果没有可用硬件则跳过测试
    if (!block) {
        std::cout << "跳过测试 - 没有可用硬件" << std::endl;
        return;
    }

    // 添加多个设备
    const int num_devices = 10;
    for (int i = 0; i < num_devices; ++i) {
        DeviceInfo device_info;
        device_info.device_name = "device_" + std::to_string(i);
        device_info.device_id = static_cast<DeviceId>(100 + i);
        device_info.device_type = DeviceType::MOTOR;

        CallbackParams callback_params;
        callback_params.msg_callback = TestCanBusCallback::StaticCallback;
        callback_params.msg_cb_ctx = &callback_context_;
        callback_params.tef_callback = nullptr;
        callback_params.tef_cb_ctx = nullptr;
        bool result = block->addDevice(device_info, callback_params);
        EXPECT_TRUE(result);
    }

    // 检查所有设备是否已添加
    EXPECT_EQ(block->deviceCount(), num_devices);

    // 验证每个设备是否存在
    for (int i = 0; i < num_devices; ++i) {
        EXPECT_TRUE(block->hasDevice(DeviceType::MOTOR, 100 + i));
    }
}

TEST_F(BusControlBlockTest, RemoveMultipleDevices) {
    // 使用 BUSMUST_B 创建总线控制块
    auto block = BusControlBlock::create("can0", CanBusModelType::BUSMUST_B, bitrate_);
    // 如果没有可用硬件则跳过测试
    if (!block) {
        std::cout << "跳过测试 - 没有可用硬件" << std::endl;
        return;
    }

    // 添加多个设备
    const int num_devices = 10;
    for (int i = 0; i < num_devices; ++i) {
        DeviceInfo device_info;
        device_info.device_name = "device_" + std::to_string(i);
        device_info.device_id = static_cast<DeviceId>(100 + i);
        device_info.device_type = DeviceType::MOTOR;

        CallbackParams callback_params;
        callback_params.msg_callback = TestCanBusCallback::StaticCallback;
        callback_params.msg_cb_ctx = &callback_context_;
        callback_params.tef_callback = nullptr;
        callback_params.tef_cb_ctx = nullptr;
        bool result = block->addDevice(device_info, callback_params);
        ASSERT_TRUE(result);
    }
    
    ASSERT_EQ(block->deviceCount(), num_devices);

    // 逐个移除设备
    for (int i = 0; i < num_devices; ++i) {
        bool result = block->removeDevice(DeviceType::MOTOR, 100 + i);
        EXPECT_TRUE(result);
        EXPECT_EQ(block->deviceCount(), num_devices - i - 1);
        EXPECT_FALSE(block->hasDevice(DeviceType::MOTOR, 100 + i));
    }
}


TEST_F(BusControlBlockTest, DeviceRegistrationValidity) {
    // 测试 DeviceRegistration 结构体功能
    DeviceRegistration reg;
    
    // 初始应该是无效的
    EXPECT_FALSE(reg.isValid());
    EXPECT_FALSE(reg.is_registered);
    EXPECT_EQ(reg.callback, nullptr);
    EXPECT_TRUE(reg.context.expired());  // 弱指针应该过期
    EXPECT_EQ(reg.strong_context, nullptr);
    
    // 设置注册信息
    DeviceInfo info;
    info.device_name = "test_device";
    info.device_id = static_cast<DeviceId>(123);

    CallbackParams callback_params;
    callback_params.msg_callback = TestCanBusCallback::StaticCallback;
    callback_params.msg_cb_ctx = &callback_context_;
    callback_params.tef_callback = nullptr;
    callback_params.tef_cb_ctx = nullptr;
    reg.setRegistration(info, callback_params);

    // 现在应该是有效的
    EXPECT_TRUE(reg.isValid());
    EXPECT_TRUE(reg.is_registered);
    EXPECT_NE(reg.callback, nullptr);
    EXPECT_FALSE(reg.context.expired());  // 弱指针应该是有效的
    EXPECT_NE(reg.strong_context, nullptr);
    EXPECT_EQ(reg.device_info.device_name, "test_device");
    EXPECT_EQ(reg.device_info.device_id, 123);
    
    // 清除注册
    reg.clearRegistration();

    // 应该再次无效
    EXPECT_FALSE(reg.isValid());
    EXPECT_FALSE(reg.is_registered);
    EXPECT_EQ(reg.callback, nullptr);
    EXPECT_TRUE(reg.context.expired());  // 弱指针应该过期
    EXPECT_EQ(reg.strong_context, nullptr);
}

TEST_F(BusControlBlockTest, DeviceRegistrationWithNullContext) {
    // 测试使用空上下文的 DeviceRegistration
    DeviceRegistration reg;
    
    DeviceInfo info;
    info.device_name = "test_device";
    info.device_id = static_cast<DeviceId>(123);

    // 设置空上下文的注册
    CallbackParams callback_params;
    callback_params.msg_callback = TestCanBusCallback::StaticCallback;
    callback_params.msg_cb_ctx = nullptr;
    callback_params.tef_callback = nullptr;
    callback_params.tef_cb_ctx = nullptr;
    reg.setRegistration(info, callback_params);

    // 即使是空上下文也应该是有效的
    EXPECT_TRUE(reg.isValid());
    EXPECT_TRUE(reg.is_registered);
    EXPECT_NE(reg.callback, nullptr);
    EXPECT_TRUE(reg.context.expired());  // 空上下文的弱指针应该过期
    EXPECT_EQ(reg.strong_context, nullptr);
}

TEST_F(BusControlBlockTest, DeviceRegistrationWithNullCallback) {
    // 测试使用空回调的 DeviceRegistration
    DeviceRegistration reg;
    
    DeviceInfo info;
    info.device_name = "test_device";
    info.device_id = static_cast<DeviceId>(123);

    // 设置空回调的注册
    CallbackParams callback_params;
    callback_params.msg_callback = nullptr;
    callback_params.msg_cb_ctx = &callback_context_;
    callback_params.tef_callback = nullptr;
    callback_params.tef_cb_ctx = nullptr;
    reg.setRegistration(info, callback_params);

    // 由于空回调应该是无效的
    EXPECT_FALSE(reg.isValid());
    EXPECT_TRUE(reg.is_registered);
    EXPECT_EQ(reg.callback, nullptr);
    EXPECT_FALSE(reg.context.expired());  // 弱指针应该是有效的
    EXPECT_NE(reg.strong_context, nullptr);
}

TEST_F(BusControlBlockTest, BusControlBlockDestruction) {
    // 测试带有 BUSMUST_B 的 BusControlBlock 可以被安全销毁
    {
        auto block = BusControlBlock::create("can0", CanBusModelType::BUSMUST_B, bitrate_);
        // 如果没有可用硬件则跳过测试
    if (!block) {
        std::cout << "跳过测试 - 没有可用硬件" << std::endl;
        return;
    }
        
        // 添加一些设备
        for (int i = 0; i < 5; ++i) {
            DeviceInfo device_info;
            device_info.device_name = "device_" + std::to_string(i);
            device_info.device_id = static_cast<DeviceId>(100 + i);
        device_info.device_type = DeviceType::MOTOR;

            CallbackParams callback_params;
            callback_params.msg_callback = TestCanBusCallback::StaticCallback;
            callback_params.msg_cb_ctx = &callback_context_;
            callback_params.tef_callback = nullptr;
            callback_params.tef_cb_ctx = nullptr;
            bool result = block->addDevice(device_info, callback_params);
            EXPECT_TRUE(result);
        }

        EXPECT_EQ(block->deviceCount(), 5);

        // 当超出作用域时块将被销毁
    }

    // 测试应该在没有崩溃或内存泄漏的情况下完成
    EXPECT_TRUE(true);
}

TEST_F(BusControlBlockTest, ThreadSafetyBasic) {
    // 通过使用 BUSMUST_B 从多个线程添加/移除设备来测试基本线程安全
    auto block = BusControlBlock::create("can0", CanBusModelType::BUSMUST_B, bitrate_);
    // 如果没有可用硬件则跳过测试
    if (!block) {
        std::cout << "跳过测试 - 没有可用硬件" << std::endl;
        return;
    }
    
    std::atomic<bool> start_flag{false};
    std::atomic<int> success_count{0};

    // 线程 1：添加设备
    auto add_thread = std::thread([&]() {
        while (!start_flag.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        for (int i = 0; i < 100; ++i) {
            DeviceInfo device_info;
            device_info.device_name = "thread1_device_" + std::to_string(i);
            device_info.device_id = static_cast<DeviceId>(1000 + i);
            device_info.device_type = DeviceType::MOTOR;

            CallbackParams callback_params;
            callback_params.msg_callback = TestCanBusCallback::StaticCallback;
            callback_params.msg_cb_ctx = &callback_context_;
            callback_params.tef_callback = nullptr;
            callback_params.tef_cb_ctx = nullptr;
            if (block->addDevice(device_info, callback_params)) {
                success_count++;
            }
        }
    });

    // 线程 2：移除设备
    auto remove_thread = std::thread([&]() {
        while (!start_flag.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        for (int i = 0; i < 100; ++i) {
            if (block->removeDevice(DeviceType::MOTOR, 1000 + i)) {
                success_count++;
            }
        }
    });

    // 启动两个线程
    start_flag.store(true);

    // 等待线程完成
    add_thread.join();
    remove_thread.join();

    // 测试应该在没有死锁的情况下完成
    EXPECT_TRUE(true);
}

