#include <gtest/gtest.h>
#include "canbus_sdk/canbus_sdk.h"
#include <unordered_map>
#include <string>

using namespace canbus_sdk;

class DeviceUniqueIdTest : public ::testing::Test {
protected:
    void SetUp() override {
        // 不需要任何设置
    }

    void TearDown() override {
        // 不需要任何清理
    }
};

// 测试1: 基本ID转换 - 设备类型和ID转换为唯一ID
TEST_F(DeviceUniqueIdTest, BasicIdConversion) {
    // 测试基本转换功能
    DeviceUniqueId motor_id(DeviceType::MOTOR, 123);
    DeviceUniqueId revo2_id(DeviceType::REVO2_HAND, 123);

    // 验证相同设备ID但不同类型的唯一性
    EXPECT_NE(motor_id.value, revo2_id.value);

    // 验证类型和ID转换正确
    EXPECT_EQ(motor_id.getType(), DeviceType::MOTOR);
    EXPECT_EQ(motor_id.getDeviceId(), 123);
    EXPECT_EQ(revo2_id.getType(), DeviceType::REVO2_HAND);
    EXPECT_EQ(revo2_id.getDeviceId(), 123); 
}

// 测试2: 位域布局验证 - 验证8位类型+24位ID的布局
TEST_F(DeviceUniqueIdTest, BitfieldLayout) {
    // 测试设备类型在高8位，设备ID在低24位
    DeviceUniqueId motor_id(DeviceType::MOTOR, 0x00123456);
    DeviceUniqueId revo2_id(DeviceType::REVO2_HAND, 0x00123456);

    // 手动计算期望值
    uint32_t expected_motor_value = (static_cast<uint8_t>(DeviceType::MOTOR) << 24) | 0x00123456;
    uint32_t expected_revo2_value = (static_cast<uint8_t>(DeviceType::REVO2_HAND) << 24) | 0x00123456;

    // 验证位域布局正确
    EXPECT_EQ(motor_id.value, expected_motor_value);
    EXPECT_EQ(revo2_id.value, expected_revo2_value);

    // 验证从唯一ID反向解析正确
    EXPECT_EQ(motor_id.getType(), DeviceType::MOTOR);
    EXPECT_EQ(motor_id.getDeviceId(), 0x00123456);
    EXPECT_EQ(revo2_id.getType(), DeviceType::REVO2_HAND);
    EXPECT_EQ(revo2_id.getDeviceId(), 0x00123456);
}

// 测试3: 设备ID范围测试 - 验证24位ID范围
TEST_F(DeviceUniqueIdTest, DeviceIdRange) {
    // 测试边界值
    DeviceUniqueId min_id(DeviceType::MOTOR, 0);
    DeviceUniqueId max_id(DeviceType::MOTOR, 0x00FFFFFF);

    EXPECT_EQ(min_id.getDeviceId(), 0);
    EXPECT_EQ(max_id.getDeviceId(), 0x00FFFFFF);

    // 测试24位限制
    DeviceUniqueId overflow_id(DeviceType::MOTOR, 0x01FFFFFF);
    EXPECT_EQ(overflow_id.getDeviceId(), 0x00FFFFFF); // 应该被截断到24位
}

// 测试4: 冲突解决 - 相同数字ID不同类型的唯一性
TEST_F(DeviceUniqueIdTest, ConflictResolution) {
    // 创建相同数字ID但不同类型的设备
    DeviceUniqueId motor_id(DeviceType::MOTOR, 100);
    DeviceUniqueId revo1_id(DeviceType::REVO1_HAND, 100);
    DeviceUniqueId revo2_id(DeviceType::REVO2_HAND, 100);
    DeviceUniqueId lejuclaw_id(DeviceType::LEJUCLAW, 100);

    // 验证所有唯一ID都不同
    EXPECT_NE(motor_id.value, revo1_id.value);
    EXPECT_NE(motor_id.value, revo2_id.value);
    EXPECT_NE(motor_id.value, lejuclaw_id.value);
    EXPECT_NE(revo1_id.value, revo2_id.value);
    EXPECT_NE(revo1_id.value, lejuclaw_id.value);
    EXPECT_NE(revo2_id.value, lejuclaw_id.value);

    // 验证设备类型正确
    EXPECT_EQ(motor_id.getType(), DeviceType::MOTOR);
    EXPECT_EQ(revo1_id.getType(), DeviceType::REVO1_HAND);
    EXPECT_EQ(revo2_id.getType(), DeviceType::REVO2_HAND);
    EXPECT_EQ(lejuclaw_id.getType(), DeviceType::LEJUCLAW);

    // 验证设备ID都相同
    EXPECT_EQ(motor_id.getDeviceId(), 100);
    EXPECT_EQ(revo1_id.getDeviceId(), 100);
    EXPECT_EQ(revo2_id.getDeviceId(), 100);
    EXPECT_EQ(lejuclaw_id.getDeviceId(), 100);
}

// 测试5: DeviceInfo集成测试 - getUniqueId方法
TEST_F(DeviceUniqueIdTest, DeviceInfoGetUniqueId) {
    DeviceInfo motor_info;
    motor_info.device_name = "test_motor";
    motor_info.device_type = DeviceType::MOTOR;
    motor_info.device_id = 456;

    DeviceInfo revo2_info;
    revo2_info.device_name = "test_revo2";
    revo2_info.device_type = DeviceType::REVO2_HAND;
    revo2_info.device_id = 456;

    // 测试getUniqueId方法
    DeviceUniqueId motor_unique_id = motor_info.getUniqueId();
    DeviceUniqueId revo2_unique_id = revo2_info.getUniqueId();

    // 验证唯一性
    EXPECT_NE(motor_unique_id.value, revo2_unique_id.value);

    // 验证转换正确
    EXPECT_EQ(motor_unique_id.getType(), DeviceType::MOTOR);
    EXPECT_EQ(motor_unique_id.getDeviceId(), 456);
    EXPECT_EQ(revo2_unique_id.getType(), DeviceType::REVO2_HAND);
    EXPECT_EQ(revo2_unique_id.getDeviceId(), 456);
}

// 测试6: 哈希功能测试 - 用于unordered_map
TEST_F(DeviceUniqueIdTest, HashFunctionality) {
    std::unordered_map<DeviceUniqueId, std::string> device_map;

    DeviceUniqueId motor_id(DeviceType::MOTOR, 100);
    DeviceUniqueId revo2_id(DeviceType::REVO2_HAND, 100);

    // 插入到哈希表
    device_map[motor_id] = "motor_device";
    device_map[revo2_id] = "revo2_device";

    // 验证可以正确检索
    EXPECT_EQ(device_map[motor_id], "motor_device");
    EXPECT_EQ(device_map[revo2_id], "revo2_device");

    // 验证不同的设备ID可以共存
    EXPECT_EQ(device_map.size(), 2);

    // 测试查找功能
    EXPECT_NE(device_map.find(motor_id), device_map.end());
    EXPECT_NE(device_map.find(revo2_id), device_map.end());
}

// 测试7: 未知设备类型处理
TEST_F(DeviceUniqueIdTest, UnknownDeviceType) {
    DeviceUniqueId unknown_id(DeviceType::UNKNOWN, 999);

    // 验证未知设备类型也能正常工作
    EXPECT_EQ(unknown_id.getType(), DeviceType::UNKNOWN);
    EXPECT_EQ(unknown_id.getDeviceId(), 999);

    // 验证位域计算
    uint32_t expected_value = (static_cast<uint8_t>(DeviceType::UNKNOWN) << 24) | 999;
    EXPECT_EQ(unknown_id.value, expected_value);
}

// 测试8: 边界情况测试
TEST_F(DeviceUniqueIdTest, EdgeCases) {
    // 测试默认构造
    DeviceUniqueId default_id;
    EXPECT_EQ(default_id.value, 0);
    EXPECT_EQ(default_id.getType(), DeviceType::UNKNOWN);
    EXPECT_EQ(default_id.getDeviceId(), 0);

    // 测试最大值
    DeviceUniqueId max_id(DeviceType::LEJUCLAW, 0xFFFFFF);
    EXPECT_EQ(max_id.getType(), DeviceType::LEJUCLAW);
    EXPECT_EQ(max_id.getDeviceId(), 0xFFFFFF);

    // 测试相等运算符
    DeviceUniqueId id1(DeviceType::MOTOR, 123);
    DeviceUniqueId id2(DeviceType::MOTOR, 123);
    DeviceUniqueId id3(DeviceType::REVO2_HAND, 123);

    EXPECT_EQ(id1, id2);
    EXPECT_NE(id1, id3);
}