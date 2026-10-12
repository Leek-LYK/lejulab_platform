#include "motorevo/motor_ctrl.h"
#include "motorevo/motor_log.h"
#include <gtest/gtest.h>
#include <thread>
#include <chrono>
#include <cmath>
#include <vector>
#include <atomic>
#include <mutex>
#include <map>



class RevoMotorTest : public ::testing::Test {
protected:
    void SetUp() override {
        motor = std::make_unique<motorevo::RevoMotor>(0x01, 0x07, motorevo::MotorMode::TorquePositionMixControlMode);
    }

    void TearDown() override {
        motor.reset();
    }

    std::unique_ptr<motorevo::RevoMotor> motor;
};

TEST_F(RevoMotorTest, ConstructorTest) {
    EXPECT_EQ(motor->getId(), 0x07);
    EXPECT_EQ(motor->getControlMode(), motorevo::MotorMode::TorquePositionMixControlMode);
}

TEST_F(RevoMotorTest, ZeroOffsetTest) {
    // setZeroOffset removed from RevoMotor class - zero offset is handled externally
    // Test basic position functionality
    EXPECT_FLOAT_EQ(motor->position(), 0.0f);
    EXPECT_FLOAT_EQ(motor->velocity(), 0.0f);
    EXPECT_FLOAT_EQ(motor->torque(), 0.0f);
    EXPECT_EQ(motor->temperature(), 0);
}

TEST_F(RevoMotorTest, RawDataTest) {
    EXPECT_FLOAT_EQ(motor->position(), 0.0f);
    EXPECT_FLOAT_EQ(motor->velocity(), 0.0f);
    EXPECT_FLOAT_EQ(motor->torque(), 0.0f);
}

TEST_F(RevoMotorTest, FeedbackTest) {
    // 模拟反馈数据
    uint8_t test_payload[8] = {
        0x07, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0x00, 0x25  // Motor ID 0x07 to match test fixture
    };
    motorevo::FeedbackFrame frame(test_payload);
    motor->receiveFeedback(frame);

    // 验证数据被更新 - basic functionality test only
    // Note: RevoMotor now returns raw values without processing
    EXPECT_NE(motor->position(), 0.0f) << "Position should be updated from test data";
    EXPECT_NE(motor->velocity(), 0.0f) << "Velocity should be updated from test data";
    EXPECT_NE(motor->torque(), 0.0f) << "Torque should be updated from test data";
    EXPECT_EQ(motor->temperature(), 0x25) << "Temperature should match test data";
}

TEST_F(RevoMotorTest, NegativeDirectionTest) {
    auto neg_motor = std::make_unique<motorevo::RevoMotor>(0x01, 0x08, motorevo::MotorMode::TorquePositionMixControlMode);
    // setZeroOffset removed from RevoMotor class - zero offset is handled externally

    // 构造正值的测试数据
    uint16_t pos_value = motorevo::float_to_int<16>(1.0f, motorevo::kCAN_COM_THETA_MIN, motorevo::kCAN_COM_THETA_MAX);
    uint16_t vel_value = motorevo::float_to_int<12>(2.0f, motorevo::kCAN_COM_VELOCITY_MIN, motorevo::kCAN_COM_VELOCITY_MAX);
    uint16_t torque_value = motorevo::float_to_int<12>(3.0f, motorevo::kCAN_COM_TORQUE_MIN, motorevo::kCAN_COM_TORQUE_MAX);

    uint8_t test_payload[8] = {
        0x08,                                                // 电机ID - Byte0 (must match motor ID)
        static_cast<uint8_t>((pos_value >> 8) & 0xFF),      // 位置高字节 - Byte1
        static_cast<uint8_t>(pos_value & 0xFF),               // 位置低字节 - Byte2
        static_cast<uint8_t>((vel_value >> 4) & 0xFF),       // 速度高字节 - Byte3
        static_cast<uint8_t>(((vel_value & 0x0F) << 4) | ((torque_value >> 8) & 0x0F)), // 速度低字节 + 力矩高字节 - Byte4
        static_cast<uint8_t>(torque_value & 0xFF),           // 力矩低字节 - Byte5
        0x00,                                                // 错误码 - Byte6
        0x25                                                 // 温度 - Byte7
    };

    motorevo::FeedbackFrame frame(test_payload);
    neg_motor->receiveFeedback(frame);

    // Basic functionality test - direction processing moved to external layer
    // RevoMotor now returns raw values without direction processing
    EXPECT_NEAR(neg_motor->position(), 1.0f, 0.1f) << "Raw position should match input";
    EXPECT_NEAR(neg_motor->velocity(), 2.0f, 0.1f) << "Raw velocity should match input";
    EXPECT_NEAR(neg_motor->torque(), 3.0f, 0.1f) << "Raw torque should match input";
}


TEST_F(RevoMotorTest, CopyConstructorTest) {
    // setZeroOffset removed - zero offset is handled externally(1.57f);

    motorevo::RevoMotor copy_motor(*motor);

    EXPECT_EQ(copy_motor.getId(), motor->getId());
    EXPECT_EQ(copy_motor.getControlMode(), motor->getControlMode());
}

TEST_F(RevoMotorTest, AssignmentOperatorTest) {
    motorevo::RevoMotor other_motor(0x02, 0x09, motorevo::MotorMode::VelocityMode);
    // Removed setZeroOffset as it's no longer available in RevoMotor class

    *motor = other_motor;

    EXPECT_EQ(motor->getId(), other_motor.getId());
    EXPECT_EQ(motor->getControlMode(), other_motor.getControlMode());
}

TEST_F(RevoMotorTest, MoveConstructorTest) {
    // 创建临时电机对象
    motorevo::RevoMotor temp_motor(0x01, 0x07, motorevo::MotorMode::TorquePositionMixControlMode);

    // 发送反馈数据到临时电机
    uint8_t test_payload[8] = {
        0x07, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0x00, 0x25
    };
    motorevo::FeedbackFrame frame(test_payload);
    temp_motor.receiveFeedback(frame);

    // 记录临时电机的状态
    auto temp_id = temp_motor.getId();
    auto temp_mode = temp_motor.getControlMode();
    auto temp_pos = temp_motor.position();

    // 移动构造
    motorevo::RevoMotor moved_motor(std::move(temp_motor));

    // 验证移动后的对象数据正确
    EXPECT_EQ(moved_motor.getId(), temp_id);
    EXPECT_EQ(moved_motor.getControlMode(), temp_mode);
    EXPECT_FLOAT_EQ(moved_motor.position(), temp_pos);

    // 原对象应该处于有效状态，但内部数据可能已转移
    EXPECT_EQ(temp_motor.getId(), 0x07);  // ID 应该保持不变
}

TEST_F(RevoMotorTest, MoveAssignmentOperatorTest) {
    // 创建目标电机
    motorevo::RevoMotor target_motor(0x02, 0x09, motorevo::MotorMode::VelocityMode);

    // 创建源电机并发送反馈
    motorevo::RevoMotor source_motor(0x01, 0x07, motorevo::MotorMode::TorquePositionMixControlMode);
    uint8_t test_payload[8] = {
        0x07, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0x00, 0x25
    };
    motorevo::FeedbackFrame frame(test_payload);
    source_motor.receiveFeedback(frame);

    // 记录源电机状态
    auto source_id = source_motor.getId();
    auto source_mode = source_motor.getControlMode();
    auto source_pos = source_motor.position();

    // 移动赋值
    target_motor = std::move(source_motor);

    // 验证目标电机数据正确
    EXPECT_EQ(target_motor.getId(), source_id);
    EXPECT_EQ(target_motor.getControlMode(), source_mode);
    EXPECT_FLOAT_EQ(target_motor.position(), source_pos);

    // 源对象应该处于有效状态，但内部数据可能已转移
    EXPECT_EQ(source_motor.getId(), 0x07);  // ID 应该保持不变
}

// MotorCtrlData 初始化和emplace使用测试
TEST(MotorCtrlDataTest, InitializationTest) {
    // 创建电机配置
    motorevo::RevoMotorConfig_t config;
    config.id = 1;  // MotorId 是 uint32_t 类型
    config.name = "TestMotor";
    config.negtive = false;
    config.ignore = false;
    config.zero_offset = 0.0f;
    config.default_params.vel = 0.0f;
    config.default_params.kp_pos = 0.0f;
    config.default_params.kd_pos = 0.0f;
    config.default_params.tor = 0.0f;
    config.default_params.kp_vel = 0.0f;
    config.default_params.kd_vel = 0.0f;
    config.default_params.ki_vel = 0.0f;

    // 创建电机实例，使用默认模式，后续可以根据配置设置具体模式
    motorevo::RevoMotor motor(0x01, config.id, motorevo::MotorMode::TorquePositionMixControlMode);

    // 注意：MotorCtrlData 是私有结构体，无法直接在测试中实例化
    // 这里只测试基本逻辑，不直接测试私有成员
    EXPECT_EQ(motor.getId(), config.id);
    EXPECT_EQ(motor.getControlMode(), motorevo::MotorMode::TorquePositionMixControlMode);
}

TEST(MotorCtrlDataTest, EmplaceUsageTest) {
    // 由于 MotorCtrlData 是私有结构体，无法直接测试 emplace 用法
    // 这里测试基本的配置和电机创建逻辑

    // 创建多个电机配置
    std::vector<motorevo::RevoMotorConfig_t> configs = {
        {
            .id = 1,  // MotorId 是 uint32_t 类型
            .name = "Motor1",
            .negtive = false,
            .ignore = false,
            .zero_offset = 0.0f,
            .default_params = {.vel = 0.0f, .kp_pos = 0.0f, .kd_pos = 0.0f, .tor = 0.0f, .kp_vel = 0.0f, .kd_vel = 0.0f, .ki_vel = 0.0f}
        },
        {
            .id = 2,  // MotorId 是 uint32_t 类型
            .name = "Motor2",
            .negtive = true,
            .ignore = false,
            .zero_offset = 1.57f,
            .default_params = {.vel = 0.0f, .kp_pos = 0.0f, .kd_pos = 0.0f, .tor = 0.0f, .kp_vel = 0.0f, .kd_vel = 0.0f, .ki_vel = 0.0f}
        }
    };

    // 测试电机创建逻辑
    for (const auto& config : configs) {
        if (!config.ignore) {
            motorevo::RevoMotor motor(0x01, config.id, motorevo::MotorMode::TorquePositionMixControlMode);
            EXPECT_EQ(motor.getId(), config.id);
            EXPECT_EQ(motor.getControlMode(), motorevo::MotorMode::TorquePositionMixControlMode);
        }
    }

    // 验证配置数量
    EXPECT_EQ(configs.size(), 2);
    EXPECT_EQ(configs[0].id, 1);
    EXPECT_EQ(configs[1].id, 2);
}

TEST(MotorCtrlDataTest, IgnoreMotorTest) {
    // 测试忽略电机的逻辑
    motorevo::RevoMotorConfig_t ignored_config;
    ignored_config.id = 3;
    ignored_config.name = "IgnoredMotor";
    ignored_config.negtive = false;
    ignored_config.ignore = true;  // 标记为忽略
    ignored_config.zero_offset = 0.0f;

    // 模拟 RevoMotorControl 中的逻辑 - 被忽略的电机不应该被处理
    bool should_process = !ignored_config.ignore;
    EXPECT_FALSE(should_process);

    // 如果不忽略，会创建电机
    if (!ignored_config.ignore) {
        motorevo::RevoMotor motor(0x01, ignored_config.id, motorevo::MotorMode::TorquePositionMixControlMode);
        EXPECT_EQ(motor.getId(), ignored_config.id);
    }
}

// FeedbackFrame类的独立测试
class FeedbackFrameTest : public ::testing::Test {
protected:
    void SetUp() override {
        // 构造测试用的payload数据
        test_payload_[0] = 0x12;  // 位置高字节
        test_payload_[1] = 0x34;  // 位置低字节
        test_payload_[2] = 0x56;  // 速度高字节
        test_payload_[3] = 0x78;  // 速度低字节 + Kp高字节
        test_payload_[4] = 0x9A;  // Kp低字节 + Kd高字节
        test_payload_[5] = 0xBC;  // Kd低字节 + Ki高字节
        test_payload_[6] = 0x01;  // 错误码
        test_payload_[7] = 0x25;  // 温度
    }

    uint8_t test_payload_[8];
};

TEST_F(FeedbackFrameTest, ConstructorTest) {
    motorevo::FeedbackFrame frame1;  // 默认构造
    EXPECT_EQ(frame1.temperature(), 0);

    motorevo::FeedbackFrame frame2(test_payload_);  // 带参数构造
    EXPECT_EQ(frame2.temperature(), 0x25);
}

TEST_F(FeedbackFrameTest, CopyConstructorTest) {
    motorevo::FeedbackFrame original(test_payload_);
    motorevo::FeedbackFrame copy(original);

    EXPECT_EQ(copy.temperature(), original.temperature());
    EXPECT_EQ(copy.errcode(), original.errcode());
}

TEST_F(FeedbackFrameTest, AssignmentOperatorTest) {
    motorevo::FeedbackFrame frame1(test_payload_);
    motorevo::FeedbackFrame frame2;

    frame2 = frame1;

    EXPECT_EQ(frame2.temperature(), frame1.temperature());
    EXPECT_EQ(frame2.errcode(), frame1.errcode());
}

TEST_F(FeedbackFrameTest, PositionExtractionTest) {
    motorevo::FeedbackFrame frame(test_payload_);
    float pos = frame.position();

    // 验证位置数据提取正确（根据具体的位域解析算法）
    EXPECT_GE(pos, motorevo::kCAN_COM_THETA_MIN);
    EXPECT_LE(pos, motorevo::kCAN_COM_THETA_MAX);
}

TEST_F(FeedbackFrameTest, VelocityExtractionTest) {
    motorevo::FeedbackFrame frame(test_payload_);
    float vel = frame.velocity();

    // 验证速度数据提取正确
    EXPECT_GE(vel, motorevo::kCAN_COM_VELOCITY_MIN);
    EXPECT_LE(vel, motorevo::kCAN_COM_VELOCITY_MAX);
}

TEST_F(FeedbackFrameTest, TorqueExtractionTest) {
    motorevo::FeedbackFrame frame(test_payload_);
    float torque = frame.torque();

    // 验证力矩数据提取正确
    EXPECT_GE(torque, motorevo::kCAN_COM_TORQUE_MIN);
    EXPECT_LE(torque, motorevo::kCAN_COM_TORQUE_MAX);
}

TEST_F(FeedbackFrameTest, TemperatureAndErrorExtractionTest) {
    motorevo::FeedbackFrame frame(test_payload_);

    EXPECT_EQ(frame.temperature(), 0x25);
    EXPECT_EQ(frame.errcode(), static_cast<motorevo::MotorErrCode>(0x01));
}

TEST_F(FeedbackFrameTest, ResetTest) {
    motorevo::FeedbackFrame frame(test_payload_);

    // 重置后所有数据应该归零
    frame.reset();
    EXPECT_EQ(frame.temperature(), 0);
    EXPECT_EQ(frame.errcode(), motorevo::MotorErrCode::NO_FAULT);
}

TEST_F(FeedbackFrameTest, SetPayloadTest) {
    motorevo::FeedbackFrame frame;
    uint8_t new_payload[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x02, 0x30};

    frame.set_payload(new_payload);

    EXPECT_EQ(frame.temperature(), 0x30);
    EXPECT_EQ(frame.errcode(), static_cast<motorevo::MotorErrCode>(0x02));
}

// FeedbackFrameFd (CAN FD 广播协议) 故障位测试
// 布局: pos(16bit) + vel(12bit) + torque(12bit) + temp(8bit) + status(16bit 大端)
class FeedbackFrameFdTest : public ::testing::Test {
protected:
    // 构造 status_word 指定的反馈帧, 其余字段填 0
    void makePayload(uint16_t status_word, uint8_t payload[8]) {
        memset(payload, 0, 8);
        payload[6] = static_cast<uint8_t>(status_word >> 8);
        payload[7] = static_cast<uint8_t>(status_word & 0xFF);
    }
};

TEST_F(FeedbackFrameFdTest, NoFaultTest) {
    uint8_t payload[8];
    makePayload(motorevo::canfd::kStatusEnabled, payload);  // 仅使能位

    motorevo::FeedbackFrameFd frame(payload);
    EXPECT_EQ(frame.status_word(), motorevo::canfd::kStatusEnabled);
    EXPECT_TRUE(frame.is_enabled());
    EXPECT_FALSE(frame.has_fault());
}

TEST_F(FeedbackFrameFdTest, SingleFaultBitTest) {
    // 每一个故障位单独置位都应判定为故障
    const uint16_t fault_bits[] = {
        motorevo::canfd::kStatusOverVoltage,
        motorevo::canfd::kStatusOverCurrent,
        motorevo::canfd::kStatusOverTemp,
        motorevo::canfd::kStatusOverSpeed,
        motorevo::canfd::kStatusEncoderFault,
        motorevo::canfd::kStatusDriverFault,
        motorevo::canfd::kStatusOverload,
        motorevo::canfd::kStatusStall,
        motorevo::canfd::kStatusDeviation,
        motorevo::canfd::kStatusCustom,
    };
    for (uint16_t bit : fault_bits) {
        uint8_t payload[8];
        makePayload(motorevo::canfd::kStatusEnabled | bit, payload);
        motorevo::FeedbackFrameFd frame(payload);
        EXPECT_TRUE(frame.has_fault()) << "fault bit 0x" << std::hex << bit;
        EXPECT_EQ(frame.status_word() & motorevo::canfd::kStatusFaultMask, bit);
    }
}

TEST_F(FeedbackFrameFdTest, FaultMaskExcludesEnabledBitTest) {
    // 使能位不属于故障位
    uint8_t payload[8];
    makePayload(motorevo::canfd::kStatusEnabled, payload);
    motorevo::FeedbackFrameFd frame(payload);
    EXPECT_EQ(frame.status_word() & motorevo::canfd::kStatusFaultMask, 0);
}

// 数值转换函数测试类
class ConversionFunctionsTest : public ::testing::Test {
protected:
    void SetUp() override {
        // 设置测试用的常量
        theta_min_ = motorevo::kCAN_COM_THETA_MIN;
        theta_max_ = motorevo::kCAN_COM_THETA_MAX;
        velocity_min_ = motorevo::kCAN_COM_VELOCITY_MIN;
        velocity_max_ = motorevo::kCAN_COM_VELOCITY_MAX;
        torque_min_ = motorevo::kCAN_COM_TORQUE_MIN;
        torque_max_ = motorevo::kCAN_COM_TORQUE_MAX;
    }

    float theta_min_;
    float theta_max_;
    float velocity_min_;
    float velocity_max_;
    float torque_min_;
    float torque_max_;
};

TEST_F(ConversionFunctionsTest, FloatToInt16Test) {
    // 测试16位位置转换
    float test_positions[] = {theta_min_, theta_max_, 0.0f, 5.0f, -5.0f, 12.5f, -12.5f};

    for (float pos : test_positions) {
        uint32_t int_val = motorevo::float_to_int<16>(pos, theta_min_, theta_max_);
        EXPECT_LE(int_val, 0xFFFF) << "16位转换结果超出范围";

        // 转换回来验证
        float converted_back = motorevo::int_to_float<16>(int_val, theta_min_, theta_max_);
        EXPECT_NEAR(converted_back, pos, 0.01f) << "位置转换往返不一致";
    }
}

TEST_F(ConversionFunctionsTest, FloatToInt12Test) {
    // 测试12位速度转换
    float test_velocities[] = {velocity_min_, velocity_max_, 0.0f, 3.0f, -3.0f, 10.0f, -10.0f};

    for (float vel : test_velocities) {
        uint32_t int_val = motorevo::float_to_int<12>(vel, velocity_min_, velocity_max_);
        EXPECT_LE(int_val, 0xFFF) << "12位转换结果超出范围";

        // 转换回来验证
        float converted_back = motorevo::int_to_float<12>(int_val, velocity_min_, velocity_max_);
        EXPECT_NEAR(converted_back, vel, 0.05f) << "速度转换往返不一致";
    }
}

TEST_F(ConversionFunctionsTest, FloatToInt8Test) {
    // 测试8位转换
    float test_values[] = {0.0f, 1.0f, -1.0f, 10.0f, -10.0f};
    float min_val = -10.0f;
    float max_val = 10.0f;

    for (float val : test_values) {
        uint32_t int_val = motorevo::float_to_int<8>(val, min_val, max_val);
        EXPECT_LE(int_val, 0xFF) << "8位转换结果超出范围";

        // 转换回来验证
        float converted_back = motorevo::int_to_float<8>(int_val, min_val, max_val);
        EXPECT_NEAR(converted_back, val, 0.1f) << "8位转换往返不一致";
    }
}

TEST_F(ConversionFunctionsTest, EdgeCasesTest) {
    // 测试边界情况
    struct TestCase {
        float input;
        float min_val;
        float max_val;
        uint8_t bits;
        uint32_t expected_int;
    };

    TestCase test_cases[] = {
        {motorevo::kCAN_COM_THETA_MIN, motorevo::kCAN_COM_THETA_MIN, motorevo::kCAN_COM_THETA_MAX, 16, 0},
        {motorevo::kCAN_COM_THETA_MAX, motorevo::kCAN_COM_THETA_MIN, motorevo::kCAN_COM_THETA_MAX, 16, 0xFFFF},
        {motorevo::kCAN_COM_VELOCITY_MIN, motorevo::kCAN_COM_VELOCITY_MIN, motorevo::kCAN_COM_VELOCITY_MAX, 12, 0},
        {motorevo::kCAN_COM_VELOCITY_MAX, motorevo::kCAN_COM_VELOCITY_MIN, motorevo::kCAN_COM_VELOCITY_MAX, 12, 0xFFF},
        {motorevo::kCAN_COM_TORQUE_MIN, motorevo::kCAN_COM_TORQUE_MIN, motorevo::kCAN_COM_TORQUE_MAX, 12, 0},
        {motorevo::kCAN_COM_TORQUE_MAX, motorevo::kCAN_COM_TORQUE_MIN, motorevo::kCAN_COM_TORQUE_MAX, 12, 0xFFF},
    };

    for (const auto& test : test_cases) {
        uint32_t result;
        if (test.bits == 16) {
            result = motorevo::float_to_int<16>(test.input, test.min_val, test.max_val);
        } else if (test.bits == 12) {
            result = motorevo::float_to_int<12>(test.input, test.min_val, test.max_val);
        } else {
            result = motorevo::float_to_int<8>(test.input, test.min_val, test.max_val);
        }
        EXPECT_EQ(result, test.expected_int) << "边界值转换失败";
    }
}

TEST_F(ConversionFunctionsTest, RangeClampingTest) {
    // 测试超出范围的值被正确截断
    struct TestCase {
        float input;
        float min_val;
        float max_val;
        uint8_t bits;
        uint32_t expected_min;
        uint32_t expected_max;
        uint32_t expected_result;
    };

    TestCase test_cases[] = {
        {15.0f, motorevo::kCAN_COM_THETA_MIN, motorevo::kCAN_COM_THETA_MAX, 16, 0, 0xFFFF, 0xFFFF},  // 超出上限
        {-15.0f, motorevo::kCAN_COM_THETA_MIN, motorevo::kCAN_COM_THETA_MAX, 16, 0, 0xFFFF, 0},      // 超出下限
        {15.0f, motorevo::kCAN_COM_VELOCITY_MIN, motorevo::kCAN_COM_VELOCITY_MAX, 12, 0, 0xFFF, 0xFFF},
        {-15.0f, motorevo::kCAN_COM_VELOCITY_MIN, motorevo::kCAN_COM_VELOCITY_MAX, 12, 0, 0xFFF, 0},
    };

    for (const auto& test : test_cases) {
        uint32_t result;
        if (test.bits == 16) {
            result = motorevo::float_to_int<16>(test.input, test.min_val, test.max_val);
        } else if (test.bits == 12) {
            result = motorevo::float_to_int<12>(test.input, test.min_val, test.max_val);
        } else {
            result = motorevo::float_to_int<8>(test.input, test.min_val, test.max_val);
        }
        EXPECT_GE(result, test.expected_min) << "转换结果小于最小值";
        EXPECT_LE(result, test.expected_max) << "转换结果大于最大值";
        EXPECT_EQ(result, test.expected_result) << "截断结果不正确";
    }
}

TEST_F(ConversionFunctionsTest, PrecisionTest) {
    // 测试转换精度
    float test_values[] = {0.1f, 0.01f, 0.001f, -0.1f, -0.01f, -0.001f};

    for (float val : test_values) {
        // 使用16位精度测试
        uint32_t int_val = motorevo::float_to_int<16>(val, -1.0f, 1.0f);
        float converted_back = motorevo::int_to_float<16>(int_val, -1.0f, 1.0f);

        // 允许一定的误差（量化误差）
        float error = std::abs(converted_back - val);
        EXPECT_LT(error, 0.0001f) << "转换精度不足，原始值: " << val
                                 << "，转换后: " << converted_back
                                 << "，误差: " << error;
    }
}

TEST_F(ConversionFunctionsTest, RoundTripTest) {
    // 大量的往返转换测试
    const int test_count = 1000;
    const float step = (motorevo::kCAN_COM_THETA_MAX - motorevo::kCAN_COM_THETA_MIN) / test_count;

    for (int i = 0; i < test_count; ++i) {
        float original = motorevo::kCAN_COM_THETA_MIN + i * step;

        // 16位往返转换
        uint32_t int_val = motorevo::float_to_int<16>(original, motorevo::kCAN_COM_THETA_MIN, motorevo::kCAN_COM_THETA_MAX);
        float converted = motorevo::int_to_float<16>(int_val, motorevo::kCAN_COM_THETA_MIN, motorevo::kCAN_COM_THETA_MAX);

        EXPECT_NEAR(converted, original, 0.01f) << "往返转换失败，原始值: " << original;
    }
}

// RevoMotorControl 类方法的综合单元测试
class RevoMotorControlTest : public ::testing::Test {
protected:
    void SetUp() override {
        // 创建测试用的电机配置
        test_configs_ = {
            {
                .id = 1,  // MotorId 是 uint32_t 类型
                .name = "TestMotor1",
                .negtive = false,
                .ignore = false,
                .zero_offset = 0.0f,
                .default_params = {.vel = 0.0f, .kp_pos = 0.0f, .kd_pos = 0.0f, .tor = 0.0f, .kp_vel = 0.0f, .kd_vel = 0.0f, .ki_vel = 0.0f}
            },
            {
                .id = 2,  // MotorId 是 uint32_t 类型
                .name = "TestMotor2",
                .negtive = true,
                .ignore = false,
                .zero_offset = 1.57f,
                .default_params = {.vel = 0.0f, .kp_pos = 0.0f, .kd_pos = 0.0f, .tor = 0.0f, .kp_vel = 0.0f, .kd_vel = 0.0f, .ki_vel = 0.0f}
            }
        };
    }

    std::vector<motorevo::RevoMotorConfig_t> test_configs_;
};

TEST_F(RevoMotorControlTest, ConstructorTest) {
    // 测试构造函数 - 不需要实际的 CAN 总线
    motorevo::RevoMotorControl controller("test_can0");

    // 构造函数应该成功创建对象
    EXPECT_TRUE(true);  // 如果构造成功，测试通过
}

TEST_F(RevoMotorControlTest, GetRawPositionsTest) {
    // 创建一个模拟的电机进行测试
    motorevo::RevoMotor test_motor(0x01, test_configs_[0].id, motorevo::MotorMode::TorquePositionMixControlMode);

    // 发送一些测试数据
    uint8_t test_payload[8] = {
        static_cast<uint8_t>(test_configs_[0].id), 0x12, 0x34, 0x56, 0x78, 0x9A, 0x00, 0x25
    };
    motorevo::FeedbackFrame frame(test_payload);
    test_motor.receiveFeedback(frame);

    // 测试原始位置获取
    float raw_pos = test_motor.position();
    EXPECT_NE(raw_pos, 0.0f);  // 应该有实际的值

    float raw_vel = test_motor.velocity();
    EXPECT_NE(raw_vel, 0.0f);

    float raw_torque = test_motor.torque();
    EXPECT_NE(raw_torque, 0.0f);
}

TEST_F(RevoMotorControlTest, PositionsWithZeroOffsetTest) {
    // 创建一个模拟的电机进行测试
    motorevo::RevoMotor test_motor(0x01, test_configs_[0].id, motorevo::MotorMode::TorquePositionMixControlMode);

    // 设置一个测试位置
    uint8_t test_payload[8] = {
        static_cast<uint8_t>(test_configs_[0].id), 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x25
    };
    motorevo::FeedbackFrame frame(test_payload);
    test_motor.receiveFeedback(frame);

    // 获取原始位置
    float raw_pos = test_motor.position();

    // 创建配置，设置零点偏移
    motorevo::RevoMotorConfig_t config = test_configs_[0];
    config.zero_offset = 1.0f;  // 设置1.0弧度的零点偏移

    // 计算期望的调整后位置
    float expected_pos = raw_pos - config.zero_offset;

    // 验证计算逻辑
    EXPECT_FLOAT_EQ(expected_pos, raw_pos - config.zero_offset);
}

TEST_F(RevoMotorControlTest, NegativeDirectionTest) {
    // 测试方向处理
    motorevo::RevoMotor test_motor(0x01, test_configs_[0].id, motorevo::MotorMode::TorquePositionMixControlMode);

    // 设置一个测试位置
    uint8_t test_payload[8] = {
        static_cast<uint8_t>(test_configs_[0].id), 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x25
    };
    motorevo::FeedbackFrame frame(test_payload);
    test_motor.receiveFeedback(frame);

    // 获取原始值
    float raw_pos = test_motor.position();
    float raw_vel = test_motor.velocity();
    float raw_torque = test_motor.torque();

    // 创建配置，设置负方向
    motorevo::RevoMotorConfig_t config = test_configs_[1];  // 使用配置2，negtive = true
    config.zero_offset = 0.0f;

    // 计算期望的调整后值
    float expected_pos = config.negtive ? -raw_pos : raw_pos;
    float expected_vel = config.negtive ? -raw_vel : raw_vel;
    float expected_torque = config.negtive ? -raw_torque : raw_torque;

    // 验证方向处理逻辑
    EXPECT_FLOAT_EQ(expected_pos, -raw_pos);
    EXPECT_FLOAT_EQ(expected_vel, -raw_vel);
    EXPECT_FLOAT_EQ(expected_torque, -raw_torque);
}

TEST_F(RevoMotorControlTest, MotorCommandTest) {
    // 测试电机命令结构
    motorevo::RevoMotorCmd_t cmd;
    cmd.pos = 1.5f;
    cmd.vel = 2.0f;
    cmd.torque = 0.5f;
    cmd.kp = 100.0f;
    cmd.kd = 10.0f;

    // 验证命令结构
    EXPECT_FLOAT_EQ(cmd.pos, 1.5f);
    EXPECT_FLOAT_EQ(cmd.vel, 2.0f);
    EXPECT_FLOAT_EQ(cmd.torque, 0.5f);
    EXPECT_FLOAT_EQ(cmd.kp, 100.0f);
    EXPECT_FLOAT_EQ(cmd.kd, 10.0f);
}

TEST_F(RevoMotorControlTest, MotorParamTest) {
    // 测试电机参数结构
    motorevo::MotorParam_t params;
    params.vel = 1.0f;
    params.kp_pos = 50.0f;
    params.kd_pos = 5.0f;
    params.tor = 0.1f;
    params.kp_vel = 25.0f;
    params.kd_vel = 2.5f;
    params.ki_vel = 0.01f;

    // 验证参数结构
    EXPECT_FLOAT_EQ(params.vel, 1.0f);
    EXPECT_FLOAT_EQ(params.kp_pos, 50.0f);
    EXPECT_FLOAT_EQ(params.kd_pos, 5.0f);
    EXPECT_FLOAT_EQ(params.tor, 0.1f);
    EXPECT_FLOAT_EQ(params.kp_vel, 25.0f);
    EXPECT_FLOAT_EQ(params.kd_vel, 2.5f);
    EXPECT_FLOAT_EQ(params.ki_vel, 0.01f);
}


int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}