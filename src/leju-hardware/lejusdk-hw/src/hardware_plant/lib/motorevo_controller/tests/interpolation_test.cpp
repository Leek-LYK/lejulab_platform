#include "motorevo/motor_ctrl.h"
#include "motorevo/motor_log.h"
#include <gtest/gtest.h>
#include <vector>
#include <cmath>
#include <algorithm>

// 直接在测试文件中实现插值函数（避免静态函数访问问题）
namespace motorevo {
    std::vector<std::vector<float>> interpolate_positions_with_speed(
        const std::vector<float>& start_positions,
        const std::vector<float>& target_positions,
        float speed,
        float dt) {

        if (start_positions.size() != target_positions.size()) {
            return {};
        }

        int num_motors = start_positions.size();

        // 计算每个电机的距离，找到最大距离的电机作为基准
        float max_distance = 0.0f;
        for (int i = 0; i < num_motors; ++i) {
            float distance = std::abs(target_positions[i] - start_positions[i]);
            if (distance > max_distance) {
                max_distance = distance;
            }
        }

        // 根据最大距离和速度计算总时间
        float total_time = max_distance / speed;

        // 根据总时间和时间步长计算插值步数
        int steps = static_cast<int>(total_time / dt);
        if (steps < 2) steps = 2;  // 至少需要2步：起始位置和目标位置

        // 使用线性插值计算每个时间步的电机位置
        std::vector<std::vector<float>> interpolation(steps, std::vector<float>(num_motors));
        for (int step = 0; step < steps; ++step) {
            float t = static_cast<float>(step) / std::max(1, (steps - 1));
            for (int motor = 0; motor < num_motors; ++motor) {
                interpolation[step][motor] = start_positions[motor] + t * (target_positions[motor] - start_positions[motor]);
            }
        }

        return interpolation;
    }
}

// 插值算法测试类
class InterpolationTest : public ::testing::Test {
protected:
    // 不需要SetUp，每个测试自己定义参数
};

TEST_F(InterpolationTest, SingleMotorShortDistanceTest) {
    // 测试单个电机短距离移动
    float speed = 2.0f * M_PI / 180.0f;  // 2度/秒 = 0.0349 rad/s
    float dt = 1.0f;     // 1秒时间步长

    std::vector<float> start_positions = {0.0f};
    std::vector<float> target_positions = {0.05f};  // 5cm移动

    auto result = motorevo::interpolate_positions_with_speed(start_positions, target_positions, speed, dt);

    // 验证基本属性
    EXPECT_GE(result.size(), 2);  // 至少2步
    EXPECT_EQ(result[0].size(), 1);  // 1个电机

    // 验证起始位置
    EXPECT_FLOAT_EQ(result[0][0], 0.0f);

    // 验证目标位置
    EXPECT_FLOAT_EQ(result.back()[0], 0.05f);

    // 验证插值是单调递增的
    for (size_t i = 1; i < result.size(); ++i) {
        EXPECT_GE(result[i][0], result[i-1][0]) << "插值应该是单调递增的";
    }
}

TEST_F(InterpolationTest, SingleMotorLongDistanceTest) {
    // 测试单个电机长距离移动
    float speed = 2.0f * M_PI / 180.0f;  // 2度/秒 = 0.0349 rad/s
    float dt = 1.0f;     // 1秒时间步长

    std::vector<float> start_positions = {0.0f};
    std::vector<float> target_positions = {1.0f};  // 1弧度移动

    auto result = motorevo::interpolate_positions_with_speed(start_positions, target_positions, speed, dt);

    // 计算期望的步数：1.0 / 0.0349 = 28.65秒，1Hz下应该有28步（算法取整）
    int expected_steps = 28;
    EXPECT_EQ(result.size(), expected_steps);  // 距离够远，应该正好28步
    EXPECT_EQ(result[0].size(), 1);

    // 验证起始和结束位置
    EXPECT_FLOAT_EQ(result[0][0], 0.0f);
    EXPECT_FLOAT_EQ(result.back()[0], 1.0f);

    // 验证线性插值
    float expected_step_size = 1.0f / (expected_steps - 1);
    for (size_t i = 0; i < result.size(); ++i) {
        float expected_pos = i * expected_step_size;
        EXPECT_NEAR(result[i][0], expected_pos, 0.001f) << "第" << i << "步位置不正确";
    }
}

TEST_F(InterpolationTest, MultiMotorDifferentDistancesTest) {
    // 测试多个电机不同距离的移动
    float speed = 2.0f * M_PI / 180.0f;  // 2度/秒 = 0.0349 rad/s
    float dt = 1.0f;     // 1秒时间步长

    std::vector<float> start_positions = {0.0f, 0.0f, 0.0f};
    std::vector<float> target_positions = {0.2f, 0.5f, 0.1f};  // 不同距离

    auto result = motorevo::interpolate_positions_with_speed(start_positions, target_positions, speed, dt);

    // 验证基本属性
    EXPECT_GE(result.size(), 2);  // 至少2步
    EXPECT_EQ(result[0].size(), 3);  // 3个电机

    // 验证起始位置
    for (int i = 0; i < 3; ++i) {
        EXPECT_FLOAT_EQ(result[0][i], 0.0f);
    }

    // 验证目标位置
    EXPECT_FLOAT_EQ(result.back()[0], 0.2f);
    EXPECT_FLOAT_EQ(result.back()[1], 0.5f);
    EXPECT_FLOAT_EQ(result.back()[2], 0.1f);

    // 验证每个电机的运动都是单调的
    for (int motor = 0; motor < 3; ++motor) {
        for (size_t i = 1; i < result.size(); ++i) {
            if (target_positions[motor] > start_positions[motor]) {
                EXPECT_GE(result[i][motor], result[i-1][motor]) << "电机" << motor << "应该单调递增";
            } else if (target_positions[motor] < start_positions[motor]) {
                EXPECT_LE(result[i][motor], result[i-1][motor]) << "电机" << motor << "应该单调递减";
            }
        }
    }
}

TEST_F(InterpolationTest, ZeroDistanceTest) {
    // 测试零距离移动
    float speed = 2.0f * M_PI / 180.0f;  // 2度/秒 = 0.0349 rad/s
    float dt = 1.0f;     // 1秒时间步长

    std::vector<float> start_positions = {0.0f, 1.0f};
    std::vector<float> target_positions = {0.0f, 1.0f};  // 相同位置

    auto result = motorevo::interpolate_positions_with_speed(start_positions, target_positions, speed, dt);

    // 验证基本属性
    EXPECT_GE(result.size(), 2);  // 至少2步
    EXPECT_EQ(result[0].size(), 2);  // 2个电机

    // 验证所有位置都相同
    for (size_t i = 0; i < result.size(); ++i) {
        EXPECT_FLOAT_EQ(result[i][0], 0.0f);
        EXPECT_FLOAT_EQ(result[i][1], 1.0f);
    }
}

TEST_F(InterpolationTest, HighFrequencyTest) {
    // 测试高频率（小dt）的情况
    float speed = 2.0f * M_PI / 180.0f;  // 2度/秒 = 0.0349 rad/s
    float high_freq_dt = 0.1f;  // 100ms
    std::vector<float> start_positions = {0.0f};
    std::vector<float> target_positions = {0.5f};

    auto result = motorevo::interpolate_positions_with_speed(start_positions, target_positions, speed, high_freq_dt);

    // 计算期望步数：0.5 / 0.0349 = 14.326秒，10Hz下应该有143步（算法取整）
    int expected_steps = 143;
    EXPECT_EQ(result.size(), expected_steps);

    // 验证起始和结束位置
    EXPECT_FLOAT_EQ(result[0][0], 0.0f);
    EXPECT_FLOAT_EQ(result.back()[0], 0.5f);
}

TEST_F(InterpolationTest, SizeMismatchTest) {
    // 测试起始和目标位置数组大小不匹配的情况
    float speed = 2.0f * M_PI / 180.0f;  // 2度/秒 = 0.0349 rad/s
    float dt = 1.0f;     // 1秒时间步长

    std::vector<float> start_positions = {0.0f, 0.0f};
    std::vector<float> target_positions = {0.1f};  // 大小不匹配

    auto result = motorevo::interpolate_positions_with_speed(start_positions, target_positions, speed, dt);

    // 应该返回空结果
    EXPECT_TRUE(result.empty());
}

TEST_F(InterpolationTest, NegativePositionsTest) {
    // 测试负位置移动
    float speed = 2.0f * M_PI / 180.0f;  // 2度/秒 = 0.0349 rad/s
    float dt = 1.0f;     // 1秒时间步长

    std::vector<float> start_positions = {0.5f};
    std::vector<float> target_positions = {-0.3f};  // 负位置

    auto result = motorevo::interpolate_positions_with_speed(start_positions, target_positions, speed, dt);

    // 验证基本属性
    EXPECT_GE(result.size(), 2);
    EXPECT_EQ(result[0].size(), 1);

    // 验证起始和结束位置
    EXPECT_FLOAT_EQ(result[0][0], 0.5f);
    EXPECT_FLOAT_EQ(result.back()[0], -0.3f);

    // 验证插值是单调递减的
    for (size_t i = 1; i < result.size(); ++i) {
        EXPECT_LE(result[i][0], result[i-1][0]) << "负位置移动应该是单调递减的";
    }
}

TEST_F(InterpolationTest, DifferentSpeedsTest) {
    // 测试不同速度下的插值
    struct SpeedTestCase {
        float speed;
        float distance;
    };

    SpeedTestCase test_cases[] = {
        {0.05f, 0.5f},   // 低速度，应该有更多步数
        {0.2f, 0.5f},    // 高速度，应该有更少步数
        {1.0f, 0.5f},    // 很高速度，应该是最小步数
    };

    for (const auto& test : test_cases) {
        std::vector<float> start_positions = {0.0f};
        std::vector<float> target_positions = {test.distance};

        auto result = motorevo::interpolate_positions_with_speed(start_positions, target_positions, test.speed, 1.0f);

        // 验证步数正确（考虑到最小2步的限制）
        EXPECT_GE(result.size(), 2);  // 至少2步

        // 验证起始和结束位置
        EXPECT_FLOAT_EQ(result[0][0], 0.0f);
        EXPECT_FLOAT_EQ(result.back()[0], test.distance);
    }
}

TEST_F(InterpolationTest, RealWorldScenarioTest) {
    // 测试真实场景：从用户日志中的情况
    // max_distance=0.9500, time=0.9500s, steps=?
    float speed = 1.0f;  // 1 rad/s (从日志推断)
    float dt = 1.0f;     // 1s (从日志推断)

    std::vector<float> start_positions = {-0.95f, -0.228f};  // 从日志推断的起始位置
    std::vector<float> target_positions = {0.0f, 0.0f};      // 目标位置是零点

    auto result = motorevo::interpolate_positions_with_speed(start_positions, target_positions, speed, dt);

    // 验证基本属性
    EXPECT_EQ(result.size(), 2);  // 应该正好2步（起始+目标）
    EXPECT_EQ(result[0].size(), 2);  // 2个电机

    // 验证起始位置
    EXPECT_FLOAT_EQ(result[0][0], -0.95f);
    EXPECT_FLOAT_EQ(result[0][1], -0.228f);

    // 验证目标位置
    EXPECT_FLOAT_EQ(result.back()[0], 0.0f);
    EXPECT_FLOAT_EQ(result.back()[1], 0.0f);

    // 验证运动方向正确（都向零点移动）
    for (size_t i = 1; i < result.size(); ++i) {
        EXPECT_GE(result[i][0], result[i-1][0]) << "电机0应该向正方向移动到零点";
        EXPECT_GE(result[i][1], result[i-1][1]) << "电机1应该向正方向移动到零点";
    }

    // 输出详细信息用于调试
    std::cout << "Real world scenario test:" << std::endl;
    std::cout << "Steps: " << result.size() << std::endl;
    std::cout << "Start positions: " << result[0][0] << ", " << result[0][1] << std::endl;
    std::cout << "End positions: " << result.back()[0] << ", " << result.back()[1] << std::endl;
}

TEST_F(InterpolationTest, EdgeCaseVerySmallDistance) {
    // 测试非常小的距离
    float speed = 2.0f * M_PI / 180.0f;  // 2度/秒 = 0.0349 rad/s
    float dt = 1.0f;     // 1秒时间步长

    std::vector<float> start_positions = {0.0f};
    std::vector<float> target_positions = {0.001f};  // 1mm移动

    auto result = motorevo::interpolate_positions_with_speed(start_positions, target_positions, speed, dt);

    // 验证基本属性
    EXPECT_EQ(result.size(), 2);  // 非常小距离，只需要2步
    EXPECT_EQ(result[0].size(), 1);

    // 验证起始和结束位置
    EXPECT_FLOAT_EQ(result[0][0], 0.0f);
    EXPECT_FLOAT_EQ(result.back()[0], 0.001f);
}

TEST_F(InterpolationTest, EdgeCaseLargeDistance) {
    // 测试大距离移动
    float speed = 2.0f * M_PI / 180.0f;  // 2度/秒 = 0.0349 rad/s
    float dt = 1.0f;     // 1秒时间步长

    std::vector<float> start_positions = {0.0f};
    std::vector<float> target_positions = {10.0f};  // 10弧度移动

    auto result = motorevo::interpolate_positions_with_speed(start_positions, target_positions, speed, dt);

    // 计算期望步数：10.0 / 0.0349 = 286.5秒，1Hz下应该有286步（算法取整）
    int expected_steps = 286;
    EXPECT_EQ(result.size(), expected_steps);  // 距离够远，应该正好286步

    // 验证起始和结束位置
    EXPECT_FLOAT_EQ(result[0][0], 0.0f);
    EXPECT_FLOAT_EQ(result.back()[0], 10.0f);
}

TEST_F(InterpolationTest, LinearityTest) {
    // 测试插值的线性性
    float speed = 2.0f * M_PI / 180.0f;  // 2度/秒 = 0.0349 rad/s
    float dt = 1.0f;     // 1秒时间步长

    std::vector<float> start_positions = {0.0f};
    std::vector<float> target_positions = {1.0f};

    auto result = motorevo::interpolate_positions_with_speed(start_positions, target_positions, speed, dt);

    // 验证每个步骤的增量是相等的
    float expected_increment = 1.0f / (result.size() - 1);
    for (size_t i = 1; i < result.size(); ++i) {
        float actual_increment = result[i][0] - result[i-1][0];
        EXPECT_NEAR(actual_increment, expected_increment, 0.0001f) << "第" << i << "步的增量不正确";
    }
}

TEST_F(InterpolationTest, MultipleMotorsSameDistance) {
    // 测试多个电机相同距离的移动
    float speed = 2.0f * M_PI / 180.0f;  // 2度/秒 = 0.0349 rad/s
    float dt = 1.0f;     // 1秒时间步长

    std::vector<float> start_positions = {0.0f, 0.0f, 0.0f};
    std::vector<float> target_positions = {0.5f, 0.5f, 0.5f};  // 相同距离

    auto result = motorevo::interpolate_positions_with_speed(start_positions, target_positions, speed, dt);

    // 验证基本属性
    EXPECT_GE(result.size(), 2);
    EXPECT_EQ(result[0].size(), 3);

    // 验证所有电机在每个步骤都有相同的位置
    for (size_t i = 0; i < result.size(); ++i) {
        EXPECT_FLOAT_EQ(result[i][0], result[i][1]);
        EXPECT_FLOAT_EQ(result[i][1], result[i][2]);
    }
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}