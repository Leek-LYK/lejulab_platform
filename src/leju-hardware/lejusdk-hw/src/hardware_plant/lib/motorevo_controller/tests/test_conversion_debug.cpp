#include "motorevo/motor_def.h"
#include <iostream>
#include <iomanip>
#include <vector>

void test_conversion(uint8_t bits, float min_val, float max_val, const std::string& name) {
    std::cout << "\n=== " << name << " (" << (int)bits << "位转换) ===\n";
    std::cout << "范围: [" << min_val << ", " << max_val << "]\n";
    std::cout << std::left << std::setw(15) << "原始值"
              << std::setw(15) << "转换后"
              << std::setw(15) << "转换回"
              << "误差\n";
    std::cout << std::string(60, '-') << "\n";

    // 测试边界值和中间值
    std::vector<float> test_values = {
        min_val,          // 最小值
        max_val,          // 最大值
        0.0f,             // 零值
        (min_val + max_val) * 0.25f,  // 1/4位置
        (min_val + max_val) * 0.5f,   // 中间值
        (min_val + max_val) * 0.75f,  // 3/4位置
        min_val - 1.0f,   // 超出下限
        max_val + 1.0f    // 超出上限
    };

    for (float original : test_values) {
        uint32_t int_val = 0;
        float converted_back = original;  // 默认值，如果bits不匹配则不转换

        // 根据位数调用相应的转换函数
        if (bits == 8) {
            int_val = motorevo::float_to_int<8>(original, min_val, max_val);
            converted_back = motorevo::int_to_float<8>(int_val, min_val, max_val);
        } else if (bits == 12) {
            int_val = motorevo::float_to_int<12>(original, min_val, max_val);
            converted_back = motorevo::int_to_float<12>(int_val, min_val, max_val);
        } else if (bits == 16) {
            int_val = motorevo::float_to_int<16>(original, min_val, max_val);
            converted_back = motorevo::int_to_float<16>(int_val, min_val, max_val);
        }

        float error = std::abs(converted_back - original);

        std::cout << std::fixed << std::setprecision(6)
                  << std::setw(15) << original
                  << std::setw(15) << int_val
                  << std::setw(15) << converted_back
                  << error << "\n";
    }
}

int main() {
    std::cout << "=== 数值转换测试程序 ===\n";
    std::cout << "测试 float_to_int 和 int_to_float 函数的转换精度\n";

    // 测试位置转换 (16位)
    test_conversion(16, motorevo::kCAN_COM_THETA_MIN, motorevo::kCAN_COM_THETA_MAX, "位置转换");

    // 测试速度转换 (12位)
    test_conversion(12, motorevo::kCAN_COM_VELOCITY_MIN, motorevo::kCAN_COM_VELOCITY_MAX, "速度转换");

    // 测试力矩转换 (12位)
    test_conversion(12, motorevo::kCAN_COM_TORQUE_MIN, motorevo::kCAN_COM_TORQUE_MAX, "力矩转换");

    // 测试8位通用转换
    test_conversion(8, -10.0f, 10.0f, "8位通用转换");

    std::cout << "\n=== 边界值专项测试 ===\n";

    // 测试各种位数的最大值和最小值
    struct BitInfo {
        uint8_t bits;
        uint32_t max_int;
        std::string name;
    };

    std::vector<BitInfo> bit_infos = {
        {8, 255, "8位"},
        {12, 4095, "12位"},
        {16, 65535, "16位"}
    };

    for (const auto& info : bit_infos) {
        std::cout << "\n" << info.name << "范围: 0 ~ " << info.max_int << "\n";
        std::cout << "测试值: 0, " << info.max_int/2 << ", " << info.max_int << ", " << info.max_int+1 << "\n";

        std::vector<uint32_t> test_ints = {0, info.max_int/2, info.max_int, info.max_int+1};

        for (uint32_t int_val : test_ints) {
            float result;
            if (info.bits == 8) {
                result = motorevo::int_to_float<8>(int_val, -100.0f, 100.0f);
            } else if (info.bits == 12) {
                result = motorevo::int_to_float<12>(int_val, -100.0f, 100.0f);
            } else {
                result = motorevo::int_to_float<16>(int_val, -100.0f, 100.0f);
            }

            std::cout << "  输入: " << std::setw(6) << int_val
                      << " -> 输出: " << std::fixed << std::setprecision(4) << result << "\n";
        }
    }

    std::cout << "\n=== 测试完成 ===\n";
    return 0;
}