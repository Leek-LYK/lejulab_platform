#include "motorevo/motor_def.h"
#include <iostream>
#include <iomanip>
#include <cmath>

void analyze_quantization_error(uint8_t bits, float min_val, float max_val, const std::string& name) {
    std::cout << "\n=== " << name << " (" << (int)bits << "位) 量化误差分析 ===\n";
    std::cout << "范围: [" << min_val << ", " << max_val << "]\n";

    uint32_t max_int = (1 << bits) - 1;
    float range = max_val - min_val;
    float resolution = range / max_int;  // 每个整数代表的数值大小

    std::cout << "最大整数值: " << max_int << "\n";
    std::cout << "分辨率: " << std::scientific << resolution << " (每个整数代表的数值)\n";
    std::cout << "最大理论误差: ±" << std::fixed << resolution/2 << "\n";

    std::cout << "\n零点附近的量化情况:\n";
    std::cout << std::setw(10) << "整数值" << std::setw(15) << "对应浮点值"
              << std::setw(15) << "与0的距离" << "\n";
    std::cout << std::string(45, '-') << "\n";

    // 查看零点附近的几个整数
    int center = max_int / 2;
    for (int i = center - 2; i <= center + 2; ++i) {
        if (i >= 0 && i <= (int)max_int) {
            float float_val;
            if (bits == 8) {
                float_val = motorevo::int_to_float<8>(i, min_val, max_val);
            } else if (bits == 12) {
                float_val = motorevo::int_to_float<12>(i, min_val, max_val);
            } else {
                float_val = motorevo::int_to_float<16>(i, min_val, max_val);
            }

            std::cout << std::setw(10) << i
                      << std::setw(15) << std::fixed << std::setprecision(6) << float_val
                      << std::setw(15) << std::abs(float_val) << "\n";
        }
    }
}


int main() {
    std::cout << "=== 量化误差分析程序 ===\n";
    std::cout << "分析浮点数到整数转换的量化误差\n";

    // 分析不同位数的量化误差
    analyze_quantization_error(8, -10.0f, 10.0f, "8位转换");
    analyze_quantization_error(12, motorevo::kCAN_COM_TORQUE_MIN, motorevo::kCAN_COM_TORQUE_MAX, "12位力矩转换");
    analyze_quantization_error(16, motorevo::kCAN_COM_THETA_MIN, motorevo::kCAN_COM_THETA_MAX, "16位位置转换");

    return 0;
}