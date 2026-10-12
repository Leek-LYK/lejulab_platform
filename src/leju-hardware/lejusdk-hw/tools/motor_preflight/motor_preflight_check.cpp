// 电机固件预检独立 CLI: 不启动整机程序, 单独跑一遍固件版本比对 + 参数写入回读校验。
// 用法:
//   motor_preflight_check <firmware_json> [canbus_yaml]
//   firmware_json: 必填, 如 src/leju-hardware/config/roban_v17/motor_firmware.json
//   canbus_yaml:   留空则用默认路径 ~/.config/lejuconfig/canbus_device_cofig.yaml
// 退出码: 0 = 全部通过, 1 = 存在失败 (明细见输出), 2 = 用法错误
#include <cstdio>
#include <string>

#include "motor_preflight/motor_preflight.h"

int main(int argc, char** argv) {
    if (argc < 2 || argc > 3 || std::string(argv[1]) == "-h") {
        std::fprintf(stderr, "用法: %s <firmware_json> [canbus_yaml]\n", argv[0]);
        std::fprintf(stderr,
                     "  例: %s src/leju-hardware/config/roban_v17/motor_firmware.json\n",
                     argv[0]);
        return 2;
    }
    motor_preflight::PreflightOptions options;
    options.firmware_json_path = argv[1];
    if (argc > 2) {
        options.canbus_yaml_path = argv[2];
    }
    return motor_preflight::RunMotorParamPreflight(options) ? 0 : 1;
}
