#include "motor_preflight/motor_preflight.h"

#include <unistd.h>

#include <cstdio>
#include <exception>

#include "canbus_sdk/config_parser.h"
#include "motor_preflight/socketcan_param_transport.h"

namespace motor_preflight {

namespace {

// 写入后到回读之间的间隔, 留给电机固件应用参数
constexpr int kWriteSettleUs = 5000;

std::string Hex32(uint32_t v) {
    char buf[16];
    std::snprintf(buf, sizeof(buf), "0x%08X", v);
    return buf;
}

std::string FormatValue(const ParamValue& v) {
    char buf[32];
    if (v.dtype == ParamDtype::kFloat) {
        std::snprintf(buf, sizeof(buf), "%.6g", static_cast<double>(v.f32));
    } else {
        std::snprintf(buf, sizeof(buf), "%u(0x%X)", v.u32, v.u32);
    }
    return buf;
}

// 读一个参数: Transact + 解码 (out->dtype 需预置)
bool ReadParam(ParamTransport& transport, uint8_t motor_id, uint8_t idx, ParamValue* out) {
    std::array<uint8_t, 8> resp{};
    if (!transport.Transact(motor_id, BuildReadFrame(idx), &resp)) {
        return false;
    }
    return DecodeResponse(resp.data(), resp.size(), idx, out);
}

void CheckOneMotor(ParamTransport& transport, const PreflightConfig& config,
                   MotorCheckResult& result) {
    const uint8_t id = result.item.motor_id;

    // 1. 设备名 → 型号
    auto map_it = config.device_map.find(result.item.device_name);
    if (map_it == config.device_map.end()) {
        result.errors.push_back("device_map 缺少设备 \"" + result.item.device_name +
                                "\" 的型号条目");
        return;
    }
    result.model = map_it->second;
    const ModelSpec& spec = config.models.at(result.model);

    // 2. 固件版本比对 (idx10, hex32), 命中白名单任一版本即通过。
    //    版本不匹配仅告警不拦截启动; 读取无响应(通信问题)仍算硬错误
    ParamValue fw;
    fw.dtype = ParamDtype::kUint32;
    if (!ReadParam(transport, id, kFirmwareVersionIdx, &fw)) {
        result.errors.push_back("固件版本读取无响应");
        return;
    }
    result.fw_read = fw.u32;
    bool fw_hit = false;
    for (uint32_t allowed : spec.firmware_versions) {
        if (fw.u32 == allowed) {
            fw_hit = true;
            break;
        }
    }
    if (fw_hit) {
        result.fw_valid = true;
    } else {
        std::string allowed_list;
        for (uint32_t allowed : spec.firmware_versions) {
            if (!allowed_list.empty()) allowed_list += "/";
            allowed_list += Hex32(allowed);
        }
        result.warnings.push_back("固件版本不匹配(仅警告): 期望 " + allowed_list + " 实际 " +
                                  Hex32(fw.u32));
    }

    // 3. 逐参数: 写入 → 回读 → 校验 (硬门控, 失败拒绝启动)
    result.params_total = static_cast<int>(spec.params.size());
    for (const ParamSpec& p : spec.params) {
        const std::string idx_str = "参数 idx" + std::to_string(p.idx);
        if (!transport.SendOnly(id, BuildWriteFrame(p.idx, p.value))) {
            result.errors.push_back(idx_str + " 写入帧发送失败");
            continue;
        }
        usleep(kWriteSettleUs);
        ParamValue readback;
        readback.dtype = p.value.dtype;
        if (!ReadParam(transport, id, p.idx, &readback)) {
            result.errors.push_back(idx_str + " 写入后回读无响应");
            continue;
        }
        if (!ValueMatches(p.value, readback)) {
            result.errors.push_back(idx_str + " 回读不一致: 期望 " + FormatValue(p.value) +
                                    " 实际 " + FormatValue(readback));
            continue;
        }
        result.params_ok++;
    }

    result.ok = result.errors.empty();
}

}  // namespace

std::vector<MotorCheckResult> CheckMotorsOnBus(ParamTransport& transport,
                                               const std::vector<MotorCheckItem>& items,
                                               const PreflightConfig& config) {
    std::vector<MotorCheckResult> results;
    results.reserve(items.size());
    for (const MotorCheckItem& item : items) {
        MotorCheckResult result;
        result.item = item;
        CheckOneMotor(transport, config, result);
        results.push_back(std::move(result));
    }
    return results;
}

namespace {

// 失败项红色加粗、告警项黄色加粗打印, 保证在启动日志里一眼可见
constexpr const char* kAnsiRed = "\033[1;31m";
constexpr const char* kAnsiYellow = "\033[1;33m";
constexpr const char* kAnsiReset = "\033[0m";

void PrintReport(const std::vector<MotorCheckResult>& results) {
    int passed = 0;
    int warned = 0;
    for (const MotorCheckResult& r : results) {
        const std::string model = r.model.empty() ? "?" : r.model;
        if (r.ok) {
            passed++;
            std::printf("[MotorPreflight] [%s] M%02d %s (%s): FW %s %s, params %d/%d\n",
                        r.item.bus.c_str(), r.item.motor_id, r.item.device_name.c_str(),
                        model.c_str(), Hex32(r.fw_read).c_str(),
                        r.fw_valid ? "OK" : "MISMATCH(warn)", r.params_ok, r.params_total);
        } else {
            std::fprintf(stderr, "%s[MotorPreflight] [%s] M%02d %s (%s): FAILED%s\n", kAnsiRed,
                         r.item.bus.c_str(), r.item.motor_id, r.item.device_name.c_str(),
                         model.c_str(), kAnsiReset);
            for (const std::string& e : r.errors) {
                std::fprintf(stderr, "%s[MotorPreflight]     - %s%s\n", kAnsiRed, e.c_str(),
                             kAnsiReset);
            }
        }
        if (!r.warnings.empty()) {
            warned++;
            for (const std::string& w : r.warnings) {
                std::fprintf(stderr, "%s[MotorPreflight]     ! %s%s\n", kAnsiYellow, w.c_str(),
                             kAnsiReset);
            }
        }
    }
    const size_t failed = results.size() - passed;
    if (warned > 0) {
        std::fprintf(stderr, "%s[MotorPreflight] 预检警告: %d 个电机固件版本不符 (仅告警, 未拦截启动)%s\n",
                     kAnsiYellow, warned, kAnsiReset);
    }
    if (failed > 0) {
        std::fprintf(stderr, "%s[MotorPreflight] 预检结果: %d 通过 / %zu 失败%s\n", kAnsiRed,
                     passed, failed, kAnsiReset);
    } else {
        std::printf("[MotorPreflight] 预检结果: %d 通过 / 0 失败\n", passed);
    }
    std::fflush(stdout);
}

}  // namespace

bool RunMotorParamPreflight(const PreflightOptions& options) {
    // 1. 解析 canbus yaml, 收集 SOCKETCAN 总线上的电机
    std::string yaml_path = options.canbus_yaml_path;
    canbus_sdk::ConfigParser parser;
    try {
        if (yaml_path.empty()) {
            yaml_path = canbus_sdk::ConfigParser::getDefaultConfigFilePath();
        }
        // yaml 不存在 = 该机器不走 canbus_sdk (如老 BMAPI 路径), 预检不适用;
        // 存在但解析失败 = 部署问题, 必须拒绝启动
        if (access(yaml_path.c_str(), R_OK) != 0) {
            std::printf("[MotorPreflight] canbus 配置不存在 (%s), 预检跳过\n",
                        yaml_path.c_str());
            return true;
        }
        if (!parser.parseFromFile(yaml_path)) {
            std::fprintf(stderr, "[MotorPreflight] canbus 配置解析失败 (%s)\n",
                         yaml_path.c_str());
            return false;
        }
    } catch (const std::exception& e) {
        std::fprintf(stderr, "[MotorPreflight] canbus 配置解析失败 (%s): %s\n",
                     yaml_path.c_str(), e.what());
        return false;
    }

    struct BusPlan {
        std::string name;
        bool use_fd;
        std::vector<MotorCheckItem> items;
    };
    std::vector<BusPlan> plans;
    size_t total_motors = 0;
    for (const canbus_sdk::CanBusConfig& bus : parser.getCanBusConfigs()) {
        if (!bus.enable) {
            continue;
        }
        if (bus.type != canbus_sdk::CanBusModelType::SOCKETCAN) {
            std::fprintf(stderr,
                         "[MotorPreflight] 警告: 总线 %s 非 SocketCAN, 预检不支持, 跳过\n",
                         bus.name.c_str());
            continue;
        }
        BusPlan plan;
        plan.name = bus.name;
        // FD 判定与 canbus_sdk SocketCanWrapper 一致
        plan.use_fd = (bus.bitrate.dbitrate > 0 && bus.bitrate.dbitrate != bus.bitrate.nbitrate);
        for (const canbus_sdk::DeviceConfig& dev :
             parser.getDevices(bus.name, canbus_sdk::DeviceType::MOTOR)) {
            if (dev.ignore) {
                continue;
            }
            plan.items.push_back({bus.name, static_cast<uint8_t>(dev.device_id), dev.name});
        }
        if (!plan.items.empty()) {
            total_motors += plan.items.size();
            plans.push_back(std::move(plan));
        }
    }

    // 2. 整机没有 SOCKETCAN 电机 (如 x86 BMAPI 路径): 预检不适用, 直接通过
    if (total_motors == 0) {
        std::printf("[MotorPreflight] 未发现 SocketCAN 电机设备, 预检跳过\n");
        return true;
    }

    // 3. 加载固件预检配置 (存在 SOCKETCAN 电机时必须提供, 配置随代码走)
    const std::string& json_path = options.firmware_json_path;
    if (json_path.empty()) {
        std::fprintf(stderr, "%s[MotorPreflight] 未指定固件预检配置路径 (motor_firmware.json)%s\n",
                     kAnsiRed, kAnsiReset);
        return false;
    }
    PreflightConfig config;
    try {
        config = LoadPreflightConfig(json_path);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "%s[MotorPreflight] %s%s\n", kAnsiRed, e.what(), kAnsiReset);
        std::fprintf(stderr,
                     "%s[MotorPreflight] 请补全固件预检配置 (随代码位于 "
                     "src/leju-hardware/config/<robot_type>/motor_firmware.json)%s\n",
                     kAnsiRed, kAnsiReset);
        return false;
    }

    // 4. 逐总线检查
    std::printf("[MotorPreflight] 开始电机固件预检: %zu 条总线, %zu 个电机 (配置: %s)\n",
                plans.size(), total_motors, json_path.c_str());
    std::fflush(stdout);
    bool all_ok = true;
    std::vector<MotorCheckResult> all_results;
    for (const BusPlan& plan : plans) {
        SocketCanParamTransport transport(options.timeout_ms, options.retries);
        if (!transport.Open(plan.name, plan.use_fd)) {
            std::fprintf(stderr, "%s[MotorPreflight] 总线 %s 打开失败, 该总线电机无法预检%s\n",
                         kAnsiRed, plan.name.c_str(), kAnsiReset);
            all_ok = false;
            continue;
        }
        std::vector<MotorCheckResult> results = CheckMotorsOnBus(transport, plan.items, config);
        all_results.insert(all_results.end(), results.begin(), results.end());
    }

    // 5. 汇总报告
    PrintReport(all_results);
    for (const MotorCheckResult& r : all_results) {
        all_ok = all_ok && r.ok;
    }
    if (!all_ok) {
        std::fprintf(stderr, "%s[MotorPreflight] 预检未通过, 拒绝初始化硬件层%s\n", kAnsiRed,
                     kAnsiReset);
    }
    return all_ok;
}

}  // namespace motor_preflight
