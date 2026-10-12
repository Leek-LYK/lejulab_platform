#include "motor_preflight/preflight_config.h"

#include <fstream>
#include <stdexcept>

#include "utils/json.hpp"

namespace motor_preflight {

namespace {

using nlohmann::json;

// "0x02030001" / "100" / 数字 → uint32
uint32_t ParseUint32(const json& node, const std::string& where) {
    try {
        if (node.is_string()) {
            return static_cast<uint32_t>(std::stoul(node.get<std::string>(), nullptr, 0));
        }
        if (node.is_number_unsigned() || node.is_number_integer()) {
            const int64_t v = node.get<int64_t>();
            if (v < 0 || v > static_cast<int64_t>(UINT32_MAX)) {
                throw std::out_of_range("out of uint32 range");
            }
            return static_cast<uint32_t>(v);
        }
    } catch (const std::exception& e) {
        throw std::runtime_error(where + ": 无法解析为 uint32: " + e.what());
    }
    throw std::runtime_error(where + ": 无法解析为 uint32 (需要数字或 \"0x..\" 字符串)");
}

ParamSpec ParseParamSpec(const json& node, const std::string& model_name) {
    const std::string where = "models." + model_name + ".params";
    if (!node.is_object() || !node.contains("idx") || !node.contains("dtype") ||
        !node.contains("value")) {
        throw std::runtime_error(where + ": 每项必须包含 idx/dtype/value");
    }
    const int idx = node["idx"].get<int>();
    if (idx < 0 || idx > 0xFF) {
        throw std::runtime_error(where + ": idx 超出范围 [0,255]: " + std::to_string(idx));
    }
    if (idx == kFirmwareVersionIdx) {
        throw std::runtime_error(where + ": idx 10 为固件版本(只读), 不允许写入");
    }

    ParamSpec spec;
    spec.idx = static_cast<uint8_t>(idx);
    const std::string dtype = node["dtype"].get<std::string>();
    const std::string value_where = where + "[idx=" + std::to_string(idx) + "].value";
    if (dtype == "float") {
        if (!node["value"].is_number()) {
            throw std::runtime_error(value_where + ": float 类型需要数字值");
        }
        spec.value.dtype = ParamDtype::kFloat;
        spec.value.f32 = node["value"].get<float>();
    } else if (dtype == "uint32" || dtype == "hex32") {
        spec.value.dtype = ParamDtype::kUint32;
        spec.value.u32 = ParseUint32(node["value"], value_where);
    } else {
        throw std::runtime_error(where + ": 未知 dtype \"" + dtype +
                                 "\" (支持 float/uint32/hex32)");
    }
    return spec;
}

}  // namespace

PreflightConfig LoadPreflightConfig(const std::string& json_path) {
    std::ifstream ifs(json_path);
    if (!ifs.is_open()) {
        throw std::runtime_error("无法打开固件预检配置文件: " + json_path);
    }

    json root;
    try {
        root = json::parse(ifs);
    } catch (const std::exception& e) {
        throw std::runtime_error("固件预检配置 JSON 解析失败 (" + json_path + "): " + e.what());
    }

    if (!root.contains("models") || !root["models"].is_object()) {
        throw std::runtime_error("固件预检配置缺少 models 对象: " + json_path);
    }

    PreflightConfig config;
    for (auto it = root["models"].begin(); it != root["models"].end(); ++it) {
        const std::string& model_name = it.key();
        const json& model_node = it.value();
        if (!model_node.contains("firmware_version")) {
            throw std::runtime_error("models." + model_name + ": 缺少 firmware_version");
        }
        ModelSpec spec;
        const json& fw_node = model_node["firmware_version"];
        const std::string fw_where = "models." + model_name + ".firmware_version";
        if (fw_node.is_array()) {
            if (fw_node.empty()) {
                throw std::runtime_error(fw_where + ": 白名单数组不能为空");
            }
            for (const auto& v : fw_node) {
                spec.firmware_versions.push_back(ParseUint32(v, fw_where));
            }
        } else {
            spec.firmware_versions.push_back(ParseUint32(fw_node, fw_where));
        }
        if (model_node.contains("params")) {
            if (!model_node["params"].is_array()) {
                throw std::runtime_error("models." + model_name + ".params 必须是数组");
            }
            for (const auto& p : model_node["params"]) {
                spec.params.push_back(ParseParamSpec(p, model_name));
            }
        }
        config.models[model_name] = std::move(spec);
    }

    if (root.contains("device_map")) {
        if (!root["device_map"].is_object()) {
            throw std::runtime_error("device_map 必须是对象: " + json_path);
        }
        for (auto it = root["device_map"].begin(); it != root["device_map"].end(); ++it) {
            const std::string& model = it.value().get<std::string>();
            if (config.models.find(model) == config.models.end()) {
                throw std::runtime_error("device_map." + it.key() + " 引用了未定义的型号 \"" +
                                         model + "\"");
            }
            config.device_map[it.key()] = model;
        }
    }

    return config;
}

}  // namespace motor_preflight
