#include "canbus_sdk/config_parser.h"
#include "canbus_sdk/canbus_log.h"
#include "canbus_sdk/result.h"
#include "canbus_sdk/canbus_sdk.h"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>
#include <set>
#include <unistd.h>
#include <pwd.h>
#include <algorithm>
#include <sstream>
#include <iomanip>

namespace canbus_sdk {

/////////////////////////////////////////////////////////////////////////////////////////
// Helper Functions
/////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief 解析单个CAN总线接口配置
 */
static Result<CanBusConfig> ParseCanBusConfig(const std::string& name, const YAML::Node* node) {

    CanBusConfig config;
    config.name = name;

    // 检查必需字段是否存在
    std::vector<std::string> required_fields = {"type", "name", "nbitrate", "dbitrate", "nsamplepos", "dsamplepos"};
    std::vector<std::string> missing_fields;

    for (const auto& field : required_fields) {
        if (!(*node)[field].IsDefined()) {
            missing_fields.push_back(field);
        }
    }

    if (!missing_fields.empty()) {
        std::string missing_str;
        for (size_t i = 0; i < missing_fields.size(); ++i) {
            if (i > 0) missing_str += ", ";
            missing_str += missing_fields[i];
        }
        LOG_FAILURE("[ConfigParser] 配置文件解析失败, %s缺少必需字段: %s", name.c_str(), missing_str.c_str());
        throw std::runtime_error("Missing required fields for " + name + ": " + missing_str);
    }

    try {
        // 解析类型
        std::string type_str = (*node)["type"].as<std::string>();
        config.type = canbus_model_from_string(type_str);

        // 解析设备名称
        config.device_name = (*node)["name"].as<std::string>();

        // 解析波特率配置
        config.bitrate.nbitrate = (*node)["nbitrate"].as<int>();
        config.bitrate.dbitrate = (*node)["dbitrate"].as<int>();
        config.bitrate.nsamplepos = (*node)["nsamplepos"].as<int>();
        config.bitrate.dsamplepos = (*node)["dsamplepos"].as<int>();

        // [可选字段]解析启用状态
        if ((*node)["enable"]) {
            config.enable = (*node)["enable"].as<bool>();
        } else {
            config.enable = true; // 默认启用
        }

    } catch (const YAML::Exception& e) {
        LOG_FAILURE("[ConfigParser] 配置文件解析失败, 解析%s的CAN总线配置时出错: %s", name.c_str(), e.what());
        return Result<CanBusConfig>::error(static_cast<int>(CanBusError::ERROR_INVALID_PARAMETER));
    }

    return Result<CanBusConfig>::ok(config);
}

/**
 * @brief 解析CAN总线接口配置
 */
static bool ParseCanBusInterfaces(const YAML::Node& node, std::vector<CanBusConfig>& canbus_configs) {

    if (!node.IsMap()) {
        LOG_FAILURE("[ConfigParser] 配置文件解析失败, CAN总线配置文件无效, 'canbus_interfaces'格式不正确");
        throw std::runtime_error("Invalid canbus config file, 'canbus_interfaces' must be a map");
    }

    for (const auto& kv : node) {
        // LOG_D("[ConfigParser] 解析CAN总线接口%s的配置", kv.first.as<std::string>().c_str());
        std::string canbus_name = kv.first.as<std::string>();

        auto result = ParseCanBusConfig(canbus_name, &kv.second);
        if (!result) {
            return false;
        }
        canbus_configs.push_back(result.value());
    }

    return true;
}

/**
 * @brief 解析单个设备配置
 */
static Result<DeviceConfig> ParseDeviceConfig(const YAML::Node* node, const std::string& canbus_name) {

    DeviceConfig config;
    config.canbus_name = canbus_name;

    // 检查必需字段是否存在
    std::vector<std::string> required_fields = {"name", "class", "device_id"};
    std::vector<std::string> missing_fields;

    for (const auto& field : required_fields) {
        if (!(*node)[field].IsDefined()) {
            missing_fields.push_back(field);
        }
    }

    if (!missing_fields.empty()) {
        std::string missing_str;
        for (size_t i = 0; i < missing_fields.size(); ++i) {
            if (i > 0) missing_str += ", ";
            missing_str += missing_fields[i];
        }
        LOG_FAILURE("[ConfigParser] 配置文件解析失败, %s缺少必需的设备字段: %s", canbus_name.c_str(), missing_str.c_str());
        throw std::runtime_error("Missing required device fields for " + canbus_name + ": " + missing_str);
    }

    try {
        // 解析设备名称
        config.name = (*node)["name"].as<std::string>();

        // 解析设备类型
        std::string type_str = (*node)["class"].as<std::string>();
        config.device_type = device_type_from_string(type_str);

        // 解析设备ID
        config.device_id = (*node)["device_id"].as<DeviceId>();

        // 解析忽略状态（可选）
        if ((*node)["ignore"]) {
            config.ignore = (*node)["ignore"].as<bool>();
        } else {
            config.ignore = false; // 默认不忽略
        }

    } catch (const YAML::Exception& e) {
        LOG_FAILURE("[ConfigParser]配置文件解析失败,  解析%s的设备配置时出错: %s", canbus_name.c_str(), e.what());
        return Result<DeviceConfig>::error(static_cast<int>(CanBusError::ERROR_INVALID_PARAMETER));
    }

    // 根据设备类型解析特定参数
    if (config.device_type == DeviceType::MOTOR || config.device_type == DeviceType::LEJUCLAW) {
        // 电机专用配置 - params是必选字段
        if (!(*node)["params"].IsDefined()) {
            LOG_FAILURE("[ConfigParser] 配置文件解析失败, 电机设备%s缺少必需的'params'字段", config.name.c_str());
            throw std::runtime_error("Missing required 'params' field for motor device " + config.name);
        }
        config.motor_params = (*node)["params"].as<std::vector<float>>();

        // 电机专用配置 - ratio是必选字段
        if (!(*node)["ratio"].IsDefined()) {
            LOG_FAILURE("[ConfigParser] 配置文件解析失败, 电机设备%s缺少必需的'ratio'字段", config.name.c_str());
            throw std::runtime_error("Missing required 'ratio' field for motor device " + config.name);
        }
        config.ratio = (*node)["ratio"].as<float>();

        // 解析可选的negtive字段
        if ((*node)["negtive"]) {
            config.negtive = (*node)["negtive"].as<bool>();
        } else {
            config.negtive = false; // 默认false
        }
    } else {
        // 灵巧手和其他设备类型没有额外配置
        config.negtive = false; // 其他设备，此时该字段无意义
        config.motor_params = {}; // 其他设备，此时该字段无意义
        config.ratio = 36.0f; // 其他设备，此时该字段无意义
    }

    return Result<DeviceConfig>::ok(config);
}

/**
 * @brief 验证配置有效性
 */
static void ValidateConfig(const std::vector<CanBusConfig>& canbus_configs, const std::map<std::string, std::vector<DeviceConfig>>& canbus_to_devices) {
    // 检查是否有CAN总线接口配置
    if (canbus_configs.empty()) {
        LOG_FAILURE("[ConfigParser] 配置文件解析错误, 未找到CAN总线接口配置, 可能是'canbus_interfaces'字段为空或者列表为空");
        throw std::runtime_error("No CAN bus interface configurations found");
    }

    // 检查CAN总线名称是否重复或为空
    std::set<std::string> canbus_names;
    for (const auto& canbus : canbus_configs) {
        // 检查名称是否为空
        if (canbus.name.empty()) {
            LOG_FAILURE("[ConfigParser] 配置文件解析错误, CAN总线名称不能为空");
            throw std::runtime_error("CAN bus name cannot be empty");
        }

        // 检查名称是否重复
        if (canbus_names.find(canbus.name) != canbus_names.end()) {
            LOG_FAILURE("[ConfigParser] 配置文件解析错误, 发现重复的CAN总线名称: %s", canbus.name.c_str());
            throw std::runtime_error("Duplicate CAN bus name found: " + canbus.name);
        }
        canbus_names.insert(canbus.name);
    }

    // 检查设备名称是否重复
    std::set<std::string> device_names;
    for (const auto& pair : canbus_to_devices) {
        for (const auto& device : pair.second) {
            if (device_names.find(device.name) != device_names.end()) {
                LOG_FAILURE("[ConfigParser] 配置文件解析错误, 发现重复的设备名称: %s", device.name.c_str());
                throw std::runtime_error("Duplicate device name found: " + device.name);
            }
            device_names.insert(device.name);
        }
    }

    // 检查设备ID是否在同一CAN总线上重复（结合设备类型）
    for (const auto& pair : canbus_to_devices) {
        std::set<std::pair<DeviceId, DeviceType>> device_id_type_pairs;
        for (const auto& device : pair.second) {
            if (device.device_id == 0) { // 0是无效ID
                LOG_FAILURE("[ConfigParser] 配置文件解析错误, 设备ID不能为0x0, 如果您想忽略/禁用该设备，请将'ignore'字段设置为true");
                throw std::runtime_error("Device ID cannot be 0");
            }

            // 检查相同设备类型+ID的组合是否重复
            auto id_type_pair = std::make_pair(device.device_id, device.device_type);
            if (device_id_type_pairs.find(id_type_pair) != device_id_type_pairs.end()) {
                LOG_FAILURE("[ConfigParser] 配置文件解析错误, 发现重复的设备ID和类型组合: ID=%d, 类型=%s",
                           device.device_id, device.device_type == DeviceType::REVO1_HAND ? "REVO1_HAND" :
                           device.device_type == DeviceType::REVO2_HAND ? "REVO2_HAND" :
                           device.device_type == DeviceType::MOTOR ? "MOTOR" :
                           device.device_type == DeviceType::LEJUCLAW ? "LEJUCLAW" : "UNKNOWN");
                throw std::runtime_error("Duplicate device ID and type combination found: ID=" + std::to_string(device.device_id) + ", Type=" + std::to_string(static_cast<int>(device.device_type)));
            }
            device_id_type_pairs.insert(id_type_pair);
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
// Class Member Functions
/////////////////////////////////////////////////////////////////////////////////////////

bool ConfigParser::parseFromFile(const std::string& config_path) {
    // 清空现有配置
    clear();

    // 检查文件是否存在
    std::ifstream file(config_path);
    if (!file.good()) {
        LOG_FAILURE("[ConfigParser] 配置文件不存在: %s", config_path.c_str());
        return false;
    }

    // 解析YAML文件 - 只包住可能抛出YAML异常的操作
    YAML::Node config;
    try {
        config = YAML::LoadFile(config_path);
    } catch (const YAML::Exception& e) {
        LOG_E("[ConfigParser] 解析配置文件时出错: %s", e.what());
        return false;
    }

    LOG_SUCCESS("[ConfigParser] 配置文件路径: %s", config_path.c_str());

    // 解析CAN总线接口配置 - canbus_interfaces是必需字段
    if (!config["canbus_interfaces"]) {
        LOG_FAILURE("[ConfigParser] 配置文件必须包含'canbus_interfaces'部分");
        throw std::runtime_error("Configuration file must contain 'canbus_interfaces' section");
    }

    if (!ParseCanBusInterfaces(config["canbus_interfaces"], canbus_configs_)) {
        LOG_FAILURE("[ConfigParser] 解析配置文件'canbus_interfaces'字段失败");
        return false;
    }

    // 解析设备配置 - 遍历所有CAN总线，解析对应的设备配置
    for (const auto& canbus_config : canbus_configs_) {
        std::string devices_key = canbus_config.name + "_devices";
        if (!config[devices_key]) {
            LOG_FAILURE("[ConfigParser] 缺少%s的设备配置", devices_key.c_str());
            throw std::runtime_error("Missing device configuration for " + canbus_config.name);
        }

        // 检查设备配置是否为数组格式
        if (!config[devices_key].IsSequence()) {
            LOG_FAILURE("[ConfigParser] %s的设备配置必须是数组格式", canbus_config.name.c_str());
            throw std::runtime_error("Device configuration must be a sequence for " + canbus_config.name);
        }

        // 解析设备配置
        for (const auto& device_node : config[devices_key]) {
            auto result = ParseDeviceConfig(&device_node, canbus_config.name);
            if (!result) {
                LOG_FAILURE("[ConfigParser] 解析%s的设备配置失败", canbus_config.name.c_str());
                throw std::runtime_error("Failed to parse device configuration for " + canbus_config.name);
            }
            canbus_to_devices_[canbus_config.name].push_back(result.value());
        }
    }

    // 验证配置有效性
    ValidateConfig(canbus_configs_, canbus_to_devices_);

    return true;
}

const std::vector<CanBusConfig>& ConfigParser::getCanBusConfigs() const {
    return canbus_configs_;
}

std::vector<DeviceConfig> ConfigParser::getDevices(const std::string& canbus_name) const {
    auto it = canbus_to_devices_.find(canbus_name);
    if (it != canbus_to_devices_.end()) {
        return it->second;
    }
    return {};
}

std::vector<DeviceConfig> ConfigParser::getDevices(const std::string& canbus_name, DeviceType device_type) const {
    auto it = canbus_to_devices_.find(canbus_name);
    if (it == canbus_to_devices_.end()) {
        return {};
    }

    std::vector<DeviceConfig> result;
    std::copy_if(it->second.begin(), it->second.end(), std::back_inserter(result),
                 [device_type](const DeviceConfig& device) {
                     return device.device_type == device_type;
                 });
    return result;
}

void ConfigParser::clear() {
    canbus_configs_.clear();
    canbus_to_devices_.clear();
}

bool ConfigParser::updateDeviceIgnore(const std::string& device_name, bool ignore) {
    // 遍历所有CAN总线和设备
    for (auto& pair : canbus_to_devices_) {
        for (auto& device : pair.second) {
            if (device.name == device_name) {
                device.ignore = ignore;
                return true;
            }
        }
    }
    return false; // 设备不存在
}

bool ConfigParser::setMotorNegative(const std::string& device_name, bool negtive) {
    // 遍历所有CAN总线和设备
    for (auto& pair : canbus_to_devices_) {
        for (auto& device : pair.second) {
            if (device.name == device_name) {
                // 检查是否为电机类型
                if (device.device_type == DeviceType::MOTOR || device.device_type == DeviceType::LEJUCLAW) {
                    device.negtive = negtive;
                    return true;
                } else {
                    return false; // 不是电机
                }
            }
        }
    }
    return false; // 设备不存在
}

void ConfigParser::printConfig() const {
    std::cout << "=== CAN总线配置解析结果 ===" << std::endl;

    // 打印CAN总线接口配置
    std::cout << "\nCAN总线接口配置:" << std::endl;
    for (const auto& canbus : canbus_configs_) {
        std::cout << "  " << canbus.name << ":" << std::endl;
        std::cout << "    类型: " << to_string(canbus.type) << std::endl;
        std::cout << "    设备名称: " << canbus.device_name << std::endl;
        std::cout << "    波特率: " << canbus.bitrate.nbitrate << "/" << canbus.bitrate.dbitrate << std::endl;
        std::cout << "    采样点: " << canbus.bitrate.nsamplepos << "%/" << canbus.bitrate.dsamplepos << "%" << std::endl;
        std::cout << "    启用状态: " << (canbus.enable ? "是" : "否") << std::endl;
    }

    // 打印设备配置
    std::cout << "\n设备配置:" << std::endl;
    for (const auto& pair : canbus_to_devices_) {
        std::cout << "  " << pair.first << " 设备列表:" << std::endl;
        for (const auto& device : pair.second) {
            std::cout << "    - " << device.name << ":" << std::endl;
            std::cout << "      设备类型: " << to_string(device.device_type) << std::endl;
            std::cout << "      设备ID: " << device.device_id << std::endl;
            std::cout << "      忽略状态: " << (device.ignore ? "是" : "否") << std::endl;

            // 电机专用配置
            if (device.device_type == DeviceType::MOTOR || device.device_type == DeviceType::LEJUCLAW) {
                std::cout << "      负方向: " << (device.negtive ? "是" : "否") << std::endl;
                std::cout << "      减速比: " << device.ratio << std::endl;
                std::cout << "      参数: [";
                for (size_t i = 0; i < device.motor_params.size(); ++i) {
                    if (i > 0) std::cout << ", ";
                    std::cout << device.motor_params[i];
                }
                std::cout << "]" << std::endl;
            }
        }
    }

    std::cout << "\n=========================" << std::endl;
}

/////////////////////////////////////////////////////////////////////////////////////////
// Helper Functions for Writing Config
/////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief 将CAN总线配置转换为YAML节点
 */
static YAML::Node CanBusConfigToYAML(const CanBusConfig& config) {
    YAML::Node node;
    node["type"] = canbus_sdk::to_string(config.type);
    node["name"] = config.device_name;
    node["nbitrate"] = config.bitrate.nbitrate;
    node["dbitrate"] = config.bitrate.dbitrate;
    node["nsamplepos"] = config.bitrate.nsamplepos;
    node["dsamplepos"] = config.bitrate.dsamplepos;
    node["enable"] = config.enable;
    return node;
}

/**
 * @brief 将设备配置转换为YAML节点
 */
static YAML::Node DeviceConfigToYAML(const DeviceConfig& config) {
    YAML::Node node;
    node["name"] = config.name;
    node["class"] = to_string(config.device_type);

    // 使用 YAML Emitter 生成十六进制格式的设备ID
    YAML::Emitter device_id_emitter;
    device_id_emitter.SetOutputCharset(YAML::EscapeNonAscii);
    device_id_emitter << YAML::Hex << static_cast<uint32_t>(config.device_id);

    // 解析生成的 YAML 字符串
    YAML::Node device_id_node = YAML::Load(device_id_emitter.c_str());
    node["device_id"] = device_id_node;

    node["ignore"] = config.ignore;

    // 电机专用配置
    if (config.device_type == DeviceType::MOTOR || config.device_type == DeviceType::LEJUCLAW) {
        // 对 params 数组使用紧凑格式
        YAML::Node params_node;
        for (const auto& param : config.motor_params) {
            params_node.push_back(param);
        }
        // 设置为数组样式
        params_node.SetStyle(YAML::EmitterStyle::Flow);
        node["params"] = params_node;

        node["ratio"] = config.ratio;
        node["negtive"] = config.negtive;
    }

    return node;
}

/////////////////////////////////////////////////////////////////////////////////////////
// Class Member Functions - Write Operations
/////////////////////////////////////////////////////////////////////////////////////////

bool ConfigParser::writeToFile(const std::string& config_path) const {
    // 检查配置是否为空
    if (canbus_configs_.empty()) {
        LOG_FAILURE("[ConfigParser] 配置为空，无法写入文件");
        return false;
    }

    try {
        // 创建YAML根节点
        YAML::Node config;

        // 写入CAN总线接口配置
        for (const auto& canbus_config : canbus_configs_) {
            config["canbus_interfaces"][canbus_config.name] = CanBusConfigToYAML(canbus_config);

            // 写入对应的设备配置
            std::string devices_key = canbus_config.name + "_devices";
            auto it = canbus_to_devices_.find(canbus_config.name);
            if (it != canbus_to_devices_.end() && !it->second.empty()) {
                for (const auto& device_config : it->second) {
                    config[devices_key].push_back(DeviceConfigToYAML(device_config));
                }
            }
        }

        // 写入文件
        std::ofstream file(config_path);
        if (!file.is_open()) {
            LOG_FAILURE("[ConfigParser] 无法打开文件进行写入: %s", config_path.c_str());
            return false;
        }

        // 写入文件头注释
        static const char* yaml_comments =
        "##########################################################\n"
        "# 配置说明:\n"
            "# - canbus_interfaces: CAN总线接口配置\n"
            "#   - type: CAN总线硬件类型 (BUSMUST_A/BUSMUST_B/LEJU_CAN_A/LEJU_CAN_B/SOCKETCAN)\n"
            "#   - name: 设备名称\n"
            "#   - nbitrate/dbitrate: 仲裁段波特率/数据段波特率\n"
            "#   - nsamplepos/dsamplepos: 仲裁段采样点(%)/数据段采样点(%)\n"
            "#   - enable: 是否启用，预留字段，暂时没有用到\n"
            "# \n"
            "# - {bus_name}_devices: 设备配置列表\n"
            "#   - class: 设备类型 (motor/revo1_hand/revo2_hand/lejuclaw)\n"
            "#   - device_id: 设备ID\n"
            "#   - name: 设备名称\n"
            "#   - ignore: 是否忽略该设备\n"
            "#   - params: 电机默认参数 [vel, kp_pos, kd_pos, tor, kp_vel, kd_vel, ki_vel] (仅电机)\n"
            "#   - ratio: 减速比 (仅电机)\n"
            "#   - negtive: 电机方向 (仅电机)\n"
            "# \n"
            "# 注意: 如果是想禁用或者忽略设备请使用`ignore`字段, 不需要把设备Id改成0x00!\n"
            "# \n"
            "# 注意 ignore 字段只是不使能设备，对应的数据还是会暂位的（即返回默认数据0），如果想要移除设备，请删除或者注释它\n"
            "# \n"
            "# 设备配置示例:\n"
            "# - name: Larm_joint_01\n"
            "#   class: motor\n"
            "#   device_id: 0x01\n"
            "#   params: [0, 25, 8, 0, 0, 0, 0]\n"
            "#   ratio: 25\n"
            "#   negtive: false\n"
            "#   ignore: false\n"
            "# #########################################################\n\n";
        file << yaml_comments;

        // 输出YAML配置
        file << config;
        file.close();

        if (file.fail()) {
            LOG_FAILURE("[ConfigParser] 写入文件失败: %s", config_path.c_str());
            return false;
        }

        LOG_SUCCESS("[ConfigParser] 配置已成功写入文件: %s", config_path.c_str());
        return true;

    } catch (const YAML::Exception& e) {
        LOG_FAILURE("[ConfigParser] YAML序列化失败: %s", e.what());
        return false;
    } catch (const std::exception& e) {
        LOG_FAILURE("[ConfigParser] 写入配置文件时发生错误: %s", e.what());
        return false;
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
// Static Functions
/////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief 获取配置根目录路径
 */
static std::string _GetConfigRootPath() {
    const char* sudo_user = getenv("SUDO_USER");
    passwd* pw = nullptr;

    if (sudo_user != nullptr) {
        pw = getpwnam(sudo_user);
    } else {
        pw = getpwuid(getuid());
    }

    if (pw != nullptr) {
        return std::string(pw->pw_dir) + "/.config/lejuconfig";
    }

    return "";
}

std::string ConfigParser::getDefaultConfigFilePath() {
    std::string config_root = _GetConfigRootPath();
    if (config_root.empty()) {
        throw std::runtime_error("Failed to get config root path");
    }
    return config_root + "/canbus_device_cofig.yaml";
}

} // namespace canbus_sdk