#pragma once

#include <yaml-cpp/yaml.h>
#include <string>
#include <optional>
#include <stdexcept>

#include "common/Types.hpp"
#include "common/Log.hpp"

namespace cyclone_record_replay {
namespace common {

/**
 * @brief Configuration parsing exception
 */
class ConfigException : public std::runtime_error {
public:
    explicit ConfigException(const std::string& message)
        : std::runtime_error(message) {}
};

/**
 * @brief Base DDS configuration
 */
struct DdsConfig {
    int32_t domain_id = 0;
    TopicFilter topic_filter;

    // Discovery timeout in seconds
    double discovery_timeout = 10.0;
};

/**
 * @brief Base configuration class for YAML-based configs
 */
class ConfigBase {
public:
    virtual ~ConfigBase() = default;

    /**
     * @brief Load configuration from YAML file
     * @param file_path Path to YAML configuration file
     */
    void load_from_file(const std::string& file_path);

    /**
     * @brief Load configuration from YAML string
     * @param yaml_content YAML content string
     */
    void load_from_string(const std::string& yaml_content);

    /**
     * @brief Get the DDS configuration
     */
    const DdsConfig& dds_config() const { return dds_config_; }

    void set_domain_id(int32_t id) { dds_config_.domain_id = id; }
    void set_topic_filter(const TopicFilter& filter) { dds_config_.topic_filter = filter; }

protected:
    /**
     * @brief Parse implementation-specific configuration
     * @param root Root YAML node
     */
    virtual void parse_specific(const YAML::Node& root) = 0;

    /**
     * @brief Parse common configuration sections
     * @param root Root YAML node
     */
    void parse_common(const YAML::Node& root);

    /**
     * @brief Parse DDS configuration section
     * @param node DDS YAML node
     */
    void parse_dds_config(const YAML::Node& node);

    /**
     * @brief Parse topic filter configuration
     * @param node Topic filter YAML node
     * @return Parsed TopicFilter
     */
    TopicFilter parse_topic_filter(const YAML::Node& node);

    /**
     * @brief Helper to get required value with error handling
     */
    template<typename T>
    T get_required(const YAML::Node& node, const std::string& key) {
        if (!node[key]) {
            throw ConfigException("Missing required configuration: " + key);
        }
        try {
            return node[key].as<T>();
        } catch (const YAML::Exception& e) {
            throw ConfigException("Invalid value for " + key + ": " + e.what());
        }
    }

    /**
     * @brief Helper to get optional value with default
     */
    template<typename T>
    T get_optional(const YAML::Node& node, const std::string& key, const T& default_value) {
        if (!node[key]) {
            return default_value;
        }
        try {
            return node[key].as<T>();
        } catch (const YAML::Exception& e) {
            LOG_WARN("Invalid value for {}: {}", key, e.what());
            return default_value;
        }
    }

    DdsConfig dds_config_;
};

} // namespace common
} // namespace cyclone_record_replay
