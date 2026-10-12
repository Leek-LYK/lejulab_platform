#include "common/Config.hpp"

#include <fstream>
#include <sstream>

namespace cyclone_record_replay {
namespace common {

void ConfigBase::load_from_file(const std::string& file_path) {
    try {
        YAML::Node root = YAML::LoadFile(file_path);
        parse_common(root);
        parse_specific(root);
    } catch (const YAML::Exception& e) {
        throw ConfigException("Failed to parse YAML file '" + file_path + "': " + e.what());
    }
}

void ConfigBase::load_from_string(const std::string& yaml_content) {
    try {
        YAML::Node root = YAML::Load(yaml_content);
        parse_common(root);
        parse_specific(root);
    } catch (const YAML::Exception& e) {
        throw ConfigException("Failed to parse YAML content: " + std::string(e.what()));
    }
}

void ConfigBase::parse_common(const YAML::Node& root) {
    if (root["dds"]) {
        parse_dds_config(root["dds"]);
    }
}

void ConfigBase::parse_dds_config(const YAML::Node& node) {
    dds_config_.domain_id = get_optional<int32_t>(node, "domain_id", 0);
    dds_config_.discovery_timeout = get_optional<double>(node, "discovery_timeout", 10.0);

    if (node["topics"]) {
        dds_config_.topic_filter = parse_topic_filter(node["topics"]);
    }
    dds_config_.topic_filter.use_regex = get_optional<bool>(node, "use_regex", false);
}

TopicFilter ConfigBase::parse_topic_filter(const YAML::Node& node) {
    TopicFilter filter;

    if (node.IsSequence()) {
        for (const auto& item : node) {
            filter.topics.push_back(item.as<std::string>());
        }
    }

    return filter;
}

} // namespace common
} // namespace cyclone_record_replay
