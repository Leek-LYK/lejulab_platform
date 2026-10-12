#pragma once

#include <dds/dds.h>
#include <string>
#include <vector>
#include <unordered_map>
#include <mutex>
#include <memory>

namespace cyclone_record_replay {
namespace recorder {

/**
 * @brief Collected type information for a DDS type
 */
struct CollectedType {
    std::string type_name;
    std::vector<uint8_t> serialized_type_object;
    std::string schema_encoding;  // "cdr", "ros2msg", "jsonschema"
    std::string schema_data;
    bool complete = false;
};

/**
 * @brief Topic to type mapping
 */
struct TopicTypeInfo {
    std::string topic_name;
    std::string type_name;
    std::vector<uint8_t> qos_data;
};

/**
 * @brief Collection of discovered types for recording
 *
 * Manages the collection of type information from DDS discovery
 * for storage in MCAP schema records.
 */
class TypesCollection {
public:
    TypesCollection() = default;
    ~TypesCollection() = default;

    /**
     * @brief Add or update a type
     * @param type_name Type name
     * @param type_info CycloneDDS type info
     * @param participant Participant for type resolution
     * @return true if type was added (new), false if already exists
     */
    bool add_type(
        const std::string& type_name,
        const dds_typeinfo_t* type_info,
        dds_entity_t participant);

    /**
     * @brief Register a topic with its type
     * @param topic_name Topic name
     * @param type_name Type name
     * @param qos_data Serialized QoS settings
     */
    void register_topic(
        const std::string& topic_name,
        const std::string& type_name,
        const std::vector<uint8_t>& qos_data = {});

    /**
     * @brief Check if a type is already collected
     * @param type_name Type name to check
     * @return true if type is in collection
     */
    bool has_type(const std::string& type_name) const;

    /**
     * @brief Check if a topic is registered
     * @param topic_name Topic name to check
     * @return true if topic is registered
     */
    bool has_topic(const std::string& topic_name) const;

    /**
     * @brief Get collected type information
     * @param type_name Type name
     * @return Pointer to collected type or nullptr
     */
    const CollectedType* get_type(const std::string& type_name) const;

    /**
     * @brief Get type name for a topic
     * @param topic_name Topic name
     * @return Type name or empty string if not found
     */
    std::string get_type_for_topic(const std::string& topic_name) const;

    /**
     * @brief Get all collected types
     * @return Map of type name to collected type
     */
    std::unordered_map<std::string, CollectedType> get_all_types() const;

    /**
     * @brief Get all registered topics
     * @return Map of topic name to topic info
     */
    std::unordered_map<std::string, TopicTypeInfo> get_all_topics() const;

    /**
     * @brief Get number of collected types
     */
    size_t type_count() const;

    /**
     * @brief Get number of registered topics
     */
    size_t topic_count() const;

    /**
     * @brief Clear all collected data
     */
    void clear();

    /**
     * @brief Generate schema data for MCAP
     * @param type_name Type name
     * @param encoding Desired encoding ("cdr", "jsonschema", "ros2msg")
     * @return Schema data string
     */
    std::string generate_schema(
        const std::string& type_name,
        const std::string& encoding = "cdr") const;

private:
    mutable std::mutex mutex_;
    std::unordered_map<std::string, CollectedType> types_;
    std::unordered_map<std::string, TopicTypeInfo> topics_;
};

} // namespace recorder
} // namespace cyclone_record_replay
