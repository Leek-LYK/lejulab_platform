#pragma once

#include <dds/dds.h>
#include <string>
#include <vector>
#include <memory>
#include <optional>
#include <unordered_map>
#include <mutex>

namespace cyclone_record_replay {
namespace cyclone_dds {

/**
 * @brief Serialized type object for storage
 */
struct SerializedTypeObject {
    std::vector<uint8_t> data;
    std::string type_name;
};

/**
 * @brief Type information cache entry
 */
struct TypeEntry {
    std::string type_name;
    const dds_typeinfo_t* type_info = nullptr;
    std::vector<uint8_t> serialized_type_object;
    dds_topic_descriptor_t* descriptor = nullptr;
    bool owns_descriptor = false;
};

/**
 * @brief XTypes support utilities for CycloneDDS
 *
 * Provides utilities for working with CycloneDDS XTypes including:
 * - Type object retrieval and serialization
 * - Dynamic topic descriptor creation
 * - Type information caching
 */
class TypeSupport {
public:
    TypeSupport() = default;
    ~TypeSupport();

    // Non-copyable
    TypeSupport(const TypeSupport&) = delete;
    TypeSupport& operator=(const TypeSupport&) = delete;

    /**
     * @brief Get type object from discovered type info
     * @param participant The DDS participant
     * @param type_info Type information from discovery
     * @param timeout_ms Timeout for type lookup
     * @return Serialized type object or empty on failure
     */
    static std::vector<uint8_t> get_type_object(
        dds_entity_t participant,
        const dds_typeinfo_t* type_info,
        int32_t timeout_ms = 5000);

    /**
     * @brief Create a topic descriptor from type information
     * @param participant The DDS participant
     * @param type_name Type name
     * @param type_info Type information from discovery
     * @param timeout_ms Timeout for type lookup
     * @return Topic descriptor or nullptr on failure
     */
    static dds_topic_descriptor_t* create_topic_descriptor(
        dds_entity_t participant,
        const std::string& type_name,
        const dds_typeinfo_t* type_info,
        int32_t timeout_ms = 5000);

    /**
     * @brief Free a topic descriptor created by create_topic_descriptor
     * @param descriptor The descriptor to free
     */
    static void free_topic_descriptor(dds_topic_descriptor_t* descriptor);

    /**
     * @brief Register a type with the cache
     * @param type_name Type name
     * @param type_info Type information
     * @param serialized_type_object Serialized type object data
     */
    void register_type(
        const std::string& type_name,
        const dds_typeinfo_t* type_info,
        const std::vector<uint8_t>& serialized_type_object);

    /**
     * @brief Get cached type entry
     * @param type_name Type name to look up
     * @return Optional type entry
     */
    std::optional<TypeEntry> get_type(const std::string& type_name) const;

    /**
     * @brief Check if a type is registered
     * @param type_name Type name to check
     * @return true if type is registered
     */
    bool has_type(const std::string& type_name) const;

    /**
     * @brief Get all registered type names
     * @return Vector of type names
     */
    std::vector<std::string> get_registered_types() const;

    /**
     * @brief Clear all cached types
     */
    void clear();

    /**
     * @brief Serialize type info to portable format for storage
     * @param type_info Type information to serialize
     * @return Serialized bytes
     */
    static std::vector<uint8_t> serialize_type_info(const dds_typeinfo_t* type_info);

    /**
     * @brief Generate IDL representation from type object
     * @param serialized_type_object Serialized type object
     * @return IDL string representation
     *
     * Note: This is a best-effort conversion and may not produce
     * compilable IDL for all types.
     */
    static std::string to_idl(const std::vector<uint8_t>& serialized_type_object);

    /**
     * @brief Generate JSON Schema from type object
     * @param serialized_type_object Serialized type object
     * @return JSON Schema string
     */
    static std::string to_json_schema(const std::vector<uint8_t>& serialized_type_object);

    /**
     * @brief Generate OMG IDL for a type
     * @param type_name Fully qualified type name (e.g., "leju::msgs::JointState")
     * @param descriptor Optional topic descriptor with XTypes type_mapping for auto-generation
     * @return OMG IDL string for the type
     *
     * If descriptor is provided and contains valid type_mapping data,
     * IDL is automatically generated from the XTypes CompleteTypeObject.
     * Otherwise falls back to hardcoded definitions for known types.
     */
    static std::string generate_idl_for_type(
        const std::string& type_name,
        const dds_topic_descriptor_t* descriptor = nullptr);

private:
    mutable std::mutex mutex_;
    std::unordered_map<std::string, TypeEntry> type_cache_;
};

} // namespace cyclone_dds
} // namespace cyclone_record_replay
