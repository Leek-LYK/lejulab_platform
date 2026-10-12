#include "recorder/TypesCollection.hpp"
#include "cyclone_dds/TypeSupport.hpp"
#include "common/Log.hpp"

namespace cyclone_record_replay {
namespace recorder {

bool TypesCollection::add_type(
    const std::string& type_name,
    const dds_typeinfo_t* type_info,
    dds_entity_t participant)
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (types_.find(type_name) != types_.end()) {
        return false;  // Already exists
    }

    CollectedType collected;
    collected.type_name = type_name;
    collected.schema_encoding = "omgidl";  // OMG IDL for DDS/CDR messages

    // Get serialized type object
    if (type_info) {
        collected.serialized_type_object =
            cyclone_dds::TypeSupport::get_type_object(participant, type_info);
        collected.complete = !collected.serialized_type_object.empty();
    }

    // Generate OMG IDL schema data from XTypes TypeMapping
    dds_topic_descriptor_t* descriptor = nullptr;
    if (type_info) {
        descriptor = cyclone_dds::TypeSupport::create_topic_descriptor(
            participant, type_name, type_info);
    }
    collected.schema_data = cyclone_dds::TypeSupport::generate_idl_for_type(
        type_name, descriptor);
    if (descriptor) {
        cyclone_dds::TypeSupport::free_topic_descriptor(descriptor);
    }

    types_[type_name] = std::move(collected);

    LOG_DEBUG("Added type '{}' to collection (complete: {})",
        type_name, types_[type_name].complete);

    return true;
}

void TypesCollection::register_topic(
    const std::string& topic_name,
    const std::string& type_name,
    const std::vector<uint8_t>& qos_data)
{
    std::lock_guard<std::mutex> lock(mutex_);

    TopicTypeInfo info;
    info.topic_name = topic_name;
    info.type_name = type_name;
    info.qos_data = qos_data;

    topics_[topic_name] = std::move(info);

    LOG_DEBUG("Registered topic '{}' with type '{}'", topic_name, type_name);
}

bool TypesCollection::has_type(const std::string& type_name) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return types_.find(type_name) != types_.end();
}

bool TypesCollection::has_topic(const std::string& topic_name) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return topics_.find(topic_name) != topics_.end();
}

const CollectedType* TypesCollection::get_type(const std::string& type_name) const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = types_.find(type_name);
    if (it != types_.end()) {
        return &it->second;
    }
    return nullptr;
}

std::string TypesCollection::get_type_for_topic(const std::string& topic_name) const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = topics_.find(topic_name);
    if (it != topics_.end()) {
        return it->second.type_name;
    }
    return "";
}

std::unordered_map<std::string, CollectedType> TypesCollection::get_all_types() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return types_;
}

std::unordered_map<std::string, TopicTypeInfo> TypesCollection::get_all_topics() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return topics_;
}

size_t TypesCollection::type_count() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return types_.size();
}

size_t TypesCollection::topic_count() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return topics_.size();
}

void TypesCollection::clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    types_.clear();
    topics_.clear();
}

std::string TypesCollection::generate_schema(
    const std::string& type_name,
    const std::string& encoding) const
{
    std::lock_guard<std::mutex> lock(mutex_);

    auto it = types_.find(type_name);
    if (it == types_.end()) {
        return "";
    }

    const auto& collected = it->second;

    if (encoding == "jsonschema") {
        return cyclone_dds::TypeSupport::to_json_schema(
            collected.serialized_type_object);
    } else if (encoding == "idl") {
        return cyclone_dds::TypeSupport::to_idl(
            collected.serialized_type_object);
    } else {
        // Default: return raw type object as base64 or hex
        return collected.schema_data;
    }
}

} // namespace recorder
} // namespace cyclone_record_replay
