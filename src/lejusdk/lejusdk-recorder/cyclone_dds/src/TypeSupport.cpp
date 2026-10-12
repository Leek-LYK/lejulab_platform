#include "cyclone_dds/TypeSupport.hpp"
#include "common/Log.hpp"

#include <dds/ddsc/dds_public_qos.h>
#include <dds/ddsi/ddsi_cdrstream.h>
#include <dds/ddsi/ddsi_xt_typeinfo.h>
#include <dds/ddsi/ddsi_xt_typemap.h>
#include <cstring>
#include <sstream>
#include <vector>
#include <algorithm>
#include <set>

namespace cyclone_record_replay {
namespace cyclone_dds {

TypeSupport::~TypeSupport() {
    clear();
}

std::vector<uint8_t> TypeSupport::get_type_object(
    dds_entity_t participant,
    const dds_typeinfo_t* type_info,
    int32_t timeout_ms)
{
    std::vector<uint8_t> result;

    if (!type_info) {
        LOG_ERROR("Cannot get type object: null type_info");
        return result;
    }

#ifdef DDS_HAS_TYPE_DISCOVERY
    // For CycloneDDS 0.10.x, we store the type_info pointer as a placeholder
    // The actual type information is managed by CycloneDDS internally
    // and will be used when creating topics via dds_create_topic_descriptor
    (void)participant;
    (void)timeout_ms;

    // Store type_info pointer as placeholder - actual type object retrieval
    // requires internal API access that varies between CycloneDDS versions
    result.resize(sizeof(dds_typeinfo_t*));
    std::memcpy(result.data(), &type_info, sizeof(dds_typeinfo_t*));

    LOG_DEBUG("Stored type info reference for later use");
#else
    LOG_WARN("Type discovery not enabled in CycloneDDS build");
#endif

    return result;
}

dds_topic_descriptor_t* TypeSupport::create_topic_descriptor(
    dds_entity_t participant,
    const std::string& type_name,
    const dds_typeinfo_t* type_info,
    int32_t timeout_ms)
{
    if (!type_info) {
        LOG_ERROR("Cannot create topic descriptor: null type_info");
        return nullptr;
    }

#if DDS_HAS_TYPE_DISCOVERY
    // Use CycloneDDS API to create descriptor from type info
    dds_topic_descriptor_t* descriptor = nullptr;

    dds_return_t ret = dds_create_topic_descriptor(
        DDS_FIND_SCOPE_GLOBAL,
        participant,
        type_info,
        DDS_MSECS(timeout_ms),
        &descriptor);

    if (ret != DDS_RETCODE_OK || !descriptor) {
        LOG_ERROR("Failed to create topic descriptor for '{}': {}",
            type_name, dds_strretcode(ret));
        return nullptr;
    }

    LOG_DEBUG("Created topic descriptor for type '{}'", type_name);
    return descriptor;
#else
    LOG_ERROR("Type discovery not enabled in CycloneDDS build");
    return nullptr;
#endif
}

void TypeSupport::free_topic_descriptor(dds_topic_descriptor_t* descriptor) {
    if (descriptor) {
#if DDS_HAS_TYPE_DISCOVERY
        dds_delete_topic_descriptor(descriptor);
#endif
    }
}

void TypeSupport::register_type(
    const std::string& type_name,
    const dds_typeinfo_t* type_info,
    const std::vector<uint8_t>& serialized_type_object)
{
    std::lock_guard<std::mutex> lock(mutex_);

    TypeEntry entry;
    entry.type_name = type_name;
    entry.type_info = type_info;
    entry.serialized_type_object = serialized_type_object;
    entry.descriptor = nullptr;
    entry.owns_descriptor = false;

    type_cache_[type_name] = std::move(entry);

    LOG_DEBUG("Registered type '{}'", type_name);
}

std::optional<TypeEntry> TypeSupport::get_type(const std::string& type_name) const {
    std::lock_guard<std::mutex> lock(mutex_);

    auto it = type_cache_.find(type_name);
    if (it != type_cache_.end()) {
        // Return a copy without ownership transfer
        TypeEntry copy;
        copy.type_name = it->second.type_name;
        copy.type_info = it->second.type_info;
        copy.serialized_type_object = it->second.serialized_type_object;
        copy.descriptor = it->second.descriptor;
        copy.owns_descriptor = false;  // Don't transfer ownership
        return copy;
    }
    return std::nullopt;
}

bool TypeSupport::has_type(const std::string& type_name) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return type_cache_.find(type_name) != type_cache_.end();
}

std::vector<std::string> TypeSupport::get_registered_types() const {
    std::lock_guard<std::mutex> lock(mutex_);

    std::vector<std::string> result;
    result.reserve(type_cache_.size());
    for (const auto& [name, entry] : type_cache_) {
        result.push_back(name);
    }
    return result;
}

void TypeSupport::clear() {
    std::lock_guard<std::mutex> lock(mutex_);

    for (auto& [name, entry] : type_cache_) {
        if (entry.owns_descriptor && entry.descriptor) {
            free_topic_descriptor(entry.descriptor);
        }
    }
    type_cache_.clear();
}

std::vector<uint8_t> TypeSupport::serialize_type_info(const dds_typeinfo_t* type_info) {
    std::vector<uint8_t> result;

    if (!type_info) {
        return result;
    }

    // Placeholder implementation
    // Actual implementation would use CycloneDDS CDR serialization
    // to serialize the XTypes TypeInformation structure
    LOG_DEBUG("Type info serialization is a placeholder");

    return result;
}

std::string TypeSupport::to_idl(const std::vector<uint8_t>& serialized_type_object) {
    // Placeholder - generating IDL from XTypes type object is complex
    // Would require parsing the TypeObject structure and reconstructing IDL
    return "// IDL generation not implemented";
}

// ---------------------------------------------------------------------------
// Helper: map XTypes TypeIdentifier discriminant to IDL type string
// ---------------------------------------------------------------------------
static std::string typeid_to_idl(
    const DDS_XTypes_TypeIdentifier& tid,
    const DDS_XTypes_TypeMapping& mapping)
{
    switch (tid._d) {
        case DDS_XTypes_TK_BOOLEAN:  return "boolean";
        case DDS_XTypes_TK_BYTE:     return "octet";
        case DDS_XTypes_TK_INT16:    return "int16";
        case DDS_XTypes_TK_INT32:    return "int32";
        case DDS_XTypes_TK_INT64:    return "int64";
        case DDS_XTypes_TK_UINT16:   return "uint16";
        case DDS_XTypes_TK_UINT32:   return "uint32";
        case DDS_XTypes_TK_UINT64:   return "uint64";
        case DDS_XTypes_TK_FLOAT32:  return "float";
        case DDS_XTypes_TK_FLOAT64:  return "double";
        case DDS_XTypes_TK_CHAR8:    return "char";
        case DDS_XTypes_TK_STRING8:  return "string";
        case DDS_XTypes_TI_STRING8_SMALL: return "string";
        case DDS_XTypes_TI_STRING8_LARGE: return "string";

        case DDS_XTypes_TI_PLAIN_SEQUENCE_SMALL: {
            const auto& seq = tid._u.seq_sdefn;
            if (seq.element_identifier) {
                return "sequence<" + typeid_to_idl(*seq.element_identifier, mapping) + ">";
            }
            return "sequence<octet>";
        }
        case DDS_XTypes_TI_PLAIN_SEQUENCE_LARGE: {
            const auto& seq = tid._u.seq_ldefn;
            if (seq.element_identifier) {
                return "sequence<" + typeid_to_idl(*seq.element_identifier, mapping) + ">";
            }
            return "sequence<octet>";
        }
        case DDS_XTypes_TI_PLAIN_ARRAY_SMALL: {
            const auto& arr = tid._u.array_sdefn;
            std::string elem = "octet";
            if (arr.element_identifier) {
                elem = typeid_to_idl(*arr.element_identifier, mapping);
            }
            // Build dimension suffix, e.g. [3][4]
            std::string dims;
            for (uint32_t i = 0; i < arr.array_bound_seq._length; ++i) {
                dims += "[" + std::to_string(arr.array_bound_seq._buffer[i]) + "]";
            }
            return elem + dims;  // caller places name between elem and dims
        }
        case DDS_XTypes_TI_PLAIN_ARRAY_LARGE: {
            const auto& arr = tid._u.array_ldefn;
            std::string elem = "octet";
            if (arr.element_identifier) {
                elem = typeid_to_idl(*arr.element_identifier, mapping);
            }
            std::string dims;
            for (uint32_t i = 0; i < arr.array_bound_seq._length; ++i) {
                dims += "[" + std::to_string(arr.array_bound_seq._buffer[i]) + "]";
            }
            return elem + dims;
        }

        // Handle EK_COMPLETE (242) - reference to a complete type object
        case DDS_XTypes_EK_COMPLETE: {
            // Look up the type name from TypeMapping using equivalence_hash
            const auto& hash = tid._u.equivalence_hash;
            const auto& complete_pairs = mapping.identifier_object_pair_complete;
            for (uint32_t i = 0; i < complete_pairs._length; ++i) {
                const auto& pair = complete_pairs._buffer[i];
                // Compare equivalence hash
                if (memcmp(pair.type_identifier._u.equivalence_hash, hash,
                           sizeof(DDS_XTypes_EquivalenceHash)) == 0) {
                    const auto& type_obj = pair.type_object;
                    if (type_obj._d == DDS_XTypes_EK_COMPLETE) {
                        const auto& complete = type_obj._u.complete;
                        if (complete._d == DDS_XTypes_TK_STRUCTURE) {
                            const char* name = complete._u.struct_type.header.detail.type_name;
                            if (name && name[0] != '\0') {
                                return std::string(name);
                            }
                        }
                    }
                }
            }
            // Fallback: return placeholder with hash info
            return "/* nested type hash:" +
                   std::to_string(static_cast<int>(hash[0])) + ":" +
                   std::to_string(static_cast<int>(hash[1])) + " */ octet";
        }

        // Handle EK_MINIMAL (241) - reference to a minimal type object
        case DDS_XTypes_EK_MINIMAL: {
            return "/* minimal type ref */ octet";
        }

        default:
            break;
    }
    return "octet /* unknown type discriminant " + std::to_string(tid._d) + " */";
}

// ---------------------------------------------------------------------------
// Helper: check if IDL type string contains array dimensions (e.g. "double[3]")
// If so, split into base type and dimension suffix for proper IDL syntax:
//   "double[3]" → base="double", dims="[3]"
// ---------------------------------------------------------------------------
static void split_array_type(const std::string& idl_type,
                             std::string& base_type,
                             std::string& dims)
{
    auto bracket = idl_type.find('[');
    if (bracket != std::string::npos) {
        base_type = idl_type.substr(0, bracket);
        dims = idl_type.substr(bracket);
    } else {
        base_type = idl_type;
        dims.clear();
    }
}

// ---------------------------------------------------------------------------
// Helper: generate IDL from a Complete TypeObject struct
// ---------------------------------------------------------------------------
static std::string generate_idl_from_complete_struct(
    const std::string& type_name,
    const DDS_XTypes_CompleteStructType& struct_type,
    const DDS_XTypes_TypeMapping& mapping)
{
    // Parse module hierarchy from type_name (e.g. "leju::msgs::JointState")
    std::vector<std::string> modules;
    std::string struct_name;
    {
        std::string remaining = type_name;
        size_t pos;
        while ((pos = remaining.find("::")) != std::string::npos) {
            modules.push_back(remaining.substr(0, pos));
            remaining = remaining.substr(pos + 2);
        }
        struct_name = remaining;
    }

    std::stringstream ss;
    std::string indent;

    // Open modules
    for (const auto& mod : modules) {
        ss << indent << "module " << mod << " {\n";
        indent += "  ";
    }

    // Emit extensibility annotation based on struct_flags
    auto flags = struct_type.struct_flags;
    if (flags & DDS_XTypes_IS_FINAL) {
        ss << indent << "@final\n";
    } else if (flags & DDS_XTypes_IS_MUTABLE) {
        ss << indent << "@mutable\n";
    } else if (flags & DDS_XTypes_IS_APPENDABLE) {
        ss << indent << "@appendable\n";
    }

    ss << indent << "struct " << struct_name << " {\n";

    std::string member_indent = indent + "  ";

    // Iterate struct members
    const auto& members = struct_type.member_seq;
    for (uint32_t i = 0; i < members._length; ++i) {
        const auto& member = members._buffer[i];
        const char* name = member.detail.name;
        const auto& tid = member.common.member_type_id;

        std::string idl_type = typeid_to_idl(tid, mapping);

        // Check if it's an array type (contains [N] dims)
        std::string base_type, dims;
        split_array_type(idl_type, base_type, dims);

        if (!dims.empty()) {
            // Array: "double name[3];"
            ss << member_indent << base_type << " " << name << dims << ";\n";
        } else {
            ss << member_indent << idl_type << " " << name << ";\n";
        }
    }

    ss << indent << "};\n";

    // Close modules (reverse order)
    for (auto it = modules.rbegin(); it != modules.rend(); ++it) {
        indent = indent.substr(2);
        ss << indent << "};\n";
    }

    return ss.str();
}

// ---------------------------------------------------------------------------
// Helper: Generate IDL from a Complete TypeObject enum
// ---------------------------------------------------------------------------
static std::string generate_idl_from_complete_enum(
    const std::string& type_name,
    const DDS_XTypes_CompleteEnumeratedType& enum_type)
{
    // Parse module hierarchy from type_name
    std::vector<std::string> modules;
    std::string enum_name;
    {
        std::string remaining = type_name;
        size_t pos;
        while ((pos = remaining.find("::")) != std::string::npos) {
            modules.push_back(remaining.substr(0, pos));
            remaining = remaining.substr(pos + 2);
        }
        enum_name = remaining;
    }

    std::stringstream ss;
    std::string indent;

    // Open modules
    for (const auto& mod : modules) {
        ss << indent << "module " << mod << " {\n";
        indent += "  ";
    }

    ss << indent << "enum " << enum_name << " {\n";

    const auto& literals = enum_type.literal_seq;
    for (uint32_t i = 0; i < literals._length; ++i) {
        const auto& literal = literals._buffer[i];
        ss << indent << "  " << literal.detail.name;
        if (i + 1 < literals._length) {
            ss << ",";
        }
        ss << "\n";
    }

    ss << indent << "};\n";

    // Close modules
    for (auto it = modules.rbegin(); it != modules.rend(); ++it) {
        indent = indent.substr(2);
        ss << indent << "};\n";
    }

    return ss.str();
}

// ---------------------------------------------------------------------------
// Helper: Generate IDL for all types in TypeMapping (dependency order)
// ---------------------------------------------------------------------------
static std::string generate_all_structs_idl(
    const std::string& main_type_name,
    const DDS_XTypes_TypeMapping& mapping)
{
    std::stringstream result;
    std::set<std::string> generated_types;

    const auto& complete_pairs = mapping.identifier_object_pair_complete;

    // First pass: generate all enum types (they have no dependencies)
    for (uint32_t i = 0; i < complete_pairs._length; ++i) {
        const auto& pair = complete_pairs._buffer[i];
        const auto& type_obj = pair.type_object;

        if (type_obj._d != DDS_XTypes_EK_COMPLETE) continue;
        const auto& complete = type_obj._u.complete;
        if (complete._d != DDS_XTypes_TK_ENUM) continue;

        const auto& enum_type = complete._u.enumerated_type;
        const char* name = enum_type.header.detail.type_name;
        if (!name || name[0] == '\0') continue;

        std::string type_name_str(name);
        if (generated_types.find(type_name_str) == generated_types.end()) {
            result << generate_idl_from_complete_enum(type_name_str, enum_type);
            result << "\n";
            generated_types.insert(type_name_str);
        }
    }

    // Second pass: generate all nested/dependency struct types
    for (uint32_t i = 0; i < complete_pairs._length; ++i) {
        const auto& pair = complete_pairs._buffer[i];
        const auto& type_obj = pair.type_object;

        if (type_obj._d != DDS_XTypes_EK_COMPLETE) continue;
        const auto& complete = type_obj._u.complete;
        if (complete._d != DDS_XTypes_TK_STRUCTURE) continue;

        const auto& struct_type = complete._u.struct_type;
        const char* name = struct_type.header.detail.type_name;
        if (!name || name[0] == '\0') continue;

        std::string type_name_str(name);

        // Skip the main type for now, generate dependencies first
        if (type_name_str == main_type_name) continue;

        if (generated_types.find(type_name_str) == generated_types.end()) {
            result << generate_idl_from_complete_struct(type_name_str, struct_type, mapping);
            result << "\n";
            generated_types.insert(type_name_str);
        }
    }

    // Third pass: generate the main type
    for (uint32_t i = 0; i < complete_pairs._length; ++i) {
        const auto& pair = complete_pairs._buffer[i];
        const auto& type_obj = pair.type_object;

        if (type_obj._d != DDS_XTypes_EK_COMPLETE) continue;
        const auto& complete = type_obj._u.complete;
        if (complete._d != DDS_XTypes_TK_STRUCTURE) continue;

        const auto& struct_type = complete._u.struct_type;
        const char* name = struct_type.header.detail.type_name;
        if (!name || name[0] == '\0') continue;

        if (std::string(name) == main_type_name) {
            result << generate_idl_from_complete_struct(main_type_name, struct_type, mapping);
            break;
        }
    }

    return result.str();
}

// ---------------------------------------------------------------------------
// Main: generate IDL from descriptor's type_mapping
// ---------------------------------------------------------------------------
std::string TypeSupport::generate_idl_for_type(
    const std::string& type_name,
    const dds_topic_descriptor_t* descriptor)
{
    // Try auto-generation from XTypes TypeMapping if descriptor is available
    if (descriptor &&
        (descriptor->m_flagset & DDS_TOPIC_XTYPES_METADATA) &&
        descriptor->type_mapping.data &&
        descriptor->type_mapping.sz > 0)
    {
        // Deserialize TypeMapping from XCDR2
        DDS_XTypes_TypeMapping* mapping = nullptr;
        void* mapping_sample = dds_alloc(DDS_XTypes_TypeMapping_desc.m_size);
        if (mapping_sample) {
            memset(mapping_sample, 0, DDS_XTypes_TypeMapping_desc.m_size);

            dds_istream_t is;
            dds_istream_init(&is,
                             descriptor->type_mapping.sz,
                             descriptor->type_mapping.data,
                             CDR_ENC_VERSION_2);
            dds_stream_read(&is,
                            static_cast<char*>(mapping_sample),
                            DDS_XTypes_TypeMapping_desc.m_ops);
            dds_istream_fini(&is);

            mapping = static_cast<DDS_XTypes_TypeMapping*>(mapping_sample);
        }

        if (mapping) {
            // Generate IDL for all types in the mapping (includes nested types)
            std::string idl = generate_all_structs_idl(type_name, *mapping);

            if (!idl.empty()) {
                dds_sample_free(mapping_sample, &DDS_XTypes_TypeMapping_desc, DDS_FREE_ALL);
                LOG_DEBUG("Auto-generated IDL for type '{}' with all dependencies", type_name);
                return idl;
            }

            // Fallback: try the old method if new method didn't produce output
            const auto& complete_pairs = mapping->identifier_object_pair_complete;
            for (uint32_t i = 0; i < complete_pairs._length; ++i) {
                const auto& pair = complete_pairs._buffer[i];
                const auto& type_obj = pair.type_object;

                if (type_obj._d != DDS_XTypes_EK_COMPLETE) continue;
                if (type_obj._u.complete._d != DDS_XTypes_TK_STRUCTURE) continue;

                const auto& struct_type = type_obj._u.complete._u.struct_type;

                if (struct_type.member_seq._length > 0) {
                    idl = generate_idl_from_complete_struct(type_name, struct_type, *mapping);
                    dds_sample_free(mapping_sample, &DDS_XTypes_TypeMapping_desc, DDS_FREE_ALL);
                    LOG_DEBUG("Auto-generated IDL for type '{}' from XTypes (fallback)", type_name);
                    return idl;
                }
            }

            dds_sample_free(mapping_sample, &DDS_XTypes_TypeMapping_desc, DDS_FREE_ALL);
            LOG_WARN("TypeMapping for '{}' has no complete struct type objects", type_name);
        }
    }

    // Fallback: generate minimal IDL structure
    LOG_WARN("No XTypes metadata available for '{}', generating placeholder IDL", type_name);

    std::vector<std::string> modules;
    std::string struct_name;
    {
        std::string remaining = type_name;
        size_t pos;
        while ((pos = remaining.find("::")) != std::string::npos) {
            modules.push_back(remaining.substr(0, pos));
            remaining = remaining.substr(pos + 2);
        }
        struct_name = remaining;
    }

    std::stringstream ss;
    std::string indent;
    for (const auto& mod : modules) {
        ss << indent << "module " << mod << " {\n";
        indent += "  ";
    }
    ss << indent << "struct " << struct_name << " {\n";
    ss << indent << "  sequence<octet> data;\n";
    ss << indent << "};\n";
    for (size_t i = 0; i < modules.size(); ++i) {
        indent = indent.substr(2);
        ss << indent << "};\n";
    }

    return ss.str();
}

std::string TypeSupport::to_json_schema(const std::vector<uint8_t>& serialized_type_object) {
    // Placeholder - generating JSON Schema from XTypes type object
    std::stringstream ss;
    ss << "{\n";
    ss << "  \"$schema\": \"http://json-schema.org/draft-07/schema#\",\n";
    ss << "  \"type\": \"object\",\n";
    ss << "  \"description\": \"Generated from DDS type object\"\n";
    ss << "}";
    return ss.str();
}

} // namespace cyclone_dds
} // namespace cyclone_record_replay
