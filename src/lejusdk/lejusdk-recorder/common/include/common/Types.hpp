#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <array>
#include <chrono>
#include <functional>
#include <optional>
#include <variant>
#include <ostream>

namespace cyclone_record_replay {
namespace common {

/**
 * @brief GUID structure for DDS entities
 */
struct Guid {
    std::array<uint8_t, 16> value;

    bool operator==(const Guid& other) const {
        return value == other.value;
    }

    bool operator!=(const Guid& other) const {
        return value != other.value;
    }

    bool operator<(const Guid& other) const {
        return value < other.value;
    }

    std::string to_string() const;
    static Guid from_dds_guid(const void* dds_guid);
};

/**
 * @brief Hash function for Guid
 */
struct GuidHash {
    std::size_t operator()(const Guid& guid) const;
};

/**
 * @brief Timestamp using nanoseconds since epoch
 */
using Timestamp = std::chrono::nanoseconds;

/**
 * @brief Convert DDS time to Timestamp
 */
Timestamp from_dds_time(int64_t dds_time);

/**
 * @brief Convert Timestamp to DDS time
 */
int64_t to_dds_time(Timestamp ts);

/**
 * @brief QoS reliability kind
 */
enum class ReliabilityKind {
    BestEffort,
    Reliable
};

/**
 * @brief QoS durability kind
 */
enum class DurabilityKind {
    Volatile,
    TransientLocal,
    Transient,
    Persistent
};

/**
 * @brief Simplified QoS settings
 */
struct QosSettings {
    ReliabilityKind reliability = ReliabilityKind::BestEffort;
    DurabilityKind durability = DurabilityKind::Volatile;
    int32_t history_depth = 1;

    // Deadline period in nanoseconds (0 = infinite)
    int64_t deadline_period_ns = 0;

    // Lifespan period in nanoseconds (0 = infinite)
    int64_t lifespan_period_ns = 0;
};

/**
 * @brief Topic information
 */
struct TopicInfo {
    std::string name;
    std::string type_name;
    QosSettings qos;
};

/**
 * @brief Discovered endpoint information
 */
struct EndpointInfo {
    Guid guid;
    std::string topic_name;
    std::string type_name;
    QosSettings qos;
    bool is_writer;  // true for DataWriter, false for DataReader
};

/**
 * @brief Raw CDR sample data
 */
struct RawSample {
    std::vector<uint8_t> payload;
    Timestamp timestamp;
    Guid writer_guid;
    uint64_t sequence_number;
};

/**
 * @brief Return code for operations
 */
enum class ReturnCode {
    Ok,
    Error,
    Timeout,
    NoData,
    InvalidArgument,
    NotSupported,
    AlreadyExists,
    NotFound,
    OutOfResources
};

/**
 * @brief Convert ReturnCode to string
 */
std::string to_string(ReturnCode code);

/**
 * @brief Topic filter configuration
 */
struct TopicFilter {
    std::vector<std::string> topics;  // Empty = record all
    bool use_regex = false;           // true = regex match, false = exact match
    std::string exclude_regex;        // Exclude topics matching this regex

    bool matches(const std::string& topic_name) const;
};

/**
 * @brief Recording state
 */
enum class RecordingState {
    Stopped,
    Running,
    Paused
};

/**
 * @brief Playback state
 */
enum class PlaybackState {
    Stopped,
    Running,
    Paused,
    Finished
};

// Stream operators for enum debugging
inline std::ostream& operator<<(std::ostream& os, RecordingState state) {
    switch (state) {
        case RecordingState::Stopped: return os << "Stopped";
        case RecordingState::Running: return os << "Running";
        case RecordingState::Paused: return os << "Paused";
    }
    return os << "Unknown(" << static_cast<int>(state) << ")";
}

inline std::ostream& operator<<(std::ostream& os, PlaybackState state) {
    switch (state) {
        case PlaybackState::Stopped: return os << "Stopped";
        case PlaybackState::Running: return os << "Running";
        case PlaybackState::Paused: return os << "Paused";
        case PlaybackState::Finished: return os << "Finished";
    }
    return os << "Unknown(" << static_cast<int>(state) << ")";
}

} // namespace common
} // namespace cyclone_record_replay
