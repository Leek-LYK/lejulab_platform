#pragma once

#include <dds/dds.h>
#include <string>
#include <vector>
#include <memory>
#include <functional>
#include <mutex>
#include <atomic>
#include <cstring>

#include "common/Types.hpp"
#include "cyclone_dds/CycloneParticipant.hpp"

namespace cyclone_record_replay {
namespace cyclone_dds {

/**
 * @brief Raw CDR sample with metadata
 */
struct RawCdrSample {
    std::vector<uint8_t> cdr_data;    // Raw CDR payload
    dds_time_t timestamp;              // Source timestamp
    dds_guid_t writer_guid;            // GUID of the writer
    uint64_t sequence_number;          // Sequence number

    RawCdrSample() : timestamp(0), sequence_number(0) {
        std::memset(&writer_guid, 0, sizeof(writer_guid));
    }
};

/**
 * @brief Callback for receiving raw CDR data
 */
using RawDataCallback = std::function<void(const RawCdrSample&)>;

/**
 * @brief Raw CDR data reader using dds_takecdr()
 *
 * Reads raw CDR-encoded data without deserializing it, which is essential
 * for recording arbitrary DDS messages without knowing their type at compile time.
 */
class RawDataReader {
public:
    /**
     * @brief Configuration for the reader
     */
    struct Config {
        std::string topic_name;
        std::string type_name;
        const dds_typeinfo_t* type_info = nullptr;

        // QoS settings
        common::ReliabilityKind reliability = common::ReliabilityKind::BestEffort;
        common::DurabilityKind durability = common::DurabilityKind::Volatile;
        int32_t history_depth = 100;
    };

    /**
     * @brief Construct a raw data reader
     * @param participant The DDS participant
     * @param config Reader configuration
     */
    RawDataReader(CycloneParticipant& participant, const Config& config);

    /**
     * @brief Destructor - cleans up DDS resources
     */
    ~RawDataReader();

    // Non-copyable, non-movable
    RawDataReader(const RawDataReader&) = delete;
    RawDataReader& operator=(const RawDataReader&) = delete;

    /**
     * @brief Check if reader is valid
     * @return true if reader was created successfully
     */
    bool is_valid() const { return reader_ > 0; }

    /**
     * @brief Get the topic name
     * @return Topic name
     */
    const std::string& topic_name() const { return config_.topic_name; }

    /**
     * @brief Get the type name
     * @return Type name
     */
    const std::string& type_name() const { return config_.type_name; }

    /**
     * @brief Take all available raw CDR samples
     * @return Vector of raw samples
     */
    std::vector<RawCdrSample> take_raw();

    /**
     * @brief Set callback for data reception (for async notification)
     * @param callback Callback function
     */
    void set_data_callback(RawDataCallback callback);

    /**
     * @brief Enable or disable async notification via callback
     * @param enable true to enable callbacks
     */
    void enable_callback(bool enable);

    /**
     * @brief Get the underlying reader handle
     * @return DDS reader entity handle
     */
    dds_entity_t handle() const { return reader_; }

private:
    void create_reader();
    void destroy_reader();
    static void on_data_available(dds_entity_t reader, void* arg);

    CycloneParticipant& participant_;
    Config config_;

    dds_entity_t topic_ = 0;
    dds_entity_t reader_ = 0;

    std::mutex callback_mutex_;
    RawDataCallback data_callback_;
    std::atomic<bool> callback_enabled_{false};
};

} // namespace cyclone_dds
} // namespace cyclone_record_replay
