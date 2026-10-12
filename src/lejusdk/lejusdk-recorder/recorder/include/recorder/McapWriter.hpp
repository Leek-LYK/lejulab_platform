#pragma once

#include "recorder/Configuration.hpp"
#include "recorder/TypesCollection.hpp"
#include "common/Types.hpp"

#include <mcap/writer.hpp>
#include <string>
#include <unordered_map>
#include <mutex>
#include <atomic>
#include <memory>

namespace cyclone_record_replay {
namespace recorder {

/**
 * @brief Statistics for MCAP writing
 */
struct McapWriterStats {
    std::atomic<uint64_t> messages_written{0};
    std::atomic<uint64_t> bytes_written{0};
    std::atomic<uint64_t> schemas_written{0};
    std::atomic<uint64_t> channels_written{0};
    std::atomic<uint64_t> write_errors{0};
};

/**
 * @brief MCAP file writer for DDS recordings
 *
 * Handles writing DDS data to MCAP format including:
 * - Schema management from type information
 * - Channel creation for topics
 * - Message writing with timestamps
 * - File management and flushing
 */
class McapWriter {
public:
    /**
     * @brief Construct MCAP writer
     * @param config Output configuration
     */
    explicit McapWriter(const OutputConfig& config);

    /**
     * @brief Destructor - closes file
     */
    ~McapWriter();

    // Non-copyable
    McapWriter(const McapWriter&) = delete;
    McapWriter& operator=(const McapWriter&) = delete;

    /**
     * @brief Open the MCAP file for writing
     * @param path File path to open
     * @return true if opened successfully
     */
    bool open(const std::string& path);

    /**
     * @brief Close the MCAP file
     */
    void close();

    /**
     * @brief Close current file and open a new one, re-registering schemas/channels
     * @param new_path New file path
     * @return true if reopened successfully
     */
    bool reopen(const std::string& new_path);

    /**
     * @brief Generate a filename from prefix, date, and split index
     * @param prefix File name prefix
     * @param append_date Whether to append date-time
     * @param split_index Split index (-1 = no split suffix)
     * @return Generated file path ending in .mcap
     */
    static std::string generate_filename(
        const std::string& directory,
        const std::string& prefix,
        bool append_date,
        int split_index = -1);

    /**
     * @brief Check if file is open
     */
    bool is_open() const { return is_open_.load(); }

    /**
     * @brief Register a schema for a type
     * @param type_name Type name
     * @param schema_encoding Schema encoding (e.g., "ros2msg", "cdr", "jsonschema")
     * @param schema_data Schema data
     * @return Schema ID or 0 on failure
     */
    mcap::SchemaId register_schema(
        const std::string& type_name,
        const std::string& schema_encoding,
        const std::string& schema_data);

    /**
     * @brief Register a channel for a topic
     * @param topic_name Topic name
     * @param schema_id Schema ID from register_schema
     * @param message_encoding Message encoding (e.g., "cdr")
     * @return Channel ID or 0 on failure
     */
    mcap::ChannelId register_channel(
        const std::string& topic_name,
        mcap::SchemaId schema_id,
        const std::string& message_encoding = "cdr");

    /**
     * @brief Write a message
     * @param channel_id Channel ID
     * @param timestamp Timestamp in nanoseconds
     * @param data Message data
     * @param size Data size
     * @return true if written successfully
     */
    bool write_message(
        mcap::ChannelId channel_id,
        uint64_t timestamp,
        const void* data,
        size_t size);

    /**
     * @brief Write a message from vector
     * @param channel_id Channel ID
     * @param timestamp Timestamp in nanoseconds
     * @param data Message data
     * @return true if written successfully
     */
    bool write_message(
        mcap::ChannelId channel_id,
        uint64_t timestamp,
        const std::vector<uint8_t>& data);

    /**
     * @brief Flush buffered data to file
     */
    void flush();

    /**
     * @brief Get writing statistics
     */
    const McapWriterStats& stats() const { return stats_; }

    /**
     * @brief Get current file size
     */
    size_t current_file_size() const;

    /**
     * @brief Get current output file path
     */
    const std::string& output_path() const { return current_path_; }

    /**
     * @brief Check if a schema is registered for a type
     */
    bool has_schema(const std::string& type_name) const;

    /**
     * @brief Check if a channel is registered for a topic
     */
    bool has_channel(const std::string& topic_name) const;

    /**
     * @brief Get schema ID for a type
     */
    mcap::SchemaId get_schema_id(const std::string& type_name) const;

    /**
     * @brief Get channel ID for a topic
     */
    mcap::ChannelId get_channel_id(const std::string& topic_name) const;

private:
    OutputConfig config_;
    std::string current_path_;
    std::unique_ptr<mcap::McapWriter> writer_;
    std::atomic<bool> is_open_{false};

    mutable std::mutex mutex_;
    std::unordered_map<std::string, mcap::SchemaId> schema_ids_;
    std::unordered_map<std::string, mcap::ChannelId> channel_ids_;

    McapWriterStats stats_;
};

} // namespace recorder
} // namespace cyclone_record_replay
