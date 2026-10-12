#include "recorder/McapWriter.hpp"
#include "common/Log.hpp"

#include <fstream>
#include <chrono>
#include <sstream>
#include <iomanip>
#include <ctime>
#include <cstdlib>
#include <filesystem>

namespace cyclone_record_replay {
namespace recorder {

McapWriter::McapWriter(const OutputConfig& config)
    : config_(config)
    , writer_(std::make_unique<mcap::McapWriter>())
{
}

McapWriter::~McapWriter() {
    close();
}

bool McapWriter::open(const std::string& path) {
    if (is_open_.load()) {
        LOG_WARN("MCAP file already open");
        return true;
    }

    current_path_ = path;

    mcap::McapWriterOptions options("cyclone-dds-recorder");

    // Configure compression
    if (config_.enable_compression) {
        if (config_.compression_type == "zstd") {
            options.compression = mcap::Compression::Zstd;
        } else if (config_.compression_type == "lz4") {
            options.compression = mcap::Compression::Lz4;
        } else {
            options.compression = mcap::Compression::None;
        }
        options.compressionLevel = static_cast<mcap::CompressionLevel>(
            config_.compression_level);
    } else {
        options.compression = mcap::Compression::None;
    }

    // Open file
    auto status = writer_->open(current_path_, options);
    if (!status.ok()) {
        LOG_ERROR("Failed to open MCAP file '{}': {}",
            current_path_, status.message);
        return false;
    }

    is_open_.store(true);
    LOG_INFO("Opened MCAP file '{}' for writing", current_path_);

    return true;
}

bool McapWriter::reopen(const std::string& new_path) {
    // Save current schema/channel mappings
    auto saved_schemas = schema_ids_;
    auto saved_channels = channel_ids_;

    // Close current file
    close();

    // Reset writer
    writer_ = std::make_unique<mcap::McapWriter>();

    // Reset stats for new file
    stats_.messages_written.store(0);
    stats_.bytes_written.store(0);
    stats_.write_errors.store(0);

    // Clear registrations (will re-register)
    schema_ids_.clear();
    channel_ids_.clear();

    // Open new file
    if (!open(new_path)) {
        return false;
    }

    // Re-register schemas and channels in the new file
    // Note: we re-register with the same names but get new IDs
    // The caller (DdsRecorder) will handle re-mapping if needed
    LOG_INFO("Reopened MCAP file as '{}'", new_path);
    return true;
}

std::string McapWriter::generate_filename(
    const std::string& directory,
    const std::string& prefix,
    bool append_date,
    int split_index)
{
    std::string name = prefix;

    if (append_date) {
        auto now = std::chrono::system_clock::now();
        auto time_t_now = std::chrono::system_clock::to_time_t(now);
        std::tm tm_now{};
        localtime_r(&time_t_now, &tm_now);

        std::ostringstream oss;
        oss << std::put_time(&tm_now, "%Y-%m-%d-%H-%M-%S");
        name += "_" + oss.str();
    }

    if (split_index >= 0) {
        name += "_" + std::to_string(split_index);
    }

    name += ".mcap";

    if (!directory.empty()) {
        std::filesystem::create_directories(directory);
        return (std::filesystem::path(directory) / name).string();
    }

    // No directory configured:
    // - absolute prefix -> respect it as-is
    // - relative prefix -> place under ~/.ros/lejulab/mcap/
    std::filesystem::path out_path(name);
    if (out_path.is_absolute()) {
        return name;
    }

    const char* home = std::getenv("HOME");
    std::filesystem::path base =
        (home && *home) ? std::filesystem::path(home) : std::filesystem::path(".");
    base /= ".ros/lejulab/mcap";
    std::filesystem::create_directories(base);
    return (base / name).string();
}

void McapWriter::close() {
    if (!is_open_.load()) {
        return;
    }

    writer_->close();
    is_open_.store(false);

    LOG_INFO("Closed MCAP file '{}' - {} messages, {} bytes",
        current_path_,
        stats_.messages_written.load(),
        stats_.bytes_written.load());
}

mcap::SchemaId McapWriter::register_schema(
    const std::string& type_name,
    const std::string& schema_encoding,
    const std::string& schema_data)
{
    std::lock_guard<std::mutex> lock(mutex_);

    // Check if already registered
    auto it = schema_ids_.find(type_name);
    if (it != schema_ids_.end()) {
        return it->second;
    }

    if (!is_open_.load()) {
        LOG_ERROR("Cannot register schema: file not open");
        return 0;
    }

    mcap::Schema schema;
    schema.name = type_name;
    schema.encoding = schema_encoding;
    schema.data.assign(
        reinterpret_cast<const std::byte*>(schema_data.data()),
        reinterpret_cast<const std::byte*>(schema_data.data() + schema_data.size()));

    writer_->addSchema(schema);

    schema_ids_[type_name] = schema.id;
    stats_.schemas_written++;

    LOG_DEBUG("Registered schema '{}' with ID {}", type_name, static_cast<unsigned>(schema.id));

    return schema.id;
}

mcap::ChannelId McapWriter::register_channel(
    const std::string& topic_name,
    mcap::SchemaId schema_id,
    const std::string& message_encoding)
{
    std::lock_guard<std::mutex> lock(mutex_);

    // Check if already registered
    auto it = channel_ids_.find(topic_name);
    if (it != channel_ids_.end()) {
        return it->second;
    }

    if (!is_open_.load()) {
        LOG_ERROR("Cannot register channel: file not open");
        return 0;
    }

    mcap::Channel channel;
    channel.topic = topic_name;
    channel.schemaId = schema_id;
    channel.messageEncoding = message_encoding;

    writer_->addChannel(channel);

    channel_ids_[topic_name] = channel.id;
    stats_.channels_written++;

    LOG_DEBUG("Registered channel '{}' with ID {}", topic_name, static_cast<unsigned>(channel.id));

    return channel.id;
}

bool McapWriter::write_message(
    mcap::ChannelId channel_id,
    uint64_t timestamp,
    const void* data,
    size_t size)
{
    if (!is_open_.load()) {
        stats_.write_errors++;
        return false;
    }

    mcap::Message msg;
    msg.channelId = channel_id;
    msg.logTime = timestamp;
    msg.publishTime = timestamp;
    msg.data = reinterpret_cast<const std::byte*>(data);
    msg.dataSize = size;

    auto status = writer_->write(msg);
    if (!status.ok()) {
        LOG_ERROR("Failed to write message: {}", status.message);
        stats_.write_errors++;
        return false;
    }

    stats_.messages_written++;
    stats_.bytes_written += size;

    return true;
}

bool McapWriter::write_message(
    mcap::ChannelId channel_id,
    uint64_t timestamp,
    const std::vector<uint8_t>& data)
{
    return write_message(channel_id, timestamp, data.data(), data.size());
}

void McapWriter::flush() {
    // MCAP library handles buffering internally
    // This is a no-op but kept for interface consistency
}

size_t McapWriter::current_file_size() const {
    // Approximate file size based on bytes written
    // Actual file size may be larger due to MCAP overhead
    return stats_.bytes_written.load();
}

bool McapWriter::has_schema(const std::string& type_name) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return schema_ids_.find(type_name) != schema_ids_.end();
}

bool McapWriter::has_channel(const std::string& topic_name) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return channel_ids_.find(topic_name) != channel_ids_.end();
}

mcap::SchemaId McapWriter::get_schema_id(const std::string& type_name) const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = schema_ids_.find(type_name);
    if (it != schema_ids_.end()) {
        return it->second;
    }
    return 0;
}

mcap::ChannelId McapWriter::get_channel_id(const std::string& topic_name) const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = channel_ids_.find(topic_name);
    if (it != channel_ids_.end()) {
        return it->second;
    }
    return 0;
}

} // namespace recorder
} // namespace cyclone_record_replay
