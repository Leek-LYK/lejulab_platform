#pragma once

#include "common/Config.hpp"
#include "common/Types.hpp"

#include <string>
#include <chrono>

namespace cyclone_record_replay {
namespace recorder {

/**
 * @brief Output file configuration
 */
struct OutputConfig {
    std::string directory;                     // Output directory (empty = cwd)
    std::string prefix = "recording";  // File name prefix
    bool append_date = true;           // Append date-time to filename

    // Maximum file size in bytes (0 = unlimited)
    size_t max_file_size = 0;

    // Maximum recording duration in seconds (0 = unlimited)
    double max_duration_seconds = 0.0;

    // Split settings
    bool split = false;          // Enable file splitting
    uint32_t max_splits = 0;     // Max split files to keep (0 = unlimited)

    // Disk space settings
    uint64_t min_space = 1024ULL * 1024 * 1024;  // 1 GB default

    // Buffer settings
    size_t buffer_size = 1024 * 1024;  // 1 MB default buffer

    // Compression settings
    bool enable_compression = true;
    std::string compression_type = "zstd";  // zstd, lz4, or none
    int compression_level = 3;
};

/**
 * @brief Recording timing configuration
 */
struct TimingConfig {
    // Use simulation time if available
    bool use_sim_time = false;

    // Start recording immediately or wait for trigger
    bool start_immediately = true;

    // Event to wait for before starting
    std::string start_trigger_topic;
};

/**
 * @brief Logging configuration
 */
struct LoggingConfig {
    // Log level: trace, debug, info, warn, error, critical
    std::string level = "info";
};

/**
 * @brief Recorder configuration
 */
class RecorderConfiguration : public common::ConfigBase {
public:
    RecorderConfiguration() = default;

    /**
     * @brief Get output configuration
     */
    const OutputConfig& output_config() const { return output_config_; }

    /**
     * @brief Get timing configuration
     */
    const TimingConfig& timing_config() const { return timing_config_; }

    /**
     * @brief Get logging configuration
     */
    const LoggingConfig& logging_config() const { return logging_config_; }

    /**
     * @brief Set output prefix
     */
    void set_output_prefix(const std::string& prefix) {
        output_config_.prefix = prefix;
    }

    /**
     * @brief Set max duration
     */
    void set_max_duration(double seconds) {
        output_config_.max_duration_seconds = seconds;
    }

    void set_compression(bool enabled) {
        output_config_.enable_compression = enabled;
    }

    void set_split(bool enabled) {
        output_config_.split = enabled;
    }

    void set_max_splits(uint32_t n) {
        output_config_.max_splits = n;
    }

    void set_max_file_size(size_t bytes) {
        output_config_.max_file_size = bytes;
    }

    void set_min_space(uint64_t bytes) {
        output_config_.min_space = bytes;
    }

    void set_compression_type(const std::string& type) {
        output_config_.compression_type = type;
    }

    OutputConfig& mutable_output_config() { return output_config_; }

protected:
    void parse_specific(const YAML::Node& root) override;

private:
    void parse_output_config(const YAML::Node& node);
    void parse_timing_config(const YAML::Node& node);
    void parse_logging_config(const YAML::Node& node);

    OutputConfig output_config_;
    TimingConfig timing_config_;
    LoggingConfig logging_config_;
};

} // namespace recorder
} // namespace cyclone_record_replay
