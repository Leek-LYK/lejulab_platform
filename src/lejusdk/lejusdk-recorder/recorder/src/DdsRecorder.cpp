#include "recorder/DdsRecorder.hpp"
#include "common/Log.hpp"

#include <chrono>
#include <cstdio>
#include <cerrno>
#include <cstring>
#include <sys/statvfs.h>

namespace cyclone_record_replay {
namespace recorder {

DdsRecorder::DdsRecorder(const RecorderConfiguration& config)
    : config_(config)
{
}

DdsRecorder::~DdsRecorder() {
    stop();
}

bool DdsRecorder::init() {
    LOG_INFO("Initializing DDS recorder");

    // Create participant
    cyclone_dds::CycloneParticipant::Config part_config;
    part_config.domain_id = config_.dds_config().domain_id;
    part_config.participant_name = "DdsRecorder";
    part_config.enable_topic_discovery = true;

    participant_ = std::make_unique<cyclone_dds::CycloneParticipant>(part_config);
    if (!participant_->is_valid()) {
        LOG_ERROR("Failed to create DDS participant");
        return false;
    }

    // Create topic discovery
    discovery_ = std::make_unique<cyclone_dds::TopicDiscovery>(*participant_);
    discovery_->set_endpoint_discovered_callback(
        [this](const cyclone_dds::DiscoveredEndpoint& ep) {
            on_endpoint_discovered(ep);
        });
    discovery_->set_endpoint_removed_callback(
        [this](const dds_guid_t& guid) {
            on_endpoint_removed(guid);
        });

    // Create MCAP writer
    writer_ = std::make_unique<McapWriter>(config_.output_config());

    LOG_INFO("DDS recorder initialized on domain {}",
        config_.dds_config().domain_id);

    return true;
}

bool DdsRecorder::start() {
    if (state_.load() != common::RecordingState::Stopped) {
        LOG_WARN("Recorder is not in stopped state");
        return false;
    }

    const auto& out = config_.output_config();
    split_count_ = 0;
    split_files_.clear();

    int split_idx = out.split ? 0 : -1;
    std::string first_path = McapWriter::generate_filename(
        out.directory, out.prefix, out.append_date, split_idx);

    LOG_INFO("Starting recording to '{}'", first_path);

    // Open MCAP file
    if (!writer_->open(first_path)) {
        LOG_ERROR("Failed to open MCAP file");
        return false;
    }

    if (out.split) {
        split_files_.push_back(first_path);
    }

    // Start discovery
    discovery_->start();

    // Start data polling
    polling_active_.store(true);
    polling_thread_ = std::make_unique<std::thread>(
        &DdsRecorder::data_polling_thread, this);

    stats_.start_time = std::chrono::steady_clock::now();
    writing_enabled_ = true;
    check_disk_next_ = std::chrono::steady_clock::now();
    disk_warn_next_ = std::chrono::steady_clock::time_point{};
    set_state(common::RecordingState::Running);

    LOG_INFO("Recording started");

    return true;
}

void DdsRecorder::stop() {
    // Always try to join the polling thread first, even if state is already Stopped
    // This handles the case where the thread stopped due to duration limit
    polling_active_.store(false);
    if (polling_thread_ && polling_thread_->joinable()) {
        polling_thread_->join();
    }
    polling_thread_.reset();

    // If already stopped (e.g., by polling thread due to limit), just return
    if (state_.load() == common::RecordingState::Stopped) {
        return;
    }

    LOG_INFO("Stopping recording");

    set_state(common::RecordingState::Stopped);

    // Stop discovery
    if (discovery_) {
        discovery_->stop();
    }

    // Close MCAP file
    if (writer_) {
        writer_->close();
    }

    stats_.end_time = std::chrono::steady_clock::now();

    // Notify completion
    {
        std::lock_guard<std::mutex> lock(completion_mutex_);
        completion_cv_.notify_all();
    }

    LOG_INFO("Recording stopped - {} topics, {} messages, {:.2f} seconds",
        stats_.topics_discovered.load(),
        stats_.messages_recorded.load(),
        stats_.duration_seconds());
}

void DdsRecorder::pause() {
    if (state_.load() != common::RecordingState::Running) {
        return;
    }
    set_state(common::RecordingState::Paused);
    LOG_INFO("Recording paused");
}

void DdsRecorder::resume() {
    if (state_.load() != common::RecordingState::Paused) {
        return;
    }
    set_state(common::RecordingState::Running);
    LOG_INFO("Recording resumed");
}

void DdsRecorder::set_state_callback(StateChangeCallback callback) {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    state_callback_ = std::move(callback);
}

std::string DdsRecorder::output_path() const {
    return writer_ ? writer_->output_path() : "";
}

bool DdsRecorder::wait_for_completion(int64_t timeout_ms) {
    std::unique_lock<std::mutex> lock(completion_mutex_);

    if (timeout_ms <= 0) {
        completion_cv_.wait(lock, [this]() {
            return state_.load() == common::RecordingState::Stopped;
        });
        return true;
    }

    return completion_cv_.wait_for(lock, std::chrono::milliseconds(timeout_ms),
        [this]() {
            return state_.load() == common::RecordingState::Stopped;
        });
}

void DdsRecorder::on_endpoint_discovered(const cyclone_dds::DiscoveredEndpoint& endpoint) {
    // Only interested in writers (data producers)
    if (!endpoint.is_writer) {
        return;
    }

    // Check topic filter
    if (!config_.dds_config().topic_filter.matches(endpoint.topic_name)) {
        LOG_DEBUG("Topic '{}' filtered out", endpoint.topic_name);
        return;
    }

    LOG_DEBUG("Discovered writer on topic '{}' type '{}'",
        endpoint.topic_name, endpoint.type_name);

    stats_.topics_discovered++;

    // Add type to collection
    types_collection_.add_type(
        endpoint.type_name,
        endpoint.type_info,
        participant_->handle());

    // Register topic
    types_collection_.register_topic(
        endpoint.topic_name,
        endpoint.type_name);

    // Register schema in MCAP
    if (!writer_->has_schema(endpoint.type_name)) {
        const auto* type = types_collection_.get_type(endpoint.type_name);
        if (type) {
            writer_->register_schema(
                endpoint.type_name,
                type->schema_encoding,  // OMG IDL encoding for DDS
                type->schema_data);
        }
    }

    // Register channel in MCAP
    if (!writer_->has_channel(endpoint.topic_name)) {
        auto schema_id = writer_->get_schema_id(endpoint.type_name);
        writer_->register_channel(endpoint.topic_name, schema_id);
    }

    // Setup reader for this topic
    setup_reader_for_topic(
        endpoint.topic_name,
        endpoint.type_name,
        endpoint.type_info);
}

void DdsRecorder::on_endpoint_removed(const dds_guid_t& guid) {
    // Log endpoint removal, but keep reader active
    // (other endpoints might be publishing to the same topic)
    LOG_DEBUG("Endpoint removed");
}

void DdsRecorder::data_polling_thread() {
    LOG_DEBUG("Data polling thread started");

    while (polling_active_.load()) {
        if (state_.load() != common::RecordingState::Running) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        // Poll all readers
        std::vector<std::pair<std::string, std::unique_ptr<cyclone_dds::RawDataReader>*>> readers_snapshot;

        {
            std::lock_guard<std::mutex> lock(readers_mutex_);
            for (auto& [topic, reader] : readers_) {
                readers_snapshot.emplace_back(topic, &reader);
            }
        }

        bool had_data = false;
        for (auto& [topic, reader_ptr] : readers_snapshot) {
            auto samples = (*reader_ptr)->take_raw();
            for (auto& sample : samples) {
                record_sample(topic, sample);
                had_data = true;
            }
        }

        // Check limits (may set polling_active_ to false)
        check_limits();

        // Sleep if no data
        if (!had_data && polling_active_.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    // If exiting due to limit, do cleanup here
    // (stop() will be a no-op since polling_active_ is already false)
    if (state_.load() == common::RecordingState::Running) {
        LOG_INFO("Polling thread finishing recording");

        // Stop discovery
        if (discovery_) {
            discovery_->stop();
        }

        // Close MCAP file
        if (writer_) {
            writer_->close();
        }

        stats_.end_time = std::chrono::steady_clock::now();
        set_state(common::RecordingState::Stopped);

        // Notify completion
        {
            std::lock_guard<std::mutex> lock(completion_mutex_);
            completion_cv_.notify_all();
        }

        LOG_INFO("Recording stopped - {} topics, {} messages, {:.2f} seconds",
            stats_.topics_discovered.load(),
            stats_.messages_recorded.load(),
            stats_.duration_seconds());
    }

    LOG_DEBUG("Data polling thread stopped");
}

void DdsRecorder::setup_reader_for_topic(
    const std::string& topic_name,
    const std::string& type_name,
    const dds_typeinfo_t* type_info)
{
    std::lock_guard<std::mutex> lock(readers_mutex_);

    // Check if reader already exists
    if (readers_.find(topic_name) != readers_.end()) {
        return;
    }

    cyclone_dds::RawDataReader::Config config;
    config.topic_name = topic_name;
    config.type_name = type_name;
    config.type_info = type_info;
    config.reliability = common::ReliabilityKind::BestEffort;
    config.durability = common::DurabilityKind::Volatile;
    config.history_depth = 100;

    auto reader = std::make_unique<cyclone_dds::RawDataReader>(*participant_, config);
    if (reader->is_valid()) {
        readers_[topic_name] = std::move(reader);
        LOG_DEBUG("Subscribed to topic '{}'", topic_name);
    } else {
        LOG_ERROR("Failed to create reader for topic '{}'", topic_name);
    }
}

void DdsRecorder::record_sample(
    const std::string& topic_name,
    const cyclone_dds::RawCdrSample& sample)
{
    stats_.messages_received++;

    if (!scheduled_check_disk() || !writing_enabled_) {
        return;
    }

    auto channel_id = writer_->get_channel_id(topic_name);
    if (channel_id == 0) {
        LOG_WARN("No channel for topic '{}'", topic_name);
        return;
    }

    uint64_t timestamp = static_cast<uint64_t>(sample.timestamp);

    if (writer_->write_message(channel_id, timestamp, sample.cdr_data)) {
        stats_.messages_recorded++;
        stats_.bytes_recorded += sample.cdr_data.size();
    }
}

void DdsRecorder::check_limits() {
    const auto& output = config_.output_config();

    bool limit_reached = false;

    // Check duration limit
    if (output.max_duration_seconds > 0) {
        if (stats_.duration_seconds() >= output.max_duration_seconds) {
            LOG_INFO("Duration limit reached ({:.1f}s)", output.max_duration_seconds);
            limit_reached = true;
        }
    }

    // Check file size limit
    if (!limit_reached && output.max_file_size > 0) {
        if (writer_->current_file_size() >= output.max_file_size) {
            LOG_INFO("File size limit reached ({} bytes)", output.max_file_size);
            limit_reached = true;
        }
    }

    if (!limit_reached) {
        return;
    }

    if (output.split) {
        do_split();
    } else {
        polling_active_.store(false);
    }
}

void DdsRecorder::do_split() {
    const auto& output = config_.output_config();

    split_count_++;
    std::string new_path = McapWriter::generate_filename(
        output.directory, output.prefix, output.append_date, static_cast<int>(split_count_));

    LOG_INFO("Splitting to new file '{}' (split #{})", new_path, split_count_);

    if (!writer_->reopen(new_path)) {
        LOG_ERROR("Failed to reopen MCAP file, stopping recording");
        polling_active_.store(false);
        return;
    }

    // Re-register all schemas and channels in the new file
    re_register_schemas_and_channels();

    // Track split files
    split_files_.push_back(new_path);
    check_num_splits();

    // Reset duration start time for next split
    stats_.start_time = std::chrono::steady_clock::now();
}

void DdsRecorder::check_num_splits() {
    const auto& output = config_.output_config();
    if (output.max_splits == 0) {
        return;
    }

    while (split_files_.size() > output.max_splits) {
        const std::string& oldest = split_files_.front();
        LOG_INFO("Removing oldest split file '{}'", oldest);
        if (std::remove(oldest.c_str()) != 0) {
            LOG_ERROR("Failed to remove '{}': {}", oldest, strerror(errno));
        }
        split_files_.pop_front();
    }
}

void DdsRecorder::re_register_schemas_and_channels() {
    // Re-register all known types and topics into the new MCAP file
    for (const auto& [topic_name, reader_ptr] : readers_) {
        std::string type_name = types_collection_.get_type_for_topic(topic_name);
        if (type_name.empty()) continue;

        const auto* type = types_collection_.get_type(type_name);
        if (!type) continue;

        // Register schema
        if (!writer_->has_schema(type_name)) {
            writer_->register_schema(type_name, type->schema_encoding, type->schema_data);
        }

        // Register channel
        if (!writer_->has_channel(topic_name)) {
            auto schema_id = writer_->get_schema_id(type_name);
            writer_->register_channel(topic_name, schema_id);
        }
    }
}

bool DdsRecorder::scheduled_check_disk() {
    auto now = std::chrono::steady_clock::now();
    if (now < check_disk_next_) {
        return writing_enabled_;
    }
    check_disk_next_ = now + std::chrono::seconds(20);
    return check_disk();
}

bool DdsRecorder::check_disk() {
    const auto& output = config_.output_config();
    if (output.min_space == 0) {
        return true;
    }

    std::string path = writer_->output_path();
    if (path.empty()) {
        return true;
    }

    struct statvfs stat;
    if (statvfs(path.c_str(), &stat) != 0) {
        LOG_WARN("Failed to check disk space: {}", strerror(errno));
        return true;
    }

    uint64_t available = static_cast<uint64_t>(stat.f_bsize) * static_cast<uint64_t>(stat.f_bavail);

    if (available < output.min_space) {
        LOG_ERROR("Disk space below minimum ({} bytes available, {} required). Disabling recording.",
            available,
            output.min_space);
        writing_enabled_ = false;
        return false;
    }

    if (available < 5 * output.min_space) {
        auto now = std::chrono::steady_clock::now();
        if (now >= disk_warn_next_) {
            LOG_WARN("Low disk space: {} bytes available (threshold: {})",
                available,
                output.min_space);
            disk_warn_next_ = now + std::chrono::seconds(5);
        }
    }

    writing_enabled_ = true;
    return true;
}

void DdsRecorder::set_state(common::RecordingState new_state) {
    state_.store(new_state);

    std::lock_guard<std::mutex> lock(callback_mutex_);
    if (state_callback_) {
        state_callback_(new_state);
    }
}

} // namespace recorder
} // namespace cyclone_record_replay
