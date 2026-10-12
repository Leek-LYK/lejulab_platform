#pragma once

#include "recorder/Configuration.hpp"
#include "recorder/McapWriter.hpp"
#include "recorder/TypesCollection.hpp"
#include "cyclone_dds/CycloneParticipant.hpp"
#include "cyclone_dds/TopicDiscovery.hpp"
#include "cyclone_dds/RawDataReader.hpp"
#include "common/Types.hpp"

#include <memory>
#include <atomic>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <chrono>
#include <functional>
#include <condition_variable>
#include <list>

namespace cyclone_record_replay {
namespace recorder {

/**
 * @brief Recording statistics
 */
struct RecorderStats {
    std::atomic<uint64_t> topics_discovered{0};
    std::atomic<uint64_t> messages_received{0};
    std::atomic<uint64_t> messages_recorded{0};
    std::atomic<uint64_t> bytes_recorded{0};
    std::chrono::steady_clock::time_point start_time;
    std::chrono::steady_clock::time_point end_time;

    double duration_seconds() const {
        auto end = (end_time == std::chrono::steady_clock::time_point{})
            ? std::chrono::steady_clock::now() : end_time;
        return std::chrono::duration<double>(end - start_time).count();
    }
};

/**
 * @brief State change callback
 */
using StateChangeCallback = std::function<void(common::RecordingState)>;

/**
 * @brief Main DDS recorder class
 *
 * Orchestrates DDS discovery, data reception, and MCAP writing.
 * Supports dynamic topic discovery and recording of arbitrary DDS types.
 */
class DdsRecorder {
public:
    /**
     * @brief Construct recorder with configuration
     * @param config Recorder configuration
     */
    explicit DdsRecorder(const RecorderConfiguration& config);

    /**
     * @brief Destructor - stops recording
     */
    ~DdsRecorder();

    // Non-copyable
    DdsRecorder(const DdsRecorder&) = delete;
    DdsRecorder& operator=(const DdsRecorder&) = delete;

    /**
     * @brief Initialize the recorder
     * @return true if initialization succeeded
     */
    bool init();

    /**
     * @brief Start recording
     * @return true if started successfully
     */
    bool start();

    /**
     * @brief Stop recording
     */
    void stop();

    /**
     * @brief Pause recording (keeps file open)
     */
    void pause();

    /**
     * @brief Resume recording after pause
     */
    void resume();

    /**
     * @brief Get current recording state
     */
    common::RecordingState state() const { return state_.load(); }

    /**
     * @brief Check if currently recording
     */
    bool is_recording() const {
        return state_.load() == common::RecordingState::Running;
    }

    /**
     * @brief Get recording statistics
     */
    const RecorderStats& stats() const { return stats_; }

    /**
     * @brief Get types collection
     */
    const TypesCollection& types() const { return types_collection_; }

    /**
     * @brief Set state change callback
     */
    void set_state_callback(StateChangeCallback callback);

    /**
     * @brief Get the output file path
     */
    std::string output_path() const;

    /**
     * @brief Wait for recording to complete (duration limit or file size limit)
     * @param timeout_ms Maximum wait time (0 = wait indefinitely)
     * @return true if recording completed, false if timeout
     */
    bool wait_for_completion(int64_t timeout_ms = 0);

private:
    void on_endpoint_discovered(const cyclone_dds::DiscoveredEndpoint& endpoint);
    void on_endpoint_removed(const dds_guid_t& guid);
    void data_polling_thread();
    void setup_reader_for_topic(const std::string& topic_name,
                                 const std::string& type_name,
                                 const dds_typeinfo_t* type_info);
    void record_sample(const std::string& topic_name,
                       const cyclone_dds::RawCdrSample& sample);
    void check_limits();
    void do_split();
    void check_num_splits();
    void re_register_schemas_and_channels();
    bool scheduled_check_disk();
    bool check_disk();
    void set_state(common::RecordingState new_state);

    RecorderConfiguration config_;
    std::atomic<common::RecordingState> state_{common::RecordingState::Stopped};

    std::unique_ptr<cyclone_dds::CycloneParticipant> participant_;
    std::unique_ptr<cyclone_dds::TopicDiscovery> discovery_;
    std::unique_ptr<McapWriter> writer_;

    TypesCollection types_collection_;

    mutable std::mutex readers_mutex_;
    std::unordered_map<std::string, std::unique_ptr<cyclone_dds::RawDataReader>> readers_;

    std::atomic<bool> polling_active_{false};
    std::unique_ptr<std::thread> polling_thread_;

    RecorderStats stats_;

    mutable std::mutex callback_mutex_;
    StateChangeCallback state_callback_;

    std::mutex completion_mutex_;
    std::condition_variable completion_cv_;

    uint32_t split_count_{0};
    std::list<std::string> split_files_;

    bool writing_enabled_{true};
    std::chrono::steady_clock::time_point check_disk_next_;
    std::chrono::steady_clock::time_point disk_warn_next_;
};

} // namespace recorder
} // namespace cyclone_record_replay
