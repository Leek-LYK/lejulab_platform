#pragma once

#include <dds/dds.h>
#include <cstdint>
#include <string>
#include <memory>
#include <functional>

namespace cyclone_record_replay {
namespace cyclone_dds {

/**
 * @brief RAII wrapper for CycloneDDS Participant
 *
 * Manages the lifecycle of a DDS domain participant and provides
 * configuration options for topic discovery and QoS settings.
 */
class CycloneParticipant {
public:
    /**
     * @brief Configuration for the participant
     */
    struct Config {
        int32_t domain_id = 0;
        std::string participant_name = "CycloneParticipant";

        // Enable topic/type discovery (requires CycloneDDS compiled with
        // ENABLE_TOPIC_DISCOVERY and ENABLE_TYPE_DISCOVERY)
        bool enable_topic_discovery = true;

        Config() = default;
    };

    /**
     * @brief Construct a new participant with default configuration
     */
    CycloneParticipant();

    /**
     * @brief Construct a new participant
     * @param config Participant configuration
     */
    explicit CycloneParticipant(const Config& config);

    /**
     * @brief Destructor - cleans up DDS resources
     */
    ~CycloneParticipant();

    // Non-copyable
    CycloneParticipant(const CycloneParticipant&) = delete;
    CycloneParticipant& operator=(const CycloneParticipant&) = delete;

    // Movable
    CycloneParticipant(CycloneParticipant&& other) noexcept;
    CycloneParticipant& operator=(CycloneParticipant&& other) noexcept;

    /**
     * @brief Get the underlying DDS participant handle
     * @return DDS entity handle
     */
    dds_entity_t handle() const { return participant_; }

    /**
     * @brief Check if participant is valid
     * @return true if participant was created successfully
     */
    bool is_valid() const { return participant_ > 0; }

    /**
     * @brief Get the domain ID
     * @return Domain ID
     */
    int32_t domain_id() const { return config_.domain_id; }

    /**
     * @brief Get participant GUID
     * @return DDS GUID
     */
    dds_guid_t guid() const;

private:
    Config config_;
    dds_entity_t participant_ = 0;

    void create_participant();
    void destroy_participant();
};

} // namespace cyclone_dds
} // namespace cyclone_record_replay
