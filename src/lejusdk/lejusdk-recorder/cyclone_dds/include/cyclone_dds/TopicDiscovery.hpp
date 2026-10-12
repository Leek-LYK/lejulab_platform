#pragma once

#include <dds/dds.h>
#include <string>
#include <vector>
#include <memory>
#include <functional>
#include <mutex>
#include <atomic>
#include <thread>
#include <unordered_map>
#include <unordered_set>

#include "common/Types.hpp"
#include "cyclone_dds/CycloneParticipant.hpp"

namespace cyclone_record_replay {
namespace cyclone_dds {

/**
 * @brief Discovered endpoint information from DCPSPublication/DCPSSubscription
 */
struct DiscoveredEndpoint {
    dds_guid_t guid;
    std::string topic_name;
    std::string type_name;
    dds_qos_t* qos = nullptr;
    const dds_typeinfo_t* type_info = nullptr;
    bool is_writer = false;

    DiscoveredEndpoint() = default;
    ~DiscoveredEndpoint();

    // Move only
    DiscoveredEndpoint(DiscoveredEndpoint&& other) noexcept;
    DiscoveredEndpoint& operator=(DiscoveredEndpoint&& other) noexcept;

    // No copy
    DiscoveredEndpoint(const DiscoveredEndpoint&) = delete;
    DiscoveredEndpoint& operator=(const DiscoveredEndpoint&) = delete;
};

/**
 * @brief Callback types for discovery events
 */
using EndpointDiscoveredCallback = std::function<void(const DiscoveredEndpoint&)>;
using EndpointRemovedCallback = std::function<void(const dds_guid_t&)>;

/**
 * @brief Topic and endpoint discovery using DDS builtin topics
 *
 * Uses DCPSPublication and DCPSSubscription builtin topics to discover
 * remote endpoints including their type information for dynamic type support.
 */
class TopicDiscovery {
public:
    /**
     * @brief Construct topic discovery for a participant
     * @param participant The DDS participant to monitor
     */
    explicit TopicDiscovery(CycloneParticipant& participant);

    /**
     * @brief Destructor - stops discovery and cleans up
     */
    ~TopicDiscovery();

    // Non-copyable, non-movable
    TopicDiscovery(const TopicDiscovery&) = delete;
    TopicDiscovery& operator=(const TopicDiscovery&) = delete;

    /**
     * @brief Set callback for endpoint discovered events
     * @param callback Function to call when a new endpoint is discovered
     */
    void set_endpoint_discovered_callback(EndpointDiscoveredCallback callback);

    /**
     * @brief Set callback for endpoint removed events
     * @param callback Function to call when an endpoint is removed
     */
    void set_endpoint_removed_callback(EndpointRemovedCallback callback);

    /**
     * @brief Start discovery monitoring
     */
    void start();

    /**
     * @brief Stop discovery monitoring
     */
    void stop();

    /**
     * @brief Check if discovery is running
     * @return true if actively monitoring for endpoints
     */
    bool is_running() const { return running_.load(); }

    /**
     * @brief Get all discovered topic names
     * @return Set of discovered topic names
     */
    std::unordered_set<std::string> get_discovered_topics() const;

    /**
     * @brief Get endpoints for a specific topic
     * @param topic_name Topic name to query
     * @return Vector of endpoint GUIDs publishing/subscribing to the topic
     */
    std::vector<dds_guid_t> get_endpoints_for_topic(const std::string& topic_name) const;

private:
    void discovery_thread_func();
    void process_publication_sample(const dds_builtintopic_endpoint_t* sample, bool is_alive);
    void process_subscription_sample(const dds_builtintopic_endpoint_t* sample, bool is_alive);
    void process_endpoint(const dds_builtintopic_endpoint_t* sample, bool is_writer, bool is_alive);

    CycloneParticipant& participant_;
    dds_entity_t publication_reader_ = 0;
    dds_entity_t subscription_reader_ = 0;

    std::atomic<bool> running_{false};
    std::unique_ptr<std::thread> discovery_thread_;

    mutable std::mutex callback_mutex_;
    EndpointDiscoveredCallback discovered_callback_;
    EndpointRemovedCallback removed_callback_;

    mutable std::mutex endpoints_mutex_;
    std::unordered_map<std::string, std::vector<dds_guid_t>> topic_endpoints_;
    std::unordered_set<std::string> discovered_topics_;
};

} // namespace cyclone_dds
} // namespace cyclone_record_replay
