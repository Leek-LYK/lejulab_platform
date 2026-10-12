#include "cyclone_dds/TopicDiscovery.hpp"
#include "common/Log.hpp"

#include <dds/ddsc/dds_public_qos.h>
#include <chrono>
#include <cstring>

namespace cyclone_record_replay {
namespace cyclone_dds {

DiscoveredEndpoint::~DiscoveredEndpoint() {
    if (qos) {
        dds_delete_qos(qos);
        qos = nullptr;
    }
    // Note: type_info is owned by CycloneDDS, don't free it
}

DiscoveredEndpoint::DiscoveredEndpoint(DiscoveredEndpoint&& other) noexcept
    : guid(other.guid)
    , topic_name(std::move(other.topic_name))
    , type_name(std::move(other.type_name))
    , qos(other.qos)
    , type_info(other.type_info)
    , is_writer(other.is_writer)
{
    other.qos = nullptr;
    other.type_info = nullptr;
}

DiscoveredEndpoint& DiscoveredEndpoint::operator=(DiscoveredEndpoint&& other) noexcept {
    if (this != &other) {
        if (qos) {
            dds_delete_qos(qos);
        }

        guid = other.guid;
        topic_name = std::move(other.topic_name);
        type_name = std::move(other.type_name);
        qos = other.qos;
        type_info = other.type_info;
        is_writer = other.is_writer;

        other.qos = nullptr;
        other.type_info = nullptr;
    }
    return *this;
}

TopicDiscovery::TopicDiscovery(CycloneParticipant& participant)
    : participant_(participant)
{
}

TopicDiscovery::~TopicDiscovery() {
    stop();
}

void TopicDiscovery::set_endpoint_discovered_callback(EndpointDiscoveredCallback callback) {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    discovered_callback_ = std::move(callback);
}

void TopicDiscovery::set_endpoint_removed_callback(EndpointRemovedCallback callback) {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    removed_callback_ = std::move(callback);
}

void TopicDiscovery::start() {
    if (running_.load()) {
        LOG_WARN("Topic discovery already running");
        return;
    }

    if (!participant_.is_valid()) {
        LOG_ERROR("Cannot start discovery: invalid participant");
        return;
    }

    // Create builtin topic readers
    publication_reader_ = dds_create_reader(
        participant_.handle(),
        DDS_BUILTIN_TOPIC_DCPSPUBLICATION,
        nullptr, nullptr);

    if (publication_reader_ < 0) {
        LOG_ERROR("Failed to create DCPSPublication reader: {}",
            dds_strretcode(publication_reader_));
        return;
    }

    subscription_reader_ = dds_create_reader(
        participant_.handle(),
        DDS_BUILTIN_TOPIC_DCPSSUBSCRIPTION,
        nullptr, nullptr);

    if (subscription_reader_ < 0) {
        LOG_ERROR("Failed to create DCPSSubscription reader: {}",
            dds_strretcode(subscription_reader_));
        dds_delete(publication_reader_);
        publication_reader_ = 0;
        return;
    }

    running_.store(true);
    discovery_thread_ = std::make_unique<std::thread>(&TopicDiscovery::discovery_thread_func, this);

    LOG_INFO("Topic discovery started");
}

void TopicDiscovery::stop() {
    if (!running_.load()) {
        return;
    }

    running_.store(false);

    if (discovery_thread_ && discovery_thread_->joinable()) {
        discovery_thread_->join();
    }
    discovery_thread_.reset();

    if (publication_reader_ > 0) {
        dds_delete(publication_reader_);
        publication_reader_ = 0;
    }

    if (subscription_reader_ > 0) {
        dds_delete(subscription_reader_);
        subscription_reader_ = 0;
    }

    LOG_INFO("Topic discovery stopped");
}

std::unordered_set<std::string> TopicDiscovery::get_discovered_topics() const {
    std::lock_guard<std::mutex> lock(endpoints_mutex_);
    return discovered_topics_;
}

std::vector<dds_guid_t> TopicDiscovery::get_endpoints_for_topic(const std::string& topic_name) const {
    std::lock_guard<std::mutex> lock(endpoints_mutex_);
    auto it = topic_endpoints_.find(topic_name);
    if (it != topic_endpoints_.end()) {
        return it->second;
    }
    return {};
}

void TopicDiscovery::discovery_thread_func() {
    constexpr size_t MAX_SAMPLES = 32;
    void* samples[MAX_SAMPLES];
    dds_sample_info_t infos[MAX_SAMPLES];

    for (size_t i = 0; i < MAX_SAMPLES; ++i) {
        samples[i] = nullptr;
    }

    while (running_.load()) {
        // Poll DCPSPublication
        dds_return_t n = dds_take(publication_reader_, samples, infos, MAX_SAMPLES, MAX_SAMPLES);
        if (n > 0) {
            for (int32_t i = 0; i < n; ++i) {
                if (infos[i].valid_data) {
                    auto* sample = static_cast<dds_builtintopic_endpoint_t*>(samples[i]);
                    process_publication_sample(sample, infos[i].instance_state == DDS_IST_ALIVE);
                }
            }
            dds_return_loan(publication_reader_, samples, n);
        }

        // Poll DCPSSubscription
        n = dds_take(subscription_reader_, samples, infos, MAX_SAMPLES, MAX_SAMPLES);
        if (n > 0) {
            for (int32_t i = 0; i < n; ++i) {
                if (infos[i].valid_data) {
                    auto* sample = static_cast<dds_builtintopic_endpoint_t*>(samples[i]);
                    process_subscription_sample(sample, infos[i].instance_state == DDS_IST_ALIVE);
                }
            }
            dds_return_loan(subscription_reader_, samples, n);
        }

        // Sleep to avoid busy polling
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void TopicDiscovery::process_publication_sample(const dds_builtintopic_endpoint_t* sample, bool is_alive) {
    process_endpoint(sample, true, is_alive);
}

void TopicDiscovery::process_subscription_sample(const dds_builtintopic_endpoint_t* sample, bool is_alive) {
    process_endpoint(sample, false, is_alive);
}

void TopicDiscovery::process_endpoint(const dds_builtintopic_endpoint_t* sample, bool is_writer, bool is_alive) {
    if (!sample || !sample->topic_name || !sample->type_name) {
        return;
    }

    // Skip builtin topics
    if (sample->topic_name[0] == '\0' ||
        strncmp(sample->topic_name, "DCPS", 4) == 0 ||
        strncmp(sample->topic_name, "rt/", 3) == 0) {
        return;
    }

    std::string topic_name(sample->topic_name);
    std::string type_name(sample->type_name);

    if (is_alive) {
        // Endpoint discovered
        LOG_DEBUG("Discovered {} on topic '{}' type '{}'",
            is_writer ? "writer" : "reader", topic_name, type_name);

        {
            std::lock_guard<std::mutex> lock(endpoints_mutex_);
            discovered_topics_.insert(topic_name);
            topic_endpoints_[topic_name].push_back(sample->key);
        }

        // Invoke callback
        std::lock_guard<std::mutex> lock(callback_mutex_);
        if (discovered_callback_) {
            DiscoveredEndpoint ep;
            ep.guid = sample->key;
            ep.topic_name = topic_name;
            ep.type_name = type_name;
            ep.is_writer = is_writer;

            // Copy QoS
            if (sample->qos) {
                ep.qos = dds_create_qos();
                dds_copy_qos(ep.qos, sample->qos);
            }

            // Get type info (if available)
            dds_builtintopic_get_endpoint_type_info(
                const_cast<dds_builtintopic_endpoint_t*>(sample), &ep.type_info);

            discovered_callback_(ep);
        }
    } else {
        // Endpoint removed
        LOG_DEBUG("Removed {} from topic '{}'", is_writer ? "writer" : "reader", topic_name);

        {
            std::lock_guard<std::mutex> lock(endpoints_mutex_);
            auto it = topic_endpoints_.find(topic_name);
            if (it != topic_endpoints_.end()) {
                auto& guids = it->second;
                guids.erase(
                    std::remove_if(guids.begin(), guids.end(),
                        [&sample](const dds_guid_t& g) {
                            return std::memcmp(&g, &sample->key, sizeof(dds_guid_t)) == 0;
                        }),
                    guids.end());

                if (guids.empty()) {
                    topic_endpoints_.erase(it);
                    discovered_topics_.erase(topic_name);
                }
            }
        }

        // Invoke callback
        std::lock_guard<std::mutex> lock(callback_mutex_);
        if (removed_callback_) {
            removed_callback_(sample->key);
        }
    }
}

} // namespace cyclone_dds
} // namespace cyclone_record_replay
