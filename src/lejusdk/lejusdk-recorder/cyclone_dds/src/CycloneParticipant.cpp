#include "cyclone_dds/CycloneParticipant.hpp"
#include "common/Log.hpp"

#include <dds/ddsc/dds_public_qos.h>
#include <cstring>

namespace cyclone_record_replay {
namespace cyclone_dds {

CycloneParticipant::CycloneParticipant()
    : CycloneParticipant(Config{})
{
}

CycloneParticipant::CycloneParticipant(const Config& config)
    : config_(config)
{
    create_participant();
}

CycloneParticipant::~CycloneParticipant() {
    destroy_participant();
}

CycloneParticipant::CycloneParticipant(CycloneParticipant&& other) noexcept
    : config_(std::move(other.config_))
    , participant_(other.participant_)
{
    other.participant_ = 0;
}

CycloneParticipant& CycloneParticipant::operator=(CycloneParticipant&& other) noexcept {
    if (this != &other) {
        destroy_participant();
        config_ = std::move(other.config_);
        participant_ = other.participant_;
        other.participant_ = 0;
    }
    return *this;
}

dds_guid_t CycloneParticipant::guid() const {
    dds_guid_t guid;
    std::memset(&guid, 0, sizeof(guid));

    if (is_valid()) {
        dds_return_t ret = dds_get_guid(participant_, &guid);
        if (ret != DDS_RETCODE_OK) {
            LOG_ERROR("Failed to get participant GUID: {}", dds_strretcode(ret));
        }
    }
    return guid;
}

void CycloneParticipant::create_participant() {
    // Create QoS for the participant
    dds_qos_t* qos = dds_create_qos();
    if (!qos) {
        LOG_ERROR("Failed to create participant QoS");
        return;
    }

    // Set participant name
    dds_qset_userdata(qos,
        config_.participant_name.c_str(),
        config_.participant_name.size());

    // Create the participant
    participant_ = dds_create_participant(config_.domain_id, qos, nullptr);
    dds_delete_qos(qos);

    if (participant_ < 0) {
        LOG_ERROR("Failed to create DDS participant on domain {}: {}",
            config_.domain_id, dds_strretcode(participant_));
        participant_ = 0;
        return;
    }

    LOG_INFO("Created DDS participant '{}' on domain {}",
        config_.participant_name, config_.domain_id);
}

void CycloneParticipant::destroy_participant() {
    if (participant_ > 0) {
        dds_return_t ret = dds_delete(participant_);
        if (ret != DDS_RETCODE_OK) {
            LOG_WARN("Failed to delete participant: {}", dds_strretcode(ret));
        } else {
            LOG_DEBUG("Deleted DDS participant");
        }
        participant_ = 0;
    }
}

} // namespace cyclone_dds
} // namespace cyclone_record_replay
