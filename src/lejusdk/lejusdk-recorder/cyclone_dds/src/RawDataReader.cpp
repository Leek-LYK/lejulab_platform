#include "cyclone_dds/RawDataReader.hpp"
#include "cyclone_dds/TypeSupport.hpp"
#include "common/Log.hpp"

#include <dds/ddsc/dds_public_qos.h>
#include <dds/ddsi/ddsi_serdata.h>
#include <dds/ddsi/ddsi_sertype.h>
#include <dds/ddsrt/iovec.h>
#include <cstring>

#ifdef DDS_HAS_SHM
#include <dds/ddsi/ddsi_shm_transport.h>
#endif

namespace cyclone_record_replay {
namespace cyclone_dds {

RawDataReader::RawDataReader(CycloneParticipant& participant, const Config& config)
    : participant_(participant)
    , config_(config)
{
    create_reader();
}

RawDataReader::~RawDataReader() {
    destroy_reader();
}

void RawDataReader::create_reader() {
    if (!participant_.is_valid()) {
        LOG_ERROR("Cannot create reader: invalid participant");
        return;
    }

    // Create topic descriptor from type info
    dds_topic_descriptor_t* descriptor = nullptr;
    if (config_.type_info) {
        descriptor = TypeSupport::create_topic_descriptor(
            participant_.handle(),
            config_.type_name,
            config_.type_info);
    }

    if (!descriptor) {
        LOG_ERROR("Failed to create topic descriptor for type '{}'", config_.type_name);
        return;
    }

    // Create topic
    topic_ = dds_create_topic(
        participant_.handle(),
        descriptor,
        config_.topic_name.c_str(),
        nullptr, nullptr);

    if (topic_ < 0) {
        LOG_ERROR("Failed to create topic '{}': {}", config_.topic_name, dds_strretcode(topic_));
        TypeSupport::free_topic_descriptor(descriptor);
        return;
    }

    // Create QoS
    dds_qos_t* qos = dds_create_qos();

    // Set reliability
    if (config_.reliability == common::ReliabilityKind::Reliable) {
        dds_qset_reliability(qos, DDS_RELIABILITY_RELIABLE, DDS_SECS(1));
    } else {
        dds_qset_reliability(qos, DDS_RELIABILITY_BEST_EFFORT, 0);
    }

    // Set durability
    dds_durability_kind_t durability;
    switch (config_.durability) {
        case common::DurabilityKind::Volatile:
            durability = DDS_DURABILITY_VOLATILE;
            break;
        case common::DurabilityKind::TransientLocal:
            durability = DDS_DURABILITY_TRANSIENT_LOCAL;
            break;
        case common::DurabilityKind::Transient:
            durability = DDS_DURABILITY_TRANSIENT;
            break;
        case common::DurabilityKind::Persistent:
            durability = DDS_DURABILITY_PERSISTENT;
            break;
        default:
            durability = DDS_DURABILITY_VOLATILE;
    }
    dds_qset_durability(qos, durability);

    // Set history
    dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, config_.history_depth);

    // Create reader
    reader_ = dds_create_reader(participant_.handle(), topic_, qos, nullptr);
    dds_delete_qos(qos);

    if (reader_ < 0) {
        LOG_ERROR("Failed to create reader for topic '{}': {}",
            config_.topic_name, dds_strretcode(reader_));
        dds_delete(topic_);
        topic_ = 0;
        return;
    }

    LOG_DEBUG("Created raw data reader for topic '{}'", config_.topic_name);
}

void RawDataReader::destroy_reader() {
    if (reader_ > 0) {
        dds_delete(reader_);
        reader_ = 0;
    }
    if (topic_ > 0) {
        dds_delete(topic_);
        topic_ = 0;
    }
}

std::vector<RawCdrSample> RawDataReader::take_raw() {
    std::vector<RawCdrSample> result;

    if (!is_valid()) {
        return result;
    }

    constexpr size_t MAX_SAMPLES = 100;
    dds_sample_info_t infos[MAX_SAMPLES];
    struct ddsi_serdata* serdatas[MAX_SAMPLES];

    // Use dds_takecdr to get raw CDR data
    dds_return_t n = dds_takecdr(
        reader_,
        serdatas,
        MAX_SAMPLES,
        infos,
        DDS_ANY_STATE);

    if (n < 0) {
        if (n != DDS_RETCODE_NO_DATA) {
            LOG_ERROR("dds_takecdr failed for topic '{}': {}",
                config_.topic_name, dds_strretcode(n));
        }
        return result;
    }

    for (int32_t i = 0; i < n; ++i) {
        if (!infos[i].valid_data) {
            ddsi_serdata_unref(serdatas[i]);
            continue;
        }

        RawCdrSample sample;
        sample.timestamp = infos[i].source_timestamp;
        sample.sequence_number = static_cast<uint64_t>(infos[i].publication_handle);

        // Get publication GUID
        dds_builtintopic_endpoint_t* ep = dds_get_matched_publication_data(
            reader_,
            infos[i].publication_handle);

        if (ep) {
            // Extract writer GUID from publication handle
            std::memset(&sample.writer_guid, 0, sizeof(sample.writer_guid));
            dds_builtintopic_free_endpoint(ep);
        }

        // Extract CDR data from serdata
        // ddsi_serdata_to_ser_ref returns data with a 4-byte RTPS SerializedPayload
        // encapsulation header followed by the CDR payload.
        // MCAP message_encoding="cdr" expects RTPS SerializedPayload framing,
        // so we keep the full data including the 4-byte encapsulation header.
        uint32_t ser_size = ddsi_serdata_size(serdatas[i]);
        bool cdr_extracted = false;

        if (ser_size > 4) {
            ddsrt_iovec_t ref;
            ddsi_serdata_to_ser_ref(serdatas[i], 0, ser_size, &ref);
            if (ref.iov_base && ref.iov_len > 4) {
                sample.cdr_data.resize(ref.iov_len);
                std::memcpy(sample.cdr_data.data(), ref.iov_base, ref.iov_len);
                cdr_extracted = true;
            }
            ddsi_serdata_to_ser_unref(serdatas[i], &ref);
        }

#ifdef DDS_HAS_SHM
        // When iceoryx SHM is enabled, serdata created via from_iox_buffer may
        // not have CDR data populated (m_size=0, m_data=nullptr). In that case,
        // read directly from the iox_chunk which contains either serialized CDR
        // or raw data that we can serialize via the sertype ops.
        if (!cdr_extracted && serdatas[i]->iox_chunk != nullptr) {
            iceoryx_header_t* iox_hdr = iceoryx_header_from_chunk(serdatas[i]->iox_chunk);
            if (iox_hdr->shm_data_state == IOX_CHUNK_CONTAINS_SERIALIZED_DATA) {
                if (iox_hdr->data_size > 4) {
                    sample.cdr_data.resize(iox_hdr->data_size);
                    std::memcpy(sample.cdr_data.data(), serdatas[i]->iox_chunk, iox_hdr->data_size);
                    cdr_extracted = true;
                }
            } else if (iox_hdr->shm_data_state == IOX_CHUNK_CONTAINS_RAW_DATA) {
                const struct ddsi_sertype* type = serdatas[i]->type;
                if (type && type->ops->get_serialized_size && type->ops->serialize_into) {
                    size_t sz = ddsi_sertype_get_serialized_size(type, serdatas[i]->iox_chunk);
                    if (sz > 4 && sz != SIZE_MAX) {
                        sample.cdr_data.resize(sz);
                        if (ddsi_sertype_serialize_into(type, serdatas[i]->iox_chunk,
                                                        sample.cdr_data.data(), sz)) {
                            cdr_extracted = true;
                        } else {
                            sample.cdr_data.clear();
                        }
                    }
                }
            }
        }
#endif

        ddsi_serdata_unref(serdatas[i]);

        // Ensure CDR encapsulation header is present.
        // Data received via iceoryx SHM may lack the 4-byte RTPS
        // SerializedPayload encapsulation header that MCAP "cdr"
        // encoding requires.  Detect by checking the first two bytes
        // for a valid CDR/CDR2 encapsulation identifier.
        if (sample.cdr_data.size() >= 4) {
            uint8_t b0 = sample.cdr_data[0];
            uint8_t b1 = sample.cdr_data[1];
            bool valid_header = (b0 == 0x00) &&
                (b1 == 0x00 || b1 == 0x01 ||   // CDR1 BE/LE
                 b1 == 0x06 || b1 == 0x07 ||   // PLAIN_CDR2 BE/LE
                 b1 == 0x08 || b1 == 0x09 ||   // DELIMITED_CDR2 BE/LE
                 b1 == 0x0a || b1 == 0x0b);    // PL_CDR2 BE/LE
            if (!valid_header) {
                // Prepend CDR1 little-endian encapsulation header
                static const uint8_t cdr1_le_header[4] = {0x00, 0x01, 0x00, 0x00};
                sample.cdr_data.insert(
                    sample.cdr_data.begin(),
                    cdr1_le_header, cdr1_le_header + 4);
            }
        }

        if (!sample.cdr_data.empty()) {
            result.push_back(std::move(sample));
        }
    }

    // Invoke callback if enabled
    if (callback_enabled_.load() && !result.empty()) {
        std::lock_guard<std::mutex> lock(callback_mutex_);
        if (data_callback_) {
            for (const auto& sample : result) {
                data_callback_(sample);
            }
        }
    }

    return result;
}

void RawDataReader::set_data_callback(RawDataCallback callback) {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    data_callback_ = std::move(callback);
}

void RawDataReader::enable_callback(bool enable) {
    callback_enabled_.store(enable);
}

void RawDataReader::on_data_available(dds_entity_t reader, void* arg) {
    auto* self = static_cast<RawDataReader*>(arg);
    if (self && self->callback_enabled_.load()) {
        self->take_raw();  // Will invoke callback inside
    }
}

} // namespace cyclone_dds
} // namespace cyclone_record_replay
