#include "common/Types.hpp"

#include <sstream>
#include <iomanip>
#include <regex>
#include <cstring>

namespace cyclone_record_replay {
namespace common {

std::string Guid::to_string() const {
    std::ostringstream oss;
    oss << std::hex << std::setfill('0');
    for (size_t i = 0; i < value.size(); ++i) {
        if (i == 4 || i == 8 || i == 12) {
            oss << '.';
        }
        oss << std::setw(2) << static_cast<int>(value[i]);
    }
    return oss.str();
}

Guid Guid::from_dds_guid(const void* dds_guid) {
    Guid guid;
    if (dds_guid) {
        std::memcpy(guid.value.data(), dds_guid, 16);
    } else {
        std::fill(guid.value.begin(), guid.value.end(), 0);
    }
    return guid;
}

std::size_t GuidHash::operator()(const Guid& guid) const {
    std::size_t hash = 0;
    for (size_t i = 0; i < guid.value.size(); ++i) {
        hash ^= std::hash<uint8_t>{}(guid.value[i]) << (i % 8);
    }
    return hash;
}

Timestamp from_dds_time(int64_t dds_time) {
    return Timestamp(dds_time);
}

int64_t to_dds_time(Timestamp ts) {
    return ts.count();
}

std::string to_string(ReturnCode code) {
    switch (code) {
        case ReturnCode::Ok:              return "Ok";
        case ReturnCode::Error:           return "Error";
        case ReturnCode::Timeout:         return "Timeout";
        case ReturnCode::NoData:          return "NoData";
        case ReturnCode::InvalidArgument: return "InvalidArgument";
        case ReturnCode::NotSupported:    return "NotSupported";
        case ReturnCode::AlreadyExists:   return "AlreadyExists";
        case ReturnCode::NotFound:        return "NotFound";
        case ReturnCode::OutOfResources:  return "OutOfResources";
        default:                          return "Unknown";
    }
}

bool TopicFilter::matches(const std::string& topic_name) const {
    // Check exclude first
    if (!exclude_regex.empty()) {
        std::regex ex(exclude_regex);
        if (std::regex_match(topic_name, ex)) {
            return false;
        }
    }

    // Empty list = record all topics
    if (topics.empty()) {
        return true;
    }

    for (const auto& pattern : topics) {
        if (use_regex) {
            std::regex re(pattern);
            if (std::regex_match(topic_name, re)) {
                return true;
            }
        } else {
            if (topic_name == pattern) {
                return true;
            }
        }
    }

    return false;
}

} // namespace common
} // namespace cyclone_record_replay
