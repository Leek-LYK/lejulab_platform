#include "motor_preflight/param_protocol.h"

#include <cmath>
#include <cstring>

namespace motor_preflight {

namespace {
constexpr uint8_t kFrameHead = 0x67;
constexpr uint8_t kReadTail0 = 0x04;
constexpr uint8_t kWriteTail0 = 0x15;
constexpr uint8_t kTail1 = 0x76;
}  // namespace

std::array<uint8_t, 8> BuildReadFrame(uint8_t idx) {
    return {kFrameHead, idx, 0x00, 0x00, 0x00, 0x00, kReadTail0, kTail1};
}

std::array<uint8_t, 8> BuildWriteFrame(uint8_t idx, const ParamValue& value) {
    std::array<uint8_t, 8> frame = {kFrameHead, idx, 0, 0, 0, 0, kWriteTail0, kTail1};
    if (value.dtype == ParamDtype::kFloat) {
        std::memcpy(frame.data() + 2, &value.f32, 4);
    } else {
        std::memcpy(frame.data() + 2, &value.u32, 4);
    }
    return frame;
}

bool DecodeResponse(const uint8_t* data, size_t len, uint8_t expect_idx, ParamValue* out) {
    if (data == nullptr || out == nullptr || len != 8) {
        return false;
    }
    if (data[0] == kFrameHead || data[1] != expect_idx) {
        return false;
    }
    if (out->dtype == ParamDtype::kFloat) {
        std::memcpy(&out->f32, data + 2, 4);
    } else {
        std::memcpy(&out->u32, data + 2, 4);
    }
    return true;
}

bool ValueMatches(const ParamValue& expected, const ParamValue& actual) {
    if (expected.dtype != actual.dtype) {
        return false;
    }
    if (expected.dtype == ParamDtype::kFloat) {
        return std::fabs(expected.f32 - actual.f32) < kFloatReadbackTol;
    }
    return expected.u32 == actual.u32;
}

}  // namespace motor_preflight
