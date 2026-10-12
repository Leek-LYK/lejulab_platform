// Copyright 2025 Leju Robotics

#include "dds_data_converter.h"
#include "lejusdk-lowlevel/data_types.h"

#include <algorithm>
#include <cmath>

namespace leju {

namespace detail {
    /**
     * @brief 时间戳转换工具函数
     */
    inline double ddsTimestampToDouble(int32_t sec, uint32_t nsec) noexcept {
        return static_cast<double>(sec) + static_cast<double>(nsec) * 1e-9;
    }
    
    inline std::pair<int32_t, uint32_t> doubleToDdsTimestamp(double timestamp_sec) noexcept {
        int32_t sec = static_cast<int32_t>(std::floor(timestamp_sec));
        double fractional = timestamp_sec - static_cast<double>(sec);
        uint32_t nsec = static_cast<uint32_t>(fractional * 1e9 + 0.5);
        
        if (nsec >= 1000000000) {
            nsec = 999999999;
        }
        
        return {sec, nsec};
    }
} // namespace detail

///////////////////////////////////////////////////////////////////////////////
// JointState <-> RobotState 转换实现
///////////////////////////////////////////////////////////////////////////////

// 注意：这里移除了 template<>，直接使用完全限定的函数名
RobotState DdsDataConverter<leju::msgs::JointState, RobotState>::fromDds(
    const leju::msgs::JointState& from) {

    RobotState to;
    to.q.assign(from.q().begin(), from.q().end());
    to.v.assign(from.v().begin(), from.v().end());
    to.vd.assign(from.vd().begin(), from.vd().end());
    to.tau.assign(from.tau().begin(), from.tau().end());
    to.timestamp = detail::ddsTimestampToDouble(from.header_sec(), from.header_nanosec());

    return to;
}

// 注意：这里移除了 template<>
leju::msgs::JointState DdsDataConverter<leju::msgs::JointState, RobotState>::toDds(
    const RobotState& from) {

    leju::msgs::JointState to;
    to.q(from.q);
    to.v(from.v);
    to.vd(from.vd);
    to.tau(from.tau);

    auto [sec, nsec] = detail::doubleToDdsTimestamp(from.timestamp);
    to.header_sec(sec);
    to.header_nanosec(nsec);

    return to;
}

///////////////////////////////////////////////////////////////////////////////
// ImuData 转换实现
///////////////////////////////////////////////////////////////////////////////

// 注意：这里移除了 template<>
ImuData DdsDataConverter<leju::msgs::ImuData, ImuData>::fromDds(
    const leju::msgs::ImuData& from) {

    ImuData to;

    // 使用编译时检查优化数组复制
    if constexpr (std::is_same_v<decltype(from.gyro()), std::array<double, 3>>) {
        to.gyro = from.gyro();
        to.acc = from.acc();
        to.free_acc = from.free_acc();
        to.quat = from.quat();
    } else {
        std::copy(from.gyro().begin(), from.gyro().end(), to.gyro.begin());
        std::copy(from.acc().begin(), from.acc().end(), to.acc.begin());
        std::copy(from.free_acc().begin(), from.free_acc().end(), to.free_acc.begin());
        std::copy(from.quat().begin(), from.quat().end(), to.quat.begin());
    }

    to.timestamp = detail::ddsTimestampToDouble(from.header_sec(), from.header_nanosec());

    return to;
}

// 注意：这里移除了 template<>
leju::msgs::ImuData DdsDataConverter<leju::msgs::ImuData, ImuData>::toDds(
    const ImuData& from) {

    leju::msgs::ImuData to;
    to.gyro(from.gyro);
    to.acc(from.acc);
    to.free_acc(from.free_acc);
    to.quat(from.quat);

    auto [sec, nsec] = detail::doubleToDdsTimestamp(from.timestamp);
    to.header_sec(sec);
    to.header_nanosec(nsec);

    return to;
}

///////////////////////////////////////////////////////////////////////////////
// JointCmd 转换实现
///////////////////////////////////////////////////////////////////////////////

// 注意：这里移除了 template<>
RobotCmd DdsDataConverter<leju::msgs::JointCmd, RobotCmd>::fromDds(
    const leju::msgs::JointCmd& from) {

    RobotCmd to;
    to.q.assign(from.q().begin(), from.q().end());
    to.v.assign(from.v().begin(), from.v().end());
    to.tau.assign(from.tau().begin(), from.tau().end());
    to.kp.assign(from.kp().begin(), from.kp().end());
    to.kd.assign(from.kd().begin(), from.kd().end());
    to.modes.assign(from.modes().begin(), from.modes().end());
    to.timestamp = detail::ddsTimestampToDouble(from.header_sec(), from.header_nanosec());

    return to;
}

// 注意：这里移除了 template<>
leju::msgs::JointCmd DdsDataConverter<leju::msgs::JointCmd, RobotCmd>::toDds(
    const RobotCmd& from) {

    leju::msgs::JointCmd to;
    to.q(from.q);
    to.v(from.v);
    to.tau(from.tau);
    to.kp(from.kp);
    to.kd(from.kd);
    to.modes(from.modes);

    auto [sec, nsec] = detail::doubleToDdsTimestamp(from.timestamp);
    to.header_sec(sec);
    to.header_nanosec(nsec);

    return to;
}

///////////////////////////////////////////////////////////////////////////////
// Joy 转换实现
///////////////////////////////////////////////////////////////////////////////

std::optional<JoyData> DdsDataConverter<leju::msgs::Joy, JoyData>::fromDds(
    const leju::msgs::Joy& from) {

    const auto& ax = from.axes();
    const auto& bt = from.buttons();

    if (ax.size() != 6 || bt.size() < 16) {
        return std::nullopt;
    }

    JoyData to;
    to.axes.left_x = ax[0];
    to.axes.left_y = ax[1];
    to.axes.right_x = ax[2];
    to.axes.right_y = ax[3];
    to.axes.left_trigger = ax[4];
    to.axes.right_trigger = ax[5];

    to.buttons.south = bt[0];
    to.buttons.east = bt[1];
    to.buttons.west = bt[2];
    to.buttons.north = bt[3];
    to.buttons.back = bt[4];
    to.buttons.guide = bt[5];
    to.buttons.start = bt[6];
    to.buttons.left_stick = bt[7];
    to.buttons.right_stick = bt[8];
    to.buttons.left_shoulder = bt[9];
    to.buttons.right_shoulder = bt[10];
    to.buttons.dpad_up = bt[11];
    to.buttons.dpad_down = bt[12];
    to.buttons.dpad_left = bt[13];
    to.buttons.dpad_right = bt[14];
    to.buttons.misc1 = bt[15];
    to.buttons.misc2 = (bt.size() > 16) ? bt[16] : 0;

    to.timestamp = detail::ddsTimestampToDouble(from.header_sec(), from.header_nanosec());

    return to;
}

leju::msgs::Joy DdsDataConverter<leju::msgs::Joy, JoyData>::toDds(
    const JoyData& from) {

    leju::msgs::Joy to;
    to.axes({from.axes.left_x, from.axes.left_y, from.axes.right_x,
             from.axes.right_y, from.axes.left_trigger, from.axes.right_trigger});
    to.buttons({from.buttons.south, from.buttons.east, from.buttons.west,
                from.buttons.north, from.buttons.back, from.buttons.guide,
                from.buttons.start, from.buttons.left_stick, from.buttons.right_stick,
                from.buttons.left_shoulder, from.buttons.right_shoulder,
                from.buttons.dpad_up, from.buttons.dpad_down, from.buttons.dpad_left,
                from.buttons.dpad_right, from.buttons.misc1, from.buttons.misc2});

    auto [sec, nsec] = detail::doubleToDdsTimestamp(from.timestamp);
    to.header_sec(sec);
    to.header_nanosec(nsec);

    return to;
}


///////////////////////////////////////////////////////////////////////////////
// HandCmd 转换实现
///////////////////////////////////////////////////////////////////////////////

HandCmd DdsDataConverter<leju::msgs::Float64Array, HandCmd>::fromDds(
    const leju::msgs::Float64Array& from) {

    HandCmd to;
    to.position.assign(from.data().begin(), from.data().end());
    to.timestamp = detail::ddsTimestampToDouble(from.header_sec(), from.header_nanosec());
    return to;
}

leju::msgs::Float64Array DdsDataConverter<leju::msgs::Float64Array, HandCmd>::toDds(
    const HandCmd& from) {

    leju::msgs::Float64Array to;
    auto [sec, nsec] = detail::doubleToDdsTimestamp(from.timestamp);
    to.header_sec(sec);
    to.header_nanosec(nsec);
    to.data(from.position);
    return to;
}

///////////////////////////////////////////////////////////////////////////////
// HandState 转换实现
///////////////////////////////////////////////////////////////////////////////

HandState DdsDataConverter<leju::msgs::HandState, HandState>::fromDds(
    const leju::msgs::HandState& from) {
  HandState to;
  to.left_valid = from.left_valid();
  to.right_valid = from.right_valid();
  to.left_sample_age_ms = from.left_sample_age_ms();
  to.right_sample_age_ms = from.right_sample_age_ms();
  to.position.assign(from.position().begin(), from.position().end());
  to.velocity.assign(from.velocity().begin(), from.velocity().end());
  to.current.assign(from.current().begin(), from.current().end());
  to.state.assign(from.state().begin(), from.state().end());
  to.timestamp = detail::ddsTimestampToDouble(from.header_sec(), from.header_nanosec());
  return to;
}

leju::msgs::HandState DdsDataConverter<leju::msgs::HandState, HandState>::toDds(
    const HandState& from) {
  leju::msgs::HandState to;
  auto [sec, nsec] = detail::doubleToDdsTimestamp(from.timestamp);
  to.header_sec(sec);
  to.header_nanosec(nsec);
  to.left_valid(from.left_valid);
  to.right_valid(from.right_valid);
  to.left_sample_age_ms(from.left_sample_age_ms);
  to.right_sample_age_ms(from.right_sample_age_ms);
  to.position(from.position);
  to.velocity(from.velocity);
  to.current(from.current);
  to.state(from.state);
  return to;
}

///////////////////////////////////////////////////////////////////////////////
// StringData 转换实现
///////////////////////////////////////////////////////////////////////////////

StringData DdsDataConverter<leju::msgs::StringData, StringData>::fromDds(
    const leju::msgs::StringData& from) {

    StringData to;
    to.timestamp = detail::ddsTimestampToDouble(from.header_sec(), from.header_nanosec());
    to.data = from.data();

    return to;
}

leju::msgs::StringData DdsDataConverter<leju::msgs::StringData, StringData>::toDds(
    const StringData& from) {

    leju::msgs::StringData to;

    auto [sec, nsec] = detail::doubleToDdsTimestamp(from.timestamp);
    to.header_sec(sec);
    to.header_nanosec(nsec);
    to.data(from.data);

    return to;
}

} // namespace leju
