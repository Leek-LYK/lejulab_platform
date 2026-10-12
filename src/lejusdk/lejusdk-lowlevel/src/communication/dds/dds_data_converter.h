#ifndef LEJUSDK_COMMUNICATION_DDS_DATA_CONVERTER_H_
#define LEJUSDK_COMMUNICATION_DDS_DATA_CONVERTER_H_

#include "lejusdk-lowlevel/data_types.h"
#include "lejusdk-dds-idl/JointState.hpp"
#include "lejusdk-dds-idl/ImuData.hpp"
#include "lejusdk-dds-idl/JointCmd.hpp"
#include "lejusdk-dds-idl/Joy.hpp"
#include "lejusdk-dds-idl/StringData.hpp"
#include "lejusdk-dds-idl/Float64Types.hpp"
#include "lejusdk-dds-idl/HandState.hpp"
#include <optional>

namespace leju {

/**
 * @brief DDS数据类型转换器模板类
 *
 * 提供SDK数据类型和DDS消息类型之间的双向转换功能
 */
template<typename DdsType, typename SdkType>
struct DdsDataConverter {
  /**
   * @brief 将DDS消息类型转换为SDK数据类型
   * @param from DDS源数据
   * @return 转换后的SDK数据
   */
  static SdkType fromDds(const DdsType& from);

  /**
   * @brief 将SDK数据类型转换为DDS消息类型
   * @param from SDK源数据
   * @return 转换后的DDS数据
   */
  static DdsType toDds(const SdkType& from);
};

///////////////////////////////////////////////////////////////////////////////
// 模板特化声明
///////////////////////////////////////////////////////////////////////////////

// JointState <-> RobotState 转换
template<>
struct DdsDataConverter<leju::msgs::JointState, RobotState> {
  static RobotState fromDds(const leju::msgs::JointState& from);
  static leju::msgs::JointState toDds(const RobotState& from);
};

// ImuData 转换
template<>
struct DdsDataConverter<leju::msgs::ImuData, ImuData> {
  static ImuData fromDds(const leju::msgs::ImuData& from);
  static leju::msgs::ImuData toDds(const ImuData& from);
};

// JointCmd 转换
template<>
struct DdsDataConverter<leju::msgs::JointCmd, RobotCmd> {
  static RobotCmd fromDds(const leju::msgs::JointCmd& from);
  static leju::msgs::JointCmd toDds(const RobotCmd& from);
};

// HandCmd 转换
template<>
struct DdsDataConverter<leju::msgs::Float64Array, HandCmd> {
  static HandCmd fromDds(const leju::msgs::Float64Array& from);
  static leju::msgs::Float64Array toDds(const HandCmd& from);
};

// HandState 转换
template<>
struct DdsDataConverter<leju::msgs::HandState, HandState> {
  static HandState fromDds(const leju::msgs::HandState& from);
  static leju::msgs::HandState toDds(const HandState& from);
};

// Joy 转换
template<>
struct DdsDataConverter<leju::msgs::Joy, JoyData> {
  static std::optional<JoyData> fromDds(const leju::msgs::Joy& from);
  static leju::msgs::Joy toDds(const JoyData& from);
};

// StringData 转换
template<>
struct DdsDataConverter<leju::msgs::StringData, StringData> {
  static StringData fromDds(const leju::msgs::StringData& from);
  static leju::msgs::StringData toDds(const StringData& from);
};

} // namespace leju

#endif // LEJUSDK_COMMUNICATION_DDS_DATA_CONVERTER_H_
