#include "config_utils.h"
#include "path_utils.h"
#include "file.h"

#include <iostream>

namespace leju{
namespace hw {     
namespace config_utils  {

std::string GetEcmasterType() {

    std::string  file_path =  leju::hw::path::ecmaster_type_path();
    if(!leju::hw::FileUtils::Exists(file_path)) {
        std::cerr << "\033[33mwarning: " << file_path << " 文件不存在, 未指定EcMasterType, 使用默认值 'elmo' 驱动器类型\033[0m" << std::endl;
        
        // // For Roban robots (version 1X), use "leju" as default
        // if (rb_version.major() > 1 && rb_version.major() < 2) {
        //     std::cerr << "\033[33mRoban robot detected (version " << std::to_string(rb_version.major()) << "), using 'leju' as default driver type\033[0m" << std::endl;
        //     return "youda1";
        // }
        return "elmo";  
    }

    std::string ecMasterType;

    if(!leju::hw::FileUtils::ReadToString(file_path, ecMasterType)) {
        std::cerr << "\033[33mwarning: " << file_path << " 文件读取失败, 未指定EcMasterType, 使用默认值 'elmo' 驱动器类型\033[0m" << std::endl;
        return "elmo";
    }

    if (ecMasterType != "elmo" 
        && ecMasterType != "youda" 
        && ecMasterType != "lunbi1" 
        && ecMasterType != "youda3" 
        && ecMasterType != "youda" 
        && ecMasterType != "leju") {
        std::cerr << "\033[33mwarning: ecmaster_type :" << ecMasterType 
              << " error, 使用默认值 'elmo' 驱动器类型\033[0m" << std::endl;
        return "elmo"; 
    }

    return ecMasterType;
}
std::string GetIMUType() {

    std::string file_path =  leju::hw::path::imu_type_path();

    if(!leju::hw::FileUtils::Exists(file_path)) {
        std::cerr << "\033[33mwarning: " << file_path << " 文件不存在, 未指定imuType, 使用默认值 'xsens' IMU类型\033[0m" << std::endl;
        return "xsens";  
    }

    std::string imuType;
    if(!leju::hw::FileUtils::ReadToString(file_path, imuType)) {
        std::cerr << "\033[33mwarning: " << file_path << " 文件读取失败, 未指定imuType, 使用默认值 'xsens' IMU类型\033[0m" << std::endl;
        return "xsens";
    }

    if (imuType != "xsens" && imuType != "hipnuc" && imuType != "none") {
        std::cerr << "\033[33mwarning: ecmaster_type :'" << imuType 
              << "' error, 使用默认值 'xsens' IMU类型\033[0m" << std::endl;
        return "xsens"; 
    }

    return imuType;
}

CanbusWiringType GetCanbusWiringType() {

    std::string file_path = leju::hw::path::canbus_wiring_type_path();
    if(!leju::hw::FileUtils::Exists(file_path)) {
        std::cerr << "\033[33mwarning: " << file_path << " 文件不存在, 未指定canbus接线方式, 使用默认值 'UNKNOWN'\033[0m" << std::endl;
        return CanbusWiringType::UNKNOWN;
    }

    std::string wiringType;
    if(!leju::hw::FileUtils::ReadToString(file_path, wiringType)) {
        std::cerr << "\033[33mAbort: " << file_path << " 文件读取失败, 未指定canbus接线方式, \
        请补充文件内容为 single_bus或dual_bus: \n \
        signle_bus: 单CAN总线 \n \
        dual_bus: 双CAN总线   \n\033[0m" << std::endl;
        abort();
    }
  
    if (wiringType != "single_bus" && wiringType != "dual_bus") {
        std::cerr << "\033[33mAbort: " << file_path << " 文件指定的canbus接线方式不支持, \
        请修改文件内容为 single_bus 或 dual_bus: \n \
        signle_bus: 单CAN总线 \n \
        dual_bus: 双CAN总线   \n \
        错误文件内容为: " << wiringType << "\n\033[0m" << std::endl;
        abort();
    }

    return wiringType == "single_bus" ? CanbusWiringType::SINGLE_BUS : CanbusWiringType::DUAL_BUS;
}

} // namepsace config_utils
} // namespace hw
} // namespace leju