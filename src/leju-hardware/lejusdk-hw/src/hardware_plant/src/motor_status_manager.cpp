#include "motor_status_manager.h"
#include "actuators_interface.h"  // 为了JointParam_t结构体
#include <algorithm>
#include <iomanip>

namespace HighlyDynamic
{

// 常量定义
static constexpr double EMERGENCY_STOP_ERROR_RATIO = 0.3;
static constexpr int MAX_POSITION_CHECK_JOINTS = 15;

MotorStatusManager::MotorStatusManager(size_t num_joints, int error_threshold, int log_interval_ms)
    : num_joints_(num_joints), error_threshold_(error_threshold), log_interval_ms_(log_interval_ms)
{
    motor_status_list_.reserve(num_joints_);
    for (size_t i = 0; i < num_joints_; ++i) {
        motor_status_list_.emplace_back(static_cast<int>(i + 1));
        joint_id_to_index_[i + 1] = i;
    }
    last_summary_time_ = std::chrono::steady_clock::now();
}

bool MotorStatusManager::updateJointStatus(int joint_id, int status_code, 
                                          double position, double velocity, double torque)
{
    bool result = true;
    
    bool updated = updateMotorStatusSafe(joint_id, [&](MotorStatusInfo& info) {
        info.previous_status = info.current_status;
        info.status_code = status_code;
        
        // 获取硬件状态
        MotorStatus hardware_status = statusCodeToMotorStatus(status_code);
        
        // 状态更新逻辑：
        // 1. ERROR状态一旦设置，不会自动切换回ENABLE，需要手动清除
        // 2. DISABLED状态（手动设置）不被硬件状态覆盖，除非硬件报错
        // 3. 只有从非ERROR状态才能接受硬件状态更新
        
        if (info.current_status == MotorStatus::ERROR) {
            // ERROR状态不会自动切换回ENABLE，保持ERROR状态
            // 只有手动调用setJointStatus才能清除ERROR状态
            // 不更新状态
        } else if (info.current_status == MotorStatus::DISABLED && hardware_status != MotorStatus::ERROR) {
            // 手动设置的DISABLED状态，除非硬件报错，否则保持DISABLED
            // 不更新状态
        } else {
            // 其他情况，正常更新状态
            info.current_status = hardware_status;
        }
        
        info.status_changed = (info.current_status != info.previous_status);
        
        auto now = std::chrono::steady_clock::now();
        
        // 更新错误计数
        if (info.current_status == MotorStatus::ERROR) {
            info.error_count++;
            info.last_error_time = now;
            total_errors_++;
            
            // error_threshold_ 参数现在仅用于统计目的，不触发状态改变
        } else if (info.current_status == MotorStatus::ENABLE) {
            // 正常状态，重置计数器
            info.error_count = 0;
            info.position_error_count = 0;  // 重置位置错误计数
        }
        
        // 设置返回值
        result = (info.current_status != MotorStatus::ERROR);
    });
    
    return updated && result;
}

bool MotorStatusManager::checkAllJointsStatus(const std::vector<JointParam_t> &joint_data,
                                             const std::vector<uint8_t> &joint_ids,
                                             std::string &error_msg)
{
    bool all_normal = true;
    std::ostringstream oss;
    std::vector<std::string> new_errors;
    std::vector<std::string> status_changes;
    
    for (size_t i = 0; i < joint_data.size() && i < joint_ids.size(); ++i) 
    {
        int joint_id = joint_ids[i];
        // 更新关节状态
        bool joint_ok = updateJointStatus(joint_id, joint_data[i].status, joint_data[i].position, joint_data[i].velocity, joint_data[i].torque);
        if (!joint_ok) 
        {
            collectJointStatusInfo(joint_id, new_errors, status_changes);
            continue;  // ← 关键：跳过后续位置/速度检测
        }
        
        // 检查位置限制 - 减少嵌套层级
        if (!hardware_settings_set_ || joint_id > MAX_POSITION_CHECK_JOINTS) 
        {
            // 检查状态变化和错误
            collectJointStatusInfo(joint_id, new_errors, status_changes);
            continue;
        }
        
        int index = joint_id - 1;
        bool valid_index = (index >= 0 && 
                           index < static_cast<int>(hardware_settings_.min_joint_position_limits.size()) && 
                           index < static_cast<int>(hardware_settings_.max_joint_position_limits.size()));
        
        if (!valid_index) 
        {
            // 检查状态变化和错误
            collectJointStatusInfo(joint_id, new_errors, status_changes);
            continue;
        }
        
        // 检查限制
        auto checkLimit = [&](const std::string& check_type, double value, double limit, 
                                     const std::string& limit_name, double threshold = 1.0) {
            // 修改1：区分最小/最大限制检查
            bool is_min_check = (check_type.find("minimum") != std::string::npos);
            bool is_max_check = (check_type.find("maximum") != std::string::npos);
            
            bool limit_exceeded = false;
            if (is_min_check) {
                limit_exceeded = (value < limit * threshold);  // 最小值检查：实际值 < 限制值
            } else if (is_max_check) {
                limit_exceeded = (value > limit * threshold);  // 最大值检查：实际值 > 限制值
            } else {
                // 其他检查（如速度）保持绝对值方式
                limit_exceeded = (std::fabs(value) > limit * threshold);
            }
            
            if (limit_exceeded) {
                std::string error = "Joint " + std::to_string(joint_id) + " " + check_type + 
                                   " limit exceeded: " + std::to_string(value) + 
                                   " (limit: " + std::to_string(limit) + ")";
                new_errors.push_back(error);
                all_normal = false;
                setJointStatus(joint_id, MotorStatus::ERROR, check_type + " limit exceeded");
            }
        };
        double pos = joint_data[i].position;
        double vel = joint_data[i].velocity * (M_PI / 180.0);

        // 检查位置和速度限制
        checkLimit("position minimum", pos, hardware_settings_.min_joint_position_limits[index], "min");
        checkLimit("position maximum", pos, hardware_settings_.max_joint_position_limits[index], "max");
        checkLimit("velocity", vel, hardware_settings_.joint_peak_velocity_limits[index], "velocity", 1.2);
        
        // 检查状态变化和错误
        collectJointStatusInfo(joint_id, new_errors, status_changes);
    }
    
    // 构建错误消息
    buildErrorMessage(new_errors, status_changes, oss);
    
    // 调用所有外部自定义检查函数
    for (const auto& func : custom_check_funcs_) 
    {
        if (!func()) 
        {
            all_normal = false;
            error_msg += (error_msg.empty() ? "" : "\n") + std::string("[Custom Check] Custom check failed");
        }
    }
    error_msg = oss.str() + error_msg;
    return all_normal;
}

void MotorStatusManager::collectJointStatusInfo(int joint_id, 
                                               std::vector<std::string>& new_errors,
                                               std::vector<std::string>& status_changes)
{
    auto it = joint_id_to_index_.find(joint_id);
    if (it == joint_id_to_index_.end()) {
        return;
    }
    
    MotorStatusInfo& info = motor_status_list_[it->second];
    
    // 检查是否需要输出日志
    bool should_log = info.status_changed || shouldLogError(info);
    if (!should_log) {
        return;
    }
    
    if (info.current_status == MotorStatus::ERROR) {
        std::string error_detail = "Joint " + std::to_string(joint_id) + 
                                 " status error! Code: " + std::to_string(info.status_code) +
                                 " (Count: " + std::to_string(info.error_count) + ")";
        new_errors.push_back(error_detail);
        info.last_log_time = std::chrono::steady_clock::now();
    }
    
    if (info.status_changed) {
        std::string change_detail = "Joint " + std::to_string(joint_id) + 
                                  " status changed: " + getStatusName(info.previous_status) +
                                  " -> " + getStatusName(info.current_status);
        status_changes.push_back(change_detail);
        
        // 重置状态变化标志，避免重复打印相同的状态变化
        info.status_changed = false;
    }
}

void MotorStatusManager::buildErrorMessage(const std::vector<std::string>& new_errors,
                                          const std::vector<std::string>& status_changes,
                                          std::ostringstream& oss)
{
    if (!new_errors.empty()) {
        oss << "Motor Status Errors:\n";
        for (const auto& error : new_errors) {
            oss << "  " << error << "\n";
        }
    }
    
    if (!status_changes.empty()) {
        oss << "Status Changes:\n";
        for (const auto& change : status_changes) {
            oss << "  " << change << "\n";
        }
    }
}

void MotorStatusManager::initMotorStatus(const std::vector<JointParam_t> &joint_data,
                                        const std::vector<uint8_t> &joint_ids)
{
    if (joint_data.size() != joint_ids.size()) {
        std::cerr << "[MotorStatusManager] Error: joint_data and joint_ids size mismatch in initMotorStatus!" << std::endl;
        return;
    }
    
    std::cout << "[MotorStatusManager] Initializing motor status..." << std::endl;
    
    int enable_count = 0, error_count = 0, disabled_count = 0, unknown_count = 0;
    
    // 使用批量更新函数，避免重复锁操作
    batchUpdateMotorStatusSafe([&](std::vector<MotorStatusInfo>& motor_list, 
                                  std::unordered_map<int, size_t>& joint_index_map,
                                  int& total_errors) {
        // 首先重置所有状态
        resetAllMotorStatus(motor_list, total_errors);
        
        // 根据实际硬件状态初始化
        for (size_t i = 0; i < joint_data.size(); ++i) {
            int joint_id = joint_ids[i];
            auto it = joint_index_map.find(joint_id);
            
            if (it == joint_index_map.end()) {
                std::cerr << "[MotorStatusManager] Warning: Joint ID " << joint_id 
                         << " not found in joint_id_to_index_ map!" << std::endl;
                continue;
            }
            
            MotorStatusInfo& info = motor_list[it->second];
            info.status_code = joint_data[i].status;
            info.current_status = statusCodeToMotorStatus(joint_data[i].status);
            info.previous_status = info.current_status;
            info.status_changed = false;
            info.error_count = 0;
            info.position_error_count = 0;  // 重置位置错误计数
            info.last_error_msg.clear();
            
            // 统计各状态数量
            countMotorStatus(info.current_status, enable_count, error_count, disabled_count, unknown_count);
        }
    });
    
    std::cout << "[MotorStatusManager] Motor status initialized - "
              << "Enable: " << enable_count << ", "
              << "Error: " << error_count << ", "
              << "Disabled: " << disabled_count << ", "
              << "Unknown: " << unknown_count << std::endl;
}

std::string MotorStatusManager::getStatusSummary(int interval_s) const
{
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_summary_time_);
    
    if (elapsed.count() < interval_s) {
        return "";  // 未到输出间隔
    }
    
    // 更新最后输出时间（需要修改mutable成员）
    last_summary_time_ = now;
    
    auto motor_list = getAllMotorStatusCopy();
    
    std::ostringstream oss;
    int enable_count = 0, error_count = 0, disabled_count = 0, unknown_count = 0;
    
    for (const auto& info : motor_list) {
        countMotorStatus(info.current_status, enable_count, error_count, disabled_count, unknown_count);
    }
    
    oss << "\n[MotorStatusManager] Status Summary:"
        << "\n  Enable: " << enable_count
        << "\n  Error: " << error_count
        << "\n  Disabled: " << disabled_count
        << "\n  Unknown: " << unknown_count
        << "\n  Total Errors: " << total_errors_;
    
    return oss.str();
}

bool MotorStatusManager::shouldTriggerEmergencyStop() const
{
    auto motor_list = getAllMotorStatusCopy();
    
    int error_count = std::count_if(motor_list.begin(), motor_list.end(),
        [](const MotorStatusInfo& info) {
            return info.current_status == MotorStatus::ERROR;
        });
    
    // 如果有超过一定比例的电机出错，触发急停
    return error_count > static_cast<int>(motor_list.size() * EMERGENCY_STOP_ERROR_RATIO);
}

void MotorStatusManager::setJointStatus(int joint_id, MotorStatus target_status, const std::string& reason)
{
    bool updated = updateMotorStatusSafe(joint_id, [&](MotorStatusInfo& info) {
        info.previous_status = info.current_status;
        info.current_status = target_status;
        info.status_changed = (info.current_status != info.previous_status);
        
        if (!reason.empty()) {
            info.last_error_msg = reason;
        }
        
        // 根据状态设置相应的状态码
        updateStatusCodeAndCounters(info, target_status);
        
        // 输出状态变化日志
        if (info.status_changed) {
            logStatusChange(joint_id, info, reason);
        }
    });
    
    if (!updated) {
        std::cerr << "[MotorStatusManager] Warning: Joint ID " << joint_id 
                 << " not found when setting status!" << std::endl;
    }
}

void MotorStatusManager::updateStatusCodeAndCounters(MotorStatusInfo& info, MotorStatus target_status)
{
    switch (target_status) {
        case MotorStatus::ENABLE:
            info.status_code = 0x01;
            info.error_count = 0;  // 重置错误计数
            info.position_error_count = 0;  // 重置位置错误计数
            break;
        case MotorStatus::ERROR:
            info.status_code = 0x02;
            info.error_count++;
            info.last_error_time = std::chrono::steady_clock::now();
            break;
        case MotorStatus::DISABLED:
            info.status_code = 0x03;
            break;
        default:
            info.status_code = 0x00;
            break;
    }
}

void MotorStatusManager::logStatusChange(int joint_id, const MotorStatusInfo& info, const std::string& reason)
{
    std::cout << "[MotorStatusManager] Joint " << joint_id 
              << " status changed: " << getStatusName(info.previous_status)
              << " -> " << getStatusName(info.current_status);
    if (!reason.empty()) {
        std::cout << " (Reason: " << reason << ")";
    }
    std::cout << std::endl;
}

bool MotorStatusManager::clearJointError(int joint_id, const std::string& reason)
{
    bool cleared = false;
    bool updated = updateMotorStatusSafe(joint_id, [&](MotorStatusInfo& info) {
        if (info.current_status == MotorStatus::ERROR) {
            info.previous_status = info.current_status;
            info.current_status = MotorStatus::ENABLE;  // 清除错误，恢复到正常状态
            info.status_changed = true;
            info.error_count = 0;  // 重置错误计数
            info.position_error_count = 0;  // 重置位置错误计数
            
            if (!reason.empty()) {
                info.last_error_msg = reason;
            }
            
            // 输出状态变化日志
            logStatusChange(joint_id, info, reason);
            cleared = true;
        }
    });
    
    if (!updated) {
        std::cerr << "[MotorStatusManager] Warning: Joint ID " << joint_id 
                 << " not found when clearing error!" << std::endl;
        return false;
    }
    
    if (!cleared) {
        std::cout << "[MotorStatusManager] Joint " << joint_id 
                 << " is not in ERROR state, no need to clear. Current status: " 
                 << getStatusName(getJointStatusSafe(joint_id)) << std::endl;
    }
    
    return cleared;
}

std::map<int, MotorStatus> MotorStatusManager::getAllJointsStatus() const
{
    auto motor_list = getAllMotorStatusCopy();
    
    std::map<int, MotorStatus> status_map;
    for (const auto& info : motor_list) {
        status_map[info.joint_id] = info.current_status;
    }
    
    return status_map;
}

std::map<int, MotorStatusInfo> MotorStatusManager::getAllJointsDetailedStatus() const
{
    auto motor_list = getAllMotorStatusCopy();
    
    std::map<int, MotorStatusInfo> detailed_map;
    for (const auto& info : motor_list) {
        detailed_map[info.joint_id] = info;
    }
    
    return detailed_map;
}

void MotorStatusManager::printAllMotorStatus(const std::string& reason) const
{
    auto motor_list = getAllMotorStatusCopy();
    
    // 打印报告头部
    printReportHeader(reason);
    
    // 统计各状态数量
    int enable_count = 0, error_count = 0, disabled_count = 0, unknown_count = 0;
    for (const auto& info : motor_list) {
        countMotorStatus(info.current_status, enable_count, error_count, disabled_count, unknown_count);
    }
    
    // 打印摘要信息
    printStatusSummary(motor_list.size(), enable_count, error_count, disabled_count, unknown_count);
    
    // 打印详细状态表格
    printDetailedStatusTable(motor_list);
    
    // 打印错误和禁用电机的详细信息
    printErrorDetails(motor_list, error_count, disabled_count);
    
    std::cout << std::string(80, '=') << std::endl;
    std::cout << std::flush;
}

void MotorStatusManager::printReportHeader(const std::string& reason) const
{
    std::cout << "\n" << std::string(80, '=') << std::endl;
    std::cout << "                    MOTOR STATUS REPORT" << std::endl;
    std::cout << "Reason: " << reason << std::endl;
    std::cout << "Timestamp: " << std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::system_clock::now().time_since_epoch()).count() << std::endl;
    std::cout << std::string(80, '=') << std::endl;
}

void MotorStatusManager::printStatusSummary(size_t total_motors, int enable_count, int error_count, 
                                           int disabled_count, int unknown_count) const
{
    std::cout << "SUMMARY: Total Motors: " << total_motors
              << " | Enable: " << enable_count 
              << " | Error: " << error_count 
              << " | Disabled: " << disabled_count 
              << " | Unknown: " << unknown_count 
              << " | Total Errors: " << total_errors_ << std::endl;
    std::cout << std::string(80, '-') << std::endl;
}

void MotorStatusManager::printDetailedStatusTable(const std::vector<MotorStatusInfo>& motor_list) const
{
    std::cout << std::left << std::setw(8) << "Joint" 
              << std::setw(10) << "Status" 
              << std::setw(8) << "Code" 
              << std::setw(8) << "Errors" 
              << std::setw(25) << "Last Error Message" << std::endl;
    std::cout << std::string(80, '-') << std::endl;
    
    for (const auto& info : motor_list) {
        std::cout << std::left << std::setw(8) << info.joint_id
                  << std::setw(10) << getStatusName(info.current_status)
                  << std::setw(8) << info.status_code
                  << std::setw(8) << info.error_count
                  << std::setw(25) << (info.last_error_msg.empty() ? "None" : info.last_error_msg.substr(0, 24))
                  << std::endl;
    }
}

void MotorStatusManager::printErrorDetails(const std::vector<MotorStatusInfo>& motor_list, 
                                          int error_count, int disabled_count) const
{
    if (error_count == 0 && disabled_count == 0) {
        return;
    }
    
    std::cout << std::string(80, '-') << std::endl;
    std::cout << "ERROR & DISABLED MOTORS DETAILS:" << std::endl;
    
    for (const auto& info : motor_list) {
        bool is_error_or_disabled = (info.current_status == MotorStatus::ERROR || 
                                   info.current_status == MotorStatus::DISABLED);
        
        if (!is_error_or_disabled) {
            continue;
        }
        
        std::cout << "  Joint " << info.joint_id 
                  << " [" << getStatusName(info.current_status) << "]"
                  << " - Error Count: " << info.error_count;
        
        if (!info.last_error_msg.empty()) {
            std::cout << " - Message: " << info.last_error_msg;
        }
        
        std::cout << std::endl;
    }
}

void MotorStatusManager::setHardwareSettings(const HardwareSettings& settings)
{
    hardware_settings_ = settings;
    hardware_settings_set_ = true;
    
    std::cout << "[MotorStatusManager] Hardware settings configured for " << settings.num_joints 
              << " joints" << std::endl;
}

void MotorStatusManager::registerCustomCheckFunction(std::function<bool()> checker)
{
    if (!checker) 
    {
        return;
    }

    custom_check_funcs_.push_back(std::move(checker));
    std::cout << "[MotorStatusManager] Custom check function registered successfully" << std::endl;
}

} // namespace HighlyDynamic 