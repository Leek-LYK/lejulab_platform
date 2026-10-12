#include "hardware_plant.h"
#include <algorithm>
#include <iomanip>
#include <unistd.h>
#include <fstream>
#include <mutex>
#include <limits>
#include <sstream>
#include <deque>
#include <numeric>
#include <set>
#include <csignal>
#include <cassert>

#include "kalman_estimate.h"
#ifdef ENABLE_XSENS
#include "imu_receiver.h"
#endif
#ifdef ENABLE_ECMASTER
#include "EcDemoApp.h"
#else
#include "ec_types_compat.h"
#endif
#include "joint_test_poses.h"
#include "ruiwo_actuator.h"
#include "motorevo/motorevo_actuator.h"
#include "canbus_sdk/config_parser.h"
#include "motor_preflight/motor_preflight.h"

#include "utils/config_utils.h"

namespace HighlyDynamic
{
const int kEcMasterNilId = 254; // Mask for does not exist ecmaster Id
const int kStatusNotInAnyState = -3; // Robot status that does not belong to any state

    bool imu_invert{false};
    uint32_t num_joint, num_arm_joints;
    uint32_t num_head_joints = 2, num_waist_joints = 0;
    Eigen::Vector4d ankle_qv_old;
    Eigen::VectorXd defalutJointPos_;
#ifdef ENABLE_ECMASTER
    static ActuatorsInterface_t actuators;
    static bool ec_master_initialized = false;
#endif
    static RuiwoActuatorBase *ruiwo_actuator = nullptr;
    static AnkleSolver ankleSolver;
    static bool imu_is_recv;
    static std::mutex mtx_imu;

    std::vector<std::unique_ptr<Kalman>> joint_kf;

    // 固定大小的关节卡尔曼滤波状态 (避免 MatrixXd 动态堆分配)
    struct JointKF3 {
        Eigen::Vector3d x = Eigen::Vector3d::Zero();
        Eigen::Matrix3d P = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d Q, R;
    };
    std::vector<JointKF3> joint_kf3;

    double hard_dt = 1e-3;

    static uint8_t _th_running;
    bool hardware_ready_ = false;

    Eigen::Vector3d pre_accel, pre_gyrosco;
    Eigen::Quaterniond pre_quaternion;
    int8_t use_pre_imu_times;

    ImuType imu_type_ = IMU_TYPE_XSENS;

    inline void convert_uint8_to_uint16(const uint8_t *input, uint16_t *output, size_t length)
    {
        for (size_t i = 0; i < length; i++)
        {
            // 将每个 uint8_t 类型的元素转换为 uint16_t 类型
            output[i] = static_cast<uint16_t>(input[i]);
        }
    }

    bool HardwarePlant::disableMotor(int motorIndex, const std::string& reason)
    {
        // 电机disable
        if (motorIndex <= 12 || 13 == motorIndex || 20 == motorIndex)
        {
            // EC
            // TODO 当前ecmaster disable接口有问题，无法立刻让电机失能，采用0力控方式
            std::cout << "ecmaster motor index, current disable use zero CST mode. " << std::endl; 
        }
        else if (motorIndex > 13 && motorIndex < 20)
        {
            // ruiwo
            //TODO 当前这个接口只在c++ sdk起作用
            std::cout << "diable motor " << motorIndex << std::endl;
            auto ruiwoIndex = motorIndex - 14;
            auto isDisableFlag = ruiwo_actuator->disableMotor(ruiwoIndex);
            if (!isDisableFlag)
            {
                std::cerr << "diable motor " << motorIndex << " failed!" << std::endl;
                return false;
            }
        }
        else if (motorIndex > 20 && motorIndex < 28)
        {
            // ruiwo
            std::cout << "diable motor " << motorIndex << std::endl;
            auto ruiwoIndex = motorIndex - 15;
            auto isDisableFlag = ruiwo_actuator->disableMotor(ruiwoIndex);
            if (!isDisableFlag)
            {
                std::cerr << "diable motor " << motorIndex << "failed!" << std::endl;
                return false;
            }
        }
        else
        {
            std::cerr << "invalue motor index: " << motorIndex << std::endl;
            return false;
        }

        // 统一在这里做电机状态管理
        markJointAsDisabled(motorIndex, reason);

        return true;
    }

    void HardwarePlant::setEcmasterDriverType(std::string type)
    {
        if (type != "elmo" && type != "youda" && type != "lunbi" && type != "youda1" && type != "leju" && type != "youda3")
        {
            std::cerr << "Unsupported ecmaster driver type: " << type << std::endl;
            throw std::invalid_argument("Unsupported ecmaster driver type: " + type);
        }
        std::cout << "set ecmaster driver type to: " << type << std::endl;
        size_t driver_type_length = sizeof(driver_type) / sizeof(driver_type[0]);
        for (size_t i = 0; i < driver_type_length; i++)
        {
            if (type == "elmo")
            {
                driver_type[i] = EcMasterType::ELMO;
            }
            else if (type == "lunbi")
            {
                if(i < 4)
                {
                    driver_type[i] = EcMasterType::ELMO;
                }
                else
                {
                    driver_type[i] = EcMasterType::YD;
                }
            }
            
            else if (type == "youda3" || type == "youda" || type == "youda1")
            {
                driver_type[i] = EcMasterType::YD;
            }
            else if (type == "leju")
            {
                driver_type[i] = EcMasterType::LEJU;
            }
        }
    }

    // 预分配的 RUIWO 数据缓冲区（避免每次 GetMotorData 都 new/delete）
    static std::vector<double> rw_pos_buf_, rw_vel_buf_, rw_torque_buf_;
    static std::vector<uint8_t> rw_status_buf_;

    inline void HardwarePlant::GetMotorData(const std::vector<uint8_t> &joint_ids, std::vector<MotorParam_t> &motor_data)
    {
        // --- 1. 批量获取 RUIWO 数据（单次调用，无临时 map/vector 分配） ---
        bool has_ruiwo = false;
        if (ruiwo_actuator) {
            ruiwo_actuator->get_all_data(rw_pos_buf_, rw_vel_buf_, rw_torque_buf_);
            ruiwo_actuator->get_all_status(rw_status_buf_);
            has_ruiwo = !rw_pos_buf_.empty();
        }

#ifdef ENABLE_ECMASTER
        // --- 2. 批量获取 EC-Master 数据 ---
        // 第一遍：分类 EC 关节 ID（仍需临时容器来匹配 actuators.getJointData 接口）
        std::vector<uint8_t> ecMasterIds;
        std::vector<uint8_t> ecExistIds;
        for (size_t i = 0; i < joint_ids.size(); ++i)
        {
            size_t index = joint_ids[i] - 1;
            if (motor_info.driver[index] == EC_MASTER)
            {
                ecMasterIds.push_back(ec_index_map_[index]);
                if (motor_info.motors_exist[i]) {
                    ecExistIds.push_back(ec_index_map_[index]);
                }
            }
        }

        std::vector<JointParam_t> EC_joint_data;
        if (ecMasterIds.size())
        {
            EC_joint_data.resize(ecMasterIds.size());
            if (ec_master_initialized) {
                std::vector<JointParam_t> exsit_ec_joint_data(ecExistIds.size());
                std::vector<uint16_t> ecMasterIds16(ecExistIds.size());
                convert_uint8_to_uint16(ecExistIds.data(), ecMasterIds16.data(), ecExistIds.size());
                actuators.getJointData(ecMasterIds16.data(), driver_type, ecExistIds.size(), exsit_ec_joint_data.data());

                int ec_exist_index = 0;

                for (size_t i = 0; i < EC_joint_data.size(); i++)
                {
                    if (ecMasterIds[i] != kEcMasterNilId)
                    {
                        EC_joint_data[i] = exsit_ec_joint_data.at(ec_exist_index++);
                        /* only-half-up-body mode: always keep stance position. */
                        if (stance_leg_joint_pos_ && i < stance_leg_joint_pos_->size() && motor_info.motors_disable[i]) {
                            EC_joint_data[i].position = stance_leg_joint_pos_->at(i);
                        }
                    }
                    else {
                        // !!! Motor does not exist, set all values default.
                        EC_joint_data[i] = JointParam_t();
                        /* only-half-up-body mode: always keep stance position. */
                        if(stance_leg_joint_pos_ && i < stance_leg_joint_pos_->size()) {
                            EC_joint_data[i].position = stance_leg_joint_pos_->at(i);
                        }
                    }
                }
            } else {
                // EC-Master not available, fill with default/stance data
                for (size_t i = 0; i < EC_joint_data.size(); i++) {
                    EC_joint_data[i] = JointParam_t();
                    if (stance_leg_joint_pos_ && i < stance_leg_joint_pos_->size()) {
                        EC_joint_data[i].position = stance_leg_joint_pos_->at(i);
                    }
                }
            }
        }
#endif // ENABLE_ECMASTER

        // --- 3. 单次循环：分发到 motor_data ---
#ifdef ENABLE_ECMASTER
        uint8_t num_ec_data = 0;
#endif
        for (size_t i = 0; i < joint_ids.size(); ++i)
        {
            size_t index = joint_ids[i] - 1;
#ifdef ENABLE_ECMASTER
            if (motor_info.driver[index] == EC_MASTER)
            {
                motor_data.at(i) = EC_joint_data.at(num_ec_data);
                num_ec_data++;
            }
            else
#endif
            if (motor_info.driver[index] == RUIWO)
            {
                // 使用 name-based 映射获取正确的 motor_refs_ 索引
                uint8_t motor_idx = jointId2RuiwoId_.motor_index(joint_ids[i]);
                if (has_ruiwo && motor_idx < rw_pos_buf_.size())
                {
                    motor_data.at(i).position = (rw_pos_buf_[motor_idx] * 180.0) / M_PI;
                    motor_data.at(i).velocity = (rw_vel_buf_[motor_idx] * 180.0) / M_PI;
                    motor_data.at(i).torque   = rw_torque_buf_[motor_idx];
                    motor_data.at(i).status   = (motor_idx < rw_status_buf_.size())
                                                ? rw_status_buf_[motor_idx] : 0;
                }
            }
        }

        // 电机离线检测: 仅统计腿部电机(Ruiwo索引1~na_foot_)离线比例, 超过30%触发退出
        // 手臂/头部在控制器切换时kp/kd可能短暂为0, 不应算作离线
        {
            static int run_cycles = 0;
            run_cycles++;
            if (has_ruiwo && !rw_status_buf_.empty() && run_cycles > 1000)
            {
                int rw_offline = 0;
                for (int i = 1; i < (int)rw_status_buf_.size() && i <= na_foot_; i++) {
                    if (rw_status_buf_[i]) rw_offline++;
                }
                if (rw_offline > 0 && rw_offline >= na_foot_ * 3 / 10)
                {
                    static bool triggered = false;
                    if (!triggered) {
                        triggered = true;
                        std::cerr << "\n[HardwarePlant] Motor offline ("
                                  << (na_foot_ - rw_offline) << "/" << na_foot_
                                  << " legs online), shutting down...\n" << std::endl;
                        std::raise(SIGINT);
                    }
                }
            }
        }
    }

#ifdef ENABLE_ECMASTER
    // 辅助函数：为 EC_MASTER 电机设置默认的 kp/kd（用于 CSP 模式）
    inline void HardwarePlant::setDefaultKpKdForEcMaster(std::vector<MotorParam_t> &motor_data, const std::vector<uint8_t> &joint_ids)
    {
        const auto &default_joint_kp = kuavo_settings_.running_settings.joint_kp;
        const auto &default_joint_kd = kuavo_settings_.running_settings.joint_kd;

        for (size_t i = 0; i < joint_ids.size() && i < motor_data.size(); ++i)
        {
            size_t index = joint_ids[i] - 1;

            // 只对 EC_MASTER 电机设置默认的 kp/kd
            if (index < motor_info.driver.size() &&
                motor_info.driver[index] == EC_MASTER &&
                motor_info.motors_exist[index])
            {
                // 找到对应的 EC_MASTER 索引（在所有 EC_MASTER 电机中的索引）
                int ec_master_index = -1;
                int ec_count = 0;
                for (int j = 0; j <= static_cast<int>(index); ++j)
                {
                    if (j < static_cast<int>(motor_info.driver.size()) &&
                        motor_info.driver[j] == EC_MASTER &&
                        motor_info.motors_exist[j])
                    {
                        if (j == static_cast<int>(index))
                        {
                            ec_master_index = ec_count;
                            break;
                        }
                        ec_count++;
                    }
                }

                // 如果找到了对应的索引，且默认值数组大小足够，则使用默认值
                if (ec_master_index >= 0 &&
                    ec_master_index < static_cast<int>(default_joint_kp.size()) &&
                    ec_master_index < static_cast<int>(default_joint_kd.size()))
                {
                    motor_data[i].kp = static_cast<double>(default_joint_kp[ec_master_index]);
                    motor_data[i].kd = static_cast<double>(default_joint_kd[ec_master_index]);
                }
            }
        }
    }
#endif // ENABLE_ECMASTER

    inline void HardwarePlant::SetMotorPosition(const std::vector<uint8_t> &joint_ids, std::vector<MotorParam_t> &motor_data)
    {
        // Classify IDs based on their types
#ifdef ENABLE_ECMASTER
        std::vector<uint8_t> ecMasterIds;
        std::vector<JointParam_t> EC_joint_data;
#endif
        std::vector<uint8_t> ruiwoIdxs;
        RuiWoJointData rw_joint_data;
        // Add more vectors for other types if needed...
        for (size_t i = 0; i < joint_ids.size(); ++i)
        {
            size_t index = joint_ids[i] - 1;
            // Check the driver type and add to the appropriate vector
#ifdef ENABLE_ECMASTER
            if (motor_info.driver[index] == EC_MASTER)
            {
                // Only add the motor that exist
                if (motor_info.motors_exist[index]) {
                    EC_joint_data.push_back(motor_data.at(i));
                    ecMasterIds.push_back(ec_index_map_[index]);
                }
            }
            else
#endif
            if (motor_info.driver[index] == RUIWO)
            {
                auto ruiwo_idx = jointId2RuiwoId_.motor_index(joint_ids[i]);
                if (ruiwo_idx >= rw_pos_buf_.size()) continue;  // 跳过越界索引
                ruiwoIdxs.push_back(ruiwo_idx);
                rw_joint_data.pos.push_back(motor_data.at(i).position);
                rw_joint_data.vel.push_back(motor_data.at(i).velocityOffset);
                rw_joint_data.torque.push_back(motor_data.at(i).torqueOffset);
                rw_joint_data.kp.push_back(motor_data.at(i).kp);
                rw_joint_data.kd.push_back(motor_data.at(i).kd);
            }
        }
#ifdef ENABLE_ECMASTER
        if (ecMasterIds.size() && ec_master_initialized)
        {
            std::vector<uint16_t> ecMasterIds16(ecMasterIds.size());
            convert_uint8_to_uint16(ecMasterIds.data(), ecMasterIds16.data(), ecMasterIds.size());
            actuators.setJointPosition(ecMasterIds16.data(), driver_type, ecMasterIds.size(), EC_joint_data.data());
        }
#endif
        if (ruiwoIdxs.size())
        {
            // 透传上层 kp/kd 到 RUIWO 电机 (仅在值变化时生效, set_joint_gains 内部做去重)
            std::vector<int> ruiwo_indices(ruiwoIdxs.begin(), ruiwoIdxs.end());
            ruiwo_actuator->set_joint_gains(ruiwo_indices, rw_joint_data.kp, rw_joint_data.kd);
            // 接口传入的是角度, c++sdk中转为弧度
            ruiwo_actuator->set_positions(ruiwoIdxs, rw_joint_data.pos, rw_joint_data.torque, rw_joint_data.vel);
        }
    }

    inline void HardwarePlant::SetMotorTorque(const std::vector<uint8_t> &joint_ids, std::vector<MotorParam_t> &motor_data)
    {
#ifdef ENABLE_ECMASTER
        // Classify IDs based on their types
        std::vector<uint8_t> ecMasterIds;
        std::vector<JointParam_t> EC_joint_data;
        // Add more vectors for other types if needed...

        for (size_t i = 0; i < joint_ids.size(); ++i)
        {
            size_t index = joint_ids[i] - 1;
            // Check the driver type and add to the appropriate vector
            if (motor_info.driver[index] == EC_MASTER)
            {
                // Only add the motor that exist
                if (motor_info.motors_exist[index]) {
                    ecMasterIds.push_back(ec_index_map_[index]);
                    EC_joint_data.push_back(motor_data.at(i));
                }
            }
        }
        if (ecMasterIds.size() && ec_master_initialized)
        {
            std::vector<uint16_t> ecMasterIds16(ecMasterIds.size());
            convert_uint8_to_uint16(ecMasterIds.data(), ecMasterIds16.data(), ecMasterIds.size());
            actuators.setJointTorque(ecMasterIds16.data(), driver_type, ecMasterIds.size(), EC_joint_data.data());
        }
#endif // ENABLE_ECMASTER
    }

    inline void HardwarePlant::SetMotorVelocity(const std::vector<uint8_t> &joint_ids, std::vector<MotorParam_t> &motor_data)
    {
#ifdef ENABLE_ECMASTER
        // Classify IDs based on their types
        std::vector<uint8_t> ecMasterIds;
        std::vector<JointParam_t> EC_joint_data;
        // Add more vectors for other types if needed...

        for (size_t i = 0; i < joint_ids.size(); ++i)
        {
            size_t index = joint_ids[i] - 1;
            // Check the driver type and add to the appropriate vector
            if (motor_info.driver[index] == EC_MASTER)
            {
                // Only add the motor that exist
                if (motor_info.motors_exist[index]) {
                    ecMasterIds.push_back(ec_index_map_[index]);
                    EC_joint_data.push_back(motor_data.at(i));
                }
            }
        }
        if (ecMasterIds.size() && ec_master_initialized)
        {
            std::vector<uint16_t> ecMasterIds16(ecMasterIds.size());
            convert_uint8_to_uint16(ecMasterIds.data(), ecMasterIds16.data(), ecMasterIds.size());
            actuators.setJointVelocity(ecMasterIds16.data(), driver_type, ecMasterIds.size(), EC_joint_data.data());
        }
#endif // ENABLE_ECMASTER
    }

    static double calcCos(double start, double stop, double T, double t)
    {
        double A = (stop - start) / 2.0;
        return A * -cos(M_PI / T * t) + start + A;
    }

    Eigen::Vector3d removeGravity(const Eigen::Vector3d &rawAccel, const Eigen::Quaterniond &orientation)
    {
        // 设置重力向量在全局坐标系中的方向，假设重力向量的方向为 (0, 0, -9.81)
        Eigen::Vector3d gravity(0.0, 0.0, 9.785); // TODO: 安装方向

        // 将重力向量转换到局部坐标系中
        Eigen::Vector3d localGravity = orientation.conjugate() * gravity;

        // 计算去除重力影响的加速度
        Eigen::Vector3d accelNoGravity = rawAccel - localGravity;

        return accelNoGravity;
    }

    void HardwarePlant::jointMoveTo(std::vector<double> goal_pos, double speed, double dt, double current_limit)
    {
        if (goal_pos.size() == num_joint - num_head_joints)// 适配旧的服务接口, 补全头部自由度
        {
            std::cout << "[HardwarePlant::jointMoveTo] Warning: The goal position size is " << goal_pos.size() << ", number of joints is " << num_joint << std::endl;
            for (int i = 0; i < num_head_joints; i++)
                goal_pos.push_back(0);
        }

        Eigen::VectorXd cmd_s(goal_pos.size());
        Eigen::VectorXd motor_position(na_foot_);
        for (size_t i = 0; i < goal_pos.size(); i++)
        {
            cmd_s(i) = goal_pos[i] * TO_RADIAN;
        }
        motor_position = ankleSolver.joint_to_motor_position(cmd_s.head(na_foot_));
        for (size_t  i = 0; i < na_foot_; i++)
        {
            goal_pos[i] = motor_position[i] * TO_DEGREE;
        }
       
        GetMotorData(joint_ids, joint_data);
        usleep(10000);
        std::vector<double> start_pos(num_joint, 0);
        std::vector<double> T(num_joint, 0);
        for (uint32_t i = 0; i < num_joint; i++)
        {
            start_pos[i] = joint_data[i].position;
            T[i] = fabs(goal_pos[i] - start_pos[i]) / speed;
            printf("Joint %d from %f to %f\n", i + 1, start_pos[i], goal_pos[i]);
        }
        double max_T = *max_element(T.begin(), T.end());
        if (max_T < 0.5)
        {
            max_T = 0.5;
        }
        printf("Duration %f\n", max_T);
        uint32_t total_cnt = ceil(max_T / dt);
        uint32_t count = 0;
        struct timespec next_time;
        clock_gettime(CLOCK_MONOTONIC, &next_time);
        while (1)
        {
            int disable_id = getDisableMotorId();
            if(-1 != disable_id)
            {
                disableMotor(disable_id, "jointMoveTo");
            }

            for (uint32_t i = 0; i < num_joint; i++)
            {
                joint_cmd[i].position = calcCos(start_pos[i], goal_pos[i], total_cnt, count);
                if (current_limit > 0)
                {
                    joint_cmd[i].maxTorque = current_limit;
                }
                else
                {
                    joint_cmd[i].maxTorque = motor_info.max_current[i];
                }
            }

            // 检查腿部电机是否被disable
            bool leg_disabled = false;
            for (const auto& jointStatus : getAllJointsStatus()) 
            {
                int joint_id = jointStatus.first;
                auto status = jointStatus.second;
                if (joint_id >= 1 && joint_id <= 12) 
                {
                    if (status == MotorStatus::DISABLED || status == MotorStatus::ERROR) 
                    {
                        leg_disabled = true;
                        std::cout << "joint_id: " << joint_id << ", status: " << (int)status << std::endl;
                        break;
                    }
                }
            }

            if (!leg_disabled) 
            {
                // 在 CSP 模式下，为 EC_MASTER 电机设置默认的 kp/kd
#ifdef ENABLE_ECMASTER
                setDefaultKpKdForEcMaster(joint_cmd, joint_ids);
#endif
                SetMotorPosition(joint_ids, joint_cmd);
            } 
            else 
            {
                //TODO 因为ec sdk有问题，掉使能并不是每次起作用
                // std::vector<uint8_t> leg_joint_ids;
                // std::vector<JointParam_t> leg_joint_cmd;
                // for (int i = 0; i < num_joint; ++i) 
                // {
                //     int joint_id = joint_ids[i];
                //     if (joint_id >= 1 && joint_id <= 12) 
                //     {
                //         joint_cmd[i].torque = 0;
                //         leg_joint_ids.push_back(joint_id);
                //         leg_joint_cmd.push_back(joint_cmd[i]);
                //     }
                // }
                // SetMotorTorque(leg_joint_ids, leg_joint_cmd);
                std::cerr << "jointMoveTo trigger joint protect." << std::endl;
                std::raise(SIGINT);
            }

            count++;
            if (count > total_cnt)
            {
                OsSleep(4);
                GetMotorData(joint_ids, joint_data);
                break;
            }

            next_time.tv_sec += (next_time.tv_nsec + dt * 1e9) / 1e9;
            next_time.tv_nsec = (int)(next_time.tv_nsec + dt * 1e9) % (int)1e9;
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);
        }
    }

    bool HardwarePlant::checkJointSafety(const std::vector<JointParam_t> &joint_data, std::vector<uint8_t> ids, std::string &msg)
    {
        if (hardware_status_ < 0)
        {
            return true;
        }

        if(!motor_status_manager_)
        {
            return true;
        }

        std::string err_msg = "";
        // 使用电机状态管理器进行全面的关节安全检查（包括状态和位置限制）
        bool result = motor_status_manager_->checkAllJointsStatus(joint_data, ids, err_msg);
        msg = err_msg;
        
        // 检查是否需要触发急停
        if (motor_status_manager_->shouldTriggerEmergencyStop()) 
        {
            std::cerr << "[HardwarePlant] Emergency stop triggered due to critical motor errors!" << std::endl;
            std::raise(SIGINT);
        }
        
        // 每10秒输出一次状态摘要
        std::string summary = motor_status_manager_->getStatusSummary(30);
        if (!summary.empty()) 
        {
            std::cout << summary << std::endl;
        }
        
        return result;
    }

    bool HardwarePlant::checkLowerLimbMotorFault(std::string &fault_info, bool ready_ok)
    {
        if (!ruiwo_actuator)
        {
            return false;
        }
        return ruiwo_actuator->check_lowerlimb_fault(fault_info, ready_ok);
    }

    bool HardwarePlant::checkJointPos(JointParam_t *joint_data, std::vector<uint8_t> ids, std::string *msg)
    {
        if (hardware_status_ == -2)
            return true;
        bool result = true;
        std::string err_msg = "";
        for (uint32_t i = 0; i < ids.size(); i++)
        {
            if(ids[i] > 14) continue;
            uint8_t index = ids[i] - 1;
            if (joint_data[i].position < min_joint_position_limits[index] ? true : false)
            {
                err_msg += "Error: Joint " + std::to_string(index + 1) + " position minimum value exceeded, actual value " + std::to_string(joint_data[i].position) + "\n";
                result = false;
            }
            if (joint_data[i].position > max_joint_position_limits[index] ? true : false)
            {
                err_msg += "Error: Joint " + std::to_string(index + 1) + " position maxmum value exceeded, actual value " + std::to_string(joint_data[i].position) + "\n";
                result = false;
            }
            if (std::abs(joint_data[i].velocity) > joint_velocity_limits_[index])
            {
                err_msg += "Error: Joint " + std::to_string(index + 1) + " velocity value exceeded, actual value " + std::to_string(joint_data[i].velocity) + "\n";
                result = false;
            }
            // std::cout << joint_data[i].status << std::endl;
            if (joint_data[i].status != 0)
            {
                err_msg += "Error: Joint " + std::to_string(index + 1) + " joint status error!" + std::to_string(joint_data[i].status) + " \n";
                result = false;
            }
        }
        *msg = err_msg;
        // if (!result)
        //     std::raise(SIGINT);
        // if (!result && KuavoErrorManager.CurrentRobotState() == ROBOT_OK)
        //     KuavoErrorManager.trigger(ERROR_MAJOR, err_msg);

        return result;
    }

    void HardwarePlant::jointFiltering(std::vector<JointParam_t> &joint_data, double dt)
    {
        // 使用固定大小 Matrix3d/Vector3d，避免 MatrixXd 的堆分配开销
        // H = I (单位阵)，简化: H*P*H' = P, H'=I, H*x=x
        const Eigen::Matrix3d F = (Eigen::Matrix3d() <<
            1.0, dt, dt * dt / 2.0,
            0.0, 1.0, dt,
            0.0, 0.0, 1.0).finished();
        const Eigen::Matrix3d Ft = F.transpose();

        for (uint32_t i = 0; i < num_joint; i++)
        {
            auto& kf = joint_kf3[i];
            const Eigen::Vector3d z(joint_data[i].position, joint_data[i].velocity, 0);

            // Predict
            kf.x = F * kf.x;
            kf.P.noalias() = F * kf.P * Ft + kf.Q;

            // Update (H=I 简化)
            const Eigen::Matrix3d S = kf.P + kf.R;           // S = P + R
            const Eigen::Matrix3d K = kf.P * S.inverse();    // K = P * S^-1
            kf.x += K * (z - kf.x);                          // x += K * (z - x)
            kf.P = (Eigen::Matrix3d::Identity() - K) * kf.P; // P = (I - K) * P

            joint_data[i].position = kf.x(0);
            joint_data[i].velocity = kf.x(1);
            joint_data[i].acceleration = kf.x(2);
        }
    }

    void HardwarePlant::getRawMotorData(std::vector<double>& pos_rad, std::vector<double>& vel_rad)
    {
        std::vector<JointParam_t> motor_data(num_joint);
        GetMotorData(joint_ids, motor_data);
        pos_rad.resize(num_joint);
        vel_rad.resize(num_joint);
        for (uint32_t i = 0; i < num_joint; i++) {
            pos_rad[i] = motor_data[i].position * M_PI / 180.0;
            vel_rad[i] = motor_data[i].velocity * M_PI / 180.0;
        }
    }

    static bool readImu(bool flag_half_up_body, Eigen::Vector3d &gyro, Eigen::Vector3d &acc, Eigen::Vector4d &quat, Eigen::Vector3d &free_acc)
    {
        Eigen::Vector3d accel, gyrosco;
        Eigen::Quaterniond quaternion;

        struct timespec t0_sol, t1_sol;
        clock_gettime(CLOCK_MONOTONIC, &t0_sol);
        bool readflag = false;
        if (imu_type_ == ImuType::IMU_TYPE_NONE || flag_half_up_body) {
            accel<<0,0,0;
            gyrosco<<0,0,0;
            quaternion = Eigen::Quaterniond(1,0,0,0);
            readflag = true;
        }
        else if (imu_type_ == ImuType::IMU_TYPE_HIPNUC){
                readflag = HIPNUC_IMU::getImuDataFrame(accel, gyrosco, quaternion);
        }
        else {
                readflag = xsens_IMU::getImuDataFrame(accel, gyrosco, quaternion);
        }
        if (!readflag)
        {

            std::cout << "getImuData failed! using old data " << static_cast<int>(use_pre_imu_times) << std::endl;
            use_pre_imu_times++;
            if (use_pre_imu_times > 30)
            {
                // KuavoErrorManager.trigger(ERROR_CRITICAL, "readIMU failed");
                return false;
            }
            accel = pre_accel;
            gyrosco = pre_gyrosco;
            quaternion = pre_quaternion;
        }
        else
        {
            use_pre_imu_times = 0;
            pre_accel = accel;
            pre_gyrosco = gyrosco;
            pre_quaternion = quaternion;
        }
        clock_gettime(CLOCK_MONOTONIC, &t1_sol);
        if (TIME_DIFF(t0_sol, t1_sol) * 1000 > 0.1)
        {
            printf("getImuDataFrame timeout err: %f\n",
                   TIME_DIFF(t0_sol, t1_sol) * 1000);
        }

        static int readpacket_count;

        static struct timespec t_ini, t_end;
        if (readpacket_count == 0)
        {
            clock_gettime(CLOCK_MONOTONIC, &t_ini);
        }
        readpacket_count++;
        clock_gettime(CLOCK_MONOTONIC, &t_end);
        // printf("readpacket_count: %d, %f\n\n", readpacket_count, TIME_DIFF(t_ini, t_end) * 1000);

        acc = accel;
        gyro = gyrosco;
        quat = Eigen::Vector4d(quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z());
        free_acc = removeGravity(acc, quaternion);
        return true;
    }

    /**
     * @brief 获取 imu（角速度， 加速度， 四元数） 和 关节位置，关节速度，关节加速度，关节力矩 数值
     *
     * @param qvt
     * @param sensor_data
     * @param free_acc
     * @warning 因为加速度是根据固定的 dt 除 速度差，一个周期只能调用一次
     */
    bool HardwarePlant::readSensor(SensorData_t &sensor_data)
    {
        using Clock = std::chrono::steady_clock;
        auto ts0 = Clock::now();

        bool result = true;

        uint32_t na_r = num_joint;
        std::vector<JointParam_t> filter_data(na_r);
        for (uint32_t i = 0; i < na_r; i++)
        {
            joint_data_old[i] = joint_data[i];
        }
        auto ts1 = Clock::now();
        GetMotorData(joint_ids, joint_data);
        auto ts2 = Clock::now();
        for (uint32_t i = 0; i < na_r; i++)
        {
            joint_data[i].acceleration = (joint_data[i].velocity - joint_data_old[i].velocity) / hard_dt;
            filter_data[i] = joint_data[i];
        }
        Eigen::VectorXd tmp_torque, tmp_state;
        tmp_torque.setZero(num_joint);
        tmp_state.setZero(num_joint);
        for (int i = 0; i < num_joint; i++)
        {
            tmp_torque(i) = joint_data[i].torque_demand_trans;
            tmp_state(i) = joint_data[i].status;
        }

        std::string err_msg;
        result = result && checkJointSafety(filter_data, joint_ids, err_msg);
        if (!result)
        {
            std::cerr << err_msg << std::endl;
            // HWPlantDeInit();
            std::raise(SIGINT);
            return false;
        }
        auto ts3 = Clock::now();

        // jointPublish(lcm_ptr,"raw/joint", filter_data,na_r);

        jointFiltering(filter_data, hard_dt);
        auto ts4 = Clock::now();
        // jointPublish(lcm_ptr, "sensors/joint", filter_data, na_r);

        // Eigen::VectorXd q_r(na_r);
        // Eigen::VectorXd v_r(na_r);
        // Eigen::VectorXd vd_r(na_r);
        // Eigen::VectorXd tau_r(na_r);
        sensor_data.joint_q.resize(na_r);
        sensor_data.joint_v.resize(na_r);
        sensor_data.joint_vd.resize(na_r);
        sensor_data.joint_current.resize(na_r);
        sensor_data.joint_torque_demand.resize(na_r);
        for (uint32_t i = 0; i < na_r; i++)
        {
            sensor_data.joint_q[i] = filter_data[i].position * (M_PI / 180.0);
            sensor_data.joint_v[i] = filter_data[i].velocity * (M_PI / 180.0);
            sensor_data.joint_vd[i] = filter_data[i].acceleration * (M_PI / 180.0);
            sensor_data.joint_current[i] = filter_data[i].torque;
            sensor_data.joint_torque_demand[i] = filter_data[i].torque_demand_trans;
        }

        if(robot_module_ == "LUNBI")
        {
            //轮臂无imu提前返回
            return result;
        }

        bool flag_only_half_up_body = kuavo_settings_.running_settings.only_half_up_body;
        result = result && readImu(flag_only_half_up_body, sensor_data.gyro, sensor_data.acc, sensor_data.quat, sensor_data.free_acc);
        if (redundant_imu_)
        {
            Eigen::Vector3d accel, gyrosco;
            Eigen::Quaterniond quaternion;
            bool readflag = HIPNUC_IMU::getImuDataFrame(accel, gyrosco, quaternion);
            if (!readflag)
            {
                std::cerr << "read redundant imu failed" << std::endl;
            }
        }

        if (imu_invert)
        {
            Invt_imudate(sensor_data);
        }

        auto ts5 = Clock::now();
        // 周期性打印 readSensor 各阶段耗时
        {
            auto us = [](auto a, auto b) {
                return std::chrono::duration_cast<std::chrono::microseconds>(b - a).count();
            };
            auto total = us(ts0, ts5);
            if (total > 300) {
                static auto last_dbg = Clock::now() - std::chrono::seconds(5);
                auto now = Clock::now();
                if (std::chrono::duration_cast<std::chrono::seconds>(now - last_dbg).count() >= 5) {
                    printf("[readSensor] prep=%ldus GetMotor=%ldus safety=%ldus filter=%ldus imu+copy=%ldus total=%ldus\n",
                           us(ts0,ts1), us(ts1,ts2), us(ts2,ts3), us(ts3,ts4), us(ts4,ts5), total);
                    last_dbg = now;
                }
            }
        }

        // motor-level raw snapshot for /rt/motor_state 旁路
        if (result) {
            std::unique_lock<std::shared_mutex> lk(motor_snapshot_mtx_);
            last_motor_sensor_       = sensor_data;
            last_motor_sensor_valid_ = true;
        }

        return result;
    }

    /**
     * @brief 获取电机电流力矩转换系数
     *
     * @param ret_c2t_coeff
     */
    void HardwarePlant::GetC2Tcoeff(double *ret_c2t_coeff, size_t size)
    {
        auto len_c2t_coeff = c2t_coeff.size();
        auto  len = size > len_c2t_coeff ? len_c2t_coeff : size;
        std::cout << "ret_c2t_coeff size: " << (int)size << std::endl;
        std::cout << "len_c2t_coeff size: " << (int)len_c2t_coeff << std::endl;
        std::cout << "c2t size: " << (int)len << std::endl;
        for (auto i = 0; i < len; i++)
        {
            ret_c2t_coeff[i] = c2t_coeff[i];
        }
    }
    Eigen::VectorXd HardwarePlant::GetC2Tcoeff(Eigen::VectorXd &motor_cur)
    {
        Eigen::VectorXd motor_c2t_ = Eigen::VectorXd::Zero(motor_cur.size());
        for (int i = 0; i < motor_cul.size(); i++) {
            int index = motor_cul[i].size(); // 默认超出最大值
            for (int j = 1; j < motor_cul[i].size(); j++) {
                if (abs(motor_cur[i]) < motor_cul[i][j]) {
                    index = j - 1; // 找到区间
                    break;
                }
            }
            if (abs(motor_cur[i]) < motor_cul[i][0]) {
                index = -1; // 小于最小值
            }

            if (index == motor_cul[i].size()) {
                motor_c2t_[i] = motor_coeff[i].back(); // 超出最大值
            } else if (index == -1) {
                motor_c2t_[i] = motor_coeff[i][0]; // 小于最小值
            } else {
                double min_cul = motor_cul[i][index];
                double max_cul = motor_cul[i][index + 1];
                double min_c2t = motor_coeff[i][index];
                double max_c2t = motor_coeff[i][index + 1];
                motor_c2t_[i] = (max_c2t - min_c2t) * (abs(motor_cur[i]) - min_cul) / (max_cul - min_cul) + min_c2t;
            }
        }
        return motor_c2t_;
    }

    Eigen::VectorXd HardwarePlant::GetC2Tcoeff_Torque(Eigen::VectorXd &joint_torque)
    {
        Eigen::VectorXd torque_c2t = Eigen::VectorXd::Zero(joint_torque.size());
        for (int i = 0; i < motor_cul.size(); i++) {
            int index = motor_cul[i].size(); // 默认超出最大值
            for (int j = 1; j < motor_cul[i].size(); j++) {
                if (abs(joint_torque[i]) < motor_cul[i][j] * motor_coeff[i][j]) {
                    index = j - 1; // 找到区间
                    break;
                }
            }
            if (abs(joint_torque[i]) < motor_cul[i][0] * motor_coeff[i][0]) {
                index = -1; // 小于最小值
            }

            if (index == motor_cul[i].size()) {
                torque_c2t[i] = motor_coeff[i].back(); // 超出最大值
            } else if (index == -1) {
                torque_c2t[i] = motor_coeff[i][0]; // 小于最小值
            } else {
                double min_tau = motor_cul[i][index] * motor_coeff[i][index];
                double max_tau = motor_cul[i][index + 1] * motor_coeff[i][index + 1];
                double min_c2t = motor_coeff[i][index];
                double max_c2t = motor_coeff[i][index + 1];
                torque_c2t[i] = (max_c2t - min_c2t) * (abs(joint_torque[i]) - min_tau) / (max_tau - min_tau) + min_c2t;
            }
        }
        return torque_c2t;
    }

    void HardwarePlant::setRuntimeSkipActive(bool active)
    {
        if (ruiwo_actuator == nullptr)
        {
            return;
        }
        if (kuavo_settings_.running_settings.skip_head_runtime_comm)
        {
            std::cout << "[HardwarePlant] runtime_skip " << (active ? "激活: 头部停发运动帧 (仅使能不控制)" : "关闭: 头部恢复运动帧") << std::endl;
        }
        ruiwo_actuator->setRuntimeSkipActive(active);
    }

    void HardwarePlant::writeCommand(Eigen::VectorXd cmd_r, uint32_t na_r, std::vector<int> control_modes, Eigen::VectorXd &joint_kp, Eigen::VectorXd &joint_kd)
    {
        for (uint32_t i = 0; i < num_joint; i++)
        {
            joint_cmd_old[i] = joint_cmd[i];
        }

        // uint32_t na_r = num_joint;
        // Eigen::VectorXd cmd_r(na_r * 4);
        // cmds2Cmdr(cmd, na, cmd_r, na_r);
        std::string err_msg;
        bool result = true;
        Eigen::VectorXd tau_ratio(na_r);
        tau_ratio << cmd_r.segment(na_r * 4, na_r);

        std::vector<uint8_t> joint_tau_ids;
        std::vector<uint8_t> joint_vel_ids;
        std::vector<uint8_t> joint_pos_ids;
        std::vector<JointParam_t> joint_tau_cmd;
        std::vector<JointParam_t> joint_vel_cmd;
        std::vector<JointParam_t> joint_pos_cmd;

        uint8_t num_tau = 0, num_vel = 0, num_pos = 0;
        SensorData_t sensor_data_motor;
        SensorData_t sensor_data_joint;
        getState(sensor_data_motor,sensor_data_joint);
        Eigen::VectorXd joint_c2t(na_foot_ + num_arm_joints + num_head_joints + num_waist_joints);
        Eigen::VectorXd cmd_tau = cmd_r.segment(na_r * 2, na_r);
        joint_c2t = GetC2Tcoeff_Torque(cmd_tau);

        for (uint32_t i = 0; i < na_r; i++)
        {
            JointParam_t joint_tau_temp;
            joint_tau_temp.status = 0;
            double tmp_tau{0.0};
            switch (control_modes[i])
            {
            case MOTOR_CONTROL_MODE_TORQUE:
            {
                bool is_ruiwo = (joint_ids[i] > 0 && joint_ids[i] <= motor_info.driver.size()
                                 && motor_info.driver[joint_ids[i] - 1] == RUIWO);
                if (is_ruiwo) {
                    // RUIWO/CANFD: SetMotorTorque 为空实现，改走 CSP 路径模拟 CST
                    // kp=kd=0, 仅 torqueOffset 有效 → 电机只执行 T_ref
                    // 发 actual position 而非 0，防止 kp/kd 瞬间非零时电机跳变
                    joint_tau_temp.position = sensor_data_joint.joint_q[i] * (180.0 / M_PI);
                    joint_tau_temp.velocityOffset = 0.0;
                    joint_tau_temp.kp = 0.0;
                    joint_tau_temp.kd = 0.0;
                    joint_tau_temp.maxTorque = cmd_r[na_r * 3 + i];
                    tmp_tau = cmd_r[na_r * 2 + i] / joint_c2t[i];
#ifdef __aarch64__
                    tmp_tau *= tau_ratio[i];
#endif
                    if (tmp_tau > joint_tau_temp.maxTorque)
                        joint_tau_temp.torqueOffset = joint_tau_temp.maxTorque;
                    else if (tmp_tau < -joint_tau_temp.maxTorque)
                        joint_tau_temp.torqueOffset = -joint_tau_temp.maxTorque;
                    else
                        joint_tau_temp.torqueOffset = tmp_tau;
                    joint_pos_cmd.push_back(joint_tau_temp);
                    joint_pos_ids.push_back(joint_ids[i]);
                    num_pos++;
                } else {
                    // EC_MASTER: 走原有 SetMotorTorque 路径
                    joint_tau_temp.torqueOffset = 0;
                    joint_tau_temp.torque = cmd_r[na_r * 2 + i] / joint_c2t[i];
                    joint_tau_temp.maxTorque = cmd_r[na_r * 3 + i];
                    joint_tau_temp.position = cmd_r[i] * (180.0 / M_PI);
                    joint_tau_temp.velocityOffset = cmd_r[na_r + i] * (180.0 / M_PI);
                    joint_tau_temp.kp = joint_kp[i];
                    joint_tau_temp.kd = joint_kd[i];
                    joint_tau_cmd.push_back(joint_tau_temp);
                    joint_tau_ids.push_back(joint_ids[i]);
                    num_tau++;
                }
                break;
            }
            case MOTOR_CONTROL_MODE_VELOCITY:

                joint_tau_temp.velocityOffset = 0;
                joint_tau_temp.velocity = cmd_r[na_r + i] * (180.0 / M_PI);
                joint_tau_temp.torqueOffset = cmd_r[na_r * 2 + i] / joint_c2t[i]; //c2t_coeff[i];
                joint_tau_temp.maxTorque = cmd_r[na_r * 3 + i];
                joint_vel_cmd.push_back(joint_tau_temp);
                joint_vel_ids.push_back(joint_ids[i]);
                num_vel++;
                break;
            case MOTOR_CONTROL_MODE_POSITION:
            {
                joint_tau_temp.positionOffset = 0;
                joint_tau_temp.position = cmd_r[i] * (180.0 / M_PI);
                joint_tau_temp.velocityOffset = cmd_r[na_r + i] * (180.0 / M_PI);
                joint_tau_temp.maxTorque = cmd_r[na_r * 3 + i];

                if (host_pd_as_cst_.load(std::memory_order_relaxed))
                {
                    // 主机侧 PD 模拟 CST: τ = Kp*(q_cmd - q_fb) + Kd*(v_cmd - v_fb) + tau_ff
                    double q_cmd_rad = cmd_r[i];
                    double v_cmd_rad = cmd_r[na_r + i];
                    double q_fb_rad = sensor_data_joint.joint_q[i];
                    double v_fb_rad = sensor_data_joint.joint_v[i];
                    double host_tau = joint_kp[i] * (q_cmd_rad - q_fb_rad)
                                    + joint_kd[i] * (v_cmd_rad - v_fb_rad)
                                    + cmd_r[na_r * 2 + i];  // 上层前馈力矩

                    tmp_tau = host_tau / joint_c2t[i];
#ifdef __aarch64__
                    tmp_tau *= tau_ratio[i];
#endif
                    joint_tau_temp.kp = 0.0;
                    joint_tau_temp.kd = 0.0;
                }
                else
                {
                    // 原始 CSP 逻辑: Kp/Kd 透传给电机，电机侧做 PD
                    // EC_MASTER: 用户层 Kp/Kd -> 电机层 Kp/Kd(经 PDO 下发), Ruiwo/Motorevo: 透传
                    if (motor_info.driver[i] == EC_MASTER) {
                        convertUserKpKdToMotorGain(
                            i,
                            joint_kp[i],
                            joint_kd[i],
                            joint_tau_temp.kp,
                            joint_tau_temp.kd);
                    } else {
                        joint_tau_temp.kp = joint_kp[i];
                        joint_tau_temp.kd = joint_kd[i];
                    }
                    tmp_tau = cmd_r[na_r * 2 + i] / joint_c2t[i];
#ifdef __aarch64__
                    tmp_tau *= tau_ratio[i];
#endif
                }

                // 限幅
                if (tmp_tau > joint_tau_temp.maxTorque)
                    joint_tau_temp.torqueOffset = joint_tau_temp.maxTorque;
                else if (tmp_tau < -joint_tau_temp.maxTorque)
                    joint_tau_temp.torqueOffset = -joint_tau_temp.maxTorque;
                else
                    joint_tau_temp.torqueOffset = tmp_tau;

                joint_pos_cmd.push_back(joint_tau_temp);
                joint_pos_ids.push_back(joint_ids[i]);
                num_pos++;
                break;
            }
            default:
                break;
            }
            joint_cmd[i] = joint_tau_temp;
        }
        result = result && checkJointSafety(joint_pos_cmd, joint_pos_ids, err_msg);
        if (!result)
        {
            std::cerr << "writeCommand ERROR: \n"
                      << err_msg << std::endl;
            std::raise(SIGINT);
        }
        // std::cout<<std::endl<<std::endl;

        // motor-level final cmd snapshot for /rt/motor_cmd 旁路
        {
            std::unique_lock<std::shared_mutex> lk(motor_snapshot_mtx_);
            assert(joint_cmd.size() >= na_r && "joint_cmd must be sized to at least na_r before snapshot capture");
            if (last_motor_cmd_.q.size() != na_r) {
                last_motor_cmd_.resize(na_r);
            }
            for (uint32_t i = 0; i < na_r; ++i) {
                last_motor_cmd_.q[i] = joint_cmd[i].position * (M_PI / 180.0);     // deg → rad
                last_motor_cmd_.v[i] = joint_cmd[i].velocity * (M_PI / 180.0);
                if (control_modes[i] == MOTOR_CONTROL_MODE_TORQUE) {
                    bool is_ruiwo = (joint_ids[i] > 0
                                     && joint_ids[i] <= motor_info.driver.size()
                                     && motor_info.driver[joint_ids[i] - 1] == RUIWO);
                    last_motor_cmd_.tau[i] = is_ruiwo ? joint_cmd[i].torqueOffset
                                                     : joint_cmd[i].torque;
                } else {
                    // CSV / CSP 都走 torqueOffset
                    last_motor_cmd_.tau[i] = joint_cmd[i].torqueOffset;
                }
                last_motor_cmd_.kp[i]      = joint_cmd[i].kp;
                last_motor_cmd_.kd[i]      = joint_cmd[i].kd;
                last_motor_cmd_.tau_max[i] = joint_cmd[i].maxTorque;
                last_motor_cmd_.modes[i]   = static_cast<uint8_t>(control_modes[i]);
            }
            last_motor_cmd_.valid = true;
        }

        if (num_tau > 0)
            SetMotorTorque(joint_tau_ids, joint_tau_cmd);
        if (num_vel > 0)
        {
            SetMotorVelocity(joint_vel_ids, joint_vel_cmd);
        }
        if (num_pos > 0)
        {
            SetMotorPosition(joint_pos_ids, joint_pos_cmd);
        }
    }
    void HardwarePlant::calibrateArmJoints()
    {
        auto has_ruiwo = std::find(motor_info.driver.begin(), motor_info.driver.end(), MotorDriveType::RUIWO);
        if (has_ruiwo == motor_info.driver.end())
        {
            std::cerr << "仅支持 ruiwo 电机的手臂关节校准。" << std::endl;
            return;
        }
        if (ruiwo_actuator == nullptr)
        {
            std::cerr << "ruiwo_actuator 为 nullptr，请确保已经加载 ruiwo 驱动。" << std::endl;
            return;
        }
        std::vector<double> moving_pos(num_joint, 0);
        initial_input_cmd_ = '\0';
        int left_right = 0;
        int motor_id = 2;
        std::ostringstream cali_tips_oss;
        cali_tips_oss << "\033[32m*******校准手臂电机编码器偏移*******\n"
                      << "1. 输入 'l' 或 'r' 选择左侧或右侧手臂电机进行校准。\n"
                      << "2. 输入 2-7 选择要校准的电机。\n"
                      << "3. 输入 'w' 增加选中电机编码器的轮数，'s' 减少选中电机编码器的轮数（注意：电机会移动！）\n"
                      << "4. 输入 'e' 启用电机，'d' 禁用电机(禁用之后可以手动掰电机到期望位置), 'f' 将当前的位置用作零位置\n"
                      << "5. 输入 'c' 保存校准结果到文件\n"
                      << "6. 输入 '<Enter>' 显示当前选择，'h' 再次显示此提示，'q' 退出校准。\033[0m\n";
        std::cout << cali_tips_oss.str() << std::endl;
        auto print_selection = [&]()
        {
            std::cout << "选中" << ((left_right == 0) ? "左臂" : "右臂") << std::to_string(motor_id) + "号电机." << std::endl;
        };
        while (1)
        {
            if (kbhit())
            {
                initial_input_cmd_ = '\0';
                initial_input_cmd_ = getchar();
            }
            if (initial_input_cmd_ == 'h')
            {
                std::cout << cali_tips_oss.str() << std::endl;
                initial_input_cmd_ = '\0';
            }
            else if (initial_input_cmd_ == 'e')
            {
                std::cout << "enable the motor " << std::endl;
                ruiwo_actuator->enable();
            }
            else if (initial_input_cmd_ == 'd')
            {
                std::cout << "disable the motor " << std::endl;
                ruiwo_actuator->disable();
            }
            else if (initial_input_cmd_ == 'l')
            {
                left_right = 0;
                print_selection();
            }
            else if (initial_input_cmd_ == 'r')
            {
                left_right = 1;
                print_selection();
            }
            else if (initial_input_cmd_ >= '2' && initial_input_cmd_ <= '7')
            {
                motor_id = initial_input_cmd_ - '0';
                print_selection();
            }
            else if (initial_input_cmd_ == 'w')
            {
                std::cout << "增加" << ((left_right == 0) ? "左臂" : "右臂") << std::to_string(motor_id) + "号电机的轮数." << std::endl;
                int real_id = left_right * 6 + motor_id - 2;
                std::cout << "real_id: " << real_id << std::endl;
                ruiwo_actuator->changeEncoderZeroRound(real_id, 1);
                std::fill(moving_pos.begin(), moving_pos.end(), 0.0);
                this->jointMoveTo(moving_pos, 60.0, dt_);
            }
            else if (initial_input_cmd_ == 's')
            {
                std::cout << "减少" << ((left_right == 0) ? "左臂" : "右臂") << std::to_string(motor_id) + "号电机的轮数." << std::endl;
                int real_id = left_right * 6 + motor_id - 2;
                std::cout << "real_id: " << real_id << std::endl;
                ruiwo_actuator->changeEncoderZeroRound(real_id, -1);
                std::fill(moving_pos.begin(), moving_pos.end(), 0.0);
                this->jointMoveTo(moving_pos, 60.0, dt_);
            }
            else if (initial_input_cmd_ == 'c')
            {
                std::cout << "保存校准结果" << std::endl;
                ruiwo_actuator->saveZeroPosition();
                initial_input_cmd_ = '\0';
            }
            else if (initial_input_cmd_ == 'f')
            {
                std::cout << "将当前的位置保存为零位置." << std::endl;
                ruiwo_actuator->saveAsZeroPosition();
                initial_input_cmd_ = '\0';
            }
            else if (initial_input_cmd_ == 'q')
            {
                std::cout << "退出校准." << std::endl;
                break;
            }
            else if (initial_input_cmd_ == '\n')
            {
                print_selection();
            }
            initial_input_cmd_ = '\0';
            usleep(2000);
        }
    }

    bool HardwarePlant::calibrateArmJointsAtLimit(bool auto_mode, bool calibrate_head, bool head_only)
    {

        std::cout << "\033[32m======== 开始限位校准 ========\033[0m" << std::endl;
        
        // 模式提示
        if (head_only)
        {
            std::cout << "校准模式: \033[33m仅校准头部\033[0m - 只校准头部关节" << std::endl;
        }
        else if (auto_mode)
        {
            std::cout << "校准模式: \033[33m自动校准\033[0m - 所有关节自动依次校准" << std::endl;
        }
        else
        {
            std::cout << "校准模式: \033[33m逐个校准\033[0m - 每组关节需要手动确认" << std::endl;
            std::cout << "提示: 每组校准前按 Enter 开始，校准后按 Enter 继续下一组" << std::endl;
        }
        
        // 检查是否有 ruiwo 电机
        auto has_ruiwo = std::find(motor_info.driver.begin(), motor_info.driver.end(), MotorDriveType::RUIWO);
        if (has_ruiwo == motor_info.driver.end())
        {
            std::cerr << "警告：没有找到 ruiwo 电机" << std::endl;
        }
        else if (ruiwo_actuator == nullptr)
        {
            std::cerr << "错误：ruiwo_actuator 为 nullptr，请确保已经加载 ruiwo 驱动。" << std::endl;
            return false;
        }

        // 获取 EC 电机的当前零点偏移
        std::vector<double> ec_motor_offsets;
        getMotorPositionOffset(ec_motor_offsets);
        std::cout << "获取到 " << ec_motor_offsets.size() << " 个 EC 电机的当前零点偏移" << std::endl;
        
        // 备份原始EC电机零点偏移，用于放弃保存时恢复
        std::vector<double> original_ec_motor_offsets = ec_motor_offsets;
        
        // 用于跟踪哪些零点被修改了
        std::map<int, double> modified_ec_offsets;    // EC index -> new offset
        std::map<int, double> modified_ruiwo_offsets; // Ruiwo arm index -> offset

        // 从配置文件读取校准参数
        const double CALIBRATION_VELOCITY = kuavo_settings_.predefined_arm_pose.arm_calibration_velocity;        // 校准移动速度 (度/秒)
        const double TIMEOUT = kuavo_settings_.predefined_arm_pose.arm_calibration_timeout;                      // 超时时间 (秒)
        const double POSITION_VARIANCE_TIME = kuavo_settings_.predefined_arm_pose.arm_calibration_position_variance_time;      // 位置方差检测时间窗口 (秒)
        const double POSITION_VARIANCE_THRESHOLD = kuavo_settings_.predefined_arm_pose.arm_calibration_position_variance_threshold; // 位置方差阈值 (度^2)
        
        // 固定的校准参数
        const double CURRENT_THRESHOLD = 5.0;           // 电流阈值 (单位取决于传感器)
        const double VELOCITY_THRESHOLD = 1.5;          // 速度阈值 (度/秒)
        const double VELOCITY_STABLE_TIME = 0.3;        // 速度稳定时间 (秒)
        const double CONTROL_FREQUENCY = 500.0;         // 控制频率 (Hz)
        const double POSITION_STEP = CALIBRATION_VELOCITY / CONTROL_FREQUENCY;  // 根据速度和频率计算步长 (度)
        const int VELOCITY_STABLE_SAMPLES = static_cast<int>(VELOCITY_STABLE_TIME * CONTROL_FREQUENCY);
        const int POSITION_HISTORY_SAMPLES = static_cast<int>(POSITION_VARIANCE_TIME * CONTROL_FREQUENCY);
        
        std::cout << "校准参数: 速度=" << CALIBRATION_VELOCITY << "度/秒, 步长=" << POSITION_STEP << "度, 频率=" << CONTROL_FREQUENCY << "Hz" << std::endl;
        std::cout << "            超时=" << TIMEOUT << "秒, 方差时间窗口=" << POSITION_VARIANCE_TIME << "秒, 方差阈值=" << POSITION_VARIANCE_THRESHOLD << std::endl;

        // 获取手臂和头部关节的起始索引
        int num_leg_joints = num_joint - num_arm_joints - num_head_joints - num_waist_joints;
        int arm_start_idx = num_waist_joints + num_leg_joints;
        int num_calibration_joints = num_arm_joints + num_head_joints;  // 手臂+头部
        
        std::cout << "手臂关节数量: " << num_arm_joints << std::endl;
        std::cout << "头部关节数量: " << num_head_joints << std::endl;
        std::cout << "手臂关节起始索引: " << arm_start_idx << std::endl;
        std::cout << "总共需要校准的关节数: " << num_calibration_joints << std::endl;

        // 从 kuavo_settings_ 读取校准安全姿态 (calibration_safe_pose)
        std::vector<double> calibration_safe_pose(num_joint, 0.0);
        if (kuavo_settings_.predefined_arm_pose.calibration_safe_pose.size() >= num_calibration_joints)
        {
            // 读取手臂和头部的安全姿态
            for (size_t i = 0; i < num_calibration_joints; ++i)
            {
                calibration_safe_pose[arm_start_idx + i] = kuavo_settings_.predefined_arm_pose.calibration_safe_pose[i];
            }
            std::cout << "从配置读取到校准安全姿态 (calibration_safe_pose) - 手臂和头部" << std::endl;
        }
        else
        {
            std::cerr << "警告：无法从配置读取 calibration_safe_pose，使用零位作为安全姿态" << std::endl;
        }

        // 从 kuavo_settings_ 读取校准限位 (arm_calibration_limits) - 包含手臂和头部
        std::vector<double> arm_calibration_limits;
        if (kuavo_settings_.predefined_arm_pose.arm_calibration_limits.size() >= num_calibration_joints)
        {
            for (size_t i = 0; i < num_calibration_joints; ++i)
            {
                arm_calibration_limits.push_back(kuavo_settings_.predefined_arm_pose.arm_calibration_limits[i]);
            }
            std::cout << "从配置读取到校准限位 (arm_calibration_limits) - 手臂和头部" << std::endl;
        }
        else
        {
            std::cerr << "错误：无法从配置读取 arm_calibration_limits" << std::endl;
            return false;
        }

        // 从 kuavo_settings_ 读取校准运动方向 (arm_calibration_directions) - 包含手臂和头部
        std::vector<int> arm_calibration_directions;
        if (kuavo_settings_.predefined_arm_pose.arm_calibration_directions.size() >= num_calibration_joints)
        {
            for (size_t i = 0; i < num_calibration_joints; ++i)
            {
                arm_calibration_directions.push_back(static_cast<int>(kuavo_settings_.predefined_arm_pose.arm_calibration_directions[i]));
            }
            std::cout << "从配置读取到校准运动方向 (arm_calibration_directions) - 手臂和头部" << std::endl;
        }
        else
        {
            std::cerr << "错误：无法从配置读取 arm_calibration_directions" << std::endl;
            return false;
        }

        // 动态生成校准分组：头部每个关节独立一组（放在最前），手臂逐对校准（左右对称）
        std::vector<std::vector<int>> calibration_groups;
        int joints_per_arm = num_arm_joints / 2;  // 每条手臂的关节数
        
        // 头部关节：每个关节独立一组，添加在最前面（逆序时最后校准）
        int head_groups_count = 0;
        if (head_only)
        {
            // 只校准头部关节
            if (num_head_joints > 0)
            {
                int head_start_idx = arm_start_idx + num_arm_joints;
                for (int i = 0; i < num_head_joints; ++i)
                {
                    std::vector<int> head_group = { head_start_idx + i };
                    calibration_groups.push_back(head_group);
                    head_groups_count++;
                }
                std::cout << "仅校准头部关节，共 " << num_head_joints << " 个独立关节组" << std::endl;
            }
            else
            {
                std::cerr << "错误：没有头部关节需要校准" << std::endl;
                return false;
            }
        }
        else
        {
            // 正常模式：头部+手臂
            if (calibrate_head && num_head_joints > 0)
            {
                int head_start_idx = arm_start_idx + num_arm_joints;
                for (int i = 0; i < num_head_joints; ++i)
                {
                    std::vector<int> head_group = { head_start_idx + i };
                    calibration_groups.push_back(head_group);
                    head_groups_count++;
                }
                std::cout << "已添加头部关节组，共 " << num_head_joints << " 个独立关节组（最后校准）" << std::endl;
            }
            else if (num_head_joints > 0)
            {
                std::cout << "跳过头部关节校准" << std::endl;
            }
            
            // 手臂关节：从第一个关节开始，包括 EC master 和 ruiwo 电机
            for (int i = 0; i < joints_per_arm; ++i)
            {
                std::vector<int> group = {
                    arm_start_idx + i,                    // 左臂关节
                    arm_start_idx + joints_per_arm + i    // 右臂对应关节
                };
                calibration_groups.push_back(group);
            }
        }
        
        if (head_only)
        {
            std::cout << "生成了 " << calibration_groups.size() << " 组校准关节（仅头部 " << head_groups_count << " 组）" << std::endl;
        }
        else
        {
            std::cout << "生成了 " << calibration_groups.size() << " 组校准关节（头部 " << head_groups_count << " 组 + 手臂 " << joints_per_arm << " 组）" << std::endl;
        }

        // 步骤1: 准备全身关节的初始位置
        // 读取当前所有关节的位置（腿部位置）
        std::vector<JointParam_t> all_joint_data(num_joint);
        GetMotorData(joint_ids, all_joint_data);
        
        // 保存腿部当前位置，设置手臂为校准安全姿态
        std::vector<double> calibration_start_pose(num_joint);
        for (size_t i = 0; i < num_joint; ++i)
        {
            calibration_start_pose[i] = all_joint_data[i].position;
        }
        
        // 根据校准模式设置安全姿态
        if (head_only)
        {
            // 只校准头部时，只设置头部关节的安全姿态
            int head_start_idx = arm_start_idx + num_arm_joints;
            for (size_t i = 0; i < num_head_joints; ++i)
            {
                calibration_start_pose[head_start_idx + i] = calibration_safe_pose[head_start_idx + i];
            }
            std::cout << "\n步骤 1: 移动到校准初始位置（腿部保持当前位置，头部移动到安全姿态）..." << std::endl;
        }
        else
        {
            // 正常模式：设置手臂和头部的安全姿态
            for (size_t i = 0; i < num_calibration_joints; ++i)
            {
                calibration_start_pose[arm_start_idx + i] = calibration_safe_pose[arm_start_idx + i];
            }
            std::cout << "\n步骤 1: 移动到校准初始位置（腿部保持当前位置，手臂和头部移动到安全姿态）..." << std::endl;
        }
        jointMoveTo(calibration_start_pose, 30.0, dt_);
        usleep(100000); // 等待500ms
        
        // 读取所有关节的当前状态，用于后续控制
        std::vector<JointParam_t> current_all_joints(num_joint);
        std::vector<JointParam_t> target_all_joints(num_joint);
        GetMotorData(joint_ids, current_all_joints);
        

        // 获取默认的 kp/kd 值（用于 CSP 模式）
        const auto &default_joint_kp = kuavo_settings_.running_settings.joint_kp;
        const auto &default_joint_kd = kuavo_settings_.running_settings.joint_kd;
        
        // 初始化目标位置为当前位置
        for (size_t i = 0; i < num_joint; ++i)
        {
            target_all_joints[i].position = current_all_joints[i].position;
            target_all_joints[i].velocity = 0.0;
            target_all_joints[i].torque = 0.0;

            // 对于 CSP 模式（位置控制模式），使用默认的 kp/kd 值
            // 只有 EC_MASTER 电机在 CSP 模式下需要设置 kp/kd
            if (motor_info.driver[i] == EC_MASTER && motor_info.motors_exist[i])
            {
                // 找到对应的 EC_MASTER 索引（在所有 EC_MASTER 电机中的索引）
                int ec_master_index = -1;
                int ec_count = 0;
                for (int j = 0; j <= static_cast<int>(i); ++j)
                {
                    if (motor_info.driver[j] == EC_MASTER && motor_info.motors_exist[j])
                    {
                        if (j == static_cast<int>(i))
                        {
                            ec_master_index = ec_count;
                            break;
                        }
                        ec_count++;
                    }
                }
                
                // 如果找到了对应的索引，且默认值数组大小足够，则使用默认值
                if (ec_master_index >= 0 && 
                    ec_master_index < static_cast<int>(default_joint_kp.size()) &&
                    ec_master_index < static_cast<int>(default_joint_kd.size()))
                {
                    target_all_joints[i].kp = static_cast<double>(default_joint_kp[ec_master_index]);
                    target_all_joints[i].kd = static_cast<double>(default_joint_kd[ec_master_index]);
                }
                else
                {
                    // 如果找不到对应的默认值，使用 0（保持原有行为）
                    target_all_joints[i].kp = 0.0;
                    target_all_joints[i].kd = 0.0;
                }
            }
            else
            {
                // 非 EC_MASTER 电机或电机不存在，设置为 0
                target_all_joints[i].kp = 0.0;
                target_all_joints[i].kd = 0.0;
            }
        }
        
        std::cout << "已获取全身关节初始状态" << std::endl;
        
        // 步骤2: 逐组校准（从最后一组开始，逆序执行）
        for (int group_idx = calibration_groups.size() - 1; group_idx >= 0; --group_idx)
        {
            const auto& joint_group = calibration_groups[group_idx];
            std::cout << "\n======== 校准第 " << (group_idx + 1) << " 组关节 (从末端向近端) ========" << std::endl;
            std::cout << "关节索引: ";
            for (int joint_idx : joint_group) {
                std::cout << joint_idx << " ";
            }
            std::cout << std::endl;

            // 逐个模式：等待用户确认
            if (!auto_mode)
            {
                std::cout << "\n\033[33m按 Enter 开始校准第 " << (group_idx + 1) << " 组关节...\033[0m" << std::endl;
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            }

            // 准备组内所有有效关节
            std::vector<int> valid_joints;
            std::vector<int> valid_arm_joint_indices;
            std::vector<double> target_limits;
            std::vector<int> directions;
            
            for (int joint_idx : joint_group)
            {
                // 检查是否是需要校准的关节（手臂或头部）
                if (joint_idx < arm_start_idx || joint_idx >= arm_start_idx + num_calibration_joints)
                {
                    continue;
                }

                int arm_joint_idx = joint_idx - arm_start_idx;  // 在配置数组中的索引
                int calibration_direction = arm_calibration_directions[arm_joint_idx];
                
                valid_joints.push_back(joint_idx);
                valid_arm_joint_indices.push_back(arm_joint_idx);
                target_limits.push_back(arm_calibration_limits[arm_joint_idx]);
                directions.push_back(calibration_direction);
            }
            
            if (valid_joints.empty())
            {
                std::cout << "本组没有需要校准的关节，跳过" << std::endl;
                continue;
            }
            
            std::cout << "\n本组共 " << valid_joints.size() << " 个关节需要校准" << std::endl;
            for (size_t i = 0; i < valid_joints.size(); ++i)
            {
                std::cout << "  关节 " << valid_joints[i] << ": 目标限位=" << target_limits[i] 
                          << "度, 方向=" << (directions[i] > 0 ? "正" : "负") << std::endl;
            }
            
            // 读取全身关节当前状态
            GetMotorData(joint_ids, current_all_joints);
            
            // 记录本组关节的起始位置（使用calibration_start_pose）
            std::vector<double> start_positions(valid_joints.size());
            std::vector<double> current_target_positions(valid_joints.size());
            std::vector<bool> joint_limit_reached(valid_joints.size(), false);
            std::vector<double> limit_positions(valid_joints.size(), 0.0);  // 记录到达限位时的位置
            std::vector<int> velocity_low_counts(valid_joints.size(), 0);
            
            // 用于位置方差检测的历史记录（循环队列）
            std::vector<std::deque<double>> position_history(valid_joints.size());
            
            for (size_t i = 0; i < valid_joints.size(); ++i)
            {
                int joint_idx = valid_joints[i];
                // 使用calibration_start_pose中的位置作为起始位置
                start_positions[i] = calibration_start_pose[joint_idx];
                current_target_positions[i] = start_positions[i];
                
                std::cout << "  关节 " << joint_idx << " 起始位置: " << start_positions[i] << " 度" << std::endl;
            }
            
            // 初始化全身目标位置为calibration_start_pose
            for (size_t i = 0; i < num_joint; ++i)
            {
                target_all_joints[i].position = calibration_start_pose[i];
            }

            // 开始同时移动所有关节到限位
            auto start_time = std::chrono::steady_clock::now();
            bool all_limits_reached = false;
            std::vector<std::string> stop_reasons(valid_joints.size(), "未知");

            std::cout << "\n开始同时移动所有关节到限位..." << std::endl;

            while (!all_limits_reached)
            {
                // 读取全身关节的当前传感器数据
                GetMotorData(joint_ids, current_all_joints);
                
                auto current_time = std::chrono::steady_clock::now();
                double elapsed_time = std::chrono::duration<double>(current_time - start_time).count();
                
                // 构建实时状态打印字符串
                std::stringstream status_line;
                status_line << "\r[校准中 " << std::fixed << std::setprecision(1) << elapsed_time << "s] ";
                
                // 检查每个关节是否到达限位
                bool any_active = false;
                for (size_t i = 0; i < valid_joints.size(); ++i)
                {
                    int joint_idx = valid_joints[i];
                    
                    double current_pos = current_all_joints[joint_idx].position;
                    double current_vel = current_all_joints[joint_idx].velocity;
                    double current_torque = std::abs(current_all_joints[joint_idx].torque);
                    
                    if (joint_limit_reached[i])
                    {
                        // 已到达限位的关节显示为完成状态，并保持在限位位置
                        status_line << "J" << joint_idx << ":✓ ";
                        target_all_joints[joint_idx].position = limit_positions[i];
                        continue;
                    }
                    
                    // 记录位置历史用于方差计算
                    position_history[i].push_back(current_pos);
                    if (position_history[i].size() > POSITION_HISTORY_SAMPLES)
                    {
                        position_history[i].pop_front();
                    }
                    
                    // 添加当前关节的状态信息到打印字符串
                    status_line << "J" << joint_idx << ":[" 
                                << std::fixed << std::setprecision(1) 
                                << current_pos << "°, " 
                                << current_vel << "°/s, " 
                                << std::setprecision(2)
                                << current_torque << "Nm] Target:"
                                << current_target_positions[i] << "°,"  ;
                    
                    any_active = true;
                    
                    // 检查停止条件
                    bool limit_detected = false;
                    std::string detected_reason = "";
                    
                    // 条件1: 电流超出阈值
                    // if (current_torque > CURRENT_THRESHOLD)
                    // {
                    //     limit_detected = true;
                    //     detected_reason = "电流超出阈值";
                    // }
                    // 条件2: 速度持续低于阈值
                    if (std::abs(current_vel) < VELOCITY_THRESHOLD)
                    {
                        velocity_low_counts[i]++;
                        if (velocity_low_counts[i] >= VELOCITY_STABLE_SAMPLES)
                        {
                            limit_detected = true;
                            detected_reason = "速度持续低于阈值";
                        }
                    }
                    else
                    {
                        velocity_low_counts[i] = 0;
                    }
                    
                    // 条件3: 位置方差很小（0.4s内位置几乎不变）
                    if (!limit_detected && position_history[i].size() >= POSITION_HISTORY_SAMPLES)
                    {
                        // 计算位置方差
                        double mean = std::accumulate(position_history[i].begin(), position_history[i].end(), 0.0) / position_history[i].size();
                        double variance = 0.0;
                        for (double pos : position_history[i])
                        {
                            variance += (pos - mean) * (pos - mean);
                        }
                        variance /= position_history[i].size();
                        
                        if (variance < POSITION_VARIANCE_THRESHOLD)
                        {
                            limit_detected = true;
                            detected_reason = "检测到位置基本不变";
                        }
                    }
                    
                    // 条件4: 超时
                    if (!limit_detected && elapsed_time > TIMEOUT)
                    {
                        limit_detected = true;
                        detected_reason = "超时";
                    }
                    
                    // 如果检测到限位，立即记录当前位置
                    if (limit_detected)
                    {
                        joint_limit_reached[i] = true;
                        limit_positions[i] = current_pos;  // 立即记录限位位置
                        stop_reasons[i] = detected_reason;
                        std::cout << "\n关节 " << joint_idx << " 到达限位 (" << detected_reason 
                                  << "), 位置: " << current_pos << " 度" << std::endl;
                    }
                    
                    // 如果还没到达限位，继续位置插值
                    if (!joint_limit_reached[i])
                    {
                        current_target_positions[i] += directions[i] * POSITION_STEP;
                        
                        // 更新全身关节中该关节的目标位置
                        target_all_joints[joint_idx].position = current_target_positions[i];
                    }
                }
                
                // 发送全身关节位置命令
#ifdef ENABLE_ECMASTER
                // 在 CSP 模式下，确保 EC_MASTER 电机有默认的 kp/kd（虽然已在初始化时设置，但为了保险再次确保）
                setDefaultKpKdForEcMaster(target_all_joints, joint_ids);
#endif
                SetMotorPosition(joint_ids, target_all_joints);
                
                all_limits_reached = !any_active;
                
                // 打印实时状态（同一行刷新）
                std::cout << status_line.str() << std::flush;
                static int print_count = 0;
                if (print_count++ % 150 == 0)
                    std::cout << std::endl;
                // 控制频率
                usleep(1000000 / CONTROL_FREQUENCY);
            }
            
            std::cout << "\n所有关节已到达限位" << std::endl;
            
            // 使用记录的限位位置计算零点偏移
            for (size_t i = 0; i < valid_joints.size(); ++i)
            {
                int joint_idx = valid_joints[i];
                int arm_joint_idx = valid_arm_joint_indices[i];
                double final_position = limit_positions[i];  // 使用到达限位时立即记录的位置
                
                std::cout << "\n关节 " << joint_idx << " 校准结果:" << std::endl;
                std::cout << "  停止原因: " << stop_reasons[i] << std::endl;
                std::cout << "  实际检测到的限位位置: " << final_position << " 度" << std::endl;
                
                // 计算零点偏移
                double zero_offset = final_position - target_limits[i];
                std::cout << "  零点偏移 = " << final_position << " - " << target_limits[i] 
                          << " = " << zero_offset << " 度" << std::endl;
                
                // 根据电机类型调整零点
                MotorDriveType motor_type = motor_info.driver[joint_idx];
                
                if (motor_type == MotorDriveType::EC_MASTER)
                {
                    // EC 电机：找到对应的EC索引
                    int ec_idx = -1;
                    int ec_count = 0;
                    for (int j = 0; j <= joint_idx; ++j)
                    {
                        if (motor_info.driver[j] == MotorDriveType::EC_MASTER)
                        {
                            if (j == joint_idx)
                            {
                                ec_idx = ec_count;
                                break;
                            }
                            ec_count++;
                        }
                    }
                    
                    if (ec_idx >= 0 && ec_idx < ec_motor_offsets.size())
                    {
                        double old_offset = ec_motor_offsets[ec_idx];
                        double new_ec_offset = old_offset + zero_offset;
                        ec_motor_offsets[ec_idx] = new_ec_offset;
                        modified_ec_offsets[ec_idx] = new_ec_offset;
                        
                        // 立即更新EC电机零点
                        setMotorPositionOffset(ec_motor_offsets);
                        
                        std::cout << "  已更新 EC 电机[" << ec_idx << "] 零点偏移: " 
                                  << old_offset << " -> " << new_ec_offset << " 度" << std::endl;
                    }
                    else
                    {
                        std::cerr << "  错误：无法找到 EC 电机索引" << std::endl;
                    }
                }
                else if (motor_type == MotorDriveType::RUIWO)
                {
                    if (ruiwo_actuator != nullptr)
                    {
                        // RUIWO 电机：使用 name-based 映射获取正确的 motor_refs_ 索引
                        int ruiwo_idx = jointId2RuiwoId_.motor_index(joint_idx + 1);
                        ruiwo_actuator->adjustZeroPosition(ruiwo_idx, zero_offset * M_PI / 180.0);
                        modified_ruiwo_offsets[ruiwo_idx] = zero_offset;
                        std::cout << "  已调整 ruiwo 电机[" << ruiwo_idx << "] (关节 " << joint_idx
                                  << ") 的零点偏移: " << zero_offset << " 度" << std::endl;
                    }
                }
                else
                {
                    std::cout << "  警告：关节 " << joint_idx << " 的电机类型未知，跳过零点调整" << std::endl;
                }
            }
            
            
            // 每组校准完成后，也移回到校准初始姿态
            std::cout << "\n第 " << (group_idx + 1) << " 组关节校准完成，移回到初始姿态..." << std::endl;
            jointMoveTo(calibration_start_pose, 30.0, dt_);
            usleep(100000); // 等待500ms
        }

        // 步骤3: 移动校准关节到零位
        if (head_only)
        {
            std::cout << "\n======== 所有关节校准完成，移动头部到零位 ========" << std::endl;
            // 基于calibration_start_pose生成zero_pose，将头部设为0
            std::vector<double> zero_pose = calibration_start_pose;
            int head_start_idx = arm_start_idx + num_arm_joints;
            for (size_t i = 0; i < num_head_joints; ++i)
            {
                zero_pose[head_start_idx + i] = 0.0;
            }
            
            std::cout << "移动头部到零位..." << std::endl;
            jointMoveTo(zero_pose, 30.0, dt_);
            std::cout << "头部已移动到零位" << std::endl;
        }
        else
        {
            std::cout << "\n======== 所有关节校准完成，移动手臂和头部到零位 ========" << std::endl;
            // 基于calibration_start_pose生成zero_pose，将手臂和头部设为0
            std::vector<double> zero_pose = calibration_start_pose;
            for (size_t i = 0; i < num_calibration_joints; ++i)
            {
                zero_pose[arm_start_idx + i] = 0.0;
            }
            
            std::cout << "移动手臂和头部到零位..." << std::endl;
            jointMoveTo(zero_pose, 30.0, dt_);
            std::cout << "手臂和头部已移动到零位" << std::endl;
        }

        // 步骤4: 保存确认
        std::cout << "\n======== 校准完成，等待保存确认 ========" << std::endl;
        std::cout << "已修改 " << modified_ec_offsets.size() << " 个 EC 电机零点" << std::endl;
        std::cout << "已修改 " << modified_ruiwo_offsets.size() << " 个 ruiwo 电机零点" << std::endl;
        
        std::cout << "\n\033[33m按 's' 保存校准结果，按其他键放弃保存...\033[0m" << std::endl;
        
        char save_choice;
        std::cin >> save_choice;
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        
        if (save_choice == 's' || save_choice == 'S')
        {
            std::cout << "\n正在保存校准结果..." << std::endl;
            
            // 保存 EC 电机零点
            if (!modified_ec_offsets.empty())
            {
                // EC电机零点已在校准时实时更新，这里只需保存到文件
                saveOffset();  // 保存EC电机零点到文件
                std::cout << "✓ EC 电机零点已保存 (" << modified_ec_offsets.size() << " 个)" << std::endl;
            }
            
            // 保存 ruiwo 电机零点
            if (!modified_ruiwo_offsets.empty() && ruiwo_actuator != nullptr)
            {
                ruiwo_actuator->saveZeroPosition();
                std::cout << "✓ ruiwo 电机零点已保存 (" << modified_ruiwo_offsets.size() << " 个)" << std::endl;
            }
            
            std::cout << "\n\033[32m======== 手臂限位校准完成并已保存 ========\033[0m" << std::endl;
            return true;
        }
        else
        {
            std::cout << "\n\033[31m放弃保存，正在恢复原始零点...\033[0m" << std::endl;
            
            // 恢复EC电机零点到原始值
            if (!modified_ec_offsets.empty())
            {
                setMotorPositionOffset(original_ec_motor_offsets);
                std::cout << "已恢复 EC 电机零点到原始值" << std::endl;
            }
            
            // 注意：RUIWO电机的零点已经通过adjustZeroPosition修改，无法简单恢复
            // 用户需要重启程序或重新校准来恢复RUIWO电机
            if (!modified_ruiwo_offsets.empty())
            {
                std::cout << "\033[33m警告：RUIWO 电机零点已修改但未保存，需要重启程序恢复原始值\033[0m" << std::endl;
            }
            
            std::cout << "\n\033[32m======== 手臂限位校准已取消 ========\033[0m" << std::endl;
            return false;
        }
    }

    void HardwarePlant::calibrateBipedLoop()
    {
        std::vector<double> moving_pos(num_joint, 0);
        double cali_tau_limit = 3;
        this->jointMoveTo(moving_pos, 60.0, dt_, cali_tau_limit);
        initial_input_cmd_ = '\0';
        int left_right = 0;
        int motor_id = 1;
        std::ostringstream cali_tips_oss;
        cali_tips_oss << "\033[32m*******校准腿部电机编码器偏移*******\n"
                      << "1. 输入 'l' 或 'r' 选择要校准的是左腿或右腿。\n"
                      << "2. 输入 1-6 选择要校准的电机。\n"
                      << "3. 输入 'w' 增加电机编码器的轮数，输入 's' 减少电机编码器的轮数（注意：电机会移动！）\n"
                      << "4. 输入 'c' 保存校准结果。\n"
                      << "5. 输入 '<Enter>' 显示当前选择，输入 'h' 再次显示此提示，输入 'q' 退出校准。\033[0m\n";
        std::cout << cali_tips_oss.str() << std::endl;
        auto print_selection = [&]()
        {
            std::cout << "选中" << ((left_right == 0) ? "左腿" : "右腿") << std::to_string(motor_id) + "号电机." << std::endl;
        };
        while (1)
        {
            if (kbhit())
            {
                initial_input_cmd_ = '\0';
                initial_input_cmd_ = getchar();
            }
            if (initial_input_cmd_ == 'h')
            {
                std::cout << cali_tips_oss.str() << std::endl;
                initial_input_cmd_ = '\0';
            }
            else if (initial_input_cmd_ == 'l')
            {
                left_right = 0;
                print_selection();
            }
            else if (initial_input_cmd_ == 'r')
            {
                left_right = 1;
                print_selection();
            }
            else if (initial_input_cmd_ >= '1' && initial_input_cmd_ <= '6')
            {
                motor_id = initial_input_cmd_ - '0';
                print_selection();
            }
            if (initial_input_cmd_ == 'w')
            {
                std::cout << "增加 " << ((left_right == 0) ? "左腿" : "右腿") << std::to_string(motor_id) + "号 " + "电机的编码器轮数." << std::endl;
                this->calibrateMotor(left_right * 6 + motor_id - 1, 1);
                std::fill(moving_pos.begin(), moving_pos.end(), 0.0);
                this->jointMoveTo(moving_pos, 60.0, dt_, cali_tau_limit);
            }
            else if (initial_input_cmd_ == 's')
            {
                std::cout << "减少 " << ((left_right == 0) ? "左腿" : "右腿") << std::to_string(motor_id) + "号 " + "电机的编码器轮数." << std::endl;
                this->calibrateMotor(left_right * 6 + motor_id - 1, -1);
                std::fill(moving_pos.begin(), moving_pos.end(), 0.0);
                this->jointMoveTo(moving_pos, 60.0, dt_, cali_tau_limit);
            }
            else if (initial_input_cmd_ == 'c')
            {
                std::cout << "保存校准结果." << std::endl;
                saveOffset();  // EC 零点
                if (ruiwo_actuator) ruiwo_actuator->saveZeroPosition();  // RUIWO 零点
                initial_input_cmd_ = '\0';
            }
            else if (initial_input_cmd_ == 'q')
            {
                std::cout << "退出校准." << std::endl;
                break;
            }
            else if (initial_input_cmd_ == '\n')
            {
                print_selection();
            }
            initial_input_cmd_ = '\0';
            usleep(2000);
        }
    }

        void HardwarePlant::calibrateWheelLoop()
    {
        std::vector<double> moving_pos(num_joint, 0);
        double cali_tau_limit = 3;
        this->jointMoveTo(moving_pos, 60.0, dt_, cali_tau_limit);
        initial_input_cmd_ = '\0';
        int motor_id = 1;

        std::ostringstream cali_tips_oss;
        cali_tips_oss << "\033[32m*******校准腿部电机编码器偏移*******\n"
                      << "1. 输入 1-4 选择要校准的电机。\n"
                      << "2. 输入 'w' 增加电机编码器的轮数，输入 's' 减少电机编码器的轮数（注意：电机会移动！）\n"
                      << "3. 输入 'c' 保存校准结果。\n"
                      << "4. 输入 '<Enter>' 显示当前选择，输入 'h' 再次显示此提示，输入 'q' 退出校准。\033[0m\n";
        std::cout << cali_tips_oss.str() << std::endl;

        auto print_selection = [&]()
        {
            std::cout << "选中" << std::to_string(motor_id) + "号电机." << std::endl;
        };

        auto get_motor_id = [&]() -> int
        {
            return motor_id - 1;
        };

        auto print_operation = [&](const std::string& operation)
        {
            std::cout << operation << std::to_string(motor_id) + "号电机的编码器轮数." << std::endl;
        };

        while (true)
        {
            if (kbhit())
            {
                initial_input_cmd_ = '\0';
                initial_input_cmd_ = getchar();
            }
            if (initial_input_cmd_ == 'h')
            {
                std::cout << cali_tips_oss.str() << std::endl;
                initial_input_cmd_ = '\0';
            }
            else if (initial_input_cmd_ >= '1' && initial_input_cmd_ <= '4')
            {
                motor_id = initial_input_cmd_ - '0';
                print_selection();
            }
            else if (initial_input_cmd_ == 'w')
            {
                print_operation("增加 ");
                this->calibrateMotor(get_motor_id(), 1);
                std::fill(moving_pos.begin(), moving_pos.end(), 0.0);
                this->jointMoveTo(moving_pos, 60.0, dt_, cali_tau_limit);
            }
            else if (initial_input_cmd_ == 's')
            {
                print_operation("减少 ");
                this->calibrateMotor(get_motor_id(), -1);
                std::fill(moving_pos.begin(), moving_pos.end(), 0.0);
                this->jointMoveTo(moving_pos, 60.0, dt_, cali_tau_limit);
            }
            else if (initial_input_cmd_ == 'c')
            {
                std::cout << "保存校准结果." << std::endl;
                saveOffset();  // EC 零点
                if (ruiwo_actuator) ruiwo_actuator->saveZeroPosition();  // RUIWO 零点
                initial_input_cmd_ = '\0';
            }
            else if (initial_input_cmd_ == 'q')
            {
                std::cout << "退出校准." << std::endl;
                break;
            }
            else if (initial_input_cmd_ == '\n')
            {
                print_selection();
            }
            initial_input_cmd_ = '\0';
            usleep(2000);
        }
    }

    int8_t HardwarePlant::HWPlantInit()
    {
        std::cout << "[HWPlantInit] Starting initialization..." << std::endl;
        
        is_cali_ = hardware_param_.cali;
        hardware_status_ = hardware_param_.cali ? -1 : kStatusNotInAnyState;
        
        if (hardware_param_.cali_leg)
        {
            hardware_status_ = -2;
        }

        std::cout << "[HWPlantInit] Checking for Ruiwo motors..." << std::endl;
        auto has_ruiwo = std::find(motor_info.driver.begin(), motor_info.driver.end(), MotorDriveType::RUIWO);
        if (has_ruiwo != motor_info.driver.end())
        {
            // ===== 电机固件预检: 固件版本比对 + 运行参数写入回读校验 =====
            // 必须在任何 actuator 创建之前执行(此时总线无控制流量);
            // 参数校验失败/无响应 → 拒绝初始化硬件层; 固件版本不匹配仅黄色告警不拦截
            // 配置随代码走: config/<robot_type>/motor_firmware.json (与 kuavo.json 同目录)
            motor_preflight::PreflightOptions preflight_options;
            {
                std::string robot_type = IS_KUAVO(rb_version_)
                                             ? "kuavo_v" + rb_version_.to_string()
                                             : "roban_v" + rb_version_.to_string();
                preflight_options.firmware_json_path = hardware_param_.kuavo_assets_path +
                                                       "/config/" + robot_type +
                                                       "/motor_firmware.json";
            }
            if (!motor_preflight::RunMotorParamPreflight(preflight_options)) {
                std::cerr << "\033[1;31m[HWPlantInit] 电机固件/参数预检未通过, 禁止初始化硬件层.\033[0m" << std::endl;
                return -1;
            }

            bool cali_arm = hardware_param_.cali_arm;
            bool cali_leg = hardware_param_.cali_leg;
            // cali_motorevo 保留给下方非 MotorevoActuator 旧路径(RuiWoActuator kuavo4pro)用,
            // 新路径(MotorevoActuator)下已不再使用它.
            bool cali_motorevo = cali_arm || cali_leg;

            // 按 CAN 总线粒度选择校准范围, 避免"标上肢连同下肢一起标"的问题:
            //   - ARM  (RK3588): bcan0=左腿+腰, bcan1=右腿; bcan2=左臂+头, bcan3=右臂
            //   - x86  (EC-Master): canbus0=左臂+头, canbus1=右臂; 腿部走 EC, 不在此校准
            std::set<std::string> cali_buses;
#ifdef __aarch64__
            if (cali_leg) { cali_buses.insert("bcan0"); cali_buses.insert("bcan1"); }
            if (cali_arm) { cali_buses.insert("bcan2"); cali_buses.insert("bcan3"); }
#else
            if (cali_arm) { cali_buses.insert("canbus0"); cali_buses.insert("canbus1"); }
#endif

            if (cali_arm) {
                hardware_status_ = -1;
            }
            {
                std::string bus_list;
                for (const auto& b : cali_buses) {
                    if (!bus_list.empty()) bus_list += ",";
                    bus_list += b;
                }
                if (bus_list.empty()) bus_list = "(none)";
                std::cout << "[HWPlantInit] cali_arm=" << cali_arm
                          << ", cali_leg=" << cali_leg
                          << ", cali_buses={" << bus_list << "}" << std::endl;
            }
            
            try {
                CanbusWiringType canbus_mode = leju::hw::config_utils::GetCanbusWiringType();
                if(canbus_mode == CanbusWiringType::DUAL_BUS
                  || canbus_mode == CanbusWiringType::SINGLE_BUS) {
                    // dual bus
                    std::cout << "[HWPlantInit] Creating RuiWoActuator instance with DUAL_BUS" << std::endl;
                    auto config_file = canbus_sdk::ConfigParser::getDefaultConfigFilePath();
                    std::cout << "[HWPlantInit] MotorevoActuator config file: " << config_file << std::endl;
                    int control_frequency = 250;
                    if (hardware_param_.can_send_frequency > 0) {
                        // 使用外部指定的频率
                        control_frequency = hardware_param_.can_send_frequency;
                        printf("\033[33m CAN 发送频率(外部指定): %dHz \033[0m\n", control_frequency);
                    } else {
                        // 自动选择: 全 CAN FD 模式 (无 EC 电机) 或 Roban2 双总线: 提高到 500Hz
                        bool all_ruiwo = (std::find(motor_info.driver.begin(), motor_info.driver.end(), MotorDriveType::EC_MASTER) == motor_info.driver.end());
                        if(all_ruiwo || (rb_version_.start_with(1) && canbus_mode == CanbusWiringType::DUAL_BUS)) {
                            printf("\033[33m CAN FD 电机控制频率 500hz \033[0m\n");
                            control_frequency = 500;
                        }
                    }
                    bool skip_head_runtime = kuavo_settings_.running_settings.skip_head_runtime_comm;
                    if (skip_head_runtime) {
                        printf("\033[33m[HWPlantInit] kuavo.json skip_head_runtime_comm=true: 头部运行期不下发运动帧\033[0m\n");
                    }
                    ruiwo_actuator = new motorevo::MotorevoActuator(config_file, cali_buses, control_frequency, skip_head_runtime,
                                                                     hardware_param_.cali_single_motor_mode);
                }
                else {
                    // unknown --> 使用旧的 kuavo4pro 上的控制方式
                    std::cout << "[HWPlantInit] Creating RuiWoActuator instance..." << std::endl;
                    ruiwo_actuator = new RuiWoActuator("unused", cali_motorevo, hardware_param_.cali_single_motor_mode);
                    std::cout << "[HWPlantInit] RuiWoActuator instance created" << std::endl;
                }
            
                std::cout << "[HWPlantInit] Initializing RuiWoActuator..." << std::endl;
                auto ret = ruiwo_actuator->initialize();
                if (ret != 0)
                {
                    std::cerr << "[HWPlantInit] RuiWoActuator initialization failed with code: " << ret << std::endl;
                    return -1;
                }
                std::cout << "[HWPlantInit] RuiWoActuator initialized successfully" << std::endl;

                // ====== 建立 kuavo.json 关节顺序 → motor_refs_ 索引的 name-based 映射 ======
                // kuavo.json 关节顺序(固定): 左腿(6)+右腿(6)+腰(1)+左臂(4)+右臂(4)+头(2)=23
                //   joint 1-6:   Lleg_joint_01-06       joint 7-12:  Rleg_joint_01-06
                //   joint 13:    Waist_joint_01          joint 14-17: Larm_joint_01-04
                //   joint 18-21: Rarm_joint_01-04        joint 22:    Head_yaw
                //   joint 23:    Head_pitch
                // CAN 配置文件名由此关节位置推导, 与具体驱动(EC_MASTER/RUIWO)无关.
                // 通过名称匹配到 motor_refs_ 索引, 兼容:
                //   x86: 腿走 EC_MASTER, 仅 手臂+头 在 CAN 上 (部分 RUIWO)
                //   ARM: 全部走 CAN FD (全 RUIWO)
                {
                    auto motor_ref_names = ruiwo_actuator->getMotorRefNames();

                    if (!motor_ref_names.empty()) {
                        std::cout << "\n======== Ruiwo Motor Mapping (name-based) ========" << std::endl;
                        std::cout << "KuavoJointId -> motor_refs_[idx]  MotorName" << std::endl;
                        std::cout << "-------------------------------------------------" << std::endl;

                        uint8_t ruiwo_id = 1;
                        int kuavo_count = 0;
                        for (size_t i = 0; i < motor_info.driver.size(); ++i) {
                            if (motor_info.driver[i] == RUIWO) {
                                uint8_t joint_id = static_cast<uint8_t>(i + 1);
                                uint8_t motor_idx = static_cast<uint8_t>(kuavo_count); // fallback
                                std::string motor_name = "(unknown)";

                                // 根据关节在 driver 数组中的位置 (0-based i) 推导 CAN 配置名称
                                int j = static_cast<int>(i);
                                if (j < 6)      { char buf[32]; snprintf(buf, sizeof(buf), "Lleg_joint_%02d", j + 1);        motor_name = buf; }
                                else if (j < 12){ char buf[32]; snprintf(buf, sizeof(buf), "Rleg_joint_%02d", j - 6 + 1);    motor_name = buf; }
                                else if (j == 12)  motor_name = "Waist_joint_01";
                                else if (j < 17){ char buf[32]; snprintf(buf, sizeof(buf), "Larm_joint_%02d", j - 13 + 1);    motor_name = buf; }
                                else if (j < 21){ char buf[32]; snprintf(buf, sizeof(buf), "Rarm_joint_%02d", j - 17 + 1);    motor_name = buf; }
                                else if (j == 21)  motor_name = "Head_yaw";
                                else if (j == 22)  motor_name = "Head_pitch";

                                // 在 motor_ref_names 中查找匹配的名称
                                for (size_t k = 0; k < motor_ref_names.size(); ++k) {
                                    if (motor_ref_names[k] == motor_name) {
                                        motor_idx = static_cast<uint8_t>(k);
                                        break;
                                    }
                                }

                                jointId2RuiwoId_.add(joint_id, ruiwo_id, motor_idx);
                                std::cout << "  " << static_cast<int>(joint_id) << "\t\t-> ["
                                          << static_cast<int>(motor_idx) << "]\t\t  " << motor_name << std::endl;
                                kuavo_count++;
                                ruiwo_id++;
                            }
                        }
                        std::cout << "Total Ruiwo motors: " << kuavo_count << std::endl;
                        std::cout << "==================================================\n" << std::endl;
                    } else {
                        // Fallback: 顺序映射 (非 MotorevoActuator 或空列表)
                        std::cout << "\n======== Ruiwo Motor Mapping (sequential fallback) ========" << std::endl;
                        uint8_t ruiwo_index = 0, ruiwo_id = 1;
                        for (size_t i = 0; i < motor_info.driver.size(); ++i) {
                            if (motor_info.driver[i] == RUIWO) {
                                uint8_t joint_id = static_cast<uint8_t>(i + 1);
                                jointId2RuiwoId_.add(joint_id, ruiwo_id, ruiwo_index);
                                std::cout << "  " << static_cast<int>(joint_id) << "\t\t-> " << static_cast<int>(ruiwo_index) << std::endl;
                                ruiwo_index++;
                                ruiwo_id++;
                            }
                        }
                        std::cout << "Total Ruiwo motors: " << static_cast<int>(ruiwo_index) << std::endl;
                        std::cout << "==========================================================\n" << std::endl;
                    }
                }

                // 设置 ruiwo 电机的 kp 和 kd 参数 (使用 name-based 映射)
                if (!kuavo_settings_.running_settings.ruiwo_kp.empty() &&
                    !kuavo_settings_.running_settings.ruiwo_kd.empty()) {

                    std::vector<int> joint_indices;
                    std::vector<double> kp_values, kd_values;

                    int kuavo_idx = 0;
                    for (size_t i = 0; i < motor_info.driver.size(); ++i) {
                        if (motor_info.driver[i] == RUIWO) {
                            if (kuavo_idx < static_cast<int>(kuavo_settings_.running_settings.ruiwo_kp.size())) {
                                // motor_index() 返回正确的 motor_refs_ 索引
                                uint8_t motor_idx = jointId2RuiwoId_.motor_index(static_cast<uint8_t>(i + 1));
                                joint_indices.push_back(motor_idx);
                                kp_values.push_back(static_cast<double>(kuavo_settings_.running_settings.ruiwo_kp[kuavo_idx]));
                                kd_values.push_back(static_cast<double>(kuavo_settings_.running_settings.ruiwo_kd[kuavo_idx]));
                            }
                            kuavo_idx++;
                        }
                    }

                    std::cout << "[HWPlantInit] Setting RuiWo joint gains from config..." << std::endl;
                    ruiwo_actuator->set_joint_gains(joint_indices, kp_values, kd_values);
                    std::cout << "[HWPlantInit] RuiWo joint gains set successfully" << std::endl;
                } else {
                    std::cout << "[HWPlantInit] Warning: ruiwo_kp or ruiwo_kd not found in config, using default values" << std::endl;
                }
                
                if(1 == hardware_param_.teach_pendant_){
                    ruiwo_actuator->set_teach_pendant_mode(1);
                }
            } catch (const std::exception& e) {
                std::cerr << "[HWPlantInit] Exception during RuiWoActuator setup: " << e.what() << std::endl;
                return -1;
            } catch (...) {
                std::cerr << "[HWPlantInit] Unknown exception during RuiWoActuator setup" << std::endl;
                return -1;
            }
        }

#ifdef ENABLE_ECMASTER
        // 统计 EC 电机数量
        std::vector<uint32_t> ecMasterEncoderRange;
        std::vector<uint16_t> ec_master_nil_ids{};
        countECMasters = 0;

        for (size_t i = 0; i < motor_info.driver.size(); ++i)
        {
            if (motor_info.driver[i] == EC_MASTER)
            {
                int id = ec_index_map_[i];
                if (motor_info.motors_exist[i]) {
                    ecMasterEncoderRange.push_back(motor_info.encoder_range[i]);
                    countECMasters ++;
                }
                if(motor_info.motors_disable[i]) {
                    ec_master_nil_ids.push_back(id);
                }
            }
        }
        // 仅在有 EC 电机时初始化 EC_MASTER
        if (countECMasters > 0) {
            actuatorsInterfaceSetup("real", &actuators);
            initRobotModule();
            // add ignore ids for ec master.
            actuators.addIgnore(ec_master_nil_ids.data(), ec_master_nil_ids.size());
            actuators.setEncoderRange(ecMasterEncoderRange.data(), ecMasterEncoderRange.size());

            std::vector<int32_t> robot_running_mode_joint_kp = kuavo_settings_.running_settings.joint_kp;
            std::vector<int32_t> robot_running_mode_joint_kd = kuavo_settings_.running_settings.joint_kd;
            actuators.setJointKp(robot_running_mode_joint_kp);
            actuators.setJointKd(robot_running_mode_joint_kd);

            // only half up body.
            bool flag_half_up_body = kuavo_settings_.running_settings.only_half_up_body;
            int num_leg_ec_motors = motor_info.num_joints - motor_info.num_arm_joints - motor_info.num_head_joints;
            int num_arm_ec_motors = 2;
            if(flag_half_up_body && (num_leg_ec_motors + num_arm_ec_motors) == ecMasterEncoderRange.size()) { // Only Half Up Body
                std::cout  << "-----------------------------------------------------\n";
                std::cout  << "!!!!!!! Only Half Up Body !!!!!!!\n";
                std::cout  << "-----------------------------------------------------\n";
                std::vector<uint16_t> ignore_motor_ids{};
                std::cout << "Half Up Body Ignore Ecmaster Id:";
                for(int id = 1; id <= num_leg_ec_motors; ++id) {
                    motor_info.motors_disable[id-1] = true; // mark leg motor disable
                    ignore_motor_ids.push_back(id);
                    std::cout << id << " ";
                }
                std::cout << std::endl;
                // 设置忽略以下 Id EC_MASTER 电机
                actuators.addIgnore(ignore_motor_ids.data(), ignore_motor_ids.size());
            }
            std::cout << "actuatorsInterfaceSetup DONE \n";
            std::cout << "actuatorsInterfaceSetup countECMasters: " << countECMasters << "\n";

            EcActuatorParams params;
            params.num_actuators = countECMasters;
            params.ec_type = ecmaster_type_;
            params.robot_module = robot_module_;
            params.config_dir = hardware_param_.kuavo_assets_path + "/config/ecmaster/";
            if (actuators.init(params) != 0)
            {
                std::cout << "actuators init failed !\n";
                return -2;
            }
            ec_master_initialized = true;
        } else {
            std::cout << "[HWPlantInit] No EC_MASTER motors or EC-Master unavailable, skipping EtherCAT initialization" << std::endl;
        }
#endif // ENABLE_ECMASTER

#if 0
    exit(0);
    actuators.deInit();
#endif

        double joint_Q[] = {0.2, 1, 1};
        double joint_R[] = {0.2, 2, 4};
        for (uint32_t i = 0; i < num_joint; i++)
        {
            joint_kf.push_back(std::make_unique<Kalman>(3, 3));
            joint_kf[i]->setQR(joint_Q, joint_R);
        }
        // 初始化固定大小卡尔曼状态
        joint_kf3.resize(num_joint);
        for (uint32_t i = 0; i < num_joint; i++) {
            joint_kf3[i].Q = Eigen::Vector3d(0.2, 1, 1).asDiagonal();
            joint_kf3[i].R = Eigen::Vector3d(0.2, 2, 4).asDiagonal();
        }
        sensorsInitHW();

        // 设置电机状态管理器的硬件配置
        setMotorStatusHardwareSettings();

        // 初始化电机状态
        if (motor_status_manager_) 
        {
            GetMotorData(joint_ids, joint_data);
            motor_status_manager_->initMotorStatus(joint_data, joint_ids);
        }

        return 0;
    }
    // bool HardwarePlant::deintialCallback(std_srvs::Trigger::Request &req,
    //                                     std_srvs::Trigger::Response &res)
    // {
    //     std::cout << "HWPlantDeInit Service called!\n";
    //     HWPlantDeInit();
    //     res.success = true;                          // 设置响应字段的值
    //     res.message = "Service called successfully"; // 可选的响应消息
    //     return true;
    // }
    // void HardwarePlant::stopCallback(const std_msgs::Bool::ConstPtr &msg)
    // {
    //     if (msg->data)
    //     {
    //         std::cout << "/stop_robot received!\n";
    //         HWPlantDeInit();
    //     }
    // }
    // bool HardwarePlant::setMotorEncoderOffsetCallback(kuavo_msgs::setMotorEncoderRoundServiceRequest &req, kuavo_msgs::setMotorEncoderRoundServiceResponse &res)
    // {
    //     if (hardware_ready_)
    //     {
    //         res.success = false;
    //         res.message = "hardware is running, setMotorEncoderOffset is prohibited, please stop first!";
    //         return false;
    //     }

    //     bool ret = this->calibrateMotor(req.motor_id, req.direction, req.save_offset);
    //     res.success = ret;
    //     res.message = "";
    //     return true;
    // }
    bool HardwarePlant::setCurrentPositionAsOffset()
    {
        if (hardware_ready_)
        {
            std::cout << "[HardwarePlant]hardware is running, setMotorEncoderOffset is prohibited, please stop first!";
            return false;
        }
        
        std::vector<double> offset;
        getMotorPositionOffset(offset);
        
        GetMotorData(joint_ids, joint_data);

        std::vector<double> ec_positons;
        for (size_t i = 0; i < joint_data.size(); ++i)
        {
            if (motor_info.driver[i] == EC_MASTER)
            {
                ec_positons.push_back(joint_data[i].position);
            }
        }

        std::vector<double> current_pos_remove_offset(num_joint, 0);
        for (size_t i = 0; i < ec_positons.size(); ++i)
        {
            current_pos_remove_offset[i] = ec_positons[i] + offset[i];
            std::cout << "motor_pos_remove_offset[" << i << "]: " << current_pos_remove_offset[i] << std::endl;
            offset[i] = current_pos_remove_offset[i];
        }
        std::string tips_msg = "\033[32m***************正在设置当前位置为编码器零点，请确认当前位置是零点位置***************\n \
        \n输入'c'保存当前位置为零点; \
        \n输入'q'退出; \
        \n输入'p'打印当前位置; \
        \033[0m";
        std::cout << tips_msg << std::endl;
        while (1)
        {
            if (kbhit())
            {
                initial_input_cmd_ = '\0';
                initial_input_cmd_ = getchar();
                if (initial_input_cmd_ == 'q')
                {
                    std::cout << "退出校准\n";
                    return false;
                }
                else if (initial_input_cmd_ == 'c')
                {
                    std::cout << "save offset to file\n";
                    setMotorPositionOffset(offset);
                    saveOffset();
                    return true;
                }
                else if (initial_input_cmd_ == 'p')
                {
                    std::cout << "current position: \n";
                    for (size_t i = 0; i < ec_positons.size(); ++i)
                    {
                        std::cout << current_pos_remove_offset[i] << "\n";
                    }
                    std::cout << std::endl;
                }
                else if (initial_input_cmd_ == '\n' || initial_input_cmd_ == 'h')
                {
                    std::cout << tips_msg << std::endl;
                }
            }
            usleep(100000);
        }

        exit(0);
        // setMotorPositionOffset(offset);
        return false;
    }

    bool HardwarePlant::calibrateMotor(int motor_id, int direction, bool save_offset)
    {
        if (hardware_ready_)
        {
            std::cout << "[HardwarePlant] hardware is running, calibration prohibited\n";
            return false;
        }
        if (motor_id < 0 || motor_id >= static_cast<int>(motor_info.driver.size()))
        {
            std::cout << "[HardwarePlant] motor_id " << motor_id << " out of range " << motor_info.driver.size() << "\n";
            return false;
        }

        if (motor_info.driver[motor_id] == RUIWO)
        {
            // ---- RUIWO/CAN FD 路径 ----
            if (!ruiwo_actuator) return false;
            if (save_offset)
            {
                ruiwo_actuator->saveZeroPosition();
                return true;
            }
            // 全局关节索引 → motor_refs_ 索引 (name-based 映射)
            uint8_t ruiwo_idx = jointId2RuiwoId_.motor_index(motor_id + 1);
            ruiwo_actuator->changeEncoderZeroRound(ruiwo_idx, direction);
            return true;
        }

        // ---- EC_MASTER 路径 (原有逻辑) ----
        std::vector<double> offset;
        getMotorPositionOffset(offset);
        if (motor_id >= static_cast<int>(motor_info.encoder_range.size()))
        {
            std::cout << "[HardwarePlant] motor_id " << motor_id << " out of range " << motor_info.encoder_range.size() << "\n";
            return false;
        }
        if (save_offset)
        {
            std::cout << "[HardwarePlant] save offset to file\n";
            saveOffset();
            return true;
        }

        double degree_per_round = 360.0 / (motor_info.encoder_range[motor_id] / BIT_17);
        std::cout << "motor_id: " << motor_id << " direction: " << direction << " degree_per_round: " << degree_per_round << std::endl;
        std::cout << "offset[" + std::to_string(motor_id) + "]: " << offset[motor_id];
        offset[motor_id] += (direction > 0) ? degree_per_round : -degree_per_round;
        std::cout << " -> " << offset[motor_id] << std::endl;
        setMotorPositionOffset(offset);
        return true;
    }

    SensorData_t HardwarePlant::sensorsInitHW()
    {
        Eigen::Vector3d acc, gyro;
        Eigen::Quaterniond quat;
        std::cout << "sensor init start!!\r\n";

        //  bool flag_only_half_up_body = hardware_param_.only_half_up_body;
        bool flag_only_half_up_body = kuavo_settings_.running_settings.only_half_up_body;
        if(flag_only_half_up_body || "LUNBI" == robot_module_) { // Only Use Half Up Body.
            acc << 0,0,0;
            gyro << 0,0,0;
            quat = Eigen::Quaterniond(1,0,0,0);
        }
        else if (imu_type_ == ImuType::IMU_TYPE_NONE)
        {
            std::cout << "IMU disabled (type=none), skipping IMU init\r\n";
            acc << 0,0,0;
            gyro << 0,0,0;
            quat = Eigen::Quaterniond(1,0,0,0);
        }
        else
        {
            int ret = false;
            if (redundant_imu_)// 初始化两个imu
            {
                ret = HIPNUC_IMU::imu_init();
                ret = xsens_IMU::imu_init();
            }
            else if (imu_type_ == ImuType::IMU_TYPE_HIPNUC)
            {
                ret = HIPNUC_IMU::imu_init(); // TODO: better imu init
            }
            else
            {
                ret = xsens_IMU::imu_init();
            }
            if (ret != 0)
            {
                std::cerr << "imu init failed !\n";
                throw std::runtime_error("imu init failed");
            }
            std::cout << "imu init DONE!!\r\n";
            bool readflag = false;
            if (imu_type_ == ImuType::IMU_TYPE_HIPNUC)
            {
                readflag = HIPNUC_IMU::getImuDataFrame(acc, gyro, quat);
            }
            else
            {
                readflag = xsens_IMU::getImuDataFrame(acc, gyro, quat);
            }
            std::cout << "acc: " << readflag << " " << acc.transpose() << std::endl;
            if (readflag)
            {
                std::cout << "imu init success!!\r\n";
            }
            else
            {
                std::cerr << "get imu data failed !\n";
                throw std::runtime_error("get imu data failed");
            }
        }
        SensorData_t sensor_data_motor;
        for (uint8_t i = 0; i < 5; i++)
        {
            readSensor(sensor_data_motor);
            usleep(dt_ * 1e5);
        }

        return sensor_data_motor;
    }

    void HardwarePlant::setState(SensorData_t &sensor_data_motor,SensorData_t &sensor_data_joint)
    {
        std::lock_guard<std::mutex> lock(motor_joint_data_mtx_);
        sensor_data_motor_last = sensor_data_motor;
        sensor_data_joint_last = sensor_data_joint;
    }
    void HardwarePlant::getState(SensorData_t &sensor_data_motor,SensorData_t &sensor_data_joint)
    {
        std::lock_guard<std::mutex> lock(motor_joint_data_mtx_);
        sensor_data_motor = sensor_data_motor_last;
        sensor_data_joint = sensor_data_joint_last;
    }


    void HardwarePlant::HWPlantDeInit()
    {
        std::cerr << "HardwarePlant deinit start!!\n";
        th_running_ = false;
        hardware_ready_ = false;
        is_deinitialized_ = true;  // 设置析构标志位
        
        // std::vector<double> goal_pos(num_joint, 0);
#ifdef ENABLE_ECMASTER
        if (ec_master_initialized) actuators.deInit();
#endif
        if (ruiwo_actuator != nullptr)
        {
            std::cout << "ruiwo_actuator deinit\n";
            ruiwo_actuator->close();
            ruiwo_actuator = nullptr;
        }

        // 关闭 Revo2 灵巧手
        if (revo2_actuator_) {
            std::cout << "[HWPlantDeInit] revo2_actuator deinit" << std::endl;
            revo2_actuator_->close();
            revo2_actuator_.reset();
        }
        
        std::cout << "imu deinit" << std::endl;
        if (imu_type_ == ImuType::IMU_TYPE_NONE)
        {
            // no-op
        }
        else if (redundant_imu_)
        {
            HIPNUC_IMU::imu_stop();
            xsens_IMU::imu_stop();
        }
        else if (imu_type_ == ImuType::IMU_TYPE_HIPNUC)
            HIPNUC_IMU::imu_stop();
        else
            xsens_IMU::imu_stop();
        std::cout << "HardwarePlant deinit DONE!!\n";
    }

    void HardwarePlant::endEffectorCommand(const std::vector<EndEffectorData>& eef_data)
    {
        if (!revo2_actuator_) return;

        eef_controller::UnsignedDualHandsArray positions{};
        bool has_command = false;

        for (std::size_t hand = 0; hand < eef_data.size() && hand < 2; ++hand) {
            const auto& cmd = eef_data[hand];
            if (cmd.type != EndEffectorType::revo2) continue;

            has_command = true;
            for (int f = 0; f < 6; f++) {
                if (f < cmd.position.size()) {
                    positions[hand][f] = static_cast<uint16_t>(std::min(100.0, std::max(0.0, cmd.position[f])));
                }
            }
        }

        if (has_command) {
            revo2_actuator_->send_position(positions);
        }
    }

    bool HardwarePlant::getEndEffectorState(eef_controller::FingerStatusSnapshot& snapshot)
    {
        if (!revo2_actuator_) return false;

        snapshot = revo2_actuator_->get_finger_status_snapshot();
        return true;
    }

    void HardwarePlant::setDefaultJointPos(std::vector<double> default_joint_pos)
    {
        hardware_param_.default_joint_pos = default_joint_pos;
        defalutJointPos_.resize(hardware_param_.default_joint_pos.size());
        for (int i = 0; i < hardware_param_.default_joint_pos.size(); i++) {
            defalutJointPos_[i] = hardware_param_.default_joint_pos[i];
        }
        vector_t stance_motor_position = ankleSolver.joint_to_motor_position(defalutJointPos_.head(na_foot_));
        std::cout << ">>>>>>>>>>>>>>>>>> stance_motor_position: " << stance_motor_position.transpose()*TO_DEGREE << std::endl;

        std::array<double, 12> leg_pos_array;
        for (int i = 0; i < 12; i++) {
            leg_pos_array[i] = stance_motor_position[i]*TO_DEGREE;
        }
        if (stance_leg_joint_pos_ == nullptr)
        {
            stance_leg_joint_pos_ = std::make_unique<std::array<double, 12>>(leg_pos_array);
        }
        else
        {
            for (int i = 0; i < leg_pos_array.size(); i++) {
                stance_leg_joint_pos_->at(i) = leg_pos_array[i];
            }
        }

        hardware_status_ = 1;  // 解除checkJointSafety入口守卫
    }

    HardwarePlant::HardwarePlant(double dt, HardwareParam hardware_param, uint8_t control_mode, uint16_t num_actuated, uint16_t nq_f, uint16_t nv_f): 
                                        dt_(dt), 
                                        control_mode_{control_mode},
                                        num_actuated_{num_actuated},
                                        nq_f_{nq_f},
                                        nv_f_{nv_f},
                                        rb_version_(4,2)
    {
        hardware_param_ = hardware_param;
        hard_dt = dt_;
        // Get robot version from hardware params
        rb_version_ = hardware_param_.robot_version;
        
        std::string robot_type = "";
        if(IS_KUAVO(rb_version_)) {
            robot_type = "kuavo_v" + rb_version_.to_string();
        }
        else if(IS_ROBAN(rb_version_)) {
            robot_type = "roban_v" + rb_version_.to_string();
        }
        else {
            throw std::runtime_error("Unsupported robot type");
        }        
        kuavo_common_ptr_ = KuavoCommon::getInstancePtr(robot_type, hardware_param_.kuavo_assets_path);
        kuavo_settings_ = kuavo_common_ptr_->getKuavoSettings();
        JSONConfigReader *robot_config = kuavo_common_ptr_->getRobotConfig();
        AnkleSolverType ankleSolverType = static_cast<AnkleSolverType>(robot_config->getValue<int>("ankle_solver_type"));
        ankleSolver.getconfig(ankleSolverType);
        motor_info = kuavo_settings_.hardware_settings;
        imu_invert = motor_info.imu_invert;
        num_joint = kuavo_settings_.hardware_settings.num_joints;
        robot_module_ = kuavo_settings_.hardware_settings.robot_module;
        num_arm_joints = kuavo_settings_.hardware_settings.num_arm_joints;
        num_waist_joints = kuavo_settings_.hardware_settings.num_waist_joints;
        num_head_joints = kuavo_settings_.hardware_settings.num_head_joints;
        ankle_motor_offset = kuavo_settings_.model_settings.ankle_motor_offset_degree * TO_RADIAN;

        std::cout << " get num_joint: " << num_joint << std::endl;
        std::cout << " get num_arm_joints: " << num_arm_joints << std::endl;
        std::cout << " get num_waist_joints: " << num_waist_joints << std::endl;
        std::cout << " get num_head_joints: " << num_head_joints << std::endl;
        na_foot_ = num_joint - num_arm_joints - num_waist_joints - num_head_joints;
        std::cout << "get na_foot num: " << na_foot_ << std::endl;
        nq_ = nq_f_ + na_foot_;
        nv_ = nv_f_ + na_foot_;
        joint_data.resize(num_joint);
        joint_data_old.resize(num_joint);
        joint_cmd.resize(num_joint);
        joint_cmd_old.resize(num_joint);
        joint_ids = motor_info.joint_ids;
        c2t_coeff = motor_info.c2t_coeff_default;
        motor_cul = motor_info.c2t_cul;
        motor_coeff = motor_info.c2t_coeff;
        max_joint_position_limits = motor_info.max_joint_position_limits;
        min_joint_position_limits = motor_info.min_joint_position_limits;
        joint_velocity_limits_ = motor_info.joint_velocity_limits;
        joint_peak_velocity_limits_ = motor_info.joint_peak_velocity_limits;
        joint_lock_rotor_limits_ = motor_info.joint_lock_rotor_limits;
        joint_peak_torque_limits_ = motor_info.joint_peak_limits;
        peakTimeWin_ = motor_info.peak_timeWin;
        lockRotorTimeWin_ = motor_info.lock_rotor_timeWin;
        speedTimeWin_ = motor_info.speed_timeWin;
        std::cout << "current limit[";
        for (int i = 0; i < num_joint; i++)
        {
            std::cout << motor_info.max_current[i] << " ";
        }
        std::cout << "]" <<std::endl;
        std::cout << "c2t [";
        for (const auto& coeff_row : motor_info.c2t_coeff)
        {
            std::cout << "[";
            for (size_t j = 0; j < coeff_row.size(); ++j)
            {
                std::cout << coeff_row[j];
                if (j + 1 < coeff_row.size())
                {
                    std::cout << " ";
                }
            }
            std::cout << "] ";
        }
        std::cout << "]" << std::endl;
        std::cout << "Load protect time window, peak: " << peakTimeWin_ << "| lock rotor: " << lockRotorTimeWin_ << "| speed: " << speedTimeWin_ << std::endl;

        // 禁用电机
        if(1 == hardware_param_.teach_pendant_){

            // ROS_INFO("current is teach_pendant mode");
            std::cout << "current is teach_pendant mode" << std::endl;
            this->kuavo_settings_.running_settings.only_half_up_body = true;
            std::vector<std::string> MOTORS_TYPE = robot_config->getValue<std::vector<std::string>>("MOTORS_TYPE");
            
            for (uint8_t i = 0; i < motor_info.num_joints; i++)
            {
                std::string motor_type_name = MOTORS_TYPE[i];

                if(motor_type_name.size() >= 5){

                    if(motor_type_name.substr(0, 5) != "ruiwo"){

                        if(motor_type_name.substr(motor_type_name.size() - 5, 5) != "_none"){
                            motor_info.motors_exist[i] = true;
                            motor_info.motors_disable[i] = true;
                            // ROS_INFO_STREAM("Disable motor " << motor_type_name);
                        }
                    }
                }
                else{

                    motor_info.motors_exist[i] = true;
                    motor_info.motors_disable[i] = true;
                    // ROS_INFO_STREAM("Disable motor " << motor_type_name);
                }
            }
        }

#ifdef ENABLE_ECMASTER
        {
        int ec_count = 0;
        for (int i = 0; i < motor_info.driver.size(); ++i)
        {
            if (motor_info.driver[i] == EC_MASTER)
            {
                if(motor_info.motors_exist[i]) {
                    ec_count++;
                     ec_index_map_[i] = ec_count; // EC master not exist
                }
                else {
                    ec_index_map_[i] = kEcMasterNilId; // EC master not exist
                }
            }
        }
        }
#endif

        // NOTE: Ruiwo Motor Mapping is now built via name-based matching
        // in the ruiwo_actuator initialization section above (see "Name-based mapping").


        /* only for halfup_body mode */
        if (kuavo_settings_.running_settings.only_half_up_body)
        {
            if (hardware_param_.default_joint_pos.size() == 0)
            {
                stance_leg_joint_pos_ = std::make_unique<std::array<double, 12>>(std::array<double, 12>{});
                std::cout << "no defalut joint pos provide !!!" << std::endl;
            }
            else{
                defalutJointPos_.resize(hardware_param_.default_joint_pos.size());
                for (int i = 0; i < hardware_param_.default_joint_pos.size(); i++) {
                    defalutJointPos_[i] = hardware_param_.default_joint_pos[i];
                }
                vector_t stance_motor_position = ankleSolver.joint_to_motor_position(defalutJointPos_.head(na_foot_));
                std::cout << ">>>>>>>>>>>>>>>>>> stance_motor_position: " << stance_motor_position.transpose()*TO_DEGREE << std::endl;

                std::array<double, 12> leg_pos_array;
                for (int i = 0; i < 12; i++) {
                    leg_pos_array[i] = stance_motor_position[i]*TO_DEGREE;
                }
                stance_leg_joint_pos_ = std::make_unique<std::array<double, 12>>(leg_pos_array);
            }
        }
        /* end */

        ankle_qv_old.setZero();
        ecmaster_type_ = leju::hw::config_utils::GetEcmasterType();
        std::string imu_type_str = leju::hw::config_utils::GetIMUType();
        std::map<std::string, ImuType> imu_type_map = {{"xsens", ImuType::IMU_TYPE_XSENS}, {"hipnuc", ImuType::IMU_TYPE_HIPNUC}, {"none", ImuType::IMU_TYPE_NONE}};
        if (imu_type_map.find(imu_type_str) == imu_type_map.end())
        {
            std::cerr << "IMU type not supported: " << imu_type_str << std::endl;
            exit(1);
        }
        imu_type_ = imu_type_map[imu_type_str];
        if (redundant_imu_)
        {
            imu_type_ = ImuType::IMU_TYPE_XSENS;
        }
        this->setEcmasterDriverType(ecmaster_type_);

        const auto &eef_types = motor_info.end_effector_type;

        std::map<EndEffectorType, std::string> eef_name_maps {
            {EndEffectorType::jodell, "jodell"},
            {EndEffectorType::qiangnao, "qiangnao"},
            {EndEffectorType::qiangnao_touch, "qiangnao_touch"},
            {EndEffectorType::lejuclaw, "lejuclaw"},
            {EndEffectorType::revo2, "revo2"},
            {EndEffectorType::none, "none"}
        };

        end_effector_type_ = eef_name_maps[eef_types[0]];

        // 初始化电机状态管理器
        motor_status_manager_ = std::make_unique<MotorStatusManager>(num_joint, 10, 1000);

        // 初始化 Revo2 灵巧手控制器
        if (end_effector_type_ == "revo2") {
            std::string gesture_file_path = hardware_param_.kuavo_assets_path + "/config/gesture/preset_gestures.json";
#ifdef __aarch64__
            bool is_can_protocol = true;   // aarch64 使用 CAN FD
#else
            bool is_can_protocol = false;  // x86_64 使用串口 (Modbus RS485)
#endif
            revo2_actuator_ = eef_controller::Revo2HandController::Create(gesture_file_path, is_can_protocol);
            if (revo2_actuator_ && revo2_actuator_->init()) {
                std::cout << "[HWPlantInit] Revo2HandController initialized successfully" << std::endl;
            } else {
                std::cerr << "[HWPlantInit] Failed to initialize Revo2HandController" << std::endl;
                revo2_actuator_.reset();
            }
        }

        th_running_ = true;

    }

    size_t HardwarePlant::get_num_actuated() const
    {
        return num_joint;
    }

    void HardwarePlant::cmds2Cmdr(const Eigen::VectorXd &cmd_s, uint32_t na_src, Eigen::VectorXd &cmd_r, uint32_t na_r)
    {
        cmd_r = cmd_s;

        if("LUNBI" != robot_module_ && !bypass_ankle_solver_)
        {
            // foot-->ankle_motor
            Eigen::VectorXd motor_position(na_foot_);
            Eigen::VectorXd motor_velocity(na_foot_);
            Eigen::VectorXd motor_current(na_foot_);
            SensorData_t sensor_data_motor;
            SensorData_t sensor_data_joint;
            getState(sensor_data_motor,sensor_data_joint);
            motor_position = ankleSolver.joint_to_motor_position(cmd_s.segment(0, na_foot_));
            motor_velocity = ankleSolver.joint_to_motor_velocity(sensor_data_joint.joint_q.segment(0, na_foot_),sensor_data_motor.joint_q.segment(0, na_foot_),cmd_s.segment(na_src, na_foot_));
            motor_current = ankleSolver.joint_to_motor_current(sensor_data_joint.joint_q.segment(0, na_foot_),sensor_data_motor.joint_q.segment(0, na_foot_),cmd_s.segment(na_src * 2,na_foot_));
            
            cmd_r.segment(0, na_foot_)<< motor_position;
            cmd_r.segment(na_r, na_foot_) << motor_velocity;
            cmd_r.segment(na_r * 2, na_foot_) << motor_current;
        }

        // tau_max
        cmd_r.segment(na_r * 3, na_r) << cmd_s.segment(na_src * 3, na_src);
        // tau ratio
        cmd_r.segment(na_r * 4, na_r) << cmd_s.segment(na_src * 4, na_src);
    }

    void HardwarePlant::motor2joint(SensorData_t sensor_data_motor, SensorData_t &sensor_data_joint)
    {
        Eigen::VectorXd q_r = sensor_data_motor.joint_q;
        Eigen::VectorXd v_r = sensor_data_motor.joint_v;
        Eigen::VectorXd vd_r = sensor_data_motor.joint_vd;
        Eigen::VectorXd cur_r = sensor_data_motor.joint_current;
        Eigen::VectorXd q_joint(na_foot_);
        Eigen::VectorXd v_joint(na_foot_);
        Eigen::VectorXd tau_joint(na_foot_);
        Eigen::VectorXd joint_c2t(num_joint);
        joint_c2t = GetC2Tcoeff(sensor_data_motor.joint_current);
        sensor_data_joint.acc = sensor_data_motor.acc;
        sensor_data_joint.free_acc = sensor_data_motor.free_acc;
        sensor_data_joint.quat = sensor_data_motor.quat;
        sensor_data_joint.gyro = sensor_data_motor.gyro;
        for (size_t i = 0; i < num_joint; i++)
        {
            cur_r(i) = cur_r(i) * joint_c2t[i];//c2t_coeff[i];
        }
        // num_waist_joints 表示腰部自由度, 电机排布上在下肢电机前面, 临时做法, 腰部不参与数据反馈
        if (!bypass_ankle_solver_) {
            q_joint = ankleSolver.motor_to_joint_position(q_r.head(na_foot_));
            v_joint = ankleSolver.motor_to_joint_velocity(q_joint,q_r.head(na_foot_),v_r.head(na_foot_));
            tau_joint = ankleSolver.motor_to_joint_torque(q_joint,q_r.head(na_foot_),cur_r.head(na_foot_));
            vd_r[4] = (v_joint(4) - ankle_qv_old(0)) / hard_dt;
            vd_r[5] = (v_joint(5) - ankle_qv_old(1)) / hard_dt;
            vd_r[10] = (v_joint(10) - ankle_qv_old(2)) / hard_dt;
            vd_r[11] = (v_joint(11) - ankle_qv_old(3)) / hard_dt;
            ankle_qv_old << v_joint(4), v_joint(5), v_joint(10), v_joint(11);
        } else {
            q_joint = q_r.head(na_foot_);
            v_joint = v_r.head(na_foot_);
            tau_joint = cur_r.head(na_foot_);
        }
        // foot
        sensor_data_joint.joint_q.segment(0, na_foot_) << q_joint;
        sensor_data_joint.joint_v.segment(0, na_foot_) << v_joint;
        sensor_data_joint.joint_vd.segment(0, na_foot_) << vd_r.segment(0, na_foot_);
        sensor_data_joint.joint_current.segment(0, na_foot_) << tau_joint;
        // waist
        sensor_data_joint.joint_q.segment(na_foot_, num_waist_joints) << q_r.segment(na_foot_, num_waist_joints);
        sensor_data_joint.joint_v.segment(na_foot_, num_waist_joints) << v_r.segment(na_foot_, num_waist_joints);
        sensor_data_joint.joint_vd.segment(na_foot_, num_waist_joints) << vd_r.segment(na_foot_, num_waist_joints);
        sensor_data_joint.joint_current.segment(na_foot_, num_waist_joints) << cur_r.segment(na_foot_, num_waist_joints);
        // ARM
        sensor_data_joint.joint_q.segment(na_foot_+num_waist_joints, num_arm_joints + num_head_joints) << sensor_data_motor.joint_q.segment(na_foot_+num_waist_joints, num_arm_joints + num_head_joints);
        sensor_data_joint.joint_v.segment(na_foot_+num_waist_joints, num_arm_joints + num_head_joints) << sensor_data_motor.joint_v.segment(na_foot_+num_waist_joints, num_arm_joints + num_head_joints);
        sensor_data_joint.joint_vd.segment(na_foot_+num_waist_joints, num_arm_joints + num_head_joints) << sensor_data_motor.joint_vd.segment(na_foot_+num_waist_joints, num_arm_joints + num_head_joints);
        sensor_data_joint.joint_current.segment(na_foot_+num_waist_joints, num_arm_joints + num_head_joints) << cur_r.segment(na_foot_+num_waist_joints, num_arm_joints + num_head_joints);
    }

    bool HardwarePlant::GetMotorSensorSnapshot(SensorData_t& out) const
    {
        std::shared_lock<std::shared_mutex> lk(motor_snapshot_mtx_);
        if (!last_motor_sensor_valid_) return false;
        out = last_motor_sensor_;
        return true;
    }

    bool HardwarePlant::GetMotorCmdSnapshot(MotorCmdSnapshot& out) const
    {
        std::shared_lock<std::shared_mutex> lk(motor_snapshot_mtx_);
        if (!last_motor_cmd_.valid) return false;
        out = last_motor_cmd_;
        return true;
    }

    void HardwarePlant::performJointSymmetryCheck()
    {
        const auto pos_list = joint_test_poses::test_pos_list();

        std::cout << "按回车执行下一个动作，按'q'退出检查\n" << std::endl;
        
        int i = 0;
        while (true) {
            initial_input_cmd_ = '\0';
            initial_input_cmd_ = getchar();
            if (initial_input_cmd_ == '\n' ) {
                this->jointMoveTo(std::vector<double>(num_joint, 0), 60, hard_dt);
                usleep(1000000);
                std::cout << "执行第 " << i + 1 << " 组动作..." << std::endl;
                this->jointMoveTo(pos_list[i], 50, hard_dt);
                i++;
                i = i % pos_list.size();
            } else if (initial_input_cmd_ == 'q') {
                std::cout << "退出电机自检" << std::endl;
                break;
            }
        }

        std::cout << "所有动作检查完成！" << std::endl;
        // return to the initial position
        std::cout << "恢复初始姿势..." << std::endl;
        this->jointMoveTo(std::vector<double>(num_joint, 0), 60, hard_dt);
        return;
    }

    /**
     * @brief 翻转imu, 解决imu倒装的问题。让值绕imu自身x轴旋转180度
     *
     * @param acc 原加速度
     * @param gyro 原角速度
     * @param quat 原四元数 [w, x, y, z]
     * @param roll_acc
     * @param roll_gyro
     * @param roll_quat [w, x, y, z]
     *
     * @note roll_quat 赋值是四元数绕当前x轴旋转的一个特解, 没有使用矩阵乘法。
     */
    void Invt_imudate(SensorData_t &sensor_data)
    {
        auto roll_offset = Eigen::AngleAxisd(-M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();
        sensor_data.gyro = roll_offset * sensor_data.gyro;
        sensor_data.acc = roll_offset * sensor_data.acc;
        sensor_data.free_acc = roll_offset * sensor_data.free_acc;
        sensor_data.quat = {-sensor_data.quat[1], sensor_data.quat[0], sensor_data.quat[3], -sensor_data.quat[2]}; // [w, x, y, z]
    }

#ifdef ENABLE_ECMASTER
    void HardwarePlant::initRobotModule()
    {
        if (robot_module_ == "KUAVO") {
            actuators.setRobotMoudle(KUAVO);
        } else if (robot_module_ == "ROBAN2") {
            actuators.setRobotMoudle(ROBAN2);
        }
        else if (robot_module_ == "LUNBI") {
            actuators.setRobotMoudle(LUNBI);
        }
        else {
            std::cerr << "Unsupported robot module: " << robot_module_ << std::endl;
            exit(1);
        }
    }
#endif // ENABLE_ECMASTER

    bool HardwarePlant::changeMotorParam(const std::vector<MotorParam> &motor_params, std::string &err_msg)
    {
#ifdef ENABLE_ECMASTER
        std::vector<uint16_t> ec_ids;
        std::vector<int32_t> ec_kps;
        std::vector<int32_t> ec_kds;
#endif

        // 检查 ID 范围且是否存在
        for(const auto &param : motor_params) {
            // 检查电机ID是否在joint_ids中存在
            if(std::find(joint_ids.begin(), joint_ids.end(), param.id) == joint_ids.end()) {
                err_msg = "Motor ID " + std::to_string(param.id) + " not found in joint_ids";
                return false;
            }

            int index = param.id - 1; // 判断 ID 对应的驱动类型
#ifdef ENABLE_ECMASTER
            if(motor_info.driver[index] == EC_MASTER) {
                if(motor_info.motors_exist[index]) {
                    ec_ids.push_back(ec_index_map_[index]); // 找到实际的 ECMASTER ID
                    ec_kps.push_back(static_cast<int32_t>(param.Kp));
                    ec_kds.push_back(static_cast<int32_t>(param.Kd));
                }
                else {
                    std::cerr << "[HardwarePlant]:changeMotorParam Motor ID " << param.id << " not found in ecmaster\n";
                }
            }
            else {
                // TODO 其他类型电机更改Kp, Kd
            }
#else
            (void)index;
            // TODO 其他类型电机更改Kp, Kd
#endif
        }

#ifdef ENABLE_ECMASTER
        if(!ec_ids.empty() && ec_master_initialized) {
            if(driver_type[0] != EcMasterType::YD) {
                err_msg = "Only support Youda motor";
                return false;
            }

            std::cout << "[HardwarePlant] Writing motor parameters:" << std::endl;
            for(size_t i = 0; i < ec_ids.size(); i++) {
                std::cout << "  Motor ID: " << ec_ids[i]
                         << ", Kp: " << ec_kps[i]
                         << ", Kd: " << ec_kds[i] << std::endl;
            }

            if(0 != actuators.writeJointKd(ec_ids, driver_type[0], ec_kds)) {
                err_msg = "EcDriver writeJointKd failed";
                return false;
            }
            if(0 != actuators.writeJointKp(ec_ids, driver_type[0], ec_kps)) {
                err_msg = "EcDriver writeJointKp failed";
                return false;
            }
        }
#endif // ENABLE_ECMASTER

        return true;
    }

    bool HardwarePlant::getMotorParam(std::vector<MotorParam> &motor_params, std::string &err_msg)
    {
#ifdef ENABLE_ECMASTER
        // Classify IDs based on their types
        std::vector<uint16_t> ecMasterIds;

        for (size_t i = 0; i < joint_ids.size(); ++i)
        {
            size_t index = joint_ids[i] - 1;
            // Check the driver type and add to the appropriate vector
            if (motor_info.driver[index] == EC_MASTER)
            {
                // Only add the motor that exist
                if (motor_info.motors_exist[index]) {
                    ecMasterIds.push_back(ec_index_map_[index]);
                }
            }
        }
#endif // ENABLE_ECMASTER

        motor_params.clear();
        for (size_t i = 0; i < joint_ids.size(); ++i)
        {
            motor_params.push_back(MotorParam{joint_ids[i], 0, 0});
        }

#ifdef ENABLE_ECMASTER
        // int motor_id = 1;
        if(!ecMasterIds.empty() && ec_master_initialized) {
            if(driver_type[0] != EcMasterType::YD) {
                err_msg = "Only support Youda motor";
                return false;
            }
            std::vector<int32_t> joint_kp;
            std::vector<int32_t> joint_kd;
            if(0 != actuators.readJointKp(ecMasterIds, driver_type[0], joint_kp)) {
                err_msg = "EcDriver readJointKp failed";
                return false;
            }
            if(0 != actuators.readJointKd(ecMasterIds, driver_type[0], joint_kd)) {
                err_msg = "EcDriver readJointKd failed";
                return false;
            }

            for(size_t i = 0; i < ecMasterIds.size(); ++i) {
            // ECMASTER ID --> index
            auto it = std::find_if(ec_index_map_.begin(), ec_index_map_.end(),
                [&](const std::pair<int, int>& pair) { return pair.second == ecMasterIds[i]; });

            // index --> joint id
            if (it != ec_index_map_.end()) {
                auto id = it->first + 1;
                auto param_it = std::find_if(motor_params.begin(), motor_params.end(),
                    [&](const MotorParam& param) { return param.id == id; });
                // Found joint id
                if(param_it != motor_params.end()) {
                    param_it->Kp = static_cast<double>(joint_kp[i]);
                    param_it->Kd = static_cast<double>(joint_kd[i]);
                }
                else {
                    err_msg = "MotorParam not found";
                    return false;
                }
            }
            }
        }
#endif // ENABLE_ECMASTER

        // TODO: 先不考虑 Ruiwo 电机

        return true;
    }

    void HardwarePlant::setMotorStatusHardwareSettings() 
    {
        if (motor_status_manager_) 
        {
            motor_status_manager_->setHardwareSettings(motor_info);
        }
    }

    void HardwarePlant::markJointAsDisabled(int joint_id, const std::string& reason)
    {
        if (motor_status_manager_) {
            motor_status_manager_->setJointStatus(joint_id, 
                MotorStatus::DISABLED, reason);
        }
    }

    bool HardwarePlant::isJointMarkedAsDisabled(int joint_id) const
    {
        if (!motor_status_manager_) {
            return false;
        }
        
        auto status_map = motor_status_manager_->getAllJointsStatus();
        auto it = status_map.find(joint_id);
        return (it != status_map.end() && 
               (it->second == MotorStatus::DISABLED || 
                it->second == MotorStatus::ERROR));
    }

    std::map<int, MotorStatus> HardwarePlant::getAllJointsStatus() const
    {
        if (motor_status_manager_) {
            return motor_status_manager_->getAllJointsStatus();
        }
        return std::map<int, MotorStatus>();
    }

    bool HardwarePlant::setZeroTorqueForLegECMotors()
    {
        // 分类腿部/腰部关节 (前 num_joint - num_arm_joints - num_head_joints 个)
        int num_leg_waist = motor_info.num_joints - motor_info.num_arm_joints - motor_info.num_head_joints;

#ifdef ENABLE_ECMASTER
        std::vector<uint8_t> ec_leg_ids;
        std::vector<JointParam_t> zero_torque_cmd;
#endif
        std::vector<int> ruiwo_leg_indices;

        for (int i = 0; i < num_leg_waist && i < static_cast<int>(joint_ids.size()); ++i)
        {
            size_t index = joint_ids[i] - 1;
#ifdef ENABLE_ECMASTER
            if (motor_info.driver[index] == EC_MASTER)
            {
                ec_leg_ids.push_back(joint_ids[i]);
                JointParam_t zero_cmd;
                zero_cmd.position = 0.0;
                zero_cmd.velocity = 0.0;
                zero_cmd.torque = 0.0;
                zero_cmd.maxTorque = 0.0;
                zero_torque_cmd.push_back(zero_cmd);
            }
            else
#endif
            if (motor_info.driver[index] == RUIWO)
            {
                auto ruiwo_idx = jointId2RuiwoId_.motor_index(joint_ids[i]);
                ruiwo_leg_indices.push_back(ruiwo_idx);
            }
        }

#ifdef ENABLE_ECMASTER
        // EC 路径: 设置零力矩
        if (!ec_leg_ids.empty()) {
            SetMotorTorque(ec_leg_ids, zero_torque_cmd);
        }
#endif

        // CAN FD 路径: 设 kp=kd=0 实现零力矩
        if (!ruiwo_leg_indices.empty() && ruiwo_actuator) {
            std::vector<double> zero_gains(ruiwo_leg_indices.size(), 0.0);
            ruiwo_actuator->set_joint_gains(ruiwo_leg_indices, zero_gains, zero_gains);
        }

#ifdef ENABLE_ECMASTER
        if (ec_leg_ids.empty() && ruiwo_leg_indices.empty()) {
#else
        if (ruiwo_leg_indices.empty()) {
#endif
            std::cout << "[HardwarePlant] No leg motors found for zero torque" << std::endl;
            return false;
        }

        return true;
    }

    bool HardwarePlant::exitZeroTorqueMode()
    {
        int leg_joint_count = na_foot_;
        std::cout << "[HardwarePlant] Exiting zero torque mode for leg joints 1-" << na_foot_ << std::endl;
        GetMotorData(joint_ids, joint_data);

#ifdef ENABLE_ECMASTER
        // EC 路径: 恢复 EC_MASTER 腿部关节的位置控制
        {
        std::vector<uint8_t> ec_leg_ids;
        std::vector<JointParam_t> normal_cmd;

        for (int i = 0; i < leg_joint_count && i < static_cast<int>(joint_ids.size()); i++)
        {
            size_t index = joint_ids[i] - 1;
            if (motor_info.driver[index] == EC_MASTER) {
                ec_leg_ids.push_back(joint_ids[i]);
                JointParam_t normal_cmd_item;
                normal_cmd_item.position = joint_data[i].position;
                normal_cmd_item.velocity = 0.0;
                normal_cmd_item.torque = 0.0;
                normal_cmd_item.maxTorque = motor_info.max_current[i];
                normal_cmd.push_back(normal_cmd_item);
            }
        }

        if (!ec_leg_ids.empty()) {
            setDefaultKpKdForEcMaster(normal_cmd, ec_leg_ids);
            SetMotorPosition(ec_leg_ids, normal_cmd);
        }
        }
#endif

        // CAN FD 路径: 恢复 RUIWO 腿部关节的 kp/kd
        if (ruiwo_actuator) {
            int num_leg_waist = motor_info.num_joints - motor_info.num_arm_joints - motor_info.num_head_joints;
            std::vector<int> ruiwo_leg_indices;
            std::vector<double> restore_kp, restore_kd;

            for (int i = 0; i < num_leg_waist && i < static_cast<int>(joint_ids.size()); ++i)
            {
                size_t index = joint_ids[i] - 1;
                if (motor_info.driver[index] == RUIWO) {
                    auto ruiwo_idx = jointId2RuiwoId_.motor_index(joint_ids[i]);
                    ruiwo_leg_indices.push_back(ruiwo_idx);
                    // ruiwo_kp/kd arrays are in kuavo.json order, use `index` (kuavo 0-based) not `ruiwo_idx` (motor_refs_ index)
                    if (index < kuavo_settings_.running_settings.ruiwo_kp.size()) {
                        restore_kp.push_back(kuavo_settings_.running_settings.ruiwo_kp[index]);
                        restore_kd.push_back(kuavo_settings_.running_settings.ruiwo_kd[index]);
                    }
                }
            }

            if (!ruiwo_leg_indices.empty() && restore_kp.size() == ruiwo_leg_indices.size()) {
                ruiwo_actuator->set_joint_gains(ruiwo_leg_indices, restore_kp, restore_kd);
            }
        }

        std::cout << "[HardwarePlant] Normal control mode restored for leg joints" << std::endl;
        return true;
    }

    void HardwarePlant::adjustZeroPosition(int motor_index, double offset) {
        if (getRuiwoActuator() != nullptr) {
            // 将电机索引转换为RUIWO电机索引
            int ruiwo_motor_index = 0;
            for(int i = 0; i < motor_index; i++) {
                if(motor_info.driver[i] == RUIWO){
                    ruiwo_motor_index++;
                }
            }
            
            offset = offset * TO_RADIAN;
            getRuiwoActuator()->adjustZeroPosition(ruiwo_motor_index, offset);
            //ruiwo_actuator->adjustZeroPosition(ruiwo_motor_index, offset);
        }
    }

    std::vector<double> HardwarePlant::getMotorZeroPoints() {
        std::vector<double> zero_points;
        if (getRuiwoActuator() != nullptr) {
            zero_points = getRuiwoActuator()->getMotorZeroPoints();
            //zero_points = ruiwo_actuator->adjustZeroPosition(ruiwo_motor_index, adjust_pos)
            // Convert radians to degrees for RUIWO motors
            std::transform(zero_points.begin(), zero_points.end(), zero_points.begin(),
                        [](double rad) { return rad * TO_DEGREE; });
        }
        return zero_points;
    }

    RuiwoActuatorBase* HardwarePlant::getRuiwoActuator() 
    {
        return ruiwo_actuator;
    }

    //////////////////////////////////////////////////////////////
    int32_t HardwarePlant::readImuData(struct leju::hw::ImuData_t &imu_data) {
        
        if(robot_module_ == "LUNBI") {
            //轮臂无imu提前返回
            return -1; // None
        }

        SensorData_t sensor_data;
        bool flag_only_half_up_body = kuavo_settings_.running_settings.only_half_up_body;
        if(!readImu(flag_only_half_up_body, sensor_data.gyro, sensor_data.acc, sensor_data.quat, sensor_data.free_acc)) {
            return -2; // read imu failed
        }

        if (imu_invert) {
            Invt_imudate(sensor_data);
        }

        // Convert sensor_data.gyro, sensor_data.acc, sensor_data.quat, sensor_data.free_acc To ImuData_t
        imu_data.reset();

        // Convert angular velocity (rad/s) - from Eigen::Vector3d to std::array<double, 3>
        imu_data.gyro[0] = sensor_data.gyro(0);  // gyro_x
        imu_data.gyro[1] = sensor_data.gyro(1);  // gyro_y
        imu_data.gyro[2] = sensor_data.gyro(2);  // gyro_z

        // Convert acceleration (m/s²) - from Eigen::Vector3d to std::array<double, 3>
        imu_data.acc[0] = sensor_data.acc(0);     // acc_x
        imu_data.acc[1] = sensor_data.acc(1);     // acc_y
        imu_data.acc[2] = sensor_data.acc(2);     // acc_z

        // Convert free acceleration (m/s²) - from Eigen::Vector3d to std::array<double, 3>
        imu_data.free_acc[0] = sensor_data.free_acc(0);  // free_acc_x
        imu_data.free_acc[1] = sensor_data.free_acc(1);  // free_acc_y
        imu_data.free_acc[2] = sensor_data.free_acc(2);  // free_acc_z

        // Convert quaternion [w, x, y, z] - from Eigen::Vector4d to std::array<double, 4>
        imu_data.quat[0] = sensor_data.quat[0];  // quat_w
        imu_data.quat[1] = sensor_data.quat[1];  // quat_x
        imu_data.quat[2] = sensor_data.quat[2];  // quat_y
        imu_data.quat[3] = sensor_data.quat[3];  // quat_z

        return 0; // Success
    }

    ///////////////////////////////////////////////////////////////

    void HardwarePlant::convertUserKpKdToMotorGain(int joint_index,
                                                   double kp_user,
                                                   double kd_user,
                                                   double &kp_motor,
                                                   double &kd_motor) const
    {
        if (kd_user <= 1e-9)
        {
            kp_motor = kp_user;
            kd_motor = kd_user;
            return;
        }

        kp_motor = 10.0 * (kp_user / kd_user);

        double H_C = 0.565; // 默认 YD100_50
        if (ecmaster_type_ == "youda3")
        {
            H_C = 0.452; // YD300_40
        }
        else if (ecmaster_type_ == "youda1" or ecmaster_type_ == "youda")
        {
            if (rb_version_.major() == 5 && rb_version_.minor() == 3)
                if (joint_index == 2 || joint_index == 3 || joint_index == 8 || joint_index == 9)
                    H_C = 0.904; // YD300_80
                else if (joint_index == 0 || joint_index == 6)
                    H_C = 0.565; // YD100_50
                else
                    H_C = 0.452; // YD300_40
            else if (IS_KUAVO5_LEGGED(rb_version_))
                if (joint_index == 2 || joint_index == 3 || joint_index == 8 || joint_index == 9)
                    H_C = 0.565; // YD100_50
                else
                    H_C = 0.565; // YD100_50
            else if (IS_KUAVO4_LEGGED(rb_version_))
                H_C = 0.565; // YD100_50
        }

        const double c2t = motor_coeff[joint_index][0];
        kd_motor = kd_user * (2.0 * M_PI * 17895697.0 / H_C * 10.0) /
                   (motor_info.encoder_range[joint_index] * c2t * 1.414);
    }

}; // namespace HighlyDynamic
