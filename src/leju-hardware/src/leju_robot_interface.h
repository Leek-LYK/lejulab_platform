#ifndef _LEJU_ROBOT_INTERFACE_H_
#define _LEJU_ROBOT_INTERFACE_H_

#include <functional>
#include <memory>
#include <atomic>
#include <mutex>

#include "lejusdk-utils/loop.hpp"
#include "lejusdk-lowlevel/data_types.h"
#include "lejusdk-lowlevel/robot_api/robot_base.h"

namespace leju {

class RobotInterface {
public:
    explicit RobotInterface();
    virtual ~RobotInterface();

    /**
     * @brief Initialize the robot interface
     *
     * @return true if initialization successful, false otherwise
     */
    bool initialize();

    /**
     * @brief Publishes the robot joint state
     *
     * @param joint_state Robot joint state data
     * @return true if publish successful, false otherwise
     */
    bool publishJointState(const RobotState& joint_state);

    bool publishMotorState(const RobotState& motor_state);
    bool publishMotorCmd(const RobotCmd& motor_cmd);
    bool publishHandState(const HandState& hand_state);

    /**
     * @brief Publishes IMU sensor data
     *
     * @param imu_data IMU sensor data
     * @return true if publish successful, false otherwise
     */
    bool publishImuData(const ImuData& imu_data);


    bool publishHardwareState();

    /**
     * @brief Publishes stop robot command (protection mode / emergency stop)
     *
     * @return true if publish successful, false otherwise
     */
    bool publishStopRobot();

    /**
     * @brief Checks if stop command has been received
     *
     * @return true if stop command received, false otherwise
     */
    bool isStopReceived() const;

    /**
     * @brief Sets the stop received flag to true
     *
     */
    void setStopReceived();    
    
protected:
    void subscribeRobotCmd(const RobotCmdCallback& callback);
    void subscribeHandCmd(const HandCmdCallback& callback);
    void subscribeJoyData(const JoyDataCallback& callback);
    virtual void onStopCommandReceived(const StringDataConstPtr& msg);
    std::atomic<leju::HardwareState> hardware_state_;
    
private:
    // stop received flag
    std::atomic<bool> stop_received_;
    RobotBaseAPI* robot_api_ = nullptr;
    std::unique_ptr<leju::Loop> hw_state_loop_;
};

} // namespace leju

#endif // _LEJU_ROBOT_INTERFACE_H_
