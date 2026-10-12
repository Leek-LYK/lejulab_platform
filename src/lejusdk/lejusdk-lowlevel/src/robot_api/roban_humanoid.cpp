#include "lejusdk-lowlevel/robot_api/roban_humanoid.h"
#include "lejusdk-utils/robot_version.hpp"

namespace leju {

RobanHumanoid::RobanHumanoid(const RobotVersion& version) : RobotBaseAPI(version) {
    if(!IS_ROBAN_LEGGED(robot_version_)) {
        std::string version_str = robot_version_.to_string();
        throw std::runtime_error("RobanHumanoid: robot_version '" + version_str + "' is not a supported Roban version");
    }
}

uint8_t RobanHumanoid::getMotorNumber() const {
    return 1+6+6+4+4+2;
}

std::vector<std::string> RobanHumanoid::getMotorNames() const {
     static const std::vector<std::string> roban2_motors = {
        // // 腰部
        // "waist_yaw",
        // // 腿部
        // "l_leg_pitch", "l_leg_roll", "l_leg_yaw", "l_knee", "l_foot_pitch", "l_foot_roll",
        // "r_leg_pitch", "r_leg_roll", "r_leg_yaw", "r_knee", "r_foot_pitch", "r_foot_roll",
        // // 手臂
        // "l_arm_pitch", "l_arm_roll", "l_arm_yaw", "l_forearm",
        // "r_arm_pitch", "r_arm_roll", "r_arm_yaw", "r_forearm",
        // // 头部
        // "head_yaw", "head_pitch"
        
        "leg_l1_joint", "leg_l2_joint", "leg_l3_joint", "leg_l4_joint", "leg_l5_joint", "leg_l6_joint", 
        "leg_r1_joint", "leg_r2_joint", "leg_r3_joint", "leg_r4_joint", "leg_r5_joint", "leg_r6_joint", 
        "waist_yaw_joint",
        "zarm_l1_joint", "zarm_l2_joint", "zarm_l3_joint", "zarm_l4_joint",
        "zarm_r1_joint", "zarm_r2_joint", "zarm_r3_joint", "zarm_r4_joint",
        "zhead_1_link", "zhead_2_link"
    };

    return roban2_motors;
}

std::vector<std::string> RobanHumanoid::getLegJointNames() const {
    static const std::vector<std::string> roban2_leg_joints = {
        "leg_l1_joint", "leg_l2_joint", "leg_l3_joint", "leg_l4_joint", "leg_l5_joint", "leg_l6_joint",
        "leg_r1_joint", "leg_r2_joint", "leg_r3_joint", "leg_r4_joint", "leg_r5_joint", "leg_r6_joint"
    };
    return roban2_leg_joints;
}

std::vector<std::string> RobanHumanoid::getArmJointNames() const {
    // Roban2: 8个手臂关节 (zarm_l/r1~4_joint)
    static const std::vector<std::string> roban2_arm_joints = {
        "zarm_l1_joint", "zarm_l2_joint", "zarm_l3_joint", "zarm_l4_joint",
        "zarm_r1_joint", "zarm_r2_joint", "zarm_r3_joint", "zarm_r4_joint"
    };
    return roban2_arm_joints;
}

std::vector<std::string> RobanHumanoid::getWaistJointNames() const {
    return {"waist_yaw_joint"};
}

std::vector<std::string> RobanHumanoid::getHeadJointNames() const {
    static const std::vector<std::string> roban2_head_joints = {
        "zhead_1_link", "zhead_2_link"
    };
    return roban2_head_joints;
}

}  // namespace leju