#include "lejusdk-lowlevel/robot_api/kuavo_humanoid.h"
#include "lejusdk-lowlevel/leju_sdk.h"
#include "lejusdk-utils/robot_version.hpp"
namespace leju {

KuavoHumanoid::KuavoHumanoid(const RobotVersion& version) : RobotBaseAPI(version) {
    if(!IS_KUAVO_LEGGED(robot_version_)) {
        std::string version_str = robot_version_.to_string();
        throw std::runtime_error("KuavoHumanoid: robot_version '" + version_str + "' is not a supported Kuavo version");
    }
}

uint8_t KuavoHumanoid::getMotorNumber() const {
    // 根据Kuavo版本确定电机数量
    if (IS_KUAVO4_LEGGED(robot_version_)) {
        // Kuavo 4th: leg+leg+arm+arm+head
        return 6 + 6 + 7 + 7 + 2;  // = 28
    } else if (IS_KUAVO5_LEGGED(robot_version_)) {
        // Kuavo 5th: waist+leg+leg+arm+arm+head
        return 1 + 6 + 6 + 7 + 7 + 2;  // = 29
    }

    // WARN! default 4Gen
    return 28;
}

std::vector<std::string> KuavoHumanoid::getMotorNames() const {

    static const std::vector<std::string> kuavo4_motors = {
        // // 左腿
        // "l_leg_roll", "l_leg_yaw", "l_leg_pitch", "l_knee", "l_foot_pitch", "l_foot_roll",
        // // 右腿
        // "r_leg_roll", "r_leg_yaw", "r_leg_pitch", "r_knee", "r_foot_pitch", "r_foot_roll",
        // // 左臂
        // "l_arm_pitch", "l_arm_roll", "l_arm_yaw", "l_forearm", "l_hand_yaw", "l_hand_pitch", "l_hand_roll",
        // // 右臂
        // "r_arm_pitch", "r_arm_roll", "r_arm_yaw", "r_forearm", "r_hand_yaw", "r_hand_pitch", "r_hand_roll",
        // // 头部
        // "head_yaw", "head_pitch"
        "leg_l1_joint", "leg_l2_joint", "leg_l3_joint", "leg_l4_joint", "leg_l5_joint", "leg_l6_joint", 
        "leg_r1_joint", "leg_r2_joint", "leg_r3_joint", "leg_r4_joint", "leg_r5_joint", "leg_r6_joint", 
        "zarm_l1_joint", "zarm_l2_joint", "zarm_l3_joint", "zarm_l4_joint", "zarm_l5_joint", "zarm_l6_joint", "zarm_l7_joint", 
        "zarm_r1_joint", "zarm_r2_joint", "zarm_r3_joint", "zarm_r4_joint", "zarm_r5_joint", "zarm_r6_joint", "zarm_r7_joint",
        "zhead_1_link", "zhead_2_link"
    };

    static const std::vector<std::string> kuavo4pro_motors = {
        // // 左腿
        // "l_leg_roll", "l_leg_yaw", "l_leg_pitch", "l_knee", "l_foot_pitch", "l_foot_roll",
        // // 右腿
        // "r_leg_roll", "r_leg_yaw", "r_leg_pitch", "r_knee", "r_foot_pitch", "r_foot_roll",
        // // 左臂
        // "l_arm_pitch", "l_arm_roll", "l_arm_yaw", "l_forearm", "l_hand_yaw", "l_hand_roll", "l_hand_pitch",
        // // 右臂
        // "r_arm_pitch", "r_arm_roll", "r_arm_yaw", "r_forearm", "r_hand_yaw", "r_hand_roll", "r_hand_pitch",
        // // 头部
        // "head_yaw", "head_pitch"
        "leg_l1_joint", "leg_l2_joint", "leg_l3_joint", "leg_l4_joint", "leg_l5_joint", "leg_l6_joint", 
        "leg_r1_joint", "leg_r2_joint", "leg_r3_joint", "leg_r4_joint", "leg_r5_joint", "leg_r6_joint", 
        "zarm_l1_joint", "zarm_l2_joint", "zarm_l3_joint", "zarm_l4_joint", "zarm_l5_joint", "zarm_l6_joint", "zarm_l7_joint", 
        "zarm_r1_joint", "zarm_r2_joint", "zarm_r3_joint", "zarm_r4_joint", "zarm_r5_joint", "zarm_r6_joint", "zarm_r7_joint",
        "zhead_1_link", "zhead_2_link"
    };

    static const std::vector<std::string> kuavo5_motors = {
        // 左腿
        "l_leg_roll", "l_leg_yaw", "l_leg_pitch", "l_knee", "l_foot_pitch", "l_foot_roll",
        // 右腿
        "r_leg_roll", "r_leg_yaw", "r_leg_pitch", "r_knee", "r_foot_pitch", "r_foot_roll",
        // 腰部
        "waist_yaw",
        // 左臂
        "l_arm_pitch", "l_arm_roll", "l_arm_yaw", "l_forearm", "l_hand_yaw", "l_hand_roll", "l_hand_pitch",
        // 右臂
        "r_arm_pitch", "r_arm_roll", "r_arm_yaw", "r_forearm", "r_hand_yaw", "r_hand_roll", "r_hand_pitch",
        // 头部
        "head_yaw", "head_pitch"
    };

    // 根据Kuavo版本返回对应的电机名称
    if (IS_KUAVO4PRO_LEGGED(robot_version_)) {
        return kuavo4pro_motors;
    } else if (IS_KUAVO5_LEGGED(robot_version_)) {
        return kuavo5_motors;
    }
    
    // 默认返回4代配置
    return kuavo4_motors;
}

std::vector<std::string> KuavoHumanoid::getLegJointNames() const {
    // Kuavo4/4Pro
    static const std::vector<std::string> kuavo4_leg_joints = {
        "leg_l1_joint", "leg_l2_joint", "leg_l3_joint", "leg_l4_joint", "leg_l5_joint", "leg_l6_joint",
        "leg_r1_joint", "leg_r2_joint", "leg_r3_joint", "leg_r4_joint", "leg_r5_joint", "leg_r6_joint"
    };

    // Kuavo5
    static const std::vector<std::string> kuavo5_leg_joints = {
        "l_leg_roll", "l_leg_yaw", "l_leg_pitch", "l_knee", "l_foot_pitch", "l_foot_roll",
        "r_leg_roll", "r_leg_yaw", "r_leg_pitch", "r_knee", "r_foot_pitch", "r_foot_roll"
    };

    if (IS_KUAVO5_LEGGED(robot_version_)) {
        return kuavo5_leg_joints;
    }
    return kuavo4_leg_joints;
}

std::vector<std::string> KuavoHumanoid::getArmJointNames() const {
    // Kuavo4/4Pro: zarm_l/r1~7_joint
    static const std::vector<std::string> kuavo4_arm_joints = {
        "zarm_l1_joint", "zarm_l2_joint", "zarm_l3_joint", "zarm_l4_joint", "zarm_l5_joint", "zarm_l6_joint", "zarm_l7_joint",
        "zarm_r1_joint", "zarm_r2_joint", "zarm_r3_joint", "zarm_r4_joint", "zarm_r5_joint", "zarm_r6_joint", "zarm_r7_joint"
    };

    // Kuavo5: l/r_arm_pitch 等
    static const std::vector<std::string> kuavo5_arm_joints = {
        "l_arm_pitch", "l_arm_roll", "l_arm_yaw", "l_forearm", "l_hand_yaw", "l_hand_roll", "l_hand_pitch",
        "r_arm_pitch", "r_arm_roll", "r_arm_yaw", "r_forearm", "r_hand_yaw", "r_hand_roll", "r_hand_pitch"
    };

    if (IS_KUAVO5_LEGGED(robot_version_)) {
        return kuavo5_arm_joints;
    }
    return kuavo4_arm_joints;
}

std::vector<std::string> KuavoHumanoid::getWaistJointNames() const {
    // Kuavo4/4Pro: 无腰部关节
    // Kuavo5: waist_yaw
    if (IS_KUAVO5_LEGGED(robot_version_)) {
        return {"waist_yaw"};
    }
    return {};
}

std::vector<std::string> KuavoHumanoid::getHeadJointNames() const {
    // Kuavo4/4Pro: zhead_1_link, zhead_2_link
    static const std::vector<std::string> kuavo4_head_joints = {
        "zhead_1_link", "zhead_2_link"
    };

    // Kuavo5: head_yaw, head_pitch
    static const std::vector<std::string> kuavo5_head_joints = {
        "head_yaw", "head_pitch"
    };

    if (IS_KUAVO5_LEGGED(robot_version_)) {
        return kuavo5_head_joints;
    }
    return kuavo4_head_joints;
}

}  // namespace leju