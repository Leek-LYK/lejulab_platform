#include "lejusdk-lowlevel/robot_transmission/robot_transmission.h"
#include "robot_transmission.impl.h"
#include <iostream>
namespace leju {
namespace transmission {

RobotTransmission::RobotTransmission(RobotVersion robot_version) {

    auto ankle_type = AnkleSolverType::ANKLE_SOLVER_TYPE_4GEN;

    if(IS_KUAVO4PRO_LEGGED(robot_version)) {
        ankle_type = AnkleSolverType::ANKLE_SOLVER_TYPE_4GEN_PRO;
    }
    else if (IS_KUAVO5_LEGGED(robot_version))
    {
        ankle_type = AnkleSolverType::ANKLE_SOLVER_TYPE_5GEN;
    }
    else if (IS_ROBAN2_LEGGED(robot_version))
    {
        ankle_type = AnkleSolverType::ANKLE_SOLVER_TYPE_S2GEN;
    }
    else if (IS_KUAVO4_LEGGED(robot_version))
    {
        ankle_type = AnkleSolverType::ANKLE_SOLVER_TYPE_4GEN;
    }
    else {
        std::string version_str = robot_version.to_string();
        std::cout << "WARN: RobotVersion not supported: " << version_str << std::endl;
        throw std::runtime_error("RobotVersion not supported: " + version_str);
    }    

    pimpl_ = new RobotTransmissionImpl(ankle_type);
}

RobotTransmission::vector_t RobotTransmission::joint_to_motor_position(const vector_t &q) {
    Eigen::VectorXd q_eigen = Eigen::Map<const Eigen::VectorXd>(q.data(), q.size());
    Eigen::VectorXd result = pimpl_->joint_to_motor_position(q_eigen);
    return vector_t(result.data(), result.data() + result.size());
}

RobotTransmission::vector_t RobotTransmission::joint_to_motor_velocity(const vector_t &q, const vector_t &p, const vector_t &v) {
    Eigen::VectorXd q_eigen = Eigen::Map<const Eigen::VectorXd>(q.data(), q.size());
    Eigen::VectorXd p_eigen = Eigen::Map<const Eigen::VectorXd>(p.data(), p.size());
    Eigen::VectorXd v_eigen = Eigen::Map<const Eigen::VectorXd>(v.data(), v.size());
    Eigen::VectorXd result = pimpl_->joint_to_motor_velocity(q_eigen, p_eigen, v_eigen);
    return vector_t(result.data(), result.data() + result.size());
}

RobotTransmission::vector_t RobotTransmission::joint_to_motor_torque(const vector_t &q, const vector_t &p, const vector_t &t) {
    Eigen::VectorXd q_eigen = Eigen::Map<const Eigen::VectorXd>(q.data(), q.size());
    Eigen::VectorXd p_eigen = Eigen::Map<const Eigen::VectorXd>(p.data(), p.size());
    Eigen::VectorXd t_eigen = Eigen::Map<const Eigen::VectorXd>(t.data(), t.size());
    Eigen::VectorXd result = pimpl_->joint_to_motor_torque(q_eigen, p_eigen, t_eigen);
    return vector_t(result.data(), result.data() + result.size());
}

RobotTransmission::vector_t RobotTransmission::motor_to_joint_position(const vector_t &p) {
    Eigen::VectorXd p_eigen = Eigen::Map<const Eigen::VectorXd>(p.data(), p.size());
    Eigen::VectorXd result = pimpl_->motor_to_joint_position(p_eigen);
    return vector_t(result.data(), result.data() + result.size());
}

RobotTransmission::vector_t RobotTransmission::motor_to_joint_velocity(const vector_t &q, const vector_t &p, const vector_t &dp) {
    Eigen::VectorXd q_eigen = Eigen::Map<const Eigen::VectorXd>(q.data(), q.size());
    Eigen::VectorXd p_eigen = Eigen::Map<const Eigen::VectorXd>(p.data(), p.size());
    Eigen::VectorXd dp_eigen = Eigen::Map<const Eigen::VectorXd>(dp.data(), dp.size());
    Eigen::VectorXd result = pimpl_->motor_to_joint_velocity(q_eigen, p_eigen, dp_eigen);
    return vector_t(result.data(), result.data() + result.size());
}

RobotTransmission::vector_t RobotTransmission::motor_to_joint_torque(const vector_t &q, const vector_t &p, const vector_t &c) {
    Eigen::VectorXd q_eigen = Eigen::Map<const Eigen::VectorXd>(q.data(), q.size());
    Eigen::VectorXd p_eigen = Eigen::Map<const Eigen::VectorXd>(p.data(), p.size());
    Eigen::VectorXd c_eigen = Eigen::Map<const Eigen::VectorXd>(c.data(), c.size());
    Eigen::VectorXd result = pimpl_->motor_to_joint_torque(q_eigen, p_eigen, c_eigen);
    return vector_t(result.data(), result.data() + result.size());
}

} // namespace transmission
} // namespace leju