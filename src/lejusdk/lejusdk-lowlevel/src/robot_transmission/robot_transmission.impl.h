#ifndef _robot_transmission_h_
#define _robot_transmission_h_

#include <iostream>
#include <memory>

#include "Eigen/Dense"

namespace leju {
namespace transmission {

class RobanAnkleSolver;

enum class AnkleSolverType {
  ANKLE_SOLVER_TYPE_4GEN = 0,
  ANKLE_SOLVER_TYPE_4GEN_PRO = 1,
  ANKLE_SOLVER_TYPE_5GEN = 2,
  ANKLE_SOLVER_TYPE_S1GEN = 3,
  ANKLE_SOLVER_TYPE_S2GEN = 4,
  ANKLE_SOLVER_TYPE_S2GEN_2 = 5
};

class RobotTransmissionImpl {
 public:
 explicit RobotTransmissionImpl(AnkleSolverType ankle_solver_type);
  ~RobotTransmissionImpl();
  void setRobotVersion(AnkleSolverType ankle_solver_type);

  Eigen::VectorXd joint_to_motor_position(const Eigen::VectorXd &q);
  Eigen::VectorXd joint_to_motor_velocity(const Eigen::VectorXd &q, const Eigen::VectorXd &p,
                                          const Eigen::VectorXd &v);
  Eigen::VectorXd joint_to_motor_torque(const Eigen::VectorXd &q, const Eigen::VectorXd &p,
                                        const Eigen::VectorXd &t);
  Eigen::VectorXd motor_to_joint_position(const Eigen::VectorXd &p);
  Eigen::VectorXd motor_to_joint_velocity(const Eigen::VectorXd &q, const Eigen::VectorXd &p,
                                          const Eigen::VectorXd &dp);
  Eigen::VectorXd motor_to_joint_torque(const Eigen::VectorXd &q, const Eigen::VectorXd &p,
                                        const Eigen::VectorXd &c);

 private:
  Eigen::VectorXd joint_to_motor_position_all_(const Eigen::VectorXd &q);
  Eigen::VectorXd joint_to_motor_velocity_all_(const Eigen::VectorXd &q, const Eigen::VectorXd &p,
                                          const Eigen::VectorXd &v);
  Eigen::VectorXd joint_to_motor_torque_all_(const Eigen::VectorXd &q, const Eigen::VectorXd &p,
                                        const Eigen::VectorXd &t);
  Eigen::VectorXd motor_to_joint_position_all_(const Eigen::VectorXd &p);
  Eigen::VectorXd motor_to_joint_velocity_all_(const Eigen::VectorXd &q, const Eigen::VectorXd &p,
                                          const Eigen::VectorXd &dp);
  Eigen::VectorXd motor_to_joint_torque_all_(const Eigen::VectorXd &q, const Eigen::VectorXd &p,
                                        const Eigen::VectorXd &c);
  Eigen::VectorXd joint_to_motor_position_(const Eigen::VectorXd &q);
  Eigen::VectorXd joint_to_motor_velocity_(const Eigen::VectorXd &q, const Eigen::VectorXd &p,
                                           const Eigen::VectorXd &v);
  Eigen::VectorXd joint_to_motor_torque_(const Eigen::VectorXd &q, const Eigen::VectorXd &p,
                                          const Eigen::VectorXd &t);
  Eigen::VectorXd motor_to_joint_position_(const Eigen::VectorXd &p);
  Eigen::VectorXd motor_to_joint_velocity_(const Eigen::VectorXd &q, const Eigen::VectorXd &p,
                                           const Eigen::VectorXd &dp);
  Eigen::VectorXd motor_to_joint_torque_(const Eigen::VectorXd &q, const Eigen::VectorXd &p,
                                         const Eigen::VectorXd &c);

  Eigen::VectorXd joint_to_motor_position_pro_(const Eigen::VectorXd &q);
  Eigen::VectorXd joint_to_motor_velocity_pro_(const Eigen::VectorXd &q, const Eigen::VectorXd &p,
                                               const Eigen::VectorXd &v);
  Eigen::VectorXd joint_to_motor_torque_pro_(const Eigen::VectorXd &q, const Eigen::VectorXd &p,
                                              const Eigen::VectorXd &t);
  Eigen::VectorXd motor_to_joint_position_pro_(const Eigen::VectorXd &p);
  Eigen::VectorXd motor_to_joint_velocity_pro_(const Eigen::VectorXd &q, const Eigen::VectorXd &p,
                                               const Eigen::VectorXd &dp);
  Eigen::VectorXd motor_to_joint_torque_pro_(const Eigen::VectorXd &q, const Eigen::VectorXd &p,
                                             const Eigen::VectorXd &c);
  Eigen::VectorXd joint_to_motor_position_s2_(const Eigen::VectorXd &q);
  Eigen::VectorXd joint_to_motor_velocity_s2_(const Eigen::VectorXd &q, const Eigen::VectorXd &p,
                                              const Eigen::VectorXd &v);
  Eigen::VectorXd joint_to_motor_torque_s2_(const Eigen::VectorXd &q, const Eigen::VectorXd &p,
                                             const Eigen::VectorXd &t);
  Eigen::VectorXd motor_to_joint_position_s2_(const Eigen::VectorXd &p);
  Eigen::VectorXd motor_to_joint_velocity_s2_(const Eigen::VectorXd &q, const Eigen::VectorXd &p,
                                              const Eigen::VectorXd &dp);
  Eigen::VectorXd motor_to_joint_torque_s2_(const Eigen::VectorXd &q, const Eigen::VectorXd &p,
                                            const Eigen::VectorXd &c);

  Eigen::VectorXd joint_to_motor_position_s2_2_(const Eigen::VectorXd &q);
  Eigen::VectorXd joint_to_motor_velocity_s2_2_(const Eigen::VectorXd &q, const Eigen::VectorXd &p,
                                                const Eigen::VectorXd &v);
  Eigen::VectorXd joint_to_motor_torque_s2_2_(const Eigen::VectorXd &q, const Eigen::VectorXd &p,
                                              const Eigen::VectorXd &t);
  Eigen::VectorXd motor_to_joint_position_s2_2_(const Eigen::VectorXd &p);
  Eigen::VectorXd motor_to_joint_velocity_s2_2_(const Eigen::VectorXd &q, const Eigen::VectorXd &p,
                                                const Eigen::VectorXd &dp);
  Eigen::VectorXd motor_to_joint_torque_s2_2_(const Eigen::VectorXd &q, const Eigen::VectorXd &p,
                                              const Eigen::VectorXd &c);

  void applyRollLimitBasedOnPitch(Eigen::VectorXd &joint_q);

 private:
  enum WaistType {
    ERROR_WAIST,
    NO_WAIST,
    ROBAN_WAIST,
  };
  AnkleSolverType ankle_solver_type_ = AnkleSolverType::ANKLE_SOLVER_TYPE_4GEN;
  WaistType waist_type_;
  Eigen::VectorXd config;
  int N_ITER = 10;
  Eigen::Vector2d ankle_pitch_limits_;
  Eigen::Vector2d ankle_roll_limits_;
  std::unique_ptr<RobanAnkleSolver> roban_solver_;
};

} // namespace transmission
} // namespace leju
#endif
