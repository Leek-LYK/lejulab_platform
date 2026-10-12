/**
 * @file test_vr_api.cpp
 * @brief VR API 单元测试
 */

#include <gtest/gtest.h>
#include <lejusdk-vr/lejusdk_vr.h>

#include <chrono>
#include <thread>

// 内部头文件，用于测试时直接访问通信层
#include "../src/communication/vr_communication.h"
#include "../src/communication/vr_msg_converter.hpp"

using namespace leju::vr;

class VRAPITest : public ::testing::Test {
 protected:
  void SetUp() override {
    // 确保每个测试开始时通信层处于初始化状态
    leju::vr::VRCommunication::getInstance().initialize();
  }

  void TearDown() override {
    // 测试后不调用 shutdown()，保持单例处于可用状态
    // 如果某个测试调用了 shutdown()，下一个测试的 SetUp 会重新初始化
  }
};

// 测试数据类型
TEST_F(VRAPITest, DataTypes_JointTrajectoryPoint) {
  JointTrajectoryPoint traj;
  traj.q = {0.1, 0.2, 0.3};
  traj.v = {0.01, 0.02, 0.03};
  traj.acc = {0.001, 0.002, 0.003};

  EXPECT_EQ(traj.q.size(), 3);
  EXPECT_EQ(traj.v.size(), 3);
  EXPECT_EQ(traj.acc.size(), 3);
}

TEST_F(VRAPITest, DataTypes_VelocityCmd) {
  VelocityCmd cmd;
  cmd.linear_x = 0.5;
  cmd.linear_y = 0.1;
  cmd.angular_z = 0.2;
  cmd.timestamp = 123.456;

  EXPECT_DOUBLE_EQ(cmd.linear_x, 0.5);
  EXPECT_DOUBLE_EQ(cmd.linear_y, 0.1);
  EXPECT_DOUBLE_EQ(cmd.angular_z, 0.2);
  EXPECT_DOUBLE_EQ(cmd.timestamp, 123.456);
}

TEST_F(VRAPITest, DataTypes_VrVelocityCmd) {
  VrVelocityCmd cmd;
  cmd.linear_x = 0.6;
  cmd.linear_y = -0.2;
  cmd.angular_z = 0.3;
  cmd.timestamp = 456.789;

  EXPECT_DOUBLE_EQ(cmd.linear_x, 0.6);
  EXPECT_DOUBLE_EQ(cmd.linear_y, -0.2);
  EXPECT_DOUBLE_EQ(cmd.angular_z, 0.3);
  EXPECT_DOUBLE_EQ(cmd.timestamp, 456.789);
}

TEST_F(VRAPITest, DataTypes_QuestStructs) {
  Pose pose;
  pose.x = 1.0f;
  pose.qw = 1.0f;

  QuestBonePosesData bones;
  bones.header_sec = 12;
  bones.timestamp_ms = 345;
  bones.is_high_confidence = true;
  bones.poses.push_back(pose);

  QuestJoystickData joy;
  joy.header_sec = 2;
  joy.left_x = 0.3f;
  joy.right_second_button_pressed = true;

  EXPECT_EQ(bones.header_sec, 12);
  EXPECT_EQ(bones.timestamp_ms, 345);
  EXPECT_TRUE(bones.is_high_confidence);
  EXPECT_EQ(bones.poses.size(), 1);
  EXPECT_FLOAT_EQ(joy.left_x, 0.3f);
  EXPECT_TRUE(joy.right_second_button_pressed);
}

TEST_F(VRAPITest, DataTypes_ControlMode) {
  EXPECT_EQ(static_cast<int>(ControlMode::kKeepPose), 0);
  EXPECT_EQ(static_cast<int>(ControlMode::kAuto), 1);
  EXPECT_EQ(static_cast<int>(ControlMode::kExternal), 2);
}

TEST_F(VRAPITest, DataTypes_ControllerState) {
  ControllerState state;
  state.current_controller = "amp";
  state.arm_mode = ControlMode::kExternal;
  state.head_mode = ControlMode::kAuto;
  state.waist_mode = ControlMode::kAuto;
  state.available_controllers = {"amp", "mimic"};

  EXPECT_EQ(state.current_controller, "amp");
  EXPECT_EQ(state.arm_mode, ControlMode::kExternal);
  EXPECT_EQ(state.head_mode, ControlMode::kAuto);
  EXPECT_EQ(state.waist_mode, ControlMode::kAuto);
  EXPECT_EQ(state.available_controllers.size(), 2);
}

// 测试 KuavoVRAPI 构造和 RobotVersion
TEST_F(VRAPITest, KuavoVRAPI_Construction) {
  KuavoVRAPI api;
  EXPECT_TRUE(IS_KUAVO(api.getRobotVersion()));
  EXPECT_TRUE(api.isInitialized());
}

// 测试 RobanVRAPI 构造和 RobotVersion
TEST_F(VRAPITest, RobanVRAPI_Construction) {
  RobanVRAPI api;
  EXPECT_TRUE(IS_ROBAN(api.getRobotVersion()));
  EXPECT_TRUE(api.isInitialized());
}

// 测试带自定义版本参数的 KuavoVRAPI 构造
TEST_F(VRAPITest, KuavoVRAPI_CustomVersion) {
  KuavoVRAPI api(leju::RobotVersions::KUAVO5_BASE);
  EXPECT_TRUE(IS_KUAVO(api.getRobotVersion()));
  EXPECT_EQ(api.getRobotVersion(), leju::RobotVersions::KUAVO5_BASE);
  EXPECT_TRUE(api.isInitialized());
}

// 测试带自定义版本参数的 RobanVRAPI 构造
TEST_F(VRAPITest, RobanVRAPI_CustomVersion) {
  RobanVRAPI api(leju::RobotVersions::ROBAN2_BASE);
  EXPECT_TRUE(IS_ROBAN(api.getRobotVersion()));
  EXPECT_EQ(api.getRobotVersion(), leju::RobotVersions::ROBAN2_BASE);
  EXPECT_TRUE(api.isInitialized());
}

// 测试 Handler 类型可赋值 lambda
TEST_F(VRAPITest, TypeDefs_Handlers) {
  // SwitchControllerHandler
  SwitchControllerHandler switch_handler =
      [](const std::string& name, std::string& message) -> bool {
    message = "Switched to " + name;
    return true;
  };
  std::string msg;
  EXPECT_TRUE(switch_handler("amp", msg));
  EXPECT_EQ(msg, "Switched to amp");

  // SetModeHandler
  SetModeHandler mode_handler =
      [](ControlMode mode, std::string& message) -> bool {
    message = "Mode set to " + std::to_string(static_cast<int>(mode));
    return mode == ControlMode::kExternal;
  };
  EXPECT_TRUE(mode_handler(ControlMode::kExternal, msg));
  EXPECT_FALSE(mode_handler(ControlMode::kAuto, msg));

  // GetControllerStateHandler
  GetControllerStateHandler state_handler = []() -> ControllerState {
    ControllerState state;
    state.current_controller = "mimic";
    state.arm_mode = ControlMode::kExternal;
    state.available_controllers = {"amp", "mimic"};
    return state;
  };
  ControllerState result = state_handler();
  EXPECT_EQ(result.current_controller, "mimic");
  EXPECT_EQ(result.arm_mode, ControlMode::kExternal);
}

// 测试 Callback 类型可赋值 lambda
TEST_F(VRAPITest, TypeDefs_Callbacks) {
  bool joint_callback_called = false;
  JointTrajectoryPoint received_joint;

  // ArmTrajectoryCallback
  ArmTrajectoryCallback joint_cb =
      [&](const JointTrajectoryPoint& cmd) {
        joint_callback_called = true;
        received_joint = cmd;
      };

  JointTrajectoryPoint test_cmd;
  test_cmd.q = {0.1, 0.2, 0.3};
  test_cmd.v = {0.01, 0.02, 0.03};
  joint_cb(test_cmd);

  EXPECT_TRUE(joint_callback_called);
  EXPECT_EQ(received_joint.q.size(), 3);
  EXPECT_DOUBLE_EQ(received_joint.q[0], 0.1);

  bool vel_callback_called = false;
  VelocityCmd received_vel;

  // VelocityCmdCallback
  VelocityCmdCallback vel_cb = [&](const VelocityCmd& cmd) {
    vel_callback_called = true;
    received_vel = cmd;
  };

  VelocityCmd test_vel;
  test_vel.linear_x = 1.0;
  test_vel.angular_z = 0.5;
  vel_cb(test_vel);

  EXPECT_TRUE(vel_callback_called);
  EXPECT_DOUBLE_EQ(received_vel.linear_x, 1.0);
  EXPECT_DOUBLE_EQ(received_vel.angular_z, 0.5);

  bool vr_vel_callback_called = false;
  VrVelocityCmd received_vr_vel;

  VrVelocityCmdCallback vr_vel_cb = [&](const VrVelocityCmd& cmd) {
    vr_vel_callback_called = true;
    received_vr_vel = cmd;
  };

  VrVelocityCmd test_vr_vel;
  test_vr_vel.linear_x = 1.0;
  test_vr_vel.angular_z = -0.4;
  vr_vel_cb(test_vr_vel);

  EXPECT_TRUE(vr_vel_callback_called);
  EXPECT_DOUBLE_EQ(received_vr_vel.linear_x, 1.0);
  EXPECT_DOUBLE_EQ(received_vr_vel.angular_z, -0.4);
}

// ============================================================================
// 数据类型默认值测试
// ============================================================================

// 测试 VelocityCmd 默认值
TEST_F(VRAPITest, DataTypes_VelocityCmd_Defaults) {
  VelocityCmd cmd;
  EXPECT_DOUBLE_EQ(cmd.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(cmd.linear_y, 0.0);
  EXPECT_DOUBLE_EQ(cmd.angular_z, 0.0);
  EXPECT_DOUBLE_EQ(cmd.timestamp, 0.0);
}

TEST_F(VRAPITest, DataTypes_VrVelocityCmd_Defaults) {
  VrVelocityCmd cmd;
  EXPECT_DOUBLE_EQ(cmd.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(cmd.linear_y, 0.0);
  EXPECT_DOUBLE_EQ(cmd.angular_z, 0.0);
  EXPECT_DOUBLE_EQ(cmd.timestamp, 0.0);
}

TEST_F(VRAPITest, Converter_QuestAndVelocityRoundTrip) {
  VelocityCmd vel;
  vel.linear_x = 0.1;
  vel.linear_y = -0.2;
  vel.angular_z = 0.3;
  vel.timestamp = 42.125;

  auto vel_msg = toMsg(vel);
  auto vel_roundtrip = fromMsg(vel_msg);
  EXPECT_DOUBLE_EQ(vel_roundtrip.linear_x, vel.linear_x);
  EXPECT_DOUBLE_EQ(vel_roundtrip.linear_y, vel.linear_y);
  EXPECT_DOUBLE_EQ(vel_roundtrip.angular_z, vel.angular_z);
  EXPECT_NEAR(vel_roundtrip.timestamp, vel.timestamp, 1e-6);

  VrVelocityCmd vr_vel;
  vr_vel.linear_x = -0.4;
  vr_vel.angular_z = 0.5;
  vr_vel.timestamp = 99.75;
  auto vr_msg = toMsg(vr_vel);
  auto vr_roundtrip = fromVrMsg(vr_msg);
  EXPECT_DOUBLE_EQ(vr_roundtrip.linear_x, vr_vel.linear_x);
  EXPECT_DOUBLE_EQ(vr_roundtrip.angular_z, vr_vel.angular_z);
  EXPECT_NEAR(vr_roundtrip.timestamp, vr_vel.timestamp, 1e-6);

  QuestBonePosesData bones;
  bones.header_sec = 10;
  bones.header_nanosec = 20;
  bones.timestamp_ms = 30;
  bones.is_high_confidence = true;
  bones.is_hand_tracking = true;
  bones.poses.push_back(Pose{1.0f, 2.0f, 3.0f, 0.0f, 0.0f, 0.0f, 1.0f});

  auto bones_msg = toMsg(bones);
  auto bones_roundtrip = fromMsg(bones_msg);
  EXPECT_EQ(bones_roundtrip.header_sec, bones.header_sec);
  EXPECT_EQ(bones_roundtrip.header_nanosec, bones.header_nanosec);
  EXPECT_EQ(bones_roundtrip.timestamp_ms, bones.timestamp_ms);
  EXPECT_TRUE(bones_roundtrip.is_high_confidence);
  EXPECT_TRUE(bones_roundtrip.is_hand_tracking);
  ASSERT_EQ(bones_roundtrip.poses.size(), 1u);
  EXPECT_FLOAT_EQ(bones_roundtrip.poses.front().x, 1.0f);
  EXPECT_FLOAT_EQ(bones_roundtrip.poses.front().qw, 1.0f);

  QuestJoystickData joy;
  joy.header_sec = 7;
  joy.header_nanosec = 8;
  joy.left_x = 0.11f;
  joy.right_second_button_touched = true;
  auto joy_msg = toMsg(joy);
  auto joy_roundtrip = fromMsg(joy_msg);
  EXPECT_EQ(joy_roundtrip.header_sec, joy.header_sec);
  EXPECT_EQ(joy_roundtrip.header_nanosec, joy.header_nanosec);
  EXPECT_FLOAT_EQ(joy_roundtrip.left_x, joy.left_x);
  EXPECT_TRUE(joy_roundtrip.right_second_button_touched);
}

// 测试 ControllerState 默认值
TEST_F(VRAPITest, DataTypes_ControllerState_Defaults) {
  ControllerState state;
  EXPECT_TRUE(state.current_controller.empty());
  EXPECT_EQ(state.arm_mode, ControlMode::kAuto);
  EXPECT_EQ(state.head_mode, ControlMode::kAuto);
  EXPECT_EQ(state.waist_mode, ControlMode::kAuto);
  EXPECT_TRUE(state.available_controllers.empty());
}

// 测试 JointTrajectoryPoint 默认值（空向量）
TEST_F(VRAPITest, DataTypes_JointTrajectoryPoint_Defaults) {
  JointTrajectoryPoint traj;
  EXPECT_TRUE(traj.q.empty());
  EXPECT_TRUE(traj.v.empty());
  EXPECT_TRUE(traj.acc.empty());
}

// 测试 JointTrajectoryPoint 大向量
TEST_F(VRAPITest, DataTypes_JointTrajectoryPoint_LargeVector) {
  JointTrajectoryPoint traj;
  // 14 DOF 手臂
  traj.q.resize(14, 0.5);
  traj.v.resize(14, 0.1);
  traj.acc.resize(14, 0.01);

  EXPECT_EQ(traj.q.size(), 14);
  EXPECT_EQ(traj.v.size(), 14);
  EXPECT_EQ(traj.acc.size(), 14);
  EXPECT_DOUBLE_EQ(traj.q[13], 0.5);
}

// ============================================================================
// API 生命周期测试
// ============================================================================

// 测试 API shutdown 不会导致崩溃
// 注意：由于使用单例模式，isInitialized() 始终返回 true
TEST_F(VRAPITest, API_Shutdown) {
  KuavoVRAPI api;
  EXPECT_TRUE(api.isInitialized());

  // shutdown 释放资源，但单例模式下 API 仍然可用
  api.shutdown();

  // 再次使用时会自动重新初始化
  EXPECT_TRUE(api.initialize());
  EXPECT_TRUE(api.isInitialized());
}

// 测试多次 shutdown 是安全的（不会崩溃）
TEST_F(VRAPITest, API_MultipleShutdown) {
  KuavoVRAPI api;
  api.shutdown();
  api.shutdown();  // 多次调用不应崩溃
  // 单例模式下仍然可以重新初始化
  EXPECT_TRUE(api.initialize());
}

// 测试多次 initialize 是安全的
TEST_F(VRAPITest, API_MultipleInitialize) {
  KuavoVRAPI api;
  EXPECT_TRUE(api.isInitialized());
  EXPECT_TRUE(api.initialize());  // 重复初始化应返回 true
  EXPECT_TRUE(api.isInitialized());
}

// 测试 shutdown 后可以继续使用（单例自动重新初始化）
TEST_F(VRAPITest, API_ShutdownAndReuse) {
  KuavoVRAPI api;
  api.shutdown();

  // 在 shutdown 后调用 publish 会触发重新初始化
  JointTrajectoryPoint cmd;
  cmd.q = {0.1, 0.2};
  // 即使 shutdown 后，发布操作也应该成功（或安全地失败）
  // 这里主要验证不会崩溃
  api.publishArmJointCmd(cmd);
}

// ============================================================================
// 多实例测试
// ============================================================================

// 测试同时创建多个 Kuavo API 实例
TEST_F(VRAPITest, MultipleInstances_Kuavo) {
  KuavoVRAPI api1;
  KuavoVRAPI api2;

  EXPECT_TRUE(api1.isInitialized());
  EXPECT_TRUE(api2.isInitialized());
  EXPECT_TRUE(IS_KUAVO(api1.getRobotVersion()));
  EXPECT_TRUE(IS_KUAVO(api2.getRobotVersion()));
}

// 测试同时创建 Kuavo 和 Roban API 实例
TEST_F(VRAPITest, MultipleInstances_Mixed) {
  KuavoVRAPI kuavo_api;
  RobanVRAPI roban_api;

  EXPECT_TRUE(kuavo_api.isInitialized());
  EXPECT_TRUE(roban_api.isInitialized());
  EXPECT_TRUE(IS_KUAVO(kuavo_api.getRobotVersion()));
  EXPECT_TRUE(IS_ROBAN(roban_api.getRobotVersion()));
}

// ============================================================================
// RobotVersion 宏测试
// ============================================================================

// 测试 IS_KUAVO 宏
TEST_F(VRAPITest, RobotVersion_IS_KUAVO) {
  EXPECT_TRUE(IS_KUAVO(leju::RobotVersions::KUAVO5_BASE));
  EXPECT_TRUE(IS_KUAVO(leju::RobotVersions::KUAVO4_BASE));
  EXPECT_TRUE(IS_KUAVO(leju::RobotVersions::KUAVO4_PRO_DEXHAND));
  EXPECT_FALSE(IS_KUAVO(leju::RobotVersions::ROBAN2_BASE));
}

// 测试 IS_ROBAN 宏
TEST_F(VRAPITest, RobotVersion_IS_ROBAN) {
  EXPECT_TRUE(IS_ROBAN(leju::RobotVersions::ROBAN2_BASE));
  EXPECT_FALSE(IS_ROBAN(leju::RobotVersions::KUAVO5_BASE));
  EXPECT_FALSE(IS_ROBAN(leju::RobotVersions::KUAVO4_BASE));
}

// 测试 IS_KUAVO4_LEGGED 和 IS_KUAVO5_LEGGED 宏
TEST_F(VRAPITest, RobotVersion_KuavoGenerations) {
  EXPECT_TRUE(IS_KUAVO4_LEGGED(leju::RobotVersions::KUAVO4_BASE));
  EXPECT_TRUE(IS_KUAVO4_LEGGED(leju::RobotVersions::KUAVO4_PRO_DEXHAND));
  EXPECT_FALSE(IS_KUAVO4_LEGGED(leju::RobotVersions::KUAVO5_BASE));

  EXPECT_TRUE(IS_KUAVO5_LEGGED(leju::RobotVersions::KUAVO5_BASE));
  EXPECT_FALSE(IS_KUAVO5_LEGGED(leju::RobotVersions::KUAVO4_BASE));
}

// ============================================================================
// ControlMode 枚举测试
// ============================================================================

// 测试 ControlMode 枚举转换
TEST_F(VRAPITest, ControlMode_Conversion) {
  // int -> ControlMode
  EXPECT_EQ(static_cast<ControlMode>(0), ControlMode::kKeepPose);
  EXPECT_EQ(static_cast<ControlMode>(1), ControlMode::kAuto);
  EXPECT_EQ(static_cast<ControlMode>(2), ControlMode::kExternal);

  // ControlMode -> int
  EXPECT_EQ(static_cast<int>(ControlMode::kKeepPose), 0);
  EXPECT_EQ(static_cast<int>(ControlMode::kAuto), 1);
  EXPECT_EQ(static_cast<int>(ControlMode::kExternal), 2);
}

// 测试 ControlMode 比较
TEST_F(VRAPITest, ControlMode_Comparison) {
  ControlMode mode1 = ControlMode::kAuto;
  ControlMode mode2 = ControlMode::kAuto;
  ControlMode mode3 = ControlMode::kExternal;

  EXPECT_EQ(mode1, mode2);
  EXPECT_NE(mode1, mode3);
}

// ============================================================================
// Handler 空值测试
// ============================================================================

// 测试空 Handler
TEST_F(VRAPITest, TypeDefs_NullHandlers) {
  SwitchControllerHandler null_handler = nullptr;
  EXPECT_FALSE(static_cast<bool>(null_handler));

  SetModeHandler null_mode_handler = nullptr;
  EXPECT_FALSE(static_cast<bool>(null_mode_handler));

  GetRuntimeStateHandler null_runtime_state_handler = nullptr;
  EXPECT_FALSE(static_cast<bool>(null_runtime_state_handler));

  GetControllerStateHandler null_controller_state_handler = nullptr;
  EXPECT_FALSE(static_cast<bool>(null_controller_state_handler));

  GetMotionStateHandler null_motion_state_handler = nullptr;
  EXPECT_FALSE(static_cast<bool>(null_motion_state_handler));
}

// 测试空 Callback
TEST_F(VRAPITest, TypeDefs_NullCallbacks) {
  ArmTrajectoryCallback null_joint_cb = nullptr;
  EXPECT_FALSE(static_cast<bool>(null_joint_cb));

  VelocityCmdCallback null_vel_cb = nullptr;
  EXPECT_FALSE(static_cast<bool>(null_vel_cb));

  VrVelocityCmdCallback null_vr_vel_cb = nullptr;
  EXPECT_FALSE(static_cast<bool>(null_vr_vel_cb));
}

// ============================================================================
// 重复注册测试
// ============================================================================

// 测试重复注册 Handler（应该替换旧的）
TEST_F(VRAPITest, Handler_DuplicateRegistration) {
  KuavoVRAPI api;

  // 使用不捕获局部变量引用的回调，避免悬挂引用问题
  // 主要测试重复注册不会崩溃

  // 第一次注册
  api.registerSwitchControllerHandler(
      [](const std::string& name, std::string& message) -> bool {
        message = "Handler 1";
        return true;
      });

  // 第二次注册（应该替换第一个）
  api.registerSwitchControllerHandler(
      [](const std::string& name, std::string& message) -> bool {
        message = "Handler 2";
        return true;
      });

  // 不应崩溃，验证可以重复注册
  EXPECT_TRUE(api.isInitialized());
}

// 测试重复注册 SetModeHandler
TEST_F(VRAPITest, Handler_DuplicateSetModeRegistration) {
  KuavoVRAPI api;

  // 注册手臂模式处理器两次
  api.registerSetArmModeHandler(
      [](ControlMode mode, std::string& message) -> bool {
        message = "First handler";
        return true;
      });

  api.registerSetArmModeHandler(
      [](ControlMode mode, std::string& message) -> bool {
        message = "Second handler";
        return false;
      });

  // 同样注册头部和腰部
  api.registerSetHeadModeHandler(
      [](ControlMode mode, std::string& message) -> bool {
        return true;
      });

  api.registerSetWaistModeHandler(
      [](ControlMode mode, std::string& message) -> bool {
        return true;
      });

  EXPECT_TRUE(api.isInitialized());
}

// 测试重复注册 ControllerState Handler
TEST_F(VRAPITest, Handler_DuplicateGetControllerStateRegistration) {
  KuavoVRAPI api;

  api.registerGetControllerStateHandler([]() -> ControllerState {
    ControllerState state;
    state.current_controller = "first";
    return state;
  });

  api.registerGetControllerStateHandler([]() -> ControllerState {
    ControllerState state;
    state.current_controller = "second";
    return state;
  });

  EXPECT_TRUE(api.isInitialized());
}

// 测试重复订阅（应该替换旧的回调）
TEST_F(VRAPITest, Subscribe_DuplicateSubscription) {
  KuavoVRAPI api;

  // 使用空回调，避免捕获局部变量引用导致悬挂引用问题
  // 主要测试重复订阅不会崩溃

  // 第一次订阅
  api.subscribeArmJointCmd([](const JointTrajectoryPoint& cmd) {});

  // 第二次订阅（应该替换第一个）
  api.subscribeArmJointCmd([](const JointTrajectoryPoint& cmd) {});

  // 同样测试其他订阅
  api.subscribeHeadJointCmd([](const JointTrajectoryPoint& cmd) {});
  api.subscribeHeadJointCmd([](const JointTrajectoryPoint& cmd) {});

  api.subscribeWaistJointCmd([](const JointTrajectoryPoint& cmd) {});
  api.subscribeWaistJointCmd([](const JointTrajectoryPoint& cmd) {});

  api.subscribeVelocityCmd([](const VelocityCmd& cmd) {});
  api.subscribeVelocityCmd([](const VelocityCmd& cmd) {});
  api.subscribeVrVelocityCmd([](const VrVelocityCmd& cmd) {});
  api.subscribeVrVelocityCmd([](const VrVelocityCmd& cmd) {});

  EXPECT_TRUE(api.isInitialized());
}

// ============================================================================
// 多 Client 测试
// ============================================================================

// 测试多个 Client 同时发布指令
TEST_F(VRAPITest, MultiClient_SimultaneousPublish) {
  KuavoVRAPI client1;
  KuavoVRAPI client2;
  RobanVRAPI client3;

  // 确保通信层已初始化
  client1.initialize();

  JointTrajectoryPoint cmd;
  cmd.q = {0.1, 0.2, 0.3};
  cmd.v = {0.01, 0.02, 0.03};

  // 多个 client 同时发布不同类型的指令（不应崩溃）
  // 注意：由于单例模式，publish 可能成功或失败取决于通信状态
  client1.publishArmJointCmd(cmd);
  client2.publishHeadJointCmd(cmd);
  client3.publishWaistJointCmd(cmd);

  VelocityCmd vel;
  vel.linear_x = 0.5;
  client1.publishVelocityCmd(vel);
  client2.publishVelocityCmd(vel);

  VrVelocityCmd vr_vel;
  vr_vel.linear_x = 0.2;
  client1.publishVrVelocityCmd(vr_vel);
  client2.publishVrVelocityCmd(vr_vel);

  // 验证所有 client 仍然有效
  EXPECT_TRUE(client1.isInitialized());
  EXPECT_TRUE(client2.isInitialized());
  EXPECT_TRUE(client3.isInitialized());
}

// 测试多个 Client 同时订阅
TEST_F(VRAPITest, MultiClient_SimultaneousSubscribe) {
  KuavoVRAPI client1;
  KuavoVRAPI client2;

  // 注意：由于使用单例，后面的订阅会覆盖前面的
  // 这里主要测试不会崩溃
  // 使用空回调或简单回调，避免捕获局部变量引用
  client1.subscribeArmJointCmd([](const JointTrajectoryPoint& cmd) {
    // 空回调，仅测试订阅功能
  });

  client2.subscribeArmJointCmd([](const JointTrajectoryPoint& cmd) {
    // 空回调，仅测试订阅功能
  });

  EXPECT_TRUE(client1.isInitialized());
  EXPECT_TRUE(client2.isInitialized());
}

// 测试 Client 和 Server 同时运行
TEST_F(VRAPITest, MultiClient_ClientAndServer) {
  // 服务端 API
  KuavoVRAPI server;
  server.initialize();  // 确保初始化

  // 使用不捕获局部变量引用的回调，避免悬挂引用问题
  server.registerSetArmModeHandler(
      [](ControlMode mode, std::string& message) -> bool {
        message = "Mode set";
        return true;
      });

  server.registerGetControllerStateHandler([]() -> ControllerState {
    ControllerState state;
    state.current_controller = "test";
    state.arm_mode = ControlMode::kExternal;
    return state;
  });

  // 客户端 API（可以是不同的实例）
  KuavoVRAPI client;

  // 客户端发布指令（不检查返回值，主要验证不崩溃）
  JointTrajectoryPoint cmd;
  cmd.q = {0.1};
  client.publishArmJointCmd(cmd);

  // 验证两个实例都正常工作
  EXPECT_TRUE(server.isInitialized());
  EXPECT_TRUE(client.isInitialized());
}

// 测试多个 Client 注册不同的 Handler
TEST_F(VRAPITest, MultiClient_DifferentHandlers) {
  KuavoVRAPI api1;
  KuavoVRAPI api2;

  // api1 注册切换控制器处理
  api1.registerSwitchControllerHandler(
      [](const std::string& name, std::string& message) -> bool {
        message = "API1 handler";
        return true;
      });

  // api2 注册获取控制器状态处理
  api2.registerGetControllerStateHandler([]() -> ControllerState {
    ControllerState state;
    state.current_controller = "api2";
    return state;
  });

  EXPECT_TRUE(api1.isInitialized());
  EXPECT_TRUE(api2.isInitialized());
}

// 测试快速创建和销毁多个 Client
TEST_F(VRAPITest, MultiClient_RapidCreateDestroy) {
  for (int i = 0; i < 5; ++i) {
    KuavoVRAPI api;
    EXPECT_TRUE(api.isInitialized());

    JointTrajectoryPoint cmd;
    cmd.q = {static_cast<double>(i) * 0.1};
    api.publishArmJointCmd(cmd);
  }

  // 验证最后仍然可以正常工作
  KuavoVRAPI final_api;
  EXPECT_TRUE(final_api.isInitialized());
}

// 测试混合 Kuavo 和 Roban Client
TEST_F(VRAPITest, MultiClient_MixedRobotTypes) {
  KuavoVRAPI kuavo1;
  KuavoVRAPI kuavo2;
  RobanVRAPI roban1;
  RobanVRAPI roban2;

  // 确保初始化
  kuavo1.initialize();

  // 验证不同类型的机器人版本
  EXPECT_TRUE(IS_KUAVO(kuavo1.getRobotVersion()));
  EXPECT_TRUE(IS_KUAVO(kuavo2.getRobotVersion()));
  EXPECT_TRUE(IS_ROBAN(roban1.getRobotVersion()));
  EXPECT_TRUE(IS_ROBAN(roban2.getRobotVersion()));

  // 所有实例都可以发布（不检查返回值，主要验证不崩溃）
  JointTrajectoryPoint cmd;
  cmd.q = {0.1};
  kuavo1.publishArmJointCmd(cmd);
  kuavo2.publishArmJointCmd(cmd);
  roban1.publishArmJointCmd(cmd);
  roban2.publishArmJointCmd(cmd);

  // 验证所有实例仍然有效
  EXPECT_TRUE(kuavo1.isInitialized());
  EXPECT_TRUE(kuavo2.isInitialized());
  EXPECT_TRUE(roban1.isInitialized());
  EXPECT_TRUE(roban2.isInitialized());
}

// ============================================================================
// Phase 1 验收测试: runtime/motion 生命周期 RPC 端到端往返测试
//
// 测试策略：在同一进程内，通过单例 VRDdsCommunicator（单 participant，domain 0）
// 同时持有 replier 和 requester，利用 CycloneDDS loopback 完成端到端往返。
// 与 calculator test_end_to_end 的双 participant 模式等效，只是这里使用 VRBaseAPI
// 对外接口，而不是直接操作 dds_rpc::Requester/Replier。
// ============================================================================

class VRAPIRpcRoundTripTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // 确保通信层已初始化（单例模式）
    leju::vr::VRCommunication::getInstance().initialize();
    // 给 DDS 足够时间完成 loopback 发现（与 kServiceDiscoveryTimeoutMs=1000ms 一致）
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
  }

  void TearDown() override {
    // 单例不 shutdown，测试间共用，与其他测试保持一致
  }
};

// ------ getRuntimeState 往返 ------
TEST_F(VRAPIRpcRoundTripTest, RoundTrip_GetRuntimeState) {
  KuavoVRAPI api;
  ASSERT_TRUE(api.isInitialized());

  // 注册服务端处理函数，返回确定值
  api.registerGetRuntimeStateHandler([]() -> leju::vr::RuntimeState {
    leju::vr::RuntimeState s;
    s.data_ready = true;
    s.hardware_state = leju::vr::HardwareState::kReadyOk;
    s.lifecycle_state = leju::vr::LifecycleState::kWaitingForStart;
    return s;
  });

  // 等待 replier 被发现
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // 客户端调用（5000ms 超时覆盖 DDS 发现窗口）
  leju::vr::RuntimeState received;
  bool ok = api.getRuntimeState(received, 5000);
  ASSERT_TRUE(ok) << "getRuntimeState RPC call failed";

  EXPECT_TRUE(received.data_ready);
  EXPECT_EQ(received.hardware_state, leju::vr::HardwareState::kReadyOk);
  EXPECT_EQ(received.lifecycle_state, leju::vr::LifecycleState::kWaitingForStart);
}

// ------ getMotionState 往返 ------
TEST_F(VRAPIRpcRoundTripTest, RoundTrip_GetMotionState) {
  KuavoVRAPI api;
  ASSERT_TRUE(api.isInitialized());

  api.registerGetMotionStateHandler([]() -> leju::vr::MotionState {
    leju::vr::MotionState m;
    m.supported = true;
    m.motion_playing = false;
    m.current_motion_name = "idle_x";
    m.available_motion_names = {"idle_x", "dance_a", "walk_b"};
    return m;
  });

  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  leju::vr::MotionState received;
  bool ok = api.getMotionState(received, 5000);
  ASSERT_TRUE(ok) << "getMotionState RPC call failed";

  EXPECT_TRUE(received.supported);
  EXPECT_FALSE(received.motion_playing);
  EXPECT_EQ(received.current_motion_name, "idle_x");
  ASSERT_EQ(received.available_motion_names.size(), 3u);
  EXPECT_EQ(received.available_motion_names[0], "idle_x");
  EXPECT_EQ(received.available_motion_names[1], "dance_a");
  EXPECT_EQ(received.available_motion_names[2], "walk_b");
}

// ------ startRuntime Trigger 往返 ------
TEST_F(VRAPIRpcRoundTripTest, RoundTrip_StartRuntime) {
  KuavoVRAPI api;
  ASSERT_TRUE(api.isInitialized());

  api.registerStartRuntimeHandler([](std::string& msg) -> bool {
    msg = "runtime_started_ok";
    return true;
  });

  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  std::string msg;
  bool ok = api.startRuntime(&msg, 5000);
  EXPECT_TRUE(ok) << "startRuntime RPC call failed";
  EXPECT_EQ(msg, "runtime_started_ok");
}

// ------ stopRuntime Trigger 往返 ------
TEST_F(VRAPIRpcRoundTripTest, RoundTrip_StopRuntime) {
  KuavoVRAPI api;
  ASSERT_TRUE(api.isInitialized());

  api.registerStopRuntimeHandler([](std::string& msg) -> bool {
    msg = "runtime_stopped_ok";
    return true;
  });

  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  std::string msg;
  bool ok = api.stopRuntime(&msg, 5000);
  EXPECT_TRUE(ok) << "stopRuntime RPC call failed";
  EXPECT_EQ(msg, "runtime_stopped_ok");
}

// ------ startMotion SetString 往返（验证 name 参数传递）------
TEST_F(VRAPIRpcRoundTripTest, RoundTrip_StartMotion) {
  KuavoVRAPI api;
  ASSERT_TRUE(api.isInitialized());

  // 记录服务端收到的 name，验证参数能正确传过去
  // 使用 shared_ptr 按值捕获，避免 handler 存活于 singleton 中时悬空引用
  auto received_name_on_server = std::make_shared<std::string>();
  api.registerStartMotionHandler(
      [received_name_on_server](const std::string& name, std::string& msg) -> bool {
        *received_name_on_server = name;
        msg = "motion_started:" + name;
        return true;
      });

  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  std::string msg;
  bool ok = api.startMotion("dance_test", &msg, 5000);
  EXPECT_TRUE(ok) << "startMotion RPC call failed";
  EXPECT_EQ(msg, "motion_started:dance_test");
  EXPECT_EQ(*received_name_on_server, "dance_test");
}

// ------ getControllerState 往返（含 controller_transitioning 字段）------
TEST_F(VRAPIRpcRoundTripTest, RoundTrip_GetControllerState_WithTransitioning) {
  KuavoVRAPI api;
  ASSERT_TRUE(api.isInitialized());

  api.registerGetControllerStateHandler([]() -> leju::vr::ControllerState {
    leju::vr::ControllerState s;
    s.current_controller = "mimic_dance";
    s.arm_mode = leju::vr::ControlMode::kExternal;
    s.head_mode = leju::vr::ControlMode::kAuto;
    s.waist_mode = leju::vr::ControlMode::kAuto;
    s.controller_transitioning = true;
    s.available_controllers = {"amp", "mimic_dance"};
    return s;
  });

  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  leju::vr::ControllerState received;
  bool ok = api.getControllerState(received, 5000);
  ASSERT_TRUE(ok) << "getControllerState RPC call failed";

  EXPECT_EQ(received.current_controller, "mimic_dance");
  EXPECT_EQ(received.arm_mode, leju::vr::ControlMode::kExternal);
  EXPECT_EQ(received.head_mode, leju::vr::ControlMode::kAuto);
  EXPECT_EQ(received.waist_mode, leju::vr::ControlMode::kAuto);
  EXPECT_TRUE(received.controller_transitioning);
  ASSERT_EQ(received.available_controllers.size(), 2u);
  EXPECT_EQ(received.available_controllers[0], "amp");
  EXPECT_EQ(received.available_controllers[1], "mimic_dance");
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
