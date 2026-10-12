#include "hardware_node.h"
#include <chrono>
#include <thread>
#include <iostream>
#include <fstream>
#include <cmath>
#include <sys/stat.h>
#include <sched.h>
#include <pthread.h>
#include "hw_log.h"

#include "lejusdk-utils/time_utils.hpp"
#include "lejusdk_hw/lejusdk_hw.h"
namespace leju {
HardwareNode::HardwareNode(const std::string& config_dir, bool bypass_ankle_solver)
    : config_dir_(config_dir), bypass_ankle_solver_(bypass_ankle_solver) {
}

HardwareNode::~HardwareNode() {
    this->close();
}

bool HardwareNode::init() {
    if(hardware_state_.load() >= HardwareState::INITIALIZING) {
        HWLOG_I("Hardware node already initialized");
        return true;
    }

    this->hardware_state_.store(HardwareState::INITIALIZING);
    
    // DDS通信初始化
    if (!this->initialize()) {
        HWLOG_FAILURE("Failed to start DDS communication");
        return false;
    }

    // Initialize the Hardware Part!
    leju::hw::HardwareInterface& hw = leju::hw::HardwareInterface::GetInstance();

    // 初始化硬件接口
    HWLOG_I("Initializing hardware interface...");
    
    if (hw.Initialize(config_dir_) == leju::hw::HwErrorType::Success) {
        HWLOG_SUCCESS("Hardware interface initialized successfully!");
    } else {
        HWLOG_FAILURE("Failed to initialize hardware interface!");
        hardware_state_ = HardwareState::ERROR;
        return false;
    }

    num_joints_ = hw.GetMotorNumber();

    // EC 电机需要通过 JointMoveTo 回零 (经过 ankle solver 关节→电机坐标变换)
    // CANFD (MotorevoActuator) 电机在 hw.Initialize() 内部已完成回零, 不需要再执行
    if (hw.HasECMasterMotors()) {
        std::vector<double> goal_pos(num_joints_, 0.0);
        double speed = 60; // deg/s
        hw.JointMoveTo(goal_pos, speed);
    }

    // 踝关节解算绕过开关 (keyboard 控制器通过 CLI 传入)
    if (bypass_ankle_solver_) {
        hw.SetBypassAnkleSolver(true);
        HWLOG_I("bypass_ankle_solver enabled: ankle FK/IK transforms will be skipped");
    }

    // 创建传感器数据发布循环
    sensors_loop_ = std::make_unique<leju::Loop>(
        "hardware_sensors_loop",
        [this]() {
            this->sensorThreadFunc();
        },
        kSensorPublishPeriodMs,     // 发布周期
        kSensorLoopCpuCore,         // CPU核心绑定 (-1不绑定)
        kSensorLoopSchedPriority    // SCHED_FIFO 优先级 30
    );

    sensors_loop_->start();

    this->subscribeRobotCmd(
        std::bind(&HardwareNode::handleJointCommand, this, std::placeholders::_1)
    );

    this->subscribeHandCmd(
        std::bind(&HardwareNode::handleHandCommand, this, std::placeholders::_1)
    );

    // 订阅手柄: start 按下后激活头部 runtime_skip (启动阶段头部已随初始化回零)
    this->subscribeJoyData(
        std::bind(&HardwareNode::handleJoyData, this, std::placeholders::_1)
    );

    // 启动正弦测试监控线程
    sine_test_thread_ = std::thread(&HardwareNode::sineTestThreadFunc, this);

    hardware_state_ = HardwareState::READY_OK;
    HWLOG_SUCCESS("Node initialized successfully!");
    HWLOG_I("[SineTest] touch /tmp/sine_test_start 开始正弦测试, rm /tmp/sine_test_start 停止");
    return true;
}

void HardwareNode::handleJointCommand(const RobotCmdConstPtr& cmd) {
    if (isStopReceived()) return;

    // 正弦测试模式下忽略 DDS 命令
    if (sine_test_active_.load()) return;

    ///////////////////////////////////////////////////////////////////////////////
    if (!cmd) {
        std::cout << "\033[31m[HardwareNode] Received null joint command!\033[0m" << std::endl;
        return;
    }

    if(hardware_state_ < HardwareState::READY_OK)  {
        std::cout << "\033[31m[HardwareNode] Hardware not ready, cannot handle joint command!\033[0m" << std::endl;
        return;
    }

    leju::hw::HardwareInterface& hw = leju::hw::HardwareInterface::GetInstance();

    // 验证命令维度
    if (num_joints_ <= 0 ||
        cmd->q.size() != num_joints_ ||
        cmd->v.size() != num_joints_ ||
        cmd->tau.size() != num_joints_ ||
        cmd->kp.size() != num_joints_ ||
        cmd->kd.size() != num_joints_ ||
        cmd->modes.size() != num_joints_) {
        std::cout << "\033[31m[HardwareNode] Invalid joint command dimensions!\033[0m" << std::endl;
        std::cout << "\033[31m[HardwareNode] Expected: " << num_joints_ << " joints\033[0m" << std::endl;
        std::cout << "\033[31m[HardwareNode] Actual: q=" << cmd->q.size() << ", v=" << cmd->v.size()
                  << ", tau=" << cmd->tau.size() << ", kp=" << cmd->kp.size()
                  << ", kd=" << cmd->kd.size() << ", modes=" << cmd->modes.size() << "\033[0m" << std::endl;
        return;
    }
    ///////////////////////////////////////////////////////////////////////////////
    
    leju::hw::JointCommand_t joint_cmd;
    joint_cmd.resize(num_joints_);
    joint_cmd.q = cmd->q;
    joint_cmd.v = cmd->v;
    joint_cmd.tau = cmd->tau;
    joint_cmd.kp = cmd->kp;
    joint_cmd.kd = cmd->kd;
    joint_cmd.modes = cmd->modes;

    // 诊断: 记录命令到达时间和 joint 0 的位置
    // 默认编译关闭, 避免 DDS 命令回调线程做同步文件 IO; 开启: cmake -DLEJU_ENABLE_DIAG_FILELOG=ON
#ifdef LEJU_ENABLE_DIAG_FILELOG
    {
        static FILE* rcvlog = fopen("/tmp/cmd_receive.log", "w");
        static double last_q0 = 0;
        if (rcvlog && cmd->q.size() > 0 && std::abs(cmd->q[0] - last_q0) > 0.0001) {
            auto now = std::chrono::steady_clock::now();
            static auto t0 = now;
            auto ms = std::chrono::duration_cast<std::chrono::microseconds>(now - t0).count() / 1000.0;
            fprintf(rcvlog, "[CmdRecv] t=%.1fms q0=%.6f(%.3fdeg) kp0=%.1f kd0=%.1f mode0=%d\n",
                    ms, cmd->q[0], cmd->q[0]*180.0/3.14159265, cmd->kp[0], cmd->kd[0], (int)cmd->modes[0]);
            fflush(rcvlog);
            last_q0 = cmd->q[0];
        }
    }
#endif

    // 干运行模式: touch /tmp/hw_dry_run 启用, rm /tmp/hw_dry_run 恢复
    // 仅在 hardware READY_OK 之后生效 (不影响初始化阶段的 moveToDefaultPos)
    // 启用后 joint_cmd 照常接收和记录, 但不下发给电机
    {
        struct stat st;
        if (hardware_state_.load() == HardwareState::READY_OK &&
            stat("/tmp/hw_dry_run", &st) == 0) return;
    }

    hw.SetJointCommand(joint_cmd);
}


void HardwareNode::handleHandCommand(const HandCmdConstPtr& cmd) {
    if (sine_test_active_.load()) return;

    if (!cmd) {
        std::cout << "\033[31m[HardwareNode] Received null hand command!\033[0m" << std::endl;
        return;
    }

    if (hardware_state_ < HardwareState::READY_OK) {
        std::cout << "\033[31m[HardwareNode] Hardware not ready, cannot handle hand command!\033[0m" << std::endl;
        return;
    }

    if (!cmd->isValid()) {
        std::cout << "\033[31m[HardwareNode] Invalid hand command dimensions: "
                  << cmd->position.size() << "\033[0m" << std::endl;
        return;
    }

    struct stat st;
    if (hardware_state_.load() == HardwareState::READY_OK &&
        stat("/tmp/hw_dry_run", &st) == 0) return;

    leju::hw::HandCommand_t hand_cmd;
    hand_cmd.position = cmd->position;
    hand_cmd.control_mode = leju::hw::HandControlMode::POSITION;

    auto& hw = leju::hw::HardwareInterface::GetInstance();
    auto ret = hw.SetHandCommand(hand_cmd);
    if (ret != leju::hw::HwErrorType::Success) {
        std::cout << "\033[31m[HardwareNode] SetHandCommand failed: "
                  << static_cast<int>(ret) << "\033[0m" << std::endl;
    }
}

void HardwareNode::handleJoyData(const JoyDataConstPtr& joy) {
    if (!joy) return;

    const bool start_now = (joy->buttons.start != 0);
    const bool start_prev = prev_joy_start_.exchange(start_now);
    if (!start_now || start_prev) return;  // 仅响应按下沿

    // back+start 是退出组合 (随后整机掉使能), 不作为控制开始信号
    if (joy->buttons.back != 0) return;

    if (runtime_skip_activated_.exchange(true)) return;  // 只激活一次

    HWLOG_I("[HardwareNode] START 按下: 激活头部 runtime_skip (头部电机停发运动帧)");
    leju::hw::HardwareInterface::GetInstance().SetRuntimeSkipActive(true);
}

void HardwareNode::sensorThreadFunc() {
    leju::hw::HardwareInterface& hw = leju::hw::HardwareInterface::GetInstance();
    using Clock = std::chrono::steady_clock;

    // 电机不可用保护必须先于正弦测试 early-return，避免测试模式绕过健康监控。
    if (!lowerlimb_fault_protect_) {
        std::string fault_info;
        const bool ready_ok = hardware_state_.load() == HardwareState::READY_OK;
        if (hw.HasLowerLimbMotorFault(fault_info, ready_ok)) {
            lowerlimb_fault_protect_ = true;
            std::cerr << "\033[1;31m[HardwareNode] 电机不可用, 进入保护模式, 发布 /rt/hardware/stop 停止控制!\n"
                      << fault_info << "\033[0m" << std::endl;
            for (int i = 0; i < 3; ++i) {
                publishStopRobot();
            }
            setStopReceived();  // 兜底: 不依赖 DDS 自回环, 确保本节点退出并给电机掉使能
        }
    }

    // 正弦测试期间跳过 GetJointState，避免与测试线程竞争 Kalman 滤波器状态
    if (sine_test_active_.load()) return;

    [[maybe_unused]] auto t0 = Clock::now();

    RobotState joint_state;
    ImuData imu_data;

    bool pub_state = false, pub_imu = false;

    leju::hw::JointData_t joint_data;
    if (hw.GetJointState(joint_data) == leju::hw::HwErrorType::Success) {
        pub_state = true;
        joint_state.timestamp = leju::common::GetUnixTimestampS();
        auto num_joints_ = joint_data.q.size();
        joint_state.resize(num_joints_);
        for (size_t i = 0; i < num_joints_; i++) {
            joint_state.q[i] = joint_data.q[i];
            joint_state.v[i] = joint_data.v[i];
            joint_state.vd[i] = joint_data.vd[i];
            joint_state.tau[i] = joint_data.tau[i];
        }
    }
    [[maybe_unused]] auto t1 = Clock::now();

    leju::hw::ImuData_t hw_imu_data;
    if(hw.GetImuData(hw_imu_data) == leju::hw::HwErrorType::Success) {
        pub_imu = true;
        imu_data.timestamp = leju::common::GetUnixTimestampS();
        imu_data.gyro = hw_imu_data.gyro;
        imu_data.acc = hw_imu_data.acc;
        imu_data.quat = hw_imu_data.quat;
        imu_data.free_acc = hw_imu_data.free_acc;
    }
    [[maybe_unused]] auto t2 = Clock::now();

    if(pub_state) {
        this->publishJointState(joint_state);

        // 电机层原始反馈（pre-c2t, pre-ankle-inv）→ /rt/motor_state
        leju::hw::JointData_t motor_data;
        if (hw.GetMotorState(motor_data) == leju::hw::HwErrorType::Success) {
            RobotState motor_state;
            motor_state.timestamp = joint_state.timestamp;
            const size_t n_state = motor_data.q.size();
            motor_state.resize(n_state);
            for (size_t i = 0; i < n_state; ++i) {
                motor_state.q[i]   = motor_data.q[i];
                motor_state.v[i]   = motor_data.v[i];
                motor_state.vd[i]  = motor_data.vd[i];
                motor_state.tau[i] = motor_data.tau[i];   // 原始电机 tau，未乘 c2t
            }
            this->publishMotorState(motor_state);
        }

        // 电机层最终命令（post-ankle-fwd, post-c2t/A）→ /rt/motor_cmd
        leju::hw::JointCommand_t motor_cmd_snap;
        if (hw.GetMotorCmd(motor_cmd_snap) == leju::hw::HwErrorType::Success) {
            RobotCmd motor_cmd_msg;
            motor_cmd_msg.timestamp = joint_state.timestamp;
            const size_t n_cmd = motor_cmd_snap.q.size();
            motor_cmd_msg.resize(n_cmd);
            for (size_t i = 0; i < n_cmd; ++i) {
                motor_cmd_msg.q[i]     = motor_cmd_snap.q[i];
                motor_cmd_msg.v[i]     = motor_cmd_snap.v[i];
                motor_cmd_msg.tau[i]   = motor_cmd_snap.tau[i];
                motor_cmd_msg.kp[i]    = motor_cmd_snap.kp[i];
                motor_cmd_msg.kd[i]    = motor_cmd_snap.kd[i];
                motor_cmd_msg.modes[i] = motor_cmd_snap.modes[i];
            }
            this->publishMotorCmd(motor_cmd_msg);
        }
    }
    else {
        std::cout << "\033[31m[HardwareNode] Failed to get joint state!\033[0m" << std::endl;
    }
    [[maybe_unused]] auto t3 = Clock::now();

    if(pub_imu) {
        this->publishImuData(imu_data);
    }
    else {
        std::cout << "\033[31m[HardwareNode] Failed to get imu data!\033[0m" << std::endl;
    }
    [[maybe_unused]] auto t4 = Clock::now();

    // 发布灵巧手状态 /rt/hand_state：随传感器环 1kHz 发布（缓存快照读取，非总线同步读）
    if (hardware_state_.load() == HardwareState::READY_OK) {
        leju::hw::HandState_t hw_state;
        if (hw.GetHandState(hw_state) == leju::hw::HwErrorType::Success) {
            HandState state;
            state.timestamp = leju::common::GetUnixTimestampS();
            state.left_valid = hw_state.left_valid;
            state.right_valid = hw_state.right_valid;
            state.left_sample_age_ms = hw_state.left_sample_age_ms;
            state.right_sample_age_ms = hw_state.right_sample_age_ms;
            state.position = hw_state.position;
            state.velocity = hw_state.velocity;
            state.current = hw_state.current;
            state.state = hw_state.state;
            if (!publishHandState(state)) {
                static auto last_warning = Clock::now() - std::chrono::seconds(2);
                const auto now = Clock::now();
                if (now - last_warning >= std::chrono::seconds(2)) {
                    HWLOG_W("[HardwareNode] Failed to publish /rt/hand_state");
                    last_warning = now;
                }
            }
        }
    }

    //////////////////////////////////////////////////////////////////////////////
    // Debug: 传感器耗时写文件 (不受 ncurses 干扰)
    // 默认编译关闭, 避免 1kHz 传感器环做同步文件 IO; 开启: cmake -DLEJU_ENABLE_DIAG_FILELOG=ON
#ifdef LEJU_ENABLE_DIAG_FILELOG
    auto us = [](auto a, auto b) {
        return std::chrono::duration_cast<std::chrono::microseconds>(b - a).count();
    };
    {
        static FILE* slog = fopen("/tmp/sensor_timing.log", "w");
        static auto last_print_time = Clock::now() - std::chrono::seconds(1);
        auto now = Clock::now();
        if (slog && std::chrono::duration_cast<std::chrono::seconds>(now - last_print_time).count() >= 1) {
            fprintf(slog, "[Sensor] GetJoint=%ldus GetImu=%ldus PubJoint=%ldus PubImu=%ldus Total=%ldus\n",
                    us(t0,t1), us(t1,t2), us(t2,t3), us(t3,t4), us(t0,t4));
            fflush(slog);
            last_print_time = now;
        }
    }
#endif
    ////////////////////////////////////////////////////////////////////////////////
}

void HardwareNode::sineTestThreadFunc() {
    using Clock = std::chrono::steady_clock;
    constexpr double kPI = 3.14159265358979323846;

    // 正弦测试参数
    constexpr int    kTestJoint    = 0;        // 测试关节索引
    constexpr double kAmplitudeDeg = 5.0;      // 振幅 (度)
    constexpr double kAmplitudeRad = kAmplitudeDeg * kPI / 180.0;
    constexpr double kFrequencyHz  = 1.0;      // 正弦频率
    constexpr double kKp           = 100.0;    // 位置增益 (与 keyboard_ctrl 一致)
    constexpr double kKd           = 10.0;     // 速度增益
    constexpr int    kCtrlFreq     = 250;      // 控制频率 Hz (与 keyboard_ctrl 一致)
    constexpr auto   kPeriod       = std::chrono::microseconds(1000000 / kCtrlFreq);

    HWLOG_I("[SineTest] 监控线程已启动, 等待 /tmp/sine_test_start ...");

    while (!isStopReceived()) {
        // 轮询触发文件
        struct stat st;
        if (stat("/tmp/sine_test_start", &st) != 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            continue;
        }

        // ===== 正弦测试开始 =====
        sine_test_active_.store(true);

        // 设置 SCHED_FIFO 优先级 (避免被抢占导致 ~1s 冻结)
        {
            struct sched_param sp;
            sp.sched_priority = 20;  // 低于 sensor loop(30) 和 CAN IRQ(90)
            if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) == 0) {
                HWLOG_I("[SineTest] SCHED_FIFO priority=%d set OK", sp.sched_priority);
            }
        }

        HWLOG_SUCCESS("[SineTest] >>> 正弦测试开始! joint=%d amp=%.1fdeg freq=%.1fHz ctrl=%dHz",
                      kTestJoint, kAmplitudeDeg, kFrequencyHz, kCtrlFreq);

        leju::hw::HardwareInterface& hw = leju::hw::HardwareInterface::GetInstance();

        // 打开 CSV 日志
        FILE* csv = fopen("/tmp/sine_test.csv", "w");
        if (csv) {
            fprintf(csv, "seq,t_s,dt_ms,cmd_q0_deg,state_q0_deg,raw_q0_deg,set_cmd_us\n");
        }

        auto t_start = Clock::now();
        auto t_prev = t_start;
        int seq = 0;

        while (!isStopReceived()) {
            // 检查触发文件是否还存在
            if (stat("/tmp/sine_test_start", &st) != 0) break;

            auto t0 = Clock::now();
            double elapsed_s = std::chrono::duration<double>(t0 - t_start).count();

            // 生成正弦位置命令
            double target_rad = kAmplitudeRad * std::sin(2.0 * kPI * kFrequencyHz * elapsed_s);

            // 构建全关节命令
            leju::hw::JointCommand_t joint_cmd;
            joint_cmd.resize(num_joints_);
            for (size_t i = 0; i < num_joints_; i++) {
                joint_cmd.q[i] = (i == kTestJoint) ? target_rad : 0.0;
                joint_cmd.kp[i] = kKp;
                joint_cmd.kd[i] = kKd;
                joint_cmd.modes[i] = 2;  // CSP 位置模式
            }

            // 直接调用硬件 SDK (绕过 DDS)
            auto tc0 = Clock::now();
            hw.SetJointCommand(joint_cmd);
            auto tc1 = Clock::now();

            // 读取当前状态: 滤波后 (Kalman) + 原始 (无滤波)
            double state_q0_deg = 0, raw_q0_deg = 0;
            leju::hw::JointData_t joint_data;
            if (hw.GetJointState(joint_data) == leju::hw::HwErrorType::Success && joint_data.q.size() > 0) {
                state_q0_deg = joint_data.q[kTestJoint] * 180.0 / kPI;
            }
            leju::hw::JointData_t raw_data;
            if (hw.GetRawJointState(raw_data) == leju::hw::HwErrorType::Success && raw_data.q.size() > 0) {
                raw_q0_deg = raw_data.q[kTestJoint] * 180.0 / kPI;
            }

            // 计算 dt
            double dt_ms = std::chrono::duration<double, std::milli>(t0 - t_prev).count();
            long set_cmd_us = std::chrono::duration_cast<std::chrono::microseconds>(tc1 - tc0).count();

            // 写 CSV
            if (csv) {
                fprintf(csv, "%d,%.4f,%.2f,%.4f,%.4f,%.4f,%ld\n",
                        seq, elapsed_s, dt_ms,
                        target_rad * 180.0 / kPI, state_q0_deg, raw_q0_deg, set_cmd_us);
                if (seq % 50 == 0) fflush(csv);
            }

            // 每秒打印一次状态
            if (seq % kCtrlFreq == 0) {
                HWLOG_I("[SineTest] t=%.1fs dt=%.2fms cmd=%.2fdeg state=%.2fdeg raw=%.2fdeg set_cmd=%ldus",
                        elapsed_s, dt_ms, target_rad * 180.0 / kPI, state_q0_deg, raw_q0_deg, set_cmd_us);
            }

            t_prev = t0;
            seq++;

            // 精确等待
            auto t_end = Clock::now();
            auto work_time = t_end - t0;
            if (work_time < kPeriod)
                std::this_thread::sleep_for(kPeriod - work_time);
        }

        // ===== 正弦测试结束 =====
        // 归零
        leju::hw::JointCommand_t zero_cmd;
        zero_cmd.resize(num_joints_);
        for (size_t i = 0; i < num_joints_; i++) {
            zero_cmd.kp[i] = kKp;
            zero_cmd.kd[i] = kKd;
            zero_cmd.modes[i] = 2;
        }
        hw.SetJointCommand(zero_cmd);

        if (csv) {
            fflush(csv);
            fclose(csv);
        }

        sine_test_active_.store(false);
        HWLOG_SUCCESS("[SineTest] <<< 正弦测试结束, 共 %d 帧, CSV 已写入 /tmp/sine_test.csv", seq);
    }
}

void HardwareNode::run() {
    if(hardware_state_ < HardwareState::READY_OK) {
        HWLOG_E("[HardwareNode] Node not initialized, cannot run!");
        return;
    }

    // 运行主循环
    HWLOG_I("[HardwareNode] Starting main loop...");

    // 使用循环保持程序运行，直到收到停止命令
    // 100ms 轮询: 收到 SIGINT/SIGTERM 后需尽快进入 close() 对电机掉使能,
    // launcher 的宽限期有限, 1s 轮询会吃掉大部分预算
    while (!isStopReceived()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    HWLOG_SUCCESS("[HardwareNode] Main loop ended");
    this->close();
}

void HardwareNode::close() {
    HWLOG_I("[HardwareNode] Closing...");

    this->setStopReceived();

    this->hardware_state_.store(HardwareState::STOPPED);

    // 停止正弦测试线程
    if(sine_test_thread_.joinable()) {
        HWLOG_I("[HardwareNode] Stopping sine test thread...");
        sine_test_thread_.join();
    }

    // 停止发布线程
    if(sensors_loop_) {
        HWLOG_I("[HardwareNode] Stopping sensors loop...");
        sensors_loop_->shutdown();
        sensors_loop_ = nullptr;
    }

    leju::hw::HardwareInterface& hw = leju::hw::HardwareInterface::GetInstance();
    hw.Shutdown();

    HWLOG_SUCCESS("[HardwareNode] Closed");
}

} // namespace leju
