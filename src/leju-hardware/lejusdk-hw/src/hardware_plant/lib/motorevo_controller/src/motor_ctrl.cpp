#include "motorevo/motor_ctrl.h"
#include "motorevo/motor_log.h"
#include "canbus_sdk/canbus_sdk.h"
#include <algorithm>
#include <chrono>
#include <thread>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <unordered_set>
#include <unordered_map>
#include <limits>

namespace motorevo {

static constexpr uint8_t kEnterMotorStateFrame[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
static constexpr uint8_t kEnterResetStateFrame[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
static constexpr uint8_t kSetZeroPositionFrame[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};
static constexpr uint8_t kMultiTurnZeroFrame[8] = {0x67, 0x06, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x76}; // 多圈编码器清零

// 零点阈值常量
static constexpr float kZeroThresholdRadians = 0.5f * M_PI / 180.0f;  // 0.5度 = 0.0087弧度

// 检查位置是否存在异常跳变
static bool hasPositionJump(float previous_pos, float current_pos) {
    // 根据Motorevo电机特性，检查位置是否存在异常跳变
    // 设置安全的跳变阈值，考虑CAN通讯可能丢几帧的情况

    const float POSITION_JUMP_THRESHOLD = 0.3f;  // 位置跳变阈值（弧度）
    const float EPSILON = 1e-10f;                // 浮点数比较精度

    // 检查输入值的有效性
    if (std::isnan(current_pos) || std::isinf(current_pos) ||
        std::isnan(previous_pos) || std::isinf(previous_pos)) {
        return true;  // 无效输入认为是异常
    }

    // 计算位置变化的绝对值
    float position_change = std::abs(current_pos - previous_pos);

    // 检查是否超过阈值
    return position_change > POSITION_JUMP_THRESHOLD;
}

static bool sendCanMessage(uint8_t canbus_id, uint32_t can_id, const uint8_t* payload) {
    using namespace canbus_sdk;
    CanMessageFrame can_frame{};
    can_frame.id.SID = can_id;
    can_frame.ctrl.tx.dlc = payload_length_to_dlc(8);
    memcpy(can_frame.payload, payload, 8);
    Result<ErrorType> result = CanBusController::getInstance().sendMessage(canbus_id, can_frame);
    if (result.has_value()) {
        return result.value() == 0;
    }
    return false;
}

static bool sendCanFdBroadcast(uint8_t canbus_id, uint32_t can_id, const uint8_t payload[64]) {
    using namespace canbus_sdk;
    CanMessageFrame can_frame{};
    can_frame.id.SID = can_id;
    can_frame.ctrl.tx.dlc = payload_length_to_dlc(64);
    memcpy(can_frame.payload, payload, 64);
    Result<ErrorType> result = CanBusController::getInstance().sendMessage(canbus_id, can_frame);
    if (result.has_value()) {
        return result.value() == 0;
    }
    return false;
}

///////////////////////////////////////////////////////////////////////////////
// Class FeedbackFrame
///////////////////////////////////////////////////////////////////////////////
/**
* See Details:《Motorevo Driver User Guide v0.2.3》- 6.5.1 反馈帧格式
* 无论在什么状态和模式下，驱动板在收到报文标识符为 CAN COM ID 的报文时都会进行一
* 次应答，反馈当前电机的位置(rad)，速度(rad/s)，扭矩(N·m)。应答的报文标识符恒定本机
* CAN ID，波特率为 1Mbps，DLC 为 8 字节。
* +---------+---------+---------+---------+---------+---------+---------+---------+---------+
* | Byte0   | Byte1   | Byte2   | Byte3   | Byte4_H |Byte4_L  | Byte5   | Byte6_H | Byte7   |
* +---------+---------+---------+---------+---------+---------+---------+---------+---------+
* | Motor   |position |position |velocity |velocity | torque  | torque  | errcode | tempe-  |
* | Id      | High    | Low     | High    | Low     | High    | Low     |         | rature  |
* | 8bits   | 8bits   | 8bits   | 8bits   | 4bits   | 4bits   | 8bits   |         |         |
* +---------+---------+---------+---------+---------+---------+---------+---------+---------+
*/
FeedbackFrame::FeedbackFrame() {
    memset(payload_, 0, 8);
}

FeedbackFrame::FeedbackFrame(const uint8_t payload[8])
{
    memcpy(payload_, payload, 8);
}

FeedbackFrame::FeedbackFrame(const FeedbackFrame& other)
{
    std::memcpy(payload_, other.payload_, sizeof(payload_));
}

FeedbackFrame& FeedbackFrame::operator=(const FeedbackFrame& other)
{
    if (this != &other) {
        std::memcpy(payload_, other.payload_, sizeof(payload_));
    }
    return *this;
}

void FeedbackFrame::reset()
{
    memset(payload_, 0, 8);
}
void FeedbackFrame::set_payload(const uint8_t payload[8])
{
    memcpy(payload_, payload, 8);
}

float FeedbackFrame::position() const
{
    uint32_t pos = (payload_[1] << 8) | payload_[2];
    return int_to_float<16>(pos, kCAN_COM_THETA_MIN, kCAN_COM_THETA_MAX);
}

float FeedbackFrame::velocity() const
{
    uint32_t vel = (payload_[3] << 4) | (payload_[4] >> 4);
    return int_to_float<12>(vel, kCAN_COM_VELOCITY_MIN, kCAN_COM_VELOCITY_MAX);
}

float FeedbackFrame::torque() const
{
   uint32_t torque = ((payload_[4] & 0x0F) << 8) | payload_[5];
   return int_to_float<12>(torque, kCAN_COM_TORQUE_MIN, kCAN_COM_TORQUE_MAX);
}
uint8_t FeedbackFrame::temperature() const
{
    return payload_[7];
}

MotorErrCode FeedbackFrame::errcode() const
{
    return static_cast<MotorErrCode>(payload_[6]);
}

MotorId FeedbackFrame::motor_id() const
{
    return static_cast<MotorId>(payload_[0]);
}

///////////////////////////////////////////////////////////////////////////////
// Class FeedbackFrameFd (CAN FD 广播协议)
///////////////////////////////////////////////////////////////////////////////
FeedbackFrameFd::FeedbackFrameFd() {
    memset(payload_, 0, 8);
}

FeedbackFrameFd::FeedbackFrameFd(const uint8_t payload[8]) {
    memcpy(payload_, payload, 8);
}

void FeedbackFrameFd::set_payload(const uint8_t payload[8]) {
    memcpy(payload_, payload, 8);
}

float FeedbackFrameFd::position() const {
    uint32_t pos = (payload_[0] << 8) | payload_[1];
    return int_to_float<16>(pos, kCAN_COM_THETA_MIN, kCAN_COM_THETA_MAX);
}

float FeedbackFrameFd::velocity() const {
    uint32_t vel = (payload_[2] << 4) | (payload_[3] >> 4);
    return int_to_float<12>(vel, canfd::kVelocityMin, canfd::kVelocityMax);
}

float FeedbackFrameFd::torque() const {
    uint32_t tor = ((payload_[3] & 0x0F) << 8) | payload_[4];
    return int_to_float<12>(tor, canfd::kTorqueMin, canfd::kTorqueMax);
}

int16_t FeedbackFrameFd::temperature() const {
    return static_cast<int16_t>(payload_[5]) - 40;
}

uint16_t FeedbackFrameFd::status_word() const {
    // 大端序: 与协议帧中 pos/vel/tor 字段一致
    return (static_cast<uint16_t>(payload_[6]) << 8) | static_cast<uint16_t>(payload_[7]);
}

bool FeedbackFrameFd::is_enabled() const {
    return (status_word() & canfd::kStatusEnabled) != 0;
}

bool FeedbackFrameFd::has_fault() const {
    return (status_word() & canfd::kStatusFaultMask) != 0;
}

///////////////////////////////////////////////////////////////////////////////
// Class RevoMotor
///////////////////////////////////////////////////////////////////////////////

struct RevoMotor::data_t {
    std::atomic<float> position_raw;    // 位置(rad)，原始数据
    std::atomic<float> velocity_raw;    // 速度(rad/s)，原始数据
    std::atomic<float> torque_raw;      // 扭矩(N·m)，原始数据
    std::atomic<uint8_t> temperature;   // 温度(℃)
    std::atomic<MotorErrCode> errcode;  // 故障码

    data_t(): position_raw(0.0), velocity_raw(0.0), torque_raw(0.0), temperature(0), errcode(MotorErrCode::NO_FAULT) {}
    data_t(const data_t& other) :
        position_raw(other.position_raw.load()),
        velocity_raw(other.velocity_raw.load()),
        torque_raw(other.torque_raw.load()),
        temperature(other.temperature.load()),
        errcode(other.errcode.load()) {}

    data_t& operator=(const data_t& other) {
        if (this != &other) {
            position_raw.store(other.position_raw.load());
            velocity_raw.store(other.velocity_raw.load());
            torque_raw.store(other.torque_raw.load());
            temperature.store(other.temperature.load());
            errcode.store(other.errcode.load());
        }
        return *this;
    }
};

RevoMotor::RevoMotor(uint8_t canbus_id, MotorId id, MotorMode mode)
    : state_(MotorState::None), ctrl_mode_(mode),
    canbus_id_(canbus_id), id_(id), data_(new data_t()) {
}

RevoMotor::RevoMotor(const RevoMotor& other)
    : state_(other.state_),
      ctrl_mode_(other.ctrl_mode_),
      data_(new data_t(*other.data_)),
      canbus_id_(other.canbus_id_),
      id_(other.id_) {
}

RevoMotor::RevoMotor(RevoMotor&& other) noexcept
    : state_(other.state_),
      ctrl_mode_(other.ctrl_mode_),
      data_(other.data_),
      canbus_id_(other.canbus_id_),
      id_(other.id_) {
    other.data_ = nullptr;
}

RevoMotor& RevoMotor::operator=(const RevoMotor& other) {
    if (this != &other) {
        state_ = other.state_;
        ctrl_mode_ = other.ctrl_mode_;
        if (data_ != nullptr) {
            delete data_;
        }
        data_ = new data_t(*other.data_);
        canbus_id_ = other.canbus_id_;
        id_ = other.id_;
    }
    return *this;
}

RevoMotor& RevoMotor::operator=(RevoMotor&& other) noexcept {
    if (this != &other) {
        state_ = other.state_;
        ctrl_mode_ = other.ctrl_mode_;
        if (data_ != nullptr) {
            delete data_;  // 删除当前对象的数据
        }
        data_ = other.data_;  // 接管源对象的数据
        canbus_id_ = other.canbus_id_;
        id_ = other.id_;
        other.data_ = nullptr;  // 将源对象指针设为 nullptr
    }
    return *this;
}

uint32_t RevoMotor::getId() const {
    return id_;
}

MotorState RevoMotor::getState() const {
    return state_;
}

MotorMode RevoMotor::getControlMode() const {
    return ctrl_mode_;
}

float RevoMotor::position() const {
    return data_->position_raw.load();
}

float RevoMotor::torque() const {
    return data_->torque_raw.load();
}

float RevoMotor::velocity() const {
    return data_->velocity_raw.load();
}

uint8_t RevoMotor::temperature() const {
    return data_->temperature.load();
}

MotorErrCode RevoMotor::getErrorCode() const {
    return data_->errcode.load();
}

bool RevoMotor::receiveFeedback(const FeedbackFrame& frame) {
    /**
    * See Details:《Motorevo Driver User Guide v0.2.3》- 6.5.1 反馈帧格式
    * 无论在什么状态和模式下，驱动板在收到报文标识符为 CAN COM ID 的报文时都会进行一
    * 次应答，反馈当前电机的位置(rad)，速度(rad/s)，扭矩(N·m)。应答的报文标识符恒定本机
    * CAN ID，波特率为 1Mbps，DLC 为 8 字节。
    * +---------+---------+---------+---------+---------+---------+---------+---------+---------+
    * | Byte0   | Byte1   | Byte2   | Byte3   | Byte4_H |Byte4_L  | Byte5   | Byte6_H | Byte7   |
    * +---------+---------+---------+---------+---------+---------+---------+---------+---------+
    * | Motor   |position |position |velocity |velocity | torque  | torque  | errcode | tempe-  |
    * | Id      | High    | Low     | High    | Low     | High    | Low     |         | rature  |
    * | 8bits   | 8bits   | 8bits   | 8bits   | 4bits   | 4bits   | 8bits   |         |         |
    * +---------+---------+---------+---------+---------+---------+---------+---------+---------+
    */
    // 双重判断:报文标识符和电机ID匹配 (CAN_ID + MOTOR_ID)
    if(frame.motor_id() != id_) {
        printf("Motor feedback frame id mismatch: 0x%02X != 0x%02X\n", frame.motor_id(), id_);
        return false;
    }

    data_->position_raw.store(frame.position());
    data_->velocity_raw.store(frame.velocity());
    data_->torque_raw.store(frame.torque());
    data_->temperature.store(frame.temperature());
    data_->errcode.store(frame.errcode());

    return true;
}

RevoMotor::~RevoMotor() {
    if (data_ != nullptr) {
        delete data_;
        data_ = nullptr;
    }
}

bool RevoMotor::enterMotorState() {
    return  sendCanMessage(canbus_id_, id_, kEnterMotorStateFrame);
}

bool RevoMotor::enterRestState() {
    return  sendCanMessage(canbus_id_, id_, kEnterResetStateFrame);
}


bool RevoMotor::setZeroPosition() {
    return  sendCanMessage(canbus_id_, id_, kSetZeroPositionFrame);
}

bool RevoMotor::multiTurnZero() {
    uint32_t can_id = id_ + 0x600;
    return  sendCanMessage(canbus_id_, can_id, kMultiTurnZeroFrame);
}

/**
 * See Details:《Motorevo Driver User Guide v0.2.3》- 6.3.2 使用 CAN 控制电机在力位混合模式（P-T-M Mode）运行
 *
 * 控制帧为 CAN 标准帧，帧 ID 为 Motor ID，波特率为 1Mhz，控制帧和控制框图的映射关系定义如下:
 *
 * +---------+---------+---------+---------+---------+---------+---------+---------+---------+---------+
 * | Byte0   | Byte1   | Byte2   | Byte3_H | Byte3_L | Byte4   | Byte5   | Byte6_H | Byte6_L | Byte7   |
 * +---------+---------+---------+---------+---------+---------+---------+---------+---------+---------+
 * | θ_ref   | θ_ref   | V_ref   | V_ref   | Kp      | Kp      | Kd      | Kd      | T_ref   | T_ref   |
 * | High    | Low     | High    | Low     | High    | Low     | High    | Low     | High    | Low     |
 * | 8bits   | 8bits   | 8bits   | 4bits   | 4bits   | 8bits   | 8bits   | 4bits   | 4bits   | 8bits   |
 * +---------+---------+---------+---------+---------+---------+---------+---------+---------+---------+
 */
bool RevoMotor::controlPTM(float pos, float vel, float torque, float pos_kp, float pos_kd) {
    if (ctrl_mode_ != MotorMode::TorquePositionMixControlMode) {
        RWLOG_E("Motor is not in P-T-M mode");
        return false;
    }

    uint8_t payload[8]{};

    uint16_t theta_ref = float_to_int<16>(pos, kCAN_COM_THETA_MIN, kCAN_COM_THETA_MAX);
    uint16_t V_ref = float_to_int<12>(vel, kCAN_COM_VELOCITY_MIN, kCAN_COM_VELOCITY_MAX);
    uint16_t Kp    = float_to_int<12>(pos_kp, kCAN_COM_POS_KP_MIN, kCAN_COM_POS_KP_MAX);
    uint16_t Kd    = float_to_int<12>(pos_kd, kCAN_COM_POS_KD_MIN, kCAN_COM_POS_KD_MAX);
    uint16_t T_ref = float_to_int<12>(torque, kCAN_COM_TORQUE_MIN, kCAN_COM_TORQUE_MAX);

    // θ_ref: Byte 0, Byte1
    payload[0] = (theta_ref >> 8) & 0xFF;
    payload[1] = theta_ref & 0xFF;
    // V_ref: Byte2, Byte3_H
    payload[2] = (V_ref >> 4) & 0xFF;
    payload[3] = ((V_ref & 0x0F) << 4) | ((Kp >> 8) & 0x0F);
    // Kp: Byte3_L, Byte4
    payload[4] = Kp & 0xFF;
    // Kd: Byte5, Byte6_H
    payload[5] = (Kd >> 4) & 0xFF;
    payload[6] = ((Kd & 0x0F) << 4) | ((T_ref >> 8) & 0x0F);
    // T_ref: Byte6_L, Byte7
    payload[7] = T_ref & 0xFF;

    return sendCanMessage(canbus_id_, id_, payload);
}

/**
 * See Details:《Motorevo Driver User Guide v0.2.3》 - 6.3.3 使用 CAN 控制电机在速度模式（Velocity Mode）运行
 *
 * 控制帧为 CAN 标准帧，帧 ID 为 Motor ID，波特率为 1Mhz，控制帧和控制框图的映射关系定义如下:
 *
 * +---------+---------+---------+---------+---------+---------+---------+---------+---------+---------+
 * | Byte0   | Byte1   | Byte2   | Byte3_H | Byte3_L | Byte4   | Byte5   | Byte6_H | Byte6_L | Byte7   |
 * +---------+---------+---------+---------+---------+---------+---------+---------+---------+---------+
 * | V_ref   | V_ref   | Kp      | Kp      | Kd      | Kd      | Ki      | Ki      | unused  | 0xAC    |
 * | High    | Low     | High    | Low     | High    | Low     | High    | Low     |         | Identi~ |
 * | 8bits   | 8bits   | 8bits   | 4bits   | 4bits   | 8bits   | 8bits   | 4bits   |         |         |
 * +---------+---------+---------+---------+---------+---------+---------+---------+---------+---------+
 */
bool RevoMotor::controlVelocity(float vel, float vel_kp, float vel_ki, float vel_kd) {
    if(ctrl_mode_ != MotorMode::VelocityMode) {
        RWLOG_E("Motor is not in Velocity Mode");
    }

    uint8_t payload[8]{};

    constexpr float kCAN_COM_VEL_KP_MIN = 0.0f;
    constexpr float kCAN_COM_VEL_KP_MAX = 250.0f;
    constexpr float kCAN_COM_VEL_KD_MIN = 0.0f;
    constexpr float kCAN_COM_VEL_KD_MAX = 50.0f;
    constexpr float kCAN_COM_VEL_KI_MIN = 0.0f;
    constexpr float kCAN_COM_VEL_KI_MAX = 0.05f;

    uint16_t V_ref = float_to_int<16>(vel, kCAN_COM_VELOCITY_MIN, kCAN_COM_VELOCITY_MAX);
    uint16_t Kp    = float_to_int<12>(vel_kp, kCAN_COM_VEL_KP_MIN, kCAN_COM_VEL_KP_MAX);
    uint16_t Kd    = float_to_int<12>(vel_kd, kCAN_COM_VEL_KD_MIN, kCAN_COM_VEL_KD_MAX);
    uint16_t Ki    = float_to_int<12>(vel_ki, kCAN_COM_VEL_KI_MIN, kCAN_COM_VEL_KI_MAX);

    payload[0] = (V_ref >> 8) & 0xFF;
    payload[1] = V_ref & 0xFF;
    payload[2] = (Kp >> 4) & 0xFF;
    payload[3] = ((Kp & 0x0F) << 4) | ((Kd >> 8) & 0x0F);
    payload[4] = Kd & 0xFF;
    payload[5] = (Ki >> 4) & 0xFF;
    payload[6] = (Ki & 0x0F) << 4;
    payload[7] = 0xAC;

    return sendCanMessage(canbus_id_, id_, payload);
}

/**
 * See Details:《Motorevo Driver User Guide v0.2.3》 - 使用 CAN 控制电机在力矩模式（Torque Mode）运行
 *
 * 控制帧为 CAN 标准帧，帧 ID 为 Motor ID，波特率为 1Mhz，控制帧和控制框图的映射关系定义如下:
 *
 * +--------+--------+--------+--------+--------+--------+--------+----------------+
 * | Byte0  | Byte1  | Byte2  | Byte3  | Byte4  | Byte5  | Byte6  | Byte7          |
 * +--------+--------+--------+--------+--------+--------+--------+----------------+
 * | T_ref  | T_ref  | unused | unused | unused | unused | unused | 0xAB           |
 * | High   | Low    |        |        |        |        |        | Identification |
 * +--------+--------+--------+--------+--------+--------+--------+----------------+
 */
bool RevoMotor::controlTorque(float torque) {
    if(ctrl_mode_ != MotorMode::TorqueMode) {
        RWLOG_E("Motor is not in Torque Mode");
        return false;
    }
    uint8_t payload[8]{};

    uint16_t T_ref = float_to_int<16>(torque, kCAN_COM_TORQUE_MIN, kCAN_COM_TORQUE_MAX);

    payload[0] = (T_ref >> 8) & 0xFF;
    payload[1] = T_ref & 0xFF;
    payload[7] = 0xAB;

    return sendCanMessage(canbus_id_, id_, payload);
}

///////////////////////////////////////////////////////////////////////////////
// CAN FD 广播协议编码
///////////////////////////////////////////////////////////////////////////////

void RevoMotor::encodeMitFrame(uint8_t buf[8], float pos, float vel, float torque, float kp, float kd) const {
    uint16_t pos_raw = float_to_int<16>(pos, kCAN_COM_THETA_MIN, kCAN_COM_THETA_MAX);
    uint16_t vel_raw = float_to_int<12>(vel, canfd::kVelocityMin, canfd::kVelocityMax);
    uint16_t kp_raw  = float_to_int<12>(kp,  canfd::kKpMin, canfd::kKpMax);
    uint16_t kd_raw  = float_to_int<12>(kd,  canfd::kKdMin, canfd::kKdMax);
    uint16_t tor_raw = float_to_int<12>(torque, canfd::kTorqueMin, canfd::kTorqueMax);

    buf[0] = (pos_raw >> 8) & 0xFF;
    buf[1] = pos_raw & 0xFF;
    buf[2] = (vel_raw >> 4) & 0xFF;
    buf[3] = ((vel_raw & 0x0F) << 4) | ((kp_raw >> 8) & 0x0F);
    buf[4] = kp_raw & 0xFF;
    buf[5] = (kd_raw >> 4) & 0xFF;
    buf[6] = ((kd_raw & 0x0F) << 4) | ((tor_raw >> 8) & 0x0F);
    buf[7] = tor_raw & 0xFF;
}

void RevoMotor::encodeControlFrame(uint8_t buf[8], uint8_t cmd) {
    memset(buf, 0xFF, 7);
    buf[7] = cmd;
}

void RevoMotor::encodePosVelFrame(uint8_t buf[8], float pos, float vel) {
    memcpy(&buf[0], &pos, sizeof(float));
    memcpy(&buf[4], &vel, sizeof(float));
}

void RevoMotor::encodeVelFrame(uint8_t buf[8], float vel) {
    memcpy(&buf[0], &vel, sizeof(float));
    memset(&buf[4], 0, 4);
}

bool RevoMotor::receiveFeedbackFd(const FeedbackFrameFd& frame, MotorErrCode errcode) {
    data_->position_raw.store(frame.position());
    data_->velocity_raw.store(frame.velocity());
    data_->torque_raw.store(frame.torque());
    data_->temperature.store(static_cast<uint8_t>(std::max(0, static_cast<int>(frame.temperature()) + 40)));
    // errcode 由 internalMessageCallback 解析 status_word 后传入
    data_->errcode.store(errcode);
    return true;
}


///////////////////////////////////////////////////////////////////////////////
// RevoMotorControl
///////////////////////////////////////////////////////////////////////////////

RevoMotorControl::RevoMotorControl(const std::string& canbus_name, MotorProtocol protocol)
    : canbus_name_(canbus_name), protocol_(protocol), zero_torque_mode_(false) {
}

bool RevoMotorControl::initializeCanBusAndDevices(const std::vector<RevoMotorConfig_t>& motor_configs) {
    // 获取CAN总线控制器实例
    auto& canbus_controller = canbus_sdk::CanBusController::getInstance();

    // 获取总线ID
    auto bus_id_result = canbus_controller.getBusIdByName(canbus_name_);
    if (!bus_id_result.has_value()) {
        RWLOG_E("Failed to get bus ID for CAN bus %s", canbus_name_.c_str());
        return false;
    }

    uint8_t bus_id = bus_id_result.value();
    canfd_bus_id_ = bus_id;
    RWLOG_I("Using CAN bus %s with bus_id: %d", canbus_name_.c_str(), bus_id);

    // 初始化回调上下文
    msg_callback_context_.userdata = this;
    tef_callback_context_.userdata = this;

    // 注册CAN消息回调
    canbus_sdk::CallbackParams callback_params{
        .msg_callback = internalMessageCallback,
        .msg_cb_ctx = &msg_callback_context_,
        .tef_callback = internalTefEventCallback,
        .tef_cb_ctx = &tef_callback_context_
    };

    // 遍历电机配置，创建电机实例并注册设备
    for (const auto& config : motor_configs) {
        // 注册设备到CAN总线
        if (!config.ignore) {
            auto register_result = canbus_controller.registerDevice(
                canbus_sdk::DeviceInfo(
                    config.name.c_str(),
                    canbus_sdk::DeviceType::MOTOR,
                    static_cast<canbus_sdk::DeviceId>(config.id),
                    nullptr
                ),
                canbus_name_,
                callback_params
            );

            if (!register_result.has_value()) {
                RWLOG_E("Failed to register device %s (ID: %d): %s",
                         config.name.c_str(), config.id,
                         canbus_sdk::errorToString(register_result.error()));
                return false;
            }

            RWLOG_I("Device registered successfully: %s (ID: %d)", config.name.c_str(), config.id);
        }

        // 创建电机实例，使用默认模式，后续可以根据配置设置具体模式
        RevoMotor motor(bus_id, config.id, MotorMode::TorquePositionMixControlMode);

        // 初始化电机控制数据
        motor_ctrl_datas_.emplace(
            std::piecewise_construct,
            std::forward_as_tuple(config.id),
            std::forward_as_tuple(false, std::move(motor), config)
        );
    }

    return true;
}

bool RevoMotorControl::init(const std::vector<RevoMotorConfig_t> &motor_configs, bool calibrate, bool move_to_zero, bool enable_all) {
    // 初始化CAN总线并注册电机设备
    if (!initializeCanBusAndDevices(motor_configs)) {
        return false;
    }

    /*
     * 不管有没有故障码都执行 Enter Reset State  清除一下
     * from: 《Motorevo Driver User Guide v0.2.3》- 6.5.1 反馈帧格式 - 故障码
     * notes: "一旦驱动板检测到故障，将会从 Motor State 自动切回 Rest State 以保护驱动器和电机，同
     *        时将返回故障码，提示用户电机运行异常。此时您应仔细检查电机运行工况是否正常，在排
     *        除异常情况后，发送 Enter Rest State 命令清除故障码"
     */
    if(!disableAll()) {
        RWLOG_FAILURE("disableAll() 失败--->无法正常初始运行");
        return false;
    }

    // 零点校准: 先失能再清零，避免清零瞬间位置突变导致电机抽搐
    // multiTurnZeroAll + calibrateMotors 已移至 calibrateAllMotorsAndHold(),
    // 由用户交互确认后通过 'c' 按键触发，避免 Ctrl+C 退出时硬件已清零但零点文件未保存
    if (calibrate) {
        RWLOG_I("标定模式: 跳过 init 阶段的硬件清零和标定, 等待用户确认后执行");
    }

    // 单电机标定模式: 不整体使能, 所有电机保持失能静止;
    // calibrateSingleMotor() 会对操作者选中的那一个电机单独执行 使能->读位置->标零->失能
    if (!enable_all) {
        RWLOG_WARNING("跳过整体使能 (enable_all=false), 所有电机保持失能状态");
    } else {
    // 使能所有电机
    if(!enableAll()) {
        RWLOG_FAILURE("使能所有电机失败--->无法正常初始运行");
        return false;
    }

    // 等待所有电机第一个反馈到达
    if (!waitForAllMotorsFeedback(1500)) {
        RWLOG_W("Some motors did not receive initial feedback, continuing anyway");
        for (const auto& [id, data] : motor_ctrl_datas_) {
            if (!data.config.ignore && !data.feedback_received) {
                RWLOG_WARNING("电机 ID %u (%s) 没有收到初始反馈", id, data.config.name.c_str());
            }
        }
    }

    // 使能后: 用当前 joint position 作为 hold 目标, 用 default kp/kd 锁住
    // 目的: 避免 cmd.pos=0 带来的首帧位置跳变和使能后重力下漂,
    //       同时保留标定 zero_offset, 让后续 moveToZero / RL 控制器
    //       读到的 joint position 仍然是"相对标定零点"的真实偏差
    // 公式: joint_pos = (negtive ? -raw : raw) - zero_offset
    //       把 joint_pos 作为 cmd.pos 发送, 经 writeSingleFrame 反向变换后
    //       raw_target 恰好等于当前 raw_pos, 电机保持不动
    {
        std::map<MotorId, RevoMotorCmd_t> hold_cmd;
        for (auto& [id, data] : motor_ctrl_datas_) {
            if (!data.config.ignore && data.feedback_received) {
                float raw_pos = data.motor.position();
                float adjusted = data.config.negtive ? -raw_pos : raw_pos;
                float current_joint_pos = adjusted - data.config.zero_offset;

                hold_cmd[id] = {
                    .pos = static_cast<double>(current_joint_pos),
                    .vel = 0.0,
                    .torque = 0.0,
                    .kp = static_cast<double>(data.config.default_params.kp_pos),
                    .kd = static_cast<double>(data.config.default_params.kd_pos)
                };
            }
        }
        if (!hold_cmd.empty()) {
            setTargetPositions(hold_cmd);
            write();
            RWLOG_I("使能后锁定: 用当前 joint_pos 作为 hold 目标, 保留标定 zero_offset");
        }
    }
    }

    RWLOG_I("==================== MotorRevo Calibration: %s ====================", calibrate ? "True" : "False");
    if (calibrate) {
        // calibrateMotors 已移至 calibrateAllMotorsAndHold(), 不在此处执行
        RWLOG_I("标定模式: 跳过 init 阶段的 calibrateMotors, 等待用户确认后通过 calibrateAllMotorsAndHold 执行");
    }
    else {
        // 检查并更新带有magic number的零点偏移
        // 先获取所有电机的当前位置
        auto raw_positions = getRawPositions();
        // 遍历判断并更新offset
        for (auto& [id, data] : motor_ctrl_datas_) {
            if (data.isEnabled() && !data.config.ignore && std::abs(data.config.zero_offset - kOFFSET_MAGIC_NUMBER) < 1e-6f) {
                // 使用当前位置作为零点偏移
                auto it = raw_positions.find(id);
                if (it != raw_positions.end()) {
                    float current_raw_position = it->second;
                    data.config.zero_offset = data.config.negtive? -current_raw_position : current_raw_position;
                    RWLOG_WARNING("Saving motor ID %u zero offset: %.6f", id, current_raw_position);
                }
            }
        }
    }

    // 打印电机原始位置
    auto raw_positions = getRawPositions();
    std::cout << "\033[32m电机 IDs: \033[0m\n";
    for (const auto& [id, pos] : raw_positions) {
        std::cout << id << " ";
    }
    std::cout << std::endl;

    std::cout << "\033[32m电机原始位置: \033[0m\n";
    for (const auto& [id, pos] : raw_positions) {
        std::cout << std::fixed << std::setprecision(4) << pos << " ";
    }
    std::cout << std::endl;

    // 打印电机+上零点后的位置
    auto positions = getPositions();
    std::cout << "\033[32m电机加上零点后的位置: \033[0m\n";
    for (const auto& [id, pos] : positions) {
        std::cout << std::fixed << std::setprecision(4) << pos << " ";
    }
    std::cout << std::endl;

    // 归零 --> 移动到零点的位置
    if (move_to_zero) {
        moveToZero();
    } else {
        RWLOG_I("跳过归零动作 (move_to_zero=false)");
    }

    // 打印电机归零后的位置
    auto final_positions = getPositions();

    std::cout << "\033[32m电机回零后的位置(误差大于0.5度报红):\033[0m\n";
    for (const auto& [id, pos] : final_positions) {
        if (std::abs(pos) > kZeroThresholdRadians) {
            std::cout << "\033[31m" << std::fixed << std::setprecision(4) << pos << "\033[0m ";
        } else {
            std::cout << std::fixed << std::setprecision(4) << pos << " ";
        }
    }
    std::cout << std::endl;

    return true;
}

int RevoMotorControl::enableMotor(MotorId id, int timeout_ms) {
    /**
     * @brief 使能指定电机
     * @param id 电机ID
     * @param timeout_ms 超时时间(毫秒)
     * @return 返回值含义：
     *         - 0: 使能成功
     *         - 1: 操作超时
     *         - 2: 发送消息失败
     *         - 3: 电机不存在
     *         - 负数: 电机故障码的负数
     */
    // 查找电机控制数据
    auto data_it = motor_ctrl_datas_.find(id);
    if (data_it == motor_ctrl_datas_.end()) {
        RWLOG_W("Cannot enable unknown motor ID: 0x%02X", id);
        return 3; // 电机不存在
    }

    // 设置状态为发送中
    data_it->second.startOperation(Operation::ENABLE);

    // 发送enable指令: CANFD_BROADCAST 总线走单槽位广播控制帧, 单帧协议保持点对点寻址
    bool sent = (protocol_ == MotorProtocol::CANFD_BROADCAST)
        ? sendSingleSlotControlFrame(id, canfd::kCmdEnable)
        : data_it->second.motor.enterMotorState();
    if (!sent) {
        RWLOG_E("Failed to send enable command to motor 0x%02X", id);
        data_it->second.updateOperationStatus(OperationStatus::FAILED);
        return 2; // 发送消息失败
    }

    // RWLOG_I("Enable command sent to motor 0x%02X, waiting for confirmation...", id);

    // 等待操作完成，waitForOperationStatus会检查故障码和超时
    int result = waitForOperationStatus(id, Operation::ENABLE, OperationStatus::SUCCESS, timeout_ms, "Enable");
    if (result != 0) {
        return result; // 返回具体的错误码 (1=超时, 负数=故障码)
    }

    return 0; // 成功
}

void RevoMotorControl::setZeroTorqueMode(bool enable) {
    zero_torque_mode_ = enable;
    RWLOG_I("Zero torque mode %s", enable ? "enabled" : "disabled");
}

void RevoMotorControl::setRuntimeSkipActive(bool active) {
    bool has_skip_motor = false;
    for (const auto& [id, data] : motor_ctrl_datas_) {
        if (data.config.runtime_skip) {
            has_skip_motor = true;
            break;
        }
    }
    runtime_skip_active_.store(active, std::memory_order_relaxed);
    if (has_skip_motor) {
        RWLOG_I("Runtime skip %s: runtime_skip 电机%s发送运动帧",
                active ? "actived" : "deactived", active ? "停止" : "恢复");
    }
}

int RevoMotorControl::disableMotor(MotorId id, int timeout_ms) {
    /**
     * @brief 失能指定电机
     * @param id 电机ID
     * @param timeout_ms 超时时间(毫秒)
     * @return 返回值含义：
     *         - 0: 失能成功
     *         - 1: 操作超时
     *         - 2: 发送消息失败
     *         - 3: 电机不存在
     *         - 负数: 电机故障码的负数
     */
    // 查找电机控制数据
    auto data_it = motor_ctrl_datas_.find(id);
    if (data_it == motor_ctrl_datas_.end()) {
        RWLOG_W("Cannot disable unknown motor ID: 0x%02X", id);
        return 3; // 电机不存在
    }

    // 设置状态为发送中, 会在feedback和TEF回调中更新
    data_it->second.startOperation(Operation::DISABLE);

    // 发送 Enter Reset State 指令: CANFD_BROADCAST 总线走单槽位广播控制帧, 单帧协议保持点对点寻址
    bool sent = (protocol_ == MotorProtocol::CANFD_BROADCAST)
        ? sendSingleSlotControlFrame(id, canfd::kCmdDisable)
        : data_it->second.motor.enterRestState();
    if (!sent) {
        RWLOG_E("Failed to send disable command to motor 0x%02X", id);
        data_it->second.updateOperationStatus(OperationStatus::FAILED);
        return 2; // 发送消息失败
    }

    // RWLOG_I("Disable command sent to motor 0x%02X, waiting for confirmation...", id);

    // 等待操作完成，waitForOperationStatus会检查故障码和超时
    int result = waitForOperationStatus(id, Operation::DISABLE, OperationStatus::SUCCESS, timeout_ms, "Disable");
    if (result != 0) {
        return result; // 返回具体的错误码 (1=超时, 负数=故障码)
    }

    return 0; // 成功
}

bool RevoMotorControl::multiTurnZero(MotorId id, int timeout_ms) {
    (void)timeout_ms;
    // 查找电机控制数据
    auto data_it = motor_ctrl_datas_.find(id);
    if (data_it == motor_ctrl_datas_.end()) {
        RWLOG_W("Cannot do multi-turn zero for unknown motor ID: 0x%02X", id);
        return false;
    }

    // 多圈清零指令没有反馈帧确认(经典单帧协议和 CANFD 广播协议均如此) ——
    // multiTurnZeroAll() 的既有实现也是发送后固定 sleep, 不做状态轮询等待。
    // 曾尝试用 waitForOperationStatus 等待确认, 但单帧协议总线上电机失能时
    // writeSingleFrame() 会跳过它(motor_ctrl.cpp:1739 !isEnabled() 直接 continue),
    // 失能期间完全没有周期性反馈帧, 永远等不到状态变化, 只会超时(真机复现:
    // 手臂电机 0x03 标零成功但一直报超时) —— 因此改回跟 multiTurnZeroAll() 一致的
    // 发送后固定等待, 不依赖反馈状态。
    bool sent = (protocol_ == MotorProtocol::CANFD_BROADCAST)
        ? sendSingleSlotControlFrame(id, canfd::kCmdSaveZero)
        : data_it->second.motor.multiTurnZero();
    if (!sent) {
        RWLOG_E("Failed to send multi-turn zero command to motor 0x%02X", id);
        return false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    RWLOG_I("Multi-turn zero command sent to motor 0x%02X", id);
    return true;
}

bool RevoMotorControl::setMotorConfig(MotorId id, const RevoMotorConfig_t& config) {
    // 查找电机控制数据
    auto data_it = motor_ctrl_datas_.find(id);
    if (data_it == motor_ctrl_datas_.end()) {
        RWLOG_W("Cannot set config for unknown motor ID: 0x%02X", id);
        return false;
    }

    // 设置电机配置
    data_it->second.config = config;
    target_updated_ = true; // 需要刷新一下位置(零点可能变化)
    broadcast_ready_.store(true, std::memory_order_relaxed);

    // 记录配置设置
    RWLOG_I("Set motor config for ID: 0x%02X, name: %s, ratio: %.1f, negtive: %s",
            id, config.name.c_str(), config.ratio, config.negtive ? "true" : "false");

    return true;
}

void RevoMotorControl::internalMessageCallback(canbus_sdk::CanMessageFrame* frame, const canbus_sdk::CallbackContext* context) {
    /**
     * 高频函数最好不好使用 RWLOG_... 等日志函数
     *
     */
    if(!frame) {
        return;
    }

    if (!context || !context->userdata) {
        if (frame) canbus_sdk::freeCanMessageFrame(frame);
        return;
    }

    // 从context获取类实例指针
    RevoMotorControl* controller = static_cast<RevoMotorControl*>(context->userdata);
    if (!controller) {
        canbus_sdk::freeCanMessageFrame(frame);
        return;
    }

    // 拷贝 payload 并释放 CAN 帧
    uint8_t payload[8];
    memcpy(payload, frame->payload, 8);

    MotorId motor_id;
    if (controller->protocol_ == MotorProtocol::CANFD_BROADCAST) {
        // CAN FD: motor_id from CAN frame SID (feedback frame CAN ID = motor ID)
        motor_id = static_cast<MotorId>(frame->id.SID);
    } else {
        // Old protocol: motor_id from payload[0]
        FeedbackFrame feedback(payload);
        motor_id = feedback.motor_id();
    }

    // !!! 重要！释放CAN帧
    canbus_sdk::freeCanMessageFrame(frame);

    // 查找对应的电机控制数据
    auto data_it = controller->motor_ctrl_datas_.find(motor_id);
    if (data_it == controller->motor_ctrl_datas_.end()) {
        return;
    }

    if (controller->protocol_ == MotorProtocol::CANFD_BROADCAST) {
        // CAN FD 广播协议反馈处理
        FeedbackFrameFd feedback_fd(payload);

        // 解析 status_word 位域为 MotorErrCode (优先级从高到低)
        MotorErrCode err = MotorErrCode::NO_FAULT;
        if (feedback_fd.has_fault()) {
            uint16_t status = feedback_fd.status_word();
            if (status & canfd::kStatusOverVoltage)       err = MotorErrCode::DC_BUS_OVER_VOLTAGE;
            else if (status & canfd::kStatusOverCurrent)  err = MotorErrCode::DC_BUS_CURRENT_OVERLOAD;
            else if (status & canfd::kStatusOverTemp)     err = MotorErrCode::DRIVER_BOARD_OVERHEAT;
            else if (status & canfd::kStatusOverSpeed)    err = MotorErrCode::OUTPUT_SPEED_OUT_OF_RANGE;
            else if (status & canfd::kStatusEncoderFault) err = MotorErrCode::ENCODER_ANGLE_FAULT;
            else if (status & canfd::kStatusDriverFault)  err = MotorErrCode::DRV_DRIVER_FAULT;
            else if (status & canfd::kStatusOverload)     err = MotorErrCode::DC_BUS_CURRENT_OVERLOAD;
            else if (status & canfd::kStatusStall)        err = MotorErrCode::STUCK_PROTECTION;
            else if (status & canfd::kStatusDeviation)    err = MotorErrCode::OUTPUT_ANGLE_OUT_OF_RANGE;
            else if (status & canfd::kStatusCustom)       err = MotorErrCode::DC_BUS_OVER_VOLTAGE; // 自定义故障位
        }
        data_it->second.fault_code.store(err);
        const uint16_t status_word = feedback_fd.status_word();
        data_it->second.last_status_word.store(status_word, std::memory_order_relaxed);
        data_it->second.status_word_valid.store(true, std::memory_order_release);
        if (feedback_fd.is_enabled()) {
            data_it->second.disabled_feedback_count.store(0, std::memory_order_release);
        } else {
            uint8_t count = data_it->second.disabled_feedback_count.load(std::memory_order_relaxed);
            if (count != std::numeric_limits<uint8_t>::max()) {
                data_it->second.disabled_feedback_count.store(static_cast<uint8_t>(count + 1),
                                                               std::memory_order_release);
            }
        }

        // CAN FD: 跳过位置跳变检测 (CAN FD 广播协议首次反馈位置差异大，跳变检测会误拒绝)
        data_it->second.motor.receiveFeedbackFd(feedback_fd, err);

        // 诊断: 记录 bcan0 motor1 的 CAN 接收时间戳 + 反馈位置
        // 默认编译关闭, 避免 RX 回调线程做同步文件 IO; 开启: cmake -DLEJU_ENABLE_DIAG_FILELOG=ON
#ifdef LEJU_ENABLE_DIAG_FILELOG
        if (motor_id == 1 && controller->canfd_bus_id_ == 0) {
            static FILE* rxlog = fopen("/tmp/can_rx.log", "w");
            if (rxlog) {
                auto pos = data_it->second.motor.position();
                auto now = std::chrono::steady_clock::now();
                auto us = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count();
                fprintf(rxlog, "%ld,%.6f\n", us, pos);
                static int rx_count = 0;
                if (++rx_count % 100 == 0) fflush(rxlog);
            }
        }
#endif
    } else {
        // Old protocol (keep existing logic)
        FeedbackFrame feedback(payload);

        // 故障码检测
        auto errcode = feedback.errcode();
        if(errcode != MotorErrCode::NO_FAULT) {
            data_it->second.fault_code.store(errcode);  // 更新电机存在故障码
            std::string errcode_str = MotorErrCode2string(errcode);
            printf("\033[31m[%s] 电机 0x%02X 故障码: 0x%03X, 描述: %s\033[0m\n", controller->canbus_name_.c_str(), motor_id, static_cast<uint8_t>(errcode), errcode_str.c_str());
            // TODO: automatic recovery 自动恢复: 执行enter rest state 清除故障码

            // 有故障码也要应该要更新
            // return;
        }

        // 位置跳变检测
        auto prev_pos = data_it->second.motor.position();
        auto curr_pos = feedback.position();
        if(data_it->second.feedback_received.load() && hasPositionJump(prev_pos, curr_pos)) {
            printf("\033[31m[%s] 电机 0x%02X 位置跳变: %.4f ---> %.4f\033[0m\n", controller->canbus_name_.c_str(), motor_id, prev_pos, curr_pos);
            return;
        }

        // TODO: 温度过高检测

        // 更新电机状态
        data_it->second.motor.receiveFeedback(feedback);
    }

    // Common: update operation status
    const uint64_t now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    data_it->second.last_feedback_time_ms.store(now_ms, std::memory_order_release);
    data_it->second.ever_received_feedback.store(true, std::memory_order_release);
    controller->last_feedback_time_ms_.store(now_ms, std::memory_order_relaxed);
    data_it->second.feedback_received.store(true);
    auto current_op = data_it->second.operation.load();
    if (current_op == Operation::ENABLE || current_op == Operation::DISABLE || current_op == Operation::MULTI_TURN_ZERO) {
        bool operation_confirmed = true;
        if (controller->protocol_ == MotorProtocol::CANFD_BROADCAST) {
            const uint16_t status = data_it->second.last_status_word.load(std::memory_order_relaxed);
            const bool enabled = (status & canfd::kStatusEnabled) != 0;
            if (current_op == Operation::ENABLE) operation_confirmed = enabled;
            if (current_op == Operation::DISABLE) operation_confirmed = !enabled;
        }
        if (operation_confirmed) {
            data_it->second.updateOperationStatus(OperationStatus::SUCCESS);
        }
    }
    // printf("Received feedback from motor %d: pos=%.4f, vel=%.4f, torque=%.4f\n",
    //          motor_id, feedback_frame.position(), feedback_frame.velocity(), feedback_frame.torque());
}

void RevoMotorControl::internalTefEventCallback(canbus_sdk::CanMessageFrame* frame, const canbus_sdk::CallbackContext* context) {
    if(!frame) {
        return;
    }

    if (!context || !context->userdata) {
        if (frame) canbus_sdk::freeCanMessageFrame(frame);
        return;
    }

    // 从context获取类实例指针
    RevoMotorControl* controller = static_cast<RevoMotorControl*>(context->userdata);
    if (!controller) {
        canbus_sdk::freeCanMessageFrame(frame);
        return;
    }

    MotorId motor_id = static_cast<MotorId>(frame->id.SID); // CAN_ID ---> MotorId

    // 拷贝 payload
    uint8_t payload[8];
    memcpy(payload, frame->payload, 8);

    // 释放CAN帧
    canbus_sdk::freeCanMessageFrame(frame);

    // 查找电机控制数据
    auto data_it = controller->motor_ctrl_datas_.find(motor_id);
    if (data_it == controller->motor_ctrl_datas_.end()) {
        printf("TEF callback for unknown motor ID: 0x%02X\n", motor_id);
        return;
    }

    if (data_it->second.operation_status != OperationStatus::PENDING) {
        // 减少 memcmp 次数
        return;
    }
    
    // 根据payload判断是不是控制指令
    if (memcmp(payload, kEnterMotorStateFrame, 8) == 0) {
        // enable指令发送成功
        if (data_it->second.operation == Operation::ENABLE &&
            data_it->second.operation_status == OperationStatus::PENDING) {
                data_it->second.updateOperationStatus(OperationStatus::SUCCESS);
                printf("Motor 0x%02X enable command sent successfully\n", motor_id);
        }
    } else if (memcmp(payload, kEnterResetStateFrame, 8) == 0) {
        // disable指令发送成功
        if (data_it->second.operation == Operation::DISABLE &&
            data_it->second.operation_status == OperationStatus::PENDING) {
                data_it->second.updateOperationStatus(OperationStatus::SUCCESS);
                printf("Motor 0x%02X disable command sent successfully\n", motor_id);
        }
    } else if (memcmp(payload, kMultiTurnZeroFrame, 8) == 0) {
        // multi-turn zero指令发送成功
        if (data_it->second.operation == Operation::MULTI_TURN_ZERO &&
            data_it->second.operation_status == OperationStatus::PENDING) {
                data_it->second.updateOperationStatus(OperationStatus::SUCCESS);
                printf("Motor 0x%02X multi-turn zero command sent successfully\n", motor_id);
        }
    }
}

static std::vector<std::vector<float>> interpolate_positions_with_speed(
    const std::vector<float>& start_positions,
    const std::vector<float>& target_positions,
    float speed,
    float dt,
    float timeout = 5.0f) {

    if (start_positions.size() != target_positions.size()) {
        RWLOG_E("Start and target positions size mismatch");
        return {};
    }

    std::cout << "\033[32m插值起始位置: \033[0m\n";
    for (const auto& pos : start_positions) {
        std::cout << std::fixed << std::setprecision(4) << pos << " ";
    }
    std::cout << std::endl;
    std::cout << "\033[32m插值目标位置: \033[0m\n";
    for (const auto& pos : target_positions) {
        std::cout << std::fixed << std::setprecision(4) << pos << " ";
    }
    std::cout << std::endl;

    int num_motors = start_positions.size();

    // 计算每个电机的距离，找到最大距离的电机作为基准
    float max_distance = 0.0f;
    for (int i = 0; i < num_motors; ++i) {
        float distance = std::abs(target_positions[i] - start_positions[i]);
        if (distance > max_distance) {
            max_distance = distance;
        }
    }

    // 根据最大距离和速度计算总时间
    float total_time = std::min((max_distance / speed), timeout); // 归零运动不要超timeout秒

    // 根据总时间和时间步长计算插值步数
    int steps = static_cast<int>(total_time / dt);
    if (steps < 2) steps = 2;  // 至少需要2步：起始位置和目标位置

    RWLOG_D("Interpolation: max_distance=%.4f, speed:%.4f, time=%.4fs, steps=%d", max_distance, speed, total_time, steps);

    // 使用线性插值计算每个时间步的电机位置
    std::vector<std::vector<float>> interpolation(steps, std::vector<float>(num_motors));
    for (int step = 0; step < steps; ++step) {
        float t = static_cast<float>(step) / std::max(1, (steps - 1));
        for (int motor = 0; motor < num_motors; ++motor) {
            interpolation[step][motor] = start_positions[motor] + t * (target_positions[motor] - start_positions[motor]);
        }
    }

    return interpolation;
}

void RevoMotorControl::updateMotorCmd(MotorId id, double pos, double vel, double torque, double kp, double kd)
{
    if (motor_ctrl_datas_.find(id) == motor_ctrl_datas_.end()) {
        RWLOG_W("Motor %u not found in updateMotorCmd", id);
        return;
    }

    auto& data = motor_ctrl_datas_.at(id);

    // 线程安全地更新控制命令
    data.setCmd(RevoMotorCmd_t{
        .pos = pos,
        .vel = vel,
        .torque = torque,
        .kp = kp,
        .kd = kd
    });

    // RWLOG_D("Updated motor %u command: pos=%.3f, vel=%.3f, torque=%.3f, kp=%.1f, kd=%.1f",
    //          id, pos, vel, torque, kp, kd);

    target_updated_ = true;
    if (!broadcast_ready_.load(std::memory_order_relaxed)) {
        broadcast_ready_.store(true, std::memory_order_relaxed);
    }
}

int RevoMotorControl::waitForOperationStatus(MotorId id, Operation expected_op, OperationStatus expected_status, int timeout_ms, const char* operation_name) {
    /**
     * @brief 等待电机操作完成
     * @param id 电机ID
     * @param expected_op 期望的操作类型
     * @param expected_status 期望的操作状态
     * @param timeout_ms 超时时间(毫秒)
     * @param operation_name 操作名称(用于日志)
     * @return 返回值含义：
     *         - 0: 操作成功完成
     *         - 1: 操作超时
     *         - 负数: 电机故障码的负数
     */
    auto start_time = std::chrono::steady_clock::now();
    while (true) {
        // 检查是否超时
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start_time);
        if (elapsed.count() >= timeout_ms) {
            RWLOG_W("%s motor 0x%02X timeout after %d ms", operation_name, id, timeout_ms);
            auto data_it = motor_ctrl_datas_.find(id);
            if (data_it != motor_ctrl_datas_.end()) {
                data_it->second.updateOperationStatus(OperationStatus::TIMEOUT);
            }
            return 1; // 超时
        }

        // 检查状态是否已更新为成功
        auto data_it = motor_ctrl_datas_.find(id);
        if (data_it != motor_ctrl_datas_.end() &&
            data_it->second.operation == expected_op &&
            data_it->second.operation_status == expected_status) {
            // 多圈清零指令没有反馈帧
            if(expected_op == Operation::MULTI_TURN_ZERO) {
                RWLOG_I("Motor 0x%02X %s completed successfully", id, operation_name);
                return 0;
            }
            
            // 检查电机故障码
            auto errcode = data_it->second.getFaultCode();
            if(errcode != MotorErrCode::NO_FAULT) {  // 没有故障
                data_it->second.updateOperationStatus(OperationStatus::FAILED);
                RWLOG_E("Motor 0x%02X %s failed: %s", id, operation_name, MotorErrCode2string(errcode).c_str());
                return -static_cast<int>(errcode);
            }

            // 有收到反馈帧
            if(true == data_it->second.feedback_received.load()) {
                RWLOG_I("Motor 0x%02X %s completed successfully", id, operation_name);
                return 0;
            }
        } // end if data_it != motor_ctrl_datas_.end()

        // 10ms检查一次
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    return 1; // timeout
}

bool RevoMotorControl::waitForAllMotorsFeedback(int timeout_ms) {
    auto start_time = std::chrono::steady_clock::now();

    while (true) {
        // 检查是否超时
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start_time);
        if (elapsed.count() >= timeout_ms) {
            RWLOG_W("Timeout waiting for feedback from all motors after %d ms", timeout_ms);
            return false;
        }

        // 检查是否所有非忽略电机都收到了反馈
        bool all_feedback_received = true;
        for (const auto& [id, data] : motor_ctrl_datas_) {
            if (!data.config.ignore && !data.feedback_received) {
                all_feedback_received = false;
            }
        }

        if (all_feedback_received) {
            RWLOG_SUCCESS("All %d motors have received feedback", motor_ctrl_datas_.size());
            return true;
        }

        // 10ms检查一次
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void RevoMotorControl::moveToZero(float zero_timeout) {
    RWLOG_I(" === Start moving motors to zero position ===");

    // 定义控制参数
    constexpr float control_frequency = 100.0f;
    constexpr float dt = 1.0f / control_frequency;
    constexpr float movement_speed = 25.0f * M_PI / 180.0f; 

    // 归一化角度到 [-π, π]，避免回零时绕超过一圈
    auto normalize_angle = [](float angle) -> float {
        angle = std::fmod(angle, 2.0f * static_cast<float>(M_PI));
        if (angle > static_cast<float>(M_PI))  angle -= 2.0f * static_cast<float>(M_PI);
        if (angle < -static_cast<float>(M_PI)) angle += 2.0f * static_cast<float>(M_PI);
        return angle;
    };

    // 获取所有电机的当前位置
    std::vector<float> current_positions;
    std::vector<MotorId> motor_ids;

    auto positions = getPositions();
    for (const auto& [id, pos] : positions) {
        current_positions.push_back(normalize_angle(pos));
        motor_ids.push_back(id);
    }

    if (current_positions.empty()) {
        RWLOG_W("No motors available for zero movement");
        return;
    }

    // 目标位置都为零点
    std::vector<float> target_positions(current_positions.size(), 0.0f);

    // 计算插值路径
    auto interpolation_path = interpolate_positions_with_speed(current_positions, target_positions, movement_speed, dt);

    RWLOG_I("Moving %zu motors to zero with %zu interpolation steps", motor_ids.size(), interpolation_path.size());

    // 第一阶段：发送插值结果
    RWLOG_I("Phase 1: 所有电机开始从当前位置插值到零点");
    for (size_t step = 0; step < interpolation_path.size(); ++step) {
        // 发送位置命令到所有电机
        for (size_t i = 0; i < motor_ids.size(); ++i) {
            MotorId id = motor_ids[i];
            float target_pos = interpolation_path[step][i];
            auto& data = motor_ctrl_datas_.at(id);
            updateMotorCmd(id, target_pos, 0.0f, 0.0f, data.config.default_params.kp_pos, data.config.default_params.kd_pos);
        }
        write();
        // 按照控制频率等待
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(dt * 1000)));
    }
    
    // 诊断: 打印 raw_pos 和 zero_offset, 定位位置突变来源
    for (const auto& [id, data] : motor_ctrl_datas_) {
        if (data.config.ignore || !data.isEnabled()) continue;
        float raw = data.motor.position();
        float offset = data.config.zero_offset;
        float computed = data.config.negtive ? (-raw - offset) : (raw - offset);
        RWLOG_I("[diag] Motor 0x%02X: raw=%.4f, zero_offset=%.4f, computed_pos=%.4f, neg=%d",
                id, raw, offset, computed, data.config.negtive);
    }

    // 第二阶段：检查是否到达零点，并记录未到达阈值的电机
    // 检查当前位置 - 使用 getPositions() 获取经过零点偏移和方向处理后的位置
    auto current_positions_for_check_map = getPositions();
    std::vector<float> current_positions_for_check;
    std::unordered_set<MotorId> motors_outside_threshold;  // 只记录未到达阈值的电机

    for (size_t i = 0; i < motor_ids.size(); ++i) {
        MotorId id = motor_ids[i];
        current_positions_for_check.push_back(current_positions_for_check_map.at(id));

        // 检查是否在阈值外，只记录未到达的电机
        if (std::abs(current_positions_for_check_map.at(id)) > kZeroThresholdRadians) {
            motors_outside_threshold.insert(id);
        }
    }

    if (motors_outside_threshold.empty()) {
        RWLOG_SUCCESS("所有电机已到达零点位置");
        return;
    }

    // 第三阶段：如果没有到达零点，继续发送命令，但只给未到达阈值的电机施加扭矩
    // 打印当前位置
    std::cout << "\033[33m电机回零未到达零点(阈值相差大于0.5度),当前位置:\033[0m\n";
    for (size_t i = 0; i < current_positions_for_check.size(); ++i) {
        if (std::abs(current_positions_for_check[i]) > kZeroThresholdRadians) {
            std::cout << "\033[31m" << std::fixed << std::setprecision(4) << current_positions_for_check[i] << "\033[0m ";
        } else {
            std::cout << std::fixed << std::setprecision(4) << current_positions_for_check[i] << " ";
        }
    }
    std::cout << std::endl;

    // Lambda函数检查是否所有位置都为零
    auto all_positions_zero = [](const std::vector<float>& positions, float threshold) -> bool {
        for (const auto& pos : positions) {
            if (std::abs(pos) > threshold) {
                return false;
            }
        }
        return true;
    };

    auto start_time = std::chrono::steady_clock::now();
    bool reached_zero = false;

    while (!reached_zero) {
        // 检查是否超时
        auto elapsed = std::chrono::duration<float>(std::chrono::steady_clock::now() - start_time).count();
        if (elapsed > zero_timeout) {
            RWLOG_W("Move to zero timeout after %.1f seconds", zero_timeout);
            break;
        }

        // 给未到达阈值的电机施加0.2扭矩
        for (const auto& id : motors_outside_threshold) {
            auto& data = motor_ctrl_datas_.at(id);
            updateMotorCmd(id, 0.0f, 0.0f, 0.2f, data.config.default_params.kp_pos, data.config.default_params.kd_pos);
        }
        write();

        // 按照控制频率等待
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(dt * 1000)));

        // 检查是否所有电机都到达零点 - 使用 getPositions() 获取经过零点偏移和方向处理后的位置
        auto current_positions_for_timeout_check_map = getPositions();
        std::vector<float> current_positions_for_timeout_check;
        for (size_t i = 0; i < motor_ids.size(); ++i) {
            MotorId id = motor_ids[i];
            current_positions_for_timeout_check.push_back(current_positions_for_timeout_check_map.at(id));
        }
        reached_zero = all_positions_zero(current_positions_for_timeout_check, kZeroThresholdRadians);
    }
}

bool RevoMotorControl::calibrateMotors() {
    RWLOG_I("Waiting for all motors to receive feedback frames...");

    // 等待所有电机接收到feedback帧，超时时间1000ms
    auto start_time = std::chrono::steady_clock::now();
    const int timeout_ms = 1000;

    while (true) {
        // 检查是否所有电机都收到了反馈
        bool all_received = true;
        for (const auto& [id, data] : motor_ctrl_datas_) {
            if (!data.feedback_received && !data.config.ignore) {
                all_received = false;
                break;
            }
        }

        if (all_received) {
            RWLOG_I("All motors received feedback, breaking wait loop");
            break;
        }

        // 检查是否超时
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start_time);
        if (elapsed.count() >= timeout_ms) {
            RWLOG_W("Calibration timeout after %d ms", timeout_ms);
            break;
        }

        // 10ms检查一次
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // 检查是否所有电机都收到了反馈，因为要需要把反馈的位置记做零点
    for (auto& [id, data] : motor_ctrl_datas_) {
        if (!data.feedback_received && !data.config.ignore) {
            RWLOG_E(" --- 警告！警告！电机 0x%02X 没有收到反馈帧，无法把电机当前位置设置成零点，使用默认值(0.0)", id);
            //  WARN: 电机被标记成 ignore/电机enble失败的情况，只能把零点设置成 MAGIC_NUMBER 了
        }
        else if(data.config.ignore){
            // 忽略电机 => 使用magic_number标记，下次使能时使用当前位置作为offset
            data.config.zero_offset = kOFFSET_MAGIC_NUMBER;
        }
        else {
            // 电机正常使能，将当前反馈的位置设置为零点偏移
            float current_raw_position = data.motor.position();
            data.config.zero_offset = data.config.negtive? -current_raw_position : current_raw_position;

            // 检查 zero_offset 是否在合理范围内: kCAN_COM_THETA_MIN + π ~ kCAN_COM_THETA_MAX - π
            constexpr float min_safe_offset = kCAN_COM_THETA_MIN + M_PI;  // -12.5 + π ≈ -9.35841
            constexpr float max_safe_offset = kCAN_COM_THETA_MAX - M_PI;  // 12.5 - π ≈ 9.35841
            if (data.config.zero_offset < min_safe_offset || data.config.zero_offset > max_safe_offset) {
                RWLOG_FAILURE("电机 0x%02X 零点偏移 %.4f 超出安全范围 [%.4f, %.4f]，可能影响运动范围",
                           id, data.config.zero_offset, min_safe_offset, max_safe_offset);
                return false;
            }
        }
        
        RWLOG_I(" --- Motor 0x%02X received feedback successfully, set as zero offset: %.4f rad", id, data.config.zero_offset);
    }

    RWLOG_I("Calibration completed successfully");
    return true;
}

bool RevoMotorControl::calibrateSingleMotor(MotorId id, int timeout_ms) {
    auto it = motor_ctrl_datas_.find(id);
    if (it == motor_ctrl_datas_.end()) {
        RWLOG_W("calibrateSingleMotor: motor 0x%02X not found", id);
        return false;
    }
    auto& data = it->second;

    if (data.config.ignore) {
        RWLOG_W("calibrateSingleMotor: motor 0x%02X is ignored, skip", id);
        return false;
    }

    // Step 1: 只对这一个电机清多圈编码器 (先失能状态下清零, 避免瞬间位置突变导致抽搐)
    if (!multiTurnZero(id, timeout_ms)) {
        RWLOG_E("calibrateSingleMotor: motor 0x%02X 多圈清零失败", id);
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    // Step 2: 只使能这一个电机 (同总线其它电机在 CANFD_BROADCAST 上会被顺带发失能, 幂等无害)
    int enable_result = enableMotor(id, timeout_ms);
    if (enable_result != 0) {
        RWLOG_E("calibrateSingleMotor: motor 0x%02X 使能失败(code=%d)", id, enable_result);
        return false;
    }

    // Step 3: 读取清零/使能后的原始位置, 计算零点
    float current_raw_position = data.motor.position();
    float new_zero_offset = data.config.negtive ? -current_raw_position : current_raw_position;

    // 检查 zero_offset 是否在合理范围内: kCAN_COM_THETA_MIN + π ~ kCAN_COM_THETA_MAX - π
    constexpr float min_safe_offset = kCAN_COM_THETA_MIN + M_PI;
    constexpr float max_safe_offset = kCAN_COM_THETA_MAX - M_PI;
    bool ok = true;
    if (new_zero_offset < min_safe_offset || new_zero_offset > max_safe_offset) {
        RWLOG_FAILURE("calibrateSingleMotor: motor 0x%02X zero offset %.4f 超出安全范围 [%.4f, %.4f]",
                   id, new_zero_offset, min_safe_offset, max_safe_offset);
        ok = false;
    } else {
        data.config.zero_offset = new_zero_offset;
        RWLOG_I("calibrateSingleMotor: motor 0x%02X set as zero offset: %.4f rad", id, new_zero_offset);
    }

    // Step 4: 标完(不论成功失败)重新失能该电机, 回到跟启动时一致的安全静止状态
    int disable_result = disableMotor(id, timeout_ms);
    if (disable_result != 0) {
        RWLOG_W("calibrateSingleMotor: motor 0x%02X 标定后重新失能失败(code=%d), 请手动确认电机状态", id, disable_result);
    }

    return ok;
}

bool RevoMotorControl::calibrateAllMotorsAndHold() {
    RWLOG_I("========== 开始完整标定流程 ==========");

    // Step 1: 失能所有电机 (必须先失能再清零，避免位置突变导致电机抽搐)
    if (!disableAll()) {
        RWLOG_FAILURE("calibrateAllMotorsAndHold: disableAll 失败");
        return false;
    }

    // Step 2: 多圈编码器清零 (不可逆硬件操作, 发送 CAN 命令 0xFE)
    multiTurnZeroAll();
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    // Step 3: 使能所有电机
    if (!enableAll()) {
        RWLOG_FAILURE("calibrateAllMotorsAndHold: enableAll 失败");
        return false;
    }

    // Step 4: 等待所有电机反馈到达
    if (!waitForAllMotorsFeedback(1500)) {
        RWLOG_W("calibrateAllMotorsAndHold: 部分电机未收到初始反馈, 继续标定");
    }

    // Step 5: 把当前电机的位置设置为零点偏移
    if (!calibrateMotors()) {
        RWLOG_FAILURE("calibrateAllMotorsAndHold: calibrateMotors 失败");
        return false;
    }

    // Step 6: 锁定所有电机在零位
    // 公式: joint_pos = (negtive ? -raw : raw) - zero_offset
    // calibrateMotors 已将 zero_offset 设为当前 raw_pos, 因此 joint_pos = 0
    // 发送 cmd.pos=0 经 writeSingleFrame/writeBroadcast 变换后 raw_target = 当前 raw_pos, 电机不动
    {
        std::map<MotorId, RevoMotorCmd_t> hold_cmd;
        for (auto& [id, data] : motor_ctrl_datas_) {
            if (!data.config.ignore && data.feedback_received) {
                hold_cmd[id] = {
                    .pos = 0.0,
                    .vel = 0.0,
                    .torque = 0.0,
                    .kp = static_cast<double>(data.config.default_params.kp_pos),
                    .kd = static_cast<double>(data.config.default_params.kd_pos)
                };
            }
        }
        if (!hold_cmd.empty()) {
            setTargetPositions(hold_cmd);
            write();
            RWLOG_I("标定后锁定: 所有电机保持在零位");
        }
    }

    RWLOG_SUCCESS("========== 完整标定流程完成 ==========");
    return true;
}

std::map<MotorId, float> RevoMotorControl::getZeroOffsets() {
    std::map<MotorId, float> zero_offsets;
    for (const auto& [id, data] : motor_ctrl_datas_) {
        zero_offsets[id] = data.config.zero_offset;
    }
    return zero_offsets;
}

std::map<MotorId, float> RevoMotorControl::getRawPositions() {
    std::map<MotorId, float> raw_positions;
    for (const auto& [id, data] : motor_ctrl_datas_) {
        raw_positions[id] = data.motor.position();
    }
    return raw_positions;
}

std::map<MotorId, float> RevoMotorControl::getRawVelocities() {
    std::map<MotorId, float> raw_velocities;
    for (const auto& [id, data] : motor_ctrl_datas_) {
        raw_velocities[id] = data.motor.velocity();
    }
    return raw_velocities;
}

std::map<MotorId, float> RevoMotorControl::getRawTorques() {
    std::map<MotorId, float> raw_torques;
    for (const auto& [id, data] : motor_ctrl_datas_) {
        raw_torques[id] = data.motor.torque();
    }
    return raw_torques;
}

std::map<MotorId, float> RevoMotorControl::getPositions() {
    std::map<MotorId, float> positions;
    for (const auto& [id, data] : motor_ctrl_datas_) {
        float raw_pos = data.motor.position();
        const auto& config = data.config;

        // 应用零点偏移和方向处理
        float adjusted_pos = raw_pos;
        if (config.negtive) {
            adjusted_pos = -adjusted_pos;
        }
        positions[id] = adjusted_pos - config.zero_offset;
        // 电机被标记为忽略或者没有使能成功 返回0.0f
        if(config.ignore || !data.isEnabled()) {
            positions[id] = 0.0f;
        }
    }
    return positions;
}


RevoMotorConfig_t RevoMotorControl::getMotorConfig(MotorId id) const {
    auto it = motor_ctrl_datas_.find(id);
    if (it != motor_ctrl_datas_.end()) {
        return it->second.config;
    }

    // 返回默认配置
    RevoMotorConfig_t default_config;
    default_config.id = id;
    default_config.name = "Unknown";
    default_config.negtive = false;
    default_config.ignore = true;
    default_config.zero_offset = 0.0f;
    default_config.ratio = 1.0f;
    return default_config;
}

std::map<MotorId, float> RevoMotorControl::getVelocities() {
    std::map<MotorId, float> velocities;
    for (const auto& [id, data] : motor_ctrl_datas_) {
        float raw_vel = data.motor.velocity();
        const auto& config = data.config;

        // 应用方向处理
        float adjusted_vel = raw_vel;
        if (config.negtive) {
            adjusted_vel = -adjusted_vel;
        }
        velocities[id] = adjusted_vel;
    }
    return velocities;
}

std::map<MotorId, float> RevoMotorControl::getTorques() {
    std::map<MotorId, float> torques;
    for (const auto& [id, data] : motor_ctrl_datas_) {
        float raw_torque = data.motor.torque();
        const auto& config = data.config;

        // 应用方向处理
        float adjusted_torque = raw_torque;
        if (config.negtive) {
            adjusted_torque = -adjusted_torque;
        }
        torques[id] = adjusted_torque;
    }
    return torques;
}

void RevoMotorControl::getAllJointData(std::vector<RevoJointSnapshot>& out) const {
    out.resize(motor_ctrl_datas_.size());
    size_t idx = 0;
    for (const auto& [id, data] : motor_ctrl_datas_) {
        float pos = data.motor.position();
        float vel = data.motor.velocity();
        float tor = data.motor.torque();
        const auto& cfg = data.config;
        if (cfg.negtive) { pos = -pos; vel = -vel; tor = -tor; }
        pos -= cfg.zero_offset;
        if (cfg.ignore || !data.isEnabled()) {
            pos = 0.0f; vel = 0.0f; tor = 0.0f;
        }
        out[idx++] = {id, pos, vel, tor};
    }
}

std::map<MotorId, MotorState> RevoMotorControl::getMotorStates() {
    std::map<MotorId, MotorState> motor_states;

    for (const auto& [id, data] : motor_ctrl_datas_) {
        motor_states[id] = data.isEnabled() ? MotorState::MotorSate : MotorState::RestSate;
    }

    return motor_states;
}

bool RevoMotorControl::hasAnyFault() const {
    for (const auto& [id, data] : motor_ctrl_datas_) {
        if (data.config.ignore) continue;
        if (data.getFaultCode() != MotorErrCode::NO_FAULT) return true;
    }
    return false;
}

MotorHealthSummary RevoMotorControl::getMotorHealthIssues(uint64_t now_ms,
                                                          bool watchdog_allowed,
                                                          uint64_t feedback_timeout_ms,
                                                          uint8_t disabled_confirm_frames,
                                                          std::vector<MotorHealthIssue>& out) const {
    out.clear();
    MotorHealthSummary summary;

    for (const auto& [id, data] : motor_ctrl_datas_) {
        const MotorErrCode fault = data.fault_code.load(std::memory_order_relaxed);
        const uint64_t last_feedback = data.last_feedback_time_ms.load(std::memory_order_acquire);
        const uint64_t feedback_age = last_feedback == 0
            ? 0
            : (now_ms >= last_feedback ? now_ms - last_feedback : 0);

        MotorHealthIssue issue;
        issue.id = id;
        issue.feedback_age_ms = feedback_age;
        issue.status_word = data.last_status_word.load(std::memory_order_relaxed);
        issue.status_word_valid = data.status_word_valid.load(std::memory_order_acquire);
        issue.fault_code = fault;

        const bool watchdog_armed = watchdog_allowed
            && !data.config.ignore
            && !(data.config.runtime_skip && runtime_skip_active_.load(std::memory_order_relaxed))
            && data.ever_received_feedback.load(std::memory_order_acquire)
            && data.expectsEnabled();
        if (watchdog_armed) {
            ++summary.armed_count;
            if (last_feedback != 0 && feedback_age > feedback_timeout_ms) {
                ++summary.timed_out_armed_count;
            }
        }

        if (!watchdog_armed) {
            // Preserve the existing explicit-fault protection independently of watchdog arming.
            if (fault != MotorErrCode::NO_FAULT) {
                issue.reason = MotorHealthReason::REPORTED_FAULT;
                out.push_back(issue);
            }
            continue;
        }

        // For an armed motor, current feedback freshness takes priority over cached
        // status/fault fields from the last received frame.
        if (last_feedback != 0 && feedback_age > feedback_timeout_ms) {
            issue.reason = MotorHealthReason::FEEDBACK_TIMEOUT;
            out.push_back(issue);
            continue;
        }

        if (protocol_ == MotorProtocol::CANFD_BROADCAST
            && issue.status_word_valid
            && disabled_confirm_frames > 0
            && data.disabled_feedback_count.load(std::memory_order_acquire) >= disabled_confirm_frames) {
            issue.reason = MotorHealthReason::UNEXPECTED_DISABLED;
            out.push_back(issue);
            continue;
        }

        if (fault != MotorErrCode::NO_FAULT) {
            issue.reason = MotorHealthReason::REPORTED_FAULT;
            out.push_back(issue);
        }
    }

    return summary;
}

std::map<MotorId, MotorErrCode> RevoMotorControl::getFaultCodes() {
    std::map<MotorId, MotorErrCode> fault_codes;
    for (const auto& [id, data] : motor_ctrl_datas_) {
        fault_codes[id] = data.getFaultCode();
    }
    return fault_codes;
}

void RevoMotorControl::setTargetPositions(const std::map<MotorId, RevoMotorCmd_t>& targets)
{
    for (const auto& [id, cmd] : targets) {
        if (motor_ctrl_datas_.find(id) == motor_ctrl_datas_.end()) {
            RWLOG_W("Motor %u not found in setTargetPositions, skipping", id);
            continue;
        }

        // 调用updateMotorCmd函数，避免代码重复
        updateMotorCmd(id, cmd.pos, cmd.vel, cmd.torque, cmd.kp, cmd.kd);

        // RWLOG_D("Set target for motor %u: pos=%.3f, vel=%.3f, torque=%.3f, kp=%.1f, kd=%.1f",
        //          id, cmd.pos, cmd.vel, cmd.torque, cmd.kp, cmd.kd);
    }
}

void RevoMotorControl::write()
{
    if (protocol_ == MotorProtocol::CANFD_BROADCAST) {
        // CAN FD 广播协议: broadcast_ready_ 在首次 setTargets 后置 true
        if (!broadcast_ready_.load(std::memory_order_relaxed)) {
            return;
        }
        // 与 EC_Master cyclic thread 一致: 每周期都发送指令帧,
        // 电机收到帧后回复反馈帧, 以获取最新的关节数据
        writeBroadcast();
    } else {
        // 单帧协议: 由 setTargets() 同步调用, 仅在有新目标时发送
        if (!target_updated_.exchange(false)) {
            // ===== 诊断: 空转计数 (can_control_thread_ 轮询时大概率命中这里) =====
            single_write_miss_.fetch_add(1, std::memory_order_relaxed);
            return;
        }
        // ===== 诊断: 真实发送计数 + 来源区分 =====
        single_write_hit_.fetch_add(1, std::memory_order_relaxed);
        if (write_from_setTargets_.exchange(false, std::memory_order_relaxed))
            hit_from_setTargets_.fetch_add(1, std::memory_order_relaxed);
        else
            hit_from_canThread_.fetch_add(1, std::memory_order_relaxed);
        auto t0 = std::chrono::steady_clock::now();
        writeSingleFrame();
        auto t1 = std::chrono::steady_clock::now();
        uint64_t dt_us = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count());
        single_write_total_us_.fetch_add(dt_us, std::memory_order_relaxed);
        // 更新 max: 无锁 CAS 循环, 正常情况下很快收敛
        uint64_t prev_max = single_write_max_us_.load(std::memory_order_relaxed);
        while (dt_us > prev_max &&
               !single_write_max_us_.compare_exchange_weak(
                   prev_max, dt_us, std::memory_order_relaxed)) {
            // 重试
        }
    }
}

RevoMotorControl::SingleFrameWriteStats RevoMotorControl::getSingleFrameWriteStats() const
{
    uint64_t hit = single_write_hit_.load(std::memory_order_relaxed);
    return SingleFrameWriteStats{
        hit,
        single_write_miss_.load(std::memory_order_relaxed),
        hit,  // samples == hit (writeSingleFrame 每次 hit 调用一次)
        single_write_total_us_.load(std::memory_order_relaxed),
        single_write_max_us_.load(std::memory_order_relaxed)
    };
}

void RevoMotorControl::writeSingleFrame()
{
    // 遍历所有电机，发送控制命令
    for (auto& [id, data] : motor_ctrl_datas_) {
        if(data.config.ignore || !data.isEnabled() ||
           (data.config.runtime_skip && runtime_skip_active_.load(std::memory_order_relaxed))) {
            continue;  // runtime_skip: 头部等 policy 不控制的关节, 上层控制开始后不发单帧
        }

        // 线程安全地获取电机命令
        RevoMotorCmd_t cmd = data.getCmd();

        // 根据电机方向和零点偏移计算原始目标值
        double raw_target_position = cmd.pos;
        double raw_target_velocity = cmd.vel;
        double raw_target_torque = cmd.torque;
        float kp = static_cast<float>(cmd.kp);
        float kd = static_cast<float>(cmd.kd);

        if (zero_torque_mode_) {
            // 0-torque模式：所有控制参数设为0
            raw_target_position = 0.0;
            raw_target_velocity = 0.0;
            raw_target_torque = 0.0;
            kp = 0.0f;
            kd = 0.0f;
        } else {
            // 正常模式：根据电机方向和零点偏移计算原始目标值
            if (data.config.negtive) {
                raw_target_position = -(raw_target_position + data.config.zero_offset);
                raw_target_velocity = -raw_target_velocity;
                raw_target_torque = -raw_target_torque;
            }
            else {
                raw_target_position += data.config.zero_offset;
            }
        }

        // 发送控制命令
        data.motor.controlPTM(
            static_cast<float>(raw_target_position),
            static_cast<float>(raw_target_velocity),
            static_cast<float>(raw_target_torque),
            kp,
            kd
        );
    }
}

void RevoMotorControl::writeBroadcast()
{
    uint8_t frame[canfd::kFrameSize];
    memset(frame, 0, canfd::kFrameSize);

    // MIT zero output frame: all zeros map to mid-range values
    // pos=0 -> 0x7FFF, vel=0 -> 0x7FF, kp=0 -> 0x000, kd=0 -> 0x000, tor=0 -> 0x7FF
    static const uint8_t kMitZeroFrame[8] = {0x7F, 0xFF, 0x7F, 0xF0, 0x00, 0x00, 0x07, 0xFF};
    for (size_t i = 0; i < canfd::kMaxJointsPerFrame; ++i) {
        memcpy(&frame[i * canfd::kBytesPerJoint], kMitZeroFrame, canfd::kBytesPerJoint);
    }

    for (auto& [id, data] : motor_ctrl_datas_) {
        if (data.config.ignore || !data.isEnabled() ||
            (data.config.runtime_skip && runtime_skip_active_.load(std::memory_order_relaxed))) {
            continue;  // slot 保持 kMitZeroFrame (零输出); runtime_skip: 上层控制开始后头部不发广播
        }

        size_t slot = static_cast<size_t>(id) - 1;
        if (slot >= canfd::kMaxJointsPerFrame) {
            RWLOG_W("Motor ID %u exceeds max joints per frame (%zu)", id, canfd::kMaxJointsPerFrame);
            continue;
        }

        RevoMotorCmd_t cmd = data.getCmd();
        double raw_target_position = cmd.pos;
        double raw_target_velocity = cmd.vel;
        double raw_target_torque = cmd.torque;
        float kp = static_cast<float>(cmd.kp);
        float kd = static_cast<float>(cmd.kd);

        if (zero_torque_mode_) {
            raw_target_position = 0.0;
            raw_target_velocity = 0.0;
            raw_target_torque = 0.0;
            kp = 0.0f;
            kd = 0.0f;
        } else {
            if (data.config.negtive) {
                raw_target_position = -(raw_target_position + data.config.zero_offset);
                raw_target_velocity = -raw_target_velocity;
                raw_target_torque = -raw_target_torque;
            } else {
                raw_target_position += data.config.zero_offset;
            }
        }

        data.motor.encodeMitFrame(
            &frame[slot * canfd::kBytesPerJoint],
            static_cast<float>(raw_target_position),
            static_cast<float>(raw_target_velocity),
            static_cast<float>(raw_target_torque),
            kp, kd
        );
    }

    sendCanFdBroadcast(canfd_bus_id_, canfd::kCanIdMIT, frame);

    // 诊断: 记录 bcan0 motor1 的 CAN 发送时间戳 + 命令位置
    // 默认编译关闭, 避免 RT/CANFD 线程做同步文件 IO; 开启: cmake -DLEJU_ENABLE_DIAG_FILELOG=ON
#ifdef LEJU_ENABLE_DIAG_FILELOG
    if (canfd_bus_id_ == 0 && motor_ctrl_datas_.count(1)) {
        static FILE* txlog = fopen("/tmp/can_tx.log", "w");
        if (txlog) {
            auto cmd = motor_ctrl_datas_.at(1).getCmd();
            auto now = std::chrono::steady_clock::now();
            auto us = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count();
            fprintf(txlog, "%ld,%.6f\n", us, cmd.pos);
            static int tx_count = 0;
            if (++tx_count % 100 == 0) fflush(txlog);
        }
    }
#endif
}

RevoMotorControl::~RevoMotorControl() {
    RWLOG_I("RevoMotorControl destructor called, cleaning up resources...");

    // 先失能所有电机
    disableAll();

    // 获取CAN总线控制器实例
    auto& canbus_controller = canbus_sdk::CanBusController::getInstance();

    // 尝试获取bus_id
    auto bus_id_result = canbus_controller.getBusIdByName(canbus_name_);

    if (bus_id_result.has_value()) {
        // 反注册所有电机设备
        for (auto& [id, data] : motor_ctrl_datas_) {
            if (!data.config.ignore) {
                RWLOG_D("Unregistering motor %u device", id);
                auto devid = static_cast<canbus_sdk::DeviceId>(id);
                canbus_controller.unregisterDevice(canbus_sdk::DeviceType::MOTOR, devid, canbus_name_);
            }
        }

        RWLOG_SUCCESS("RevoMotorControl resources cleaned up successfully");
    } else {
        RWLOG_WARNING("Could not find bus_id for %s during cleanup", canbus_name_.c_str());
    }
}

bool RevoMotorControl::enableAll()
{
    if (protocol_ == MotorProtocol::CANFD_BROADCAST) {
        // Step 1: Clear any residual faults before enabling
        {
            uint8_t frame[canfd::kFrameSize];
            uint8_t joint[canfd::kBytesPerJoint];
            RevoMotor::encodeControlFrame(joint, canfd::kCmdClearErr);
            for (size_t i = 0; i < canfd::kMaxJointsPerFrame; ++i) {
                memcpy(&frame[i * canfd::kBytesPerJoint], joint, canfd::kBytesPerJoint);
            }
            if (!sendCanFdBroadcast(canfd_bus_id_, canfd::kCanIdControl, frame)) {
                RWLOG_E("Failed to send CAN FD broadcast clear error");
            } else {
                RWLOG_I("CAN FD broadcast clear error sent");
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

        // Step 2: Send enable command
        uint8_t frame[canfd::kFrameSize];
        uint8_t joint[canfd::kBytesPerJoint];
        RevoMotor::encodeControlFrame(joint, canfd::kCmdEnable);
        for (size_t i = 0; i < canfd::kMaxJointsPerFrame; ++i) {
            memcpy(&frame[i * canfd::kBytesPerJoint], joint, canfd::kBytesPerJoint);
        }
        if (!sendCanFdBroadcast(canfd_bus_id_, canfd::kCanIdControl, frame)) {
            RWLOG_E("Failed to send CAN FD broadcast enable");
            return false;
        }
        // 等待每个电机的反馈确认
        for (auto& [id, data] : motor_ctrl_datas_) {
            if (!data.config.ignore) {
                data.startOperation(Operation::ENABLE);
            }
        }
        int success_count = 0, ignore_count = 0;
        bool danger_error = false;
        for (auto& [id, data] : motor_ctrl_datas_) {
            if (data.config.ignore) {
                ignore_count++;
                RWLOG_WARNING("MOTOR ID:%u Enable: [Ignored]", id);
                continue;
            }
            int result = waitForOperationStatus(id, Operation::ENABLE, OperationStatus::SUCCESS, 200, "Enable");
            if (result == 0) {
                if (data.isEnabled()) {
                    success_count++;
                    RWLOG_SUCCESS("MOTOR ID:%u Enable: [Success]", id);
                } else {
                    RWLOG_FAILURE("MOTOR ID:%u Enable: [Failed]", id);
                }
            } else if (result == 1) {
                RWLOG_FAILURE("MOTOR ID:%u Enable: [Timeout]", id);
            } else if (result < 0) {
                uint8_t fault_code = static_cast<uint8_t>(-result);
                auto errcode = static_cast<MotorErrCode>(fault_code);
                RWLOG_FAILURE("MOTOR ID:%u Enable: [Fault 0x%02X: %s]", id, fault_code, MotorErrCode2string(errcode).c_str());
                if (errcode >= MotorErrCode::ABS_ENCODER_OFFSET_VERIFICATION_FAILURE) {
                    danger_error = true;
                }
            }
        }
        if (danger_error) return false;
        if (static_cast<size_t>(ignore_count) == motor_ctrl_datas_.size()) return true;
        const size_t required_count = motor_ctrl_datas_.size() - static_cast<size_t>(ignore_count);
        return static_cast<size_t>(success_count) == required_count;
    }

    // Existing single-frame logic follows...
    std::unordered_map<MotorId, int> results;
    for (auto& [id, data] : motor_ctrl_datas_) {
        // 电机在配置文件中被标记为 <ignore> 则跳过
        if(data.config.ignore) {
            results.insert(std::make_pair(id, 31415926)); // 31415926 magic number
            continue;
        }
        int ret = enableMotor(id, 500); // 500ms
        results.insert(std::make_pair(id, ret));
    }

    // 检查状态: enable 使能指令是否发送成功
    bool danger_error = false;   // 有严重故障码
    int ignore_count = 0, success_count = 0;
    for (auto& [id, result] : results) {
        if (result == 0) {
            auto& data = motor_ctrl_datas_.at(id);
            if (!data.isEnabled()) {
                RWLOG_FAILURE("MOTOR ID:%u Enable: [Failed]", id);
            } else {
                success_count++;
                RWLOG_SUCCESS("MOTOR ID:%u Enable: [Success]", id);
            }
        }
        else if(result == 31415926) {
            RWLOG_WARNING("MOTOR ID:%u Enable: [Ignored]", id);
            ignore_count++;
        }
        else if(result == 1) {
            RWLOG_FAILURE("MOTOR ID:%u Enable: [Timeout]", id);
        }
        else if(result == 2) {
            RWLOG_FAILURE("MOTOR ID:%u Enable: [Send message failed]", id);
        }
        else if(result == 3) {
            RWLOG_FAILURE("MOTOR ID:%u Enable: [Motor not found]", id);
        }
        else if(result < 0) {
            uint8_t fault_code = static_cast<uint8_t>(-result);
            RWLOG_FAILURE("MOTOR ID:%u Enable: [Fault code 0x%02X]", id, fault_code);
            auto  errcode = static_cast<MotorErrCode>(fault_code);
            if(errcode >= MotorErrCode::ABS_ENCODER_OFFSET_VERIFICATION_FAILURE) {
                danger_error = true;
                std::string err_str = MotorErrCode2string(errcode);
                RWLOG_FAILURE("[%s] 电机 ID:%u 使能失败: [无法清除的故障码 0x%03X: %s]", canbus_name_.c_str(), id, fault_code, err_str.c_str());
            }
        }
        else {
            RWLOG_FAILURE("MOTOR ID:%u Disable: [Unknown error %d]", id, result);
        }
    }

    // 有严重故障码存在
    if(danger_error) {
        return false;
    }

    if(ignore_count == motor_ctrl_datas_.size()) {
        // 全部忽略那就是成功
        return true;
    }
    const size_t required_count = motor_ctrl_datas_.size() - static_cast<size_t>(ignore_count);
    return static_cast<size_t>(success_count) == required_count;
}

bool RevoMotorControl::disableAll()
{
    const bool skip_wait = skip_disable_wait_.load(std::memory_order_acquire);

    if (protocol_ == MotorProtocol::CANFD_BROADCAST) {
        // Disarm the enabled watchdog before intentionally sending FD.
        for (auto& [id, data] : motor_ctrl_datas_) {
            if (!data.config.ignore) {
                data.startOperation(Operation::DISABLE);
            }
        }
        // CAN FD broadcast: send one 64-byte control frame to disable all joints
        uint8_t frame[canfd::kFrameSize];
        uint8_t joint[canfd::kBytesPerJoint];
        RevoMotor::encodeControlFrame(joint, canfd::kCmdDisable);
        for (size_t i = 0; i < canfd::kMaxJointsPerFrame; ++i) {
            memcpy(&frame[i * canfd::kBytesPerJoint], joint, canfd::kBytesPerJoint);
        }
        if (!sendCanFdBroadcast(canfd_bus_id_, canfd::kCanIdControl, frame)) {
            RWLOG_E("Failed to send CAN FD broadcast disable");
            return false;
        }
        if (skip_wait) {
            RWLOG_W("CAN FD disable broadcast sent; skipping feedback confirmation because the bus is offline");
            return true;
        }
        // 等待每个电机的反馈确认
        int success_count = 0, ignore_count = 0;
        for (auto& [id, data] : motor_ctrl_datas_) {
            if (data.config.ignore) {
                ignore_count++;
                RWLOG_SUCCESS("MOTOR ID:%u Disable: [Ignored]", id);
                continue;
            }
            int result = waitForOperationStatus(id, Operation::DISABLE, OperationStatus::SUCCESS, 200, "Disable");
            if (result == 0) {
                success_count++;
                RWLOG_SUCCESS("MOTOR ID:%u Disable: [Success]", id);
            } else if (result == 1) {
                RWLOG_FAILURE("MOTOR ID:%u Disable: [Timeout]", id);
            } else if (result < 0) {
                uint8_t fault_code = static_cast<uint8_t>(-result);
                auto errcode = static_cast<MotorErrCode>(fault_code);
                RWLOG_FAILURE("MOTOR ID:%u Disable: [Fault 0x%02X: %s]", id, fault_code, MotorErrCode2string(errcode).c_str());
            }
        }
        if (static_cast<size_t>(ignore_count) == motor_ctrl_datas_.size()) return true;
        return success_count > 0;
    }

    if (skip_wait) {
        bool all_sent = true;
        bool has_target = false;
        for (auto& [id, data] : motor_ctrl_datas_) {
            if (data.config.ignore) continue;
            has_target = true;
            data.startOperation(Operation::DISABLE);
            if (!data.motor.enterRestState()) {
                all_sent = false;
                RWLOG_E("Failed to send disable command to motor 0x%02X", id);
            }
        }
        RWLOG_W("Classic CAN disable commands sent; skipping feedback confirmation because the bus is offline");
        return !has_target || all_sent;
    }

    // Existing single-frame logic follows...
    bool danger_error = false;
    std::map<MotorId, int> results;
    for (auto& [id, data] : motor_ctrl_datas_) {
        if(data.config.ignore) {
            results.insert(std::make_pair(id, 31415926)); // 31415926 magic number
            continue;
        }
        int ret = disableMotor(id);
        results.insert(std::make_pair(id, ret));
    }
    int ignore_count = 0, success_count = 0;
    for (auto& [id, result] : results) {
        if (result == 0) {
            success_count++;
            RWLOG_SUCCESS("MOTOR ID:%u Disable: [Success]", id);
        }
        else if(result == 31415926) {
            ignore_count++;
            RWLOG_SUCCESS("MOTOR ID:%u Disable: [Ignored]", id);
        }
        else if(result == 1) {
            RWLOG_FAILURE("MOTOR ID:%u Disable: [Timeout]", id);
        }
        else if(result == 2) {
            RWLOG_FAILURE("MOTOR ID:%u Disable: [Send message failed]", id);
        }
        else if(result == 3) {
            RWLOG_FAILURE("MOTOR ID:%u Disable: [Motor not found]", id);
        }
        else if(result < 0) {
            uint8_t fault_code = static_cast<uint8_t>(-result);
            RWLOG_FAILURE("MOTOR ID:%u Disable: [Fault code 0x%02X]", id, fault_code);
            auto  errcode = static_cast<MotorErrCode>(fault_code);
            if(errcode >= MotorErrCode::ABS_ENCODER_OFFSET_VERIFICATION_FAILURE) {
                danger_error = true;
                std::string err_str = MotorErrCode2string(errcode);
                RWLOG_FAILURE("[%s] 电机 ID:%u 失能失败: [无法清除的故障码 0x%03X: %s]", canbus_name_.c_str(), id, fault_code, err_str.c_str());
            }
        }
        else {
            RWLOG_FAILURE("MOTOR ID:%u Disable: [Unknown error %d]", id, result);
        }
    }

    // 有严重故障码存在
    if(danger_error) {
        return false;
    }

    if(ignore_count == motor_ctrl_datas_.size()) {
        // 全部忽略那就是成功
        return true;
    }
    if(success_count == 0) {
        // 一个成功的都没有
        return false;
    }
    return true;
}

void RevoMotorControl::multiTurnZeroAll()
{
    if (protocol_ == MotorProtocol::CANFD_BROADCAST) {
        // CAN FD 广播协议: 通过控制帧 CAN ID 0x10 发送零点保存命令 0xFE
        uint8_t frame[canfd::kFrameSize];
        uint8_t joint[canfd::kBytesPerJoint];
        RevoMotor::encodeControlFrame(joint, canfd::kCmdSaveZero);
        for (size_t i = 0; i < canfd::kMaxJointsPerFrame; ++i) {
            memcpy(&frame[i * canfd::kBytesPerJoint], joint, canfd::kBytesPerJoint);
        }
        if (!sendCanFdBroadcast(canfd_bus_id_, canfd::kCanIdControl, frame)) {
            RWLOG_E("Failed to send CAN FD broadcast save zero");
            return;
        }
        RWLOG_I("CAN FD broadcast save zero sent (CAN ID 0x10, cmd 0xFE)");

        // 等待电机反馈确认
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        for (auto& [id, data] : motor_ctrl_datas_) {
            if (data.config.ignore) {
                RWLOG_SUCCESS("MOTOR ID:%u SaveZero: [Ignored]", id);
            } else {
                RWLOG_SUCCESS("MOTOR ID:%u SaveZero: [Broadcast sent]", id);
            }
        }
        return;
    }

    // 单帧(非广播)协议: 逐个电机发送多圈编码器清零命令
    // 经典 CAN 协议使用 multiTurnZero() (CAN ID = 0x600+ID, 数据 67 06 FF..., 76)
    std::map<MotorId, int> results;
    for (auto& [id, data] : motor_ctrl_datas_) {
        if(data.config.ignore) {
            results.insert(std::make_pair(id, -1));
            continue;
        }
        bool res = data.motor.multiTurnZero();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        results.insert(std::make_pair(id, res? 1 : 0));
    }
    for (auto& [id, res] : results) {
        if(res == 1) {
            RWLOG_SUCCESS("MOTOR ID:%u MultiTurnZero: [Success]", id);
        }
        else if(res == -1) {
            RWLOG_SUCCESS("MOTOR ID:%u MultiTurnZero: [Ignored]", id);
        }
        else {
            RWLOG_FAILURE("MOTOR ID:%u MultiTurnZero: [Failed]", id);
        }
    }
}

bool RevoMotorControl::sendSingleSlotControlFrame(MotorId target_id, uint8_t target_cmd) {
    if (protocol_ != MotorProtocol::CANFD_BROADCAST) {
        RWLOG_W("sendSingleSlotControlFrame only applies to CANFD_BROADCAST protocol");
        return false;
    }

    size_t slot = static_cast<size_t>(target_id) - 1;
    if (slot >= canfd::kMaxJointsPerFrame) {
        RWLOG_W("sendSingleSlotControlFrame: motor ID %u exceeds max joints per frame (%zu)",
                target_id, canfd::kMaxJointsPerFrame);
        return false;
    }

    // 其它槽位统一填失能命令: 单电机标定场景下同总线其它电机本就没有整体使能过,
    // 这里发送失能对它们是幂等操作, 不会引入额外风险
    uint8_t frame[canfd::kFrameSize];
    uint8_t disable_joint[canfd::kBytesPerJoint];
    RevoMotor::encodeControlFrame(disable_joint, canfd::kCmdDisable);
    for (size_t i = 0; i < canfd::kMaxJointsPerFrame; ++i) {
        memcpy(&frame[i * canfd::kBytesPerJoint], disable_joint, canfd::kBytesPerJoint);
    }

    uint8_t target_joint[canfd::kBytesPerJoint];
    RevoMotor::encodeControlFrame(target_joint, target_cmd);
    memcpy(&frame[slot * canfd::kBytesPerJoint], target_joint, canfd::kBytesPerJoint);

    return sendCanFdBroadcast(canfd_bus_id_, canfd::kCanIdControl, frame);
}

} // namespace motorevo
