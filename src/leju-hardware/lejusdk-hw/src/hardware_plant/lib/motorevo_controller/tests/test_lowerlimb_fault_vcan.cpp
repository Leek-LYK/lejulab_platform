/**
 * vcan integration test for per-motor CAN FD health classification.
 * The companion responder follows normal FD/FC operations, then injects the
 * scenario passed on the command line.
 */
#include <chrono>
#include <cstdio>
#include <string>
#include <thread>
#include <vector>

#include "motorevo/motor_ctrl.h"
#include "canbus_sdk/canbus_sdk.h"

namespace {

constexpr uint64_t kFeedbackTimeoutMs = 20;
constexpr uint8_t kDisabledConfirmFrames = 3;
int g_failures = 0;

uint64_t nowMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

void expect(bool condition, const char* message) {
    if (condition) {
        printf("[PASS] %s\n", message);
    } else {
        printf("\033[31m[FAIL] %s\033[0m\n", message);
        ++g_failures;
    }
}

motorevo::RevoMotorConfig_t makeCfg(motorevo::MotorId id, const std::string& name) {
    motorevo::RevoMotorConfig_t config;
    config.id = id;
    config.name = name;
    config.negtive = false;
    config.ignore = false;
    config.runtime_skip = false;
    config.zero_offset = 0.0f;
    config.ratio = 25.0f;
    config.default_params = {0.0f, 200.0f, 5.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    return config;
}

const motorevo::MotorHealthIssue* findIssue(
    const std::vector<motorevo::MotorHealthIssue>& issues,
    motorevo::MotorId id) {
    for (const auto& issue : issues) {
        if (issue.id == id) return &issue;
    }
    return nullptr;
}

bool waitForReason(motorevo::RevoMotorControl& control,
                   motorevo::MotorId id,
                   motorevo::MotorHealthReason reason,
                   motorevo::MotorHealthSummary& summary,
                   std::vector<motorevo::MotorHealthIssue>& issues,
                   int timeout_ms = 3000) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline) {
        summary = control.getMotorHealthIssues(nowMs(), true, kFeedbackTimeoutMs,
                                               kDisabledConfirmFrames, issues);
        const auto* issue = findIssue(issues, id);
        if (issue && issue->reason == reason) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return false;
}

}  // namespace

int main(int argc, char** argv) {
    const std::string iface = argc > 1 ? argv[1] : "vbcan0";
    const std::string scenario = argc > 2 ? argv[2] : "fault";

    auto& bus_controller = canbus_sdk::CanBusController::getInstance();
    bus_controller.init();
    canbus_sdk::CanBusBitrate bitrate{1000000, 5000000, 80, 75};
    auto open_result = bus_controller.openCanBus(
        iface, canbus_sdk::CanBusModelType::SOCKETCAN, bitrate);
    if (!open_result.has_value()) {
        printf("\033[31m[FAIL] cannot open %s\033[0m\n", iface.c_str());
        return 1;
    }

    motorevo::RevoMotorControl control(iface, motorevo::MotorProtocol::CANFD_BROADCAST);
    const bool initialized = control.init(
        {makeCfg(1, "Test_joint_01"), makeCfg(2, "Test_joint_02")},
        false, false);
    expect(initialized, "init follows FD/FC feedback and enables both motors");
    if (!initialized) return 1;

    std::vector<motorevo::MotorHealthIssue> issues;
    motorevo::MotorHealthSummary summary = control.getMotorHealthIssues(
        nowMs(), true, kFeedbackTimeoutMs, kDisabledConfirmFrames, issues);
    expect(issues.empty(), "healthy baseline has no health issue");
    expect(summary.armed_count == 2, "healthy baseline arms both motors");

    if (scenario == "fault") {
        const bool seen = waitForReason(control, 1, motorevo::MotorHealthReason::REPORTED_FAULT,
                                        summary, issues);
        expect(seen, "motor 1 reports REPORTED_FAULT");
        const auto* issue = findIssue(issues, 1);
        expect(issue && issue->fault_code == motorevo::MotorErrCode::DC_BUS_CURRENT_OVERLOAD,
               "over-current status maps to DC_BUS_CURRENT_OVERLOAD");
        expect(findIssue(issues, 2) == nullptr, "fault does not leak to motor 2");
    } else if (scenario == "disabled") {
        const bool seen = waitForReason(control, 1,
                                        motorevo::MotorHealthReason::UNEXPECTED_DISABLED,
                                        summary, issues);
        expect(seen, "fresh status 0x0000 becomes UNEXPECTED_DISABLED");
        const auto* issue = findIssue(issues, 1);
        expect(issue && issue->status_word_valid && issue->status_word == 0,
               "disabled issue retains valid status 0x0000");
        expect(issue && issue->feedback_age_ms <= kFeedbackTimeoutMs,
               "disabled feedback remains fresh");
        expect(findIssue(issues, 2) == nullptr, "disabled state does not leak to motor 2");
    } else if (scenario == "single_timeout") {
        const bool seen = waitForReason(control, 1,
                                        motorevo::MotorHealthReason::FEEDBACK_TIMEOUT,
                                        summary, issues);
        expect(seen, "missing motor 1 feedback becomes FEEDBACK_TIMEOUT");
        expect(summary.armed_count == 2 && summary.timed_out_armed_count == 1,
               "single timeout does not classify the whole bus offline");
        expect(findIssue(issues, 2) == nullptr, "motor 2 feedback remains healthy");
        expect(!control.skipDisableWait(), "single timeout does not skip disable confirmation");
    } else if (scenario == "bus_timeout") {
        const bool seen = waitForReason(control, 1,
                                        motorevo::MotorHealthReason::FEEDBACK_TIMEOUT,
                                        summary, issues);
        expect(seen, "motor 1 times out when all feedback stops");
        expect(summary.armed_count == 2 && summary.timed_out_armed_count == 2,
               "all armed motors are classified timed out");
        const auto* motor2 = findIssue(issues, 2);
        expect(motor2 && motor2->reason == motorevo::MotorHealthReason::FEEDBACK_TIMEOUT,
               "motor 2 also reports FEEDBACK_TIMEOUT");
        expect(!control.skipDisableWait(),
               "low-level observation does not skip disable confirmation before protective stop");
    } else if (scenario == "normal_lifecycle") {
        expect(control.disableAll(), "normal disableAll succeeds with status confirmation");
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        control.getMotorHealthIssues(nowMs(), true, kFeedbackTimeoutMs,
                                     kDisabledConfirmFrames, issues);
        expect(issues.empty(), "normal disable does not report UNEXPECTED_DISABLED");
        expect(control.enableAll(), "normal enableAll succeeds with status confirmation");
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        control.getMotorHealthIssues(nowMs(), true, kFeedbackTimeoutMs,
                                     kDisabledConfirmFrames, issues);
        expect(issues.empty(), "normal re-enable returns to healthy state");
    } else {
        printf("\033[31m[FAIL] unknown scenario: %s\033[0m\n", scenario.c_str());
        return 2;
    }

    if (g_failures == 0) {
        printf("\033[32mscenario %s: all checks passed\033[0m\n", scenario.c_str());
        return 0;
    }
    printf("\033[31mscenario %s: %d checks failed\033[0m\n", scenario.c_str(), g_failures);
    return 1;
}
