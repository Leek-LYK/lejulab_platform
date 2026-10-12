// motor_preflight 单元测试
// 帧编解码用例与 scripts/can_param_tools/can_param_tool.py 的输出逐字节对拍
#include <gtest/gtest.h>

#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>

#include <map>
#include <set>
#include <utility>

#include "motor_preflight/motor_preflight.h"
#include "motor_preflight/param_protocol.h"
#include "motor_preflight/preflight_config.h"

using namespace motor_preflight;

namespace {

ParamValue U32(uint32_t v) {
    ParamValue p;
    p.dtype = ParamDtype::kUint32;
    p.u32 = v;
    return p;
}

ParamValue F32(float v) {
    ParamValue p;
    p.dtype = ParamDtype::kFloat;
    p.f32 = v;
    return p;
}

}  // namespace

// ===== 帧编码 =====

TEST(ParamProtocol, BuildReadFrameFirmwareIdx) {
    // python: build_read_frame(10) -> 67 0A 00 00 00 00 04 76
    std::array<uint8_t, 8> expected = {0x67, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x04, 0x76};
    EXPECT_EQ(BuildReadFrame(kFirmwareVersionIdx), expected);
}

TEST(ParamProtocol, BuildWriteFrameUint32) {
    // python: build_write_frame(61, 100, "uint32") -> 67 3D 64 00 00 00 15 76
    std::array<uint8_t, 8> expected = {0x67, 0x3D, 0x64, 0x00, 0x00, 0x00, 0x15, 0x76};
    EXPECT_EQ(BuildWriteFrame(61, U32(100)), expected);
}

TEST(ParamProtocol, BuildWriteFrameHex32) {
    // python: build_write_frame(56, "0xFF", "hex32") -> 67 38 FF 00 00 00 15 76
    std::array<uint8_t, 8> expected = {0x67, 0x38, 0xFF, 0x00, 0x00, 0x00, 0x15, 0x76};
    EXPECT_EQ(BuildWriteFrame(56, U32(0xFF)), expected);
}

TEST(ParamProtocol, BuildWriteFrameFloat) {
    // python: build_write_frame(30, 1.5, "float") -> 67 1E 00 00 C0 3F 15 76
    std::array<uint8_t, 8> expected = {0x67, 0x1E, 0x00, 0x00, 0xC0, 0x3F, 0x15, 0x76};
    EXPECT_EQ(BuildWriteFrame(30, F32(1.5f)), expected);
}

// ===== 响应解码 =====

TEST(ParamProtocol, DecodeResponseUint32) {
    // 响应帧: <motor_id=01> <idx=0A> 01 00 03 02 <tail> -> 0x02030001
    uint8_t resp[8] = {0x01, 0x0A, 0x01, 0x00, 0x03, 0x02, 0x00, 0x76};
    ParamValue out = U32(0);
    ASSERT_TRUE(DecodeResponse(resp, 8, kFirmwareVersionIdx, &out));
    EXPECT_EQ(out.u32, 0x02030001u);
}

TEST(ParamProtocol, DecodeResponseFloat) {
    // payload 00 00 C0 3F -> 1.5f
    uint8_t resp[8] = {0x05, 0x1E, 0x00, 0x00, 0xC0, 0x3F, 0x00, 0x76};
    ParamValue out = F32(0.0f);
    ASSERT_TRUE(DecodeResponse(resp, 8, 30, &out));
    EXPECT_FLOAT_EQ(out.f32, 1.5f);
}

TEST(ParamProtocol, DecodeResponseRejectsBadLength) {
    uint8_t resp[7] = {0x01, 0x0A, 0x01, 0x00, 0x03, 0x02, 0x00};
    ParamValue out = U32(0);
    EXPECT_FALSE(DecodeResponse(resp, 7, kFirmwareVersionIdx, &out));
}

TEST(ParamProtocol, DecodeResponseRejectsEchoedRequest) {
    // 首字节 0x67 说明收到的是请求帧自环, 不是电机响应
    uint8_t resp[8] = {0x67, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x04, 0x76};
    ParamValue out = U32(0);
    EXPECT_FALSE(DecodeResponse(resp, 8, kFirmwareVersionIdx, &out));
}

TEST(ParamProtocol, DecodeResponseRejectsWrongIdx) {
    uint8_t resp[8] = {0x01, 0x0B, 0x01, 0x00, 0x03, 0x02, 0x00, 0x76};
    ParamValue out = U32(0);
    EXPECT_FALSE(DecodeResponse(resp, 8, kFirmwareVersionIdx, &out));
}

// ===== 值比较 =====

TEST(ParamProtocol, ValueMatchesFloatTolerance) {
    EXPECT_TRUE(ValueMatches(F32(1.5f), F32(1.5f + 1e-6f)));
    EXPECT_FALSE(ValueMatches(F32(1.5f), F32(1.5f + 1e-4f)));
}

TEST(ParamProtocol, ValueMatchesUint32Exact) {
    EXPECT_TRUE(ValueMatches(U32(100), U32(100)));
    EXPECT_FALSE(ValueMatches(U32(100), U32(99)));
}

TEST(ParamProtocol, ValueMatchesRejectsDtypeMismatch) {
    EXPECT_FALSE(ValueMatches(U32(0), F32(0.0f)));
}

// ===== 配置加载 =====

namespace {

// 写临时 json 文件并返回路径; TearDown 时删除
class PreflightConfigTest : public ::testing::Test {
protected:
    std::string WriteConfig(const std::string& content) {
        path_ = ::testing::TempDir() + "motor_firmware_test.json";
        std::ofstream ofs(path_);
        ofs << content;
        ofs.close();
        return path_;
    }
    void TearDown() override {
        if (!path_.empty()) std::remove(path_.c_str());
    }
    std::string path_;
};

}  // namespace

TEST_F(PreflightConfigTest, ParsesValidConfig) {
    auto path = WriteConfig(R"({
        "models": {
            "motorevo_PA81_25": {
                "firmware_version": "0x02030001",
                "params": [
                    { "idx": 61, "dtype": "uint32", "value": 100 },
                    { "idx": 30, "dtype": "float",  "value": 1.5 },
                    { "idx": 56, "dtype": "hex32",  "value": "0xFF" }
                ]
            },
            "ruiwo": { "firmware_version": 66051 }
        },
        "device_map": {
            "Lleg_joint_01": "motorevo_PA81_25",
            "Larm_joint_01": "ruiwo"
        }
    })");
    PreflightConfig cfg = LoadPreflightConfig(path);

    ASSERT_EQ(cfg.models.size(), 2u);
    const ModelSpec& pa81 = cfg.models.at("motorevo_PA81_25");
    ASSERT_EQ(pa81.firmware_versions.size(), 1u);
    EXPECT_EQ(pa81.firmware_versions[0], 0x02030001u);
    ASSERT_EQ(pa81.params.size(), 3u);
    // 参数顺序保持配置顺序
    EXPECT_EQ(pa81.params[0].idx, 61);
    EXPECT_EQ(pa81.params[0].value.u32, 100u);
    EXPECT_EQ(pa81.params[1].idx, 30);
    EXPECT_FLOAT_EQ(pa81.params[1].value.f32, 1.5f);
    EXPECT_EQ(pa81.params[2].idx, 56);
    EXPECT_EQ(pa81.params[2].value.u32, 0xFFu);
    // 数字形式的 firmware_version (66051 = 0x00010203)
    ASSERT_EQ(cfg.models.at("ruiwo").firmware_versions.size(), 1u);
    EXPECT_EQ(cfg.models.at("ruiwo").firmware_versions[0], 0x00010203u);
    EXPECT_EQ(cfg.device_map.at("Lleg_joint_01"), "motorevo_PA81_25");
    EXPECT_EQ(cfg.device_map.at("Larm_joint_01"), "ruiwo");
}

TEST_F(PreflightConfigTest, ParsesFirmwareVersionWhitelist) {
    auto path = WriteConfig(R"({ "models": { "m": {
        "firmware_version": ["0x02030001", "0x0203000A", 3] } } })");
    PreflightConfig cfg = LoadPreflightConfig(path);
    const auto& versions = cfg.models.at("m").firmware_versions;
    ASSERT_EQ(versions.size(), 3u);
    EXPECT_EQ(versions[0], 0x02030001u);
    EXPECT_EQ(versions[1], 0x0203000Au);
    EXPECT_EQ(versions[2], 3u);
}

TEST_F(PreflightConfigTest, ThrowsOnEmptyFirmwareWhitelist) {
    auto path = WriteConfig(R"({ "models": { "m": { "firmware_version": [] } } })");
    EXPECT_THROW(LoadPreflightConfig(path), std::runtime_error);
}

TEST_F(PreflightConfigTest, ThrowsOnMissingFile) {
    EXPECT_THROW(LoadPreflightConfig("/nonexistent/motor_firmware.json"), std::runtime_error);
}

TEST_F(PreflightConfigTest, ThrowsOnMissingFirmwareVersion) {
    auto path = WriteConfig(R"({ "models": { "m": { "params": [] } } })");
    EXPECT_THROW(LoadPreflightConfig(path), std::runtime_error);
}

TEST_F(PreflightConfigTest, ThrowsOnBadDtype) {
    auto path = WriteConfig(R"({ "models": { "m": {
        "firmware_version": 1,
        "params": [ { "idx": 61, "dtype": "int64", "value": 1 } ] } } })");
    EXPECT_THROW(LoadPreflightConfig(path), std::runtime_error);
}

TEST_F(PreflightConfigTest, ThrowsOnFirmwareIdxInParams) {
    auto path = WriteConfig(R"({ "models": { "m": {
        "firmware_version": 1,
        "params": [ { "idx": 10, "dtype": "hex32", "value": "0x1" } ] } } })");
    EXPECT_THROW(LoadPreflightConfig(path), std::runtime_error);
}

TEST_F(PreflightConfigTest, ThrowsOnUnknownModelInDeviceMap) {
    auto path = WriteConfig(R"({
        "models": { "m": { "firmware_version": 1 } },
        "device_map": { "Lleg_joint_01": "nonexistent_model" }
    })");
    EXPECT_THROW(LoadPreflightConfig(path), std::runtime_error);
}

// ===== CheckMotorsOnBus 编排 =====

namespace {

// 模拟电机参数存储的假传输层:
// - 写帧更新内部存储 (ignore_writes 中的 (id,idx) 除外, 模拟写不生效)
// - 读帧返回存储值; dead_motors 中的电机不响应
class FakeTransport : public ParamTransport {
public:
    void Seed(uint8_t motor_id, uint8_t idx, uint32_t raw_le) {
        store_[Key(motor_id, idx)] = raw_le;
    }
    std::set<uint8_t> dead_motors;
    std::set<std::pair<uint8_t, uint8_t>> ignore_writes;

    bool Transact(uint8_t motor_id, const std::array<uint8_t, 8>& req,
                  std::array<uint8_t, 8>* resp) override {
        if (dead_motors.count(motor_id)) return false;
        EXPECT_EQ(req[0], 0x67);
        EXPECT_EQ(req[6], 0x04) << "Transact 只应收到读帧";
        const uint8_t idx = req[1];
        auto it = store_.find(Key(motor_id, idx));
        if (it == store_.end()) return false;
        (*resp)[0] = motor_id;
        (*resp)[1] = idx;
        std::memcpy(resp->data() + 2, &it->second, 4);
        (*resp)[6] = 0x00;
        (*resp)[7] = 0x76;
        return true;
    }

    bool SendOnly(uint8_t motor_id, const std::array<uint8_t, 8>& req) override {
        EXPECT_EQ(req[0], 0x67);
        EXPECT_EQ(req[6], 0x15) << "SendOnly 只应收到写帧";
        const uint8_t idx = req[1];
        if (dead_motors.count(motor_id)) return true;  // 发送本身成功, 电机不理
        if (ignore_writes.count({motor_id, idx})) return true;
        uint32_t raw = 0;
        std::memcpy(&raw, req.data() + 2, 4);
        store_[Key(motor_id, idx)] = raw;
        return true;
    }

private:
    static uint16_t Key(uint8_t id, uint8_t idx) { return (uint16_t(id) << 8) | idx; }
    std::map<uint16_t, uint32_t> store_;
};

PreflightConfig MakeTestConfig() {
    PreflightConfig cfg;
    ModelSpec pa81;
    pa81.firmware_versions.push_back(0x02030001);
    pa81.params.push_back({61, U32(100)});
    pa81.params.push_back({30, F32(1.5f)});
    cfg.models["motorevo_PA81_25"] = pa81;
    cfg.device_map["Lleg_joint_01"] = "motorevo_PA81_25";
    cfg.device_map["Lleg_joint_02"] = "motorevo_PA81_25";
    return cfg;
}

std::vector<MotorCheckItem> MakeTestItems() {
    return {{"bcan0", 0x01, "Lleg_joint_01"}, {"bcan0", 0x02, "Lleg_joint_02"}};
}

}  // namespace

TEST(CheckMotorsOnBus, AllPass) {
    FakeTransport t;
    t.Seed(0x01, kFirmwareVersionIdx, 0x02030001);
    t.Seed(0x02, kFirmwareVersionIdx, 0x02030001);
    auto results = CheckMotorsOnBus(t, MakeTestItems(), MakeTestConfig());
    ASSERT_EQ(results.size(), 2u);
    for (const auto& r : results) {
        EXPECT_TRUE(r.ok) << r.item.device_name;
        EXPECT_TRUE(r.fw_valid);
        EXPECT_EQ(r.params_total, 2);
        EXPECT_EQ(r.params_ok, 2);
        EXPECT_TRUE(r.errors.empty());
    }
}

TEST(CheckMotorsOnBus, FirmwareWhitelistSecondVersionPasses) {
    FakeTransport t;
    t.Seed(0x01, kFirmwareVersionIdx, 0x0203000A);  // 白名单第二个版本
    t.Seed(0x02, kFirmwareVersionIdx, 0x02030001);
    auto cfg = MakeTestConfig();
    cfg.models.at("motorevo_PA81_25").firmware_versions.push_back(0x0203000A);
    auto results = CheckMotorsOnBus(t, MakeTestItems(), cfg);
    EXPECT_TRUE(results[0].ok);
    EXPECT_TRUE(results[0].fw_valid);
    EXPECT_EQ(results[0].fw_read, 0x0203000Au);
    EXPECT_TRUE(results[1].ok);
}

TEST(CheckMotorsOnBus, FirmwareMismatchWarnsButDoesNotBlock) {
    FakeTransport t;
    t.Seed(0x01, kFirmwareVersionIdx, 0x02020009);  // 旧固件
    t.Seed(0x02, kFirmwareVersionIdx, 0x02030001);
    auto results = CheckMotorsOnBus(t, MakeTestItems(), MakeTestConfig());
    ASSERT_EQ(results.size(), 2u);
    // 固件不匹配仅告警: 参数照常写入校验, 不拦截启动
    EXPECT_TRUE(results[0].ok);
    EXPECT_FALSE(results[0].fw_valid);
    EXPECT_EQ(results[0].fw_read, 0x02020009u);
    EXPECT_EQ(results[0].params_total, 2) << "固件不匹配也要写参数";
    EXPECT_EQ(results[0].params_ok, 2);
    EXPECT_TRUE(results[0].errors.empty());
    ASSERT_EQ(results[0].warnings.size(), 1u);
    EXPECT_NE(results[0].warnings[0].find("固件版本不匹配"), std::string::npos);
    EXPECT_TRUE(results[1].ok);
    EXPECT_TRUE(results[1].warnings.empty());
}

TEST(CheckMotorsOnBus, FirmwareMismatchPlusParamFailureBlocks) {
    FakeTransport t;
    t.Seed(0x01, kFirmwareVersionIdx, 0x02020009);  // 旧固件
    t.Seed(0x02, kFirmwareVersionIdx, 0x02030001);
    t.Seed(0x01, 61, 99);
    t.ignore_writes.insert({0x01, 61});  // 参数写不生效 → 硬失败
    auto results = CheckMotorsOnBus(t, MakeTestItems(), MakeTestConfig());
    EXPECT_FALSE(results[0].ok);
    ASSERT_EQ(results[0].errors.size(), 1u);
    EXPECT_NE(results[0].errors[0].find("回读不一致"), std::string::npos);
    EXPECT_EQ(results[0].warnings.size(), 1u);
}

TEST(CheckMotorsOnBus, WriteIgnoredCausesReadbackMismatch) {
    FakeTransport t;
    t.Seed(0x01, kFirmwareVersionIdx, 0x02030001);
    t.Seed(0x02, kFirmwareVersionIdx, 0x02030001);
    t.Seed(0x01, 61, 99);                // 已有旧值
    t.ignore_writes.insert({0x01, 61});  // 写不生效
    auto results = CheckMotorsOnBus(t, MakeTestItems(), MakeTestConfig());
    EXPECT_FALSE(results[0].ok);
    EXPECT_TRUE(results[0].fw_valid);
    EXPECT_EQ(results[0].params_total, 2);
    EXPECT_EQ(results[0].params_ok, 1);
    ASSERT_EQ(results[0].errors.size(), 1u);
    EXPECT_NE(results[0].errors[0].find("回读不一致"), std::string::npos);
    EXPECT_TRUE(results[1].ok);
}

TEST(CheckMotorsOnBus, DeadMotorNoResponse) {
    FakeTransport t;
    t.Seed(0x02, kFirmwareVersionIdx, 0x02030001);
    t.dead_motors.insert(0x01);
    auto results = CheckMotorsOnBus(t, MakeTestItems(), MakeTestConfig());
    EXPECT_FALSE(results[0].ok);
    ASSERT_EQ(results[0].errors.size(), 1u);
    EXPECT_NE(results[0].errors[0].find("无响应"), std::string::npos);
    EXPECT_TRUE(results[1].ok);
}

TEST(CheckMotorsOnBus, MissingDeviceMapEntry) {
    FakeTransport t;
    t.Seed(0x01, kFirmwareVersionIdx, 0x02030001);
    auto cfg = MakeTestConfig();
    cfg.device_map.erase("Lleg_joint_01");
    auto results = CheckMotorsOnBus(t, MakeTestItems(), cfg);
    EXPECT_FALSE(results[0].ok);
    ASSERT_EQ(results[0].errors.size(), 1u);
    EXPECT_NE(results[0].errors[0].find("device_map 缺少设备"), std::string::npos);
    EXPECT_TRUE(results[0].model.empty());
}
