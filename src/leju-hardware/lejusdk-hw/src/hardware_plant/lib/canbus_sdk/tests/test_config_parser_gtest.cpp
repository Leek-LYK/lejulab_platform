#include "canbus_sdk/config_parser.h"
#include <gtest/gtest.h>
#include <string>
#include <fstream>
#include <sys/stat.h>

using namespace canbus_sdk;

class ConfigParserTest : public ::testing::Test {
protected:
    void SetUp() override {
        // 创建临时测试目录
        test_dir_ = "/tmp/config_parser_test";
        mkdir(test_dir_.c_str(), 0755);
    }

    void TearDown() override {
        // 清理临时测试文件
        std::string cmd = "rm -rf " + test_dir_;
        system(cmd.c_str());
    }

    // 创建测试配置文件
    std::string createConfigFile(const std::string& content) {
        std::string filename = test_dir_ + "/test_config_" + std::to_string(test_file_counter_++) + ".yaml";
        std::ofstream file(filename);
        file << content;
        file.close();
        return filename;
    }

    std::string test_dir_;
    static int test_file_counter_;
};

int ConfigParserTest::test_file_counter_ = 0;

// 测试正常配置文件解析
TEST_F(ConfigParserTest, ValidConfigFile) {
    std::string config_content = R"(
canbus_interfaces:
  canbus0:
    type: BUSMUST_A
    name: "Test Device 0"
    nbitrate: 500000
    dbitrate: 1000000
    nsamplepos: 75
    dsamplepos: 75
    enable: true
  canbus1:
    type: BUSMUST_B
    name: "Test Device 1"
    nbitrate: 500000
    dbitrate: 1000000
    nsamplepos: 75
    dsamplepos: 75
    enable: true

canbus0_devices:
  - name: motor_01
    class: motor
    device_id: 1
    params: [0, 35, 2, 0, 0, 0, 0]
    negtive: false
    enable: true
  - name: hand_01
    class: revo2_hand
    device_id: 127
    enable: true

canbus1_devices:
  - name: motor_02
    class: motor
    device_id: 2
    params: [0, 35, 2, 0, 0, 0, 0]
    negtive: true
    enable: true
)";

    std::string config_path = createConfigFile(config_content);
    ConfigParser parser;

    EXPECT_TRUE(parser.parseFromFile(config_path));

    // 验证CAN总线配置
    auto canbus_configs = parser.getCanBusConfigs();
    EXPECT_EQ(canbus_configs.size(), 2);
    EXPECT_EQ(canbus_configs[0].name, "canbus0");
    EXPECT_EQ(canbus_configs[1].name, "canbus1");
    EXPECT_EQ(canbus_configs[0].type, CanBusModelType::BUSMUST_A);
    EXPECT_EQ(canbus_configs[1].type, CanBusModelType::BUSMUST_B);

    // 验证设备配置
    auto canbus0_devices = parser.getDevices("canbus0");
    EXPECT_EQ(canbus0_devices.size(), 2);
    EXPECT_EQ(canbus0_devices[0].name, "motor_01");
    EXPECT_EQ(canbus0_devices[0].device_type, DeviceType::MOTOR);
    EXPECT_EQ(canbus0_devices[0].device_id, 1);
    EXPECT_EQ(canbus0_devices[0].negtive, false);
    EXPECT_EQ(canbus0_devices[0].ignore, false);
    EXPECT_EQ(canbus0_devices[0].motor_params.size(), 7);

    EXPECT_EQ(canbus0_devices[1].name, "hand_01");
    EXPECT_EQ(canbus0_devices[1].device_type, DeviceType::REVO2_HAND);
    EXPECT_EQ(canbus0_devices[1].device_id, 127);
    EXPECT_EQ(canbus0_devices[1].ignore, false);

    auto canbus1_devices = parser.getDevices("canbus1");
    EXPECT_EQ(canbus1_devices.size(), 1);
    EXPECT_EQ(canbus1_devices[0].name, "motor_02");
    EXPECT_EQ(canbus1_devices[0].negtive, true);
    EXPECT_EQ(canbus1_devices[0].ignore, false);

    // 验证设备类型过滤
    auto canbus0_motors = parser.getDevices("canbus0", DeviceType::MOTOR);
    EXPECT_EQ(canbus0_motors.size(), 1);
    auto canbus0_hands = parser.getDevices("canbus0", DeviceType::REVO2_HAND);
    EXPECT_EQ(canbus0_hands.size(), 1);
}

// 测试缺少canbus_interfaces字段
TEST_F(ConfigParserTest, MissingCanbusInterfaces) {
    std::string config_content = R"(
canbus0_devices:
  - name: motor_01
    class: motor
    device_id: 1
    params: [0, 35, 2, 0, 0, 0, 0]
)";

    std::string config_path = createConfigFile(config_content);
    ConfigParser parser;

    EXPECT_THROW(parser.parseFromFile(config_path), std::runtime_error);
}

// 测试canbus_interfaces不是map类型
TEST_F(ConfigParserTest, InvalidCanbusInterfacesType) {
    std::string config_content = R"(
canbus_interfaces:
  - canbus0
  - canbus1
)";

    std::string config_path = createConfigFile(config_content);
    ConfigParser parser;

    EXPECT_THROW(parser.parseFromFile(config_path), std::runtime_error);
}

// 测试CAN总线缺少必需字段
TEST_F(ConfigParserTest, MissingRequiredCanbusFields) {
    std::string config_content = R"(
canbus_interfaces:
  canbus0:
    type: BUSMUST_A
    name: "Test Device"
    # 缺少 nbitrate, dbitrate 等必需字段
)";

    std::string config_path = createConfigFile(config_content);
    ConfigParser parser;

    EXPECT_THROW(parser.parseFromFile(config_path), std::runtime_error);
}

// 测试重复的CAN总线名称
TEST_F(ConfigParserTest, DuplicateCanbusNames) {
    std::string config_content = R"(
canbus_interfaces:
  canbus0:
    type: BUSMUST_A
    name: "Test Device 0"
    nbitrate: 500000
    dbitrate: 1000000
    nsamplepos: 75
    dsamplepos: 75
    enable: true
  canbus0:
    type: BUSMUST_B
    name: "Test Device 1"
    nbitrate: 500000
    dbitrate: 1000000
    nsamplepos: 75
    dsamplepos: 75
    enable: true

canbus0_devices:
  - name: motor_01
    class: motor
    device_id: 1
    params: [0, 35, 2, 0, 0, 0, 0]
)";

    std::string config_path = createConfigFile(config_content);
    ConfigParser parser;

    EXPECT_THROW(parser.parseFromFile(config_path), std::runtime_error);
}

// 测试空的CAN总线名称
TEST_F(ConfigParserTest, EmptyCanbusName) {
    std::string config_content = R"(
canbus_interfaces:
  "":
    type: BUSMUST_A
    name: "Test Device"
    nbitrate: 500000
    dbitrate: 1000000
    nsamplepos: 75
    dsamplepos: 75
    enable: true
)";

    std::string config_path = createConfigFile(config_content);
    ConfigParser parser;

    EXPECT_THROW(parser.parseFromFile(config_path), std::runtime_error);
}

// 测试缺少设备配置
TEST_F(ConfigParserTest, MissingDeviceConfig) {
    std::string config_content = R"(
canbus_interfaces:
  canbus0:
    type: BUSMUST_A
    name: "Test Device"
    nbitrate: 500000
    dbitrate: 1000000
    nsamplepos: 75
    dsamplepos: 75
    enable: true
# 缺少 canbus0_devices
)";

    std::string config_path = createConfigFile(config_content);
    ConfigParser parser;

    EXPECT_THROW(parser.parseFromFile(config_path), std::runtime_error);
}

// 测试设备配置不是数组
TEST_F(ConfigParserTest, InvalidDeviceConfigType) {
    std::string config_content = R"(
canbus_interfaces:
  canbus0:
    type: BUSMUST_A
    name: "Test Device"
    nbitrate: 500000
    dbitrate: 1000000
    nsamplepos: 75
    dsamplepos: 75
    enable: true

canbus0_devices:
  invalid_config: "not an array"
)";

    std::string config_path = createConfigFile(config_content);
    ConfigParser parser;

    EXPECT_THROW(parser.parseFromFile(config_path), std::runtime_error);
}

// 测试设备缺少必需字段
TEST_F(ConfigParserTest, MissingRequiredDeviceFields) {
    std::string config_content = R"(
canbus_interfaces:
  canbus0:
    type: BUSMUST_A
    name: "Test Device"
    nbitrate: 500000
    dbitrate: 1000000
    nsamplepos: 75
    dsamplepos: 75
    enable: true

canbus0_devices:
  - name: motor_01
    class: motor
    # 缺少 device_id 字段
)";

    std::string config_path = createConfigFile(config_content);
    ConfigParser parser;

    EXPECT_THROW(parser.parseFromFile(config_path), std::runtime_error);
}

// 测试电机设备缺少params字段
TEST_F(ConfigParserTest, MissingMotorParams) {
    std::string config_content = R"(
canbus_interfaces:
  canbus0:
    type: BUSMUST_A
    name: "Test Device"
    nbitrate: 500000
    dbitrate: 1000000
    nsamplepos: 75
    dsamplepos: 75
    enable: true

canbus0_devices:
  - name: motor_01
    class: motor
    device_id: 1
    # 缺少 params 字段
)";

    std::string config_path = createConfigFile(config_content);
    ConfigParser parser;

    EXPECT_THROW(parser.parseFromFile(config_path), std::runtime_error);
}

// 测试重复的设备名称
TEST_F(ConfigParserTest, DuplicateDeviceNames) {
    std::string config_content = R"(
canbus_interfaces:
  canbus0:
    type: BUSMUST_A
    name: "Test Device"
    nbitrate: 500000
    dbitrate: 1000000
    nsamplepos: 75
    dsamplepos: 75
    enable: true

canbus0_devices:
  - name: motor_01
    class: motor
    device_id: 1
    params: [0, 35, 2, 0, 0, 0, 0]
  - name: motor_01
    class: motor
    device_id: 2
    params: [0, 35, 2, 0, 0, 0, 0]
)";

    std::string config_path = createConfigFile(config_content);
    ConfigParser parser;

    EXPECT_THROW(parser.parseFromFile(config_path), std::runtime_error);
}

// 测试重复的设备ID（同一CAN总线）
TEST_F(ConfigParserTest, DuplicateDeviceIds) {
    std::string config_content = R"(
canbus_interfaces:
  canbus0:
    type: BUSMUST_A
    name: "Test Device"
    nbitrate: 500000
    dbitrate: 1000000
    nsamplepos: 75
    dsamplepos: 75
    enable: true

canbus0_devices:
  - name: motor_01
    class: motor
    device_id: 1
    params: [0, 35, 2, 0, 0, 0, 0]
  - name: motor_02
    class: motor
    device_id: 1
    params: [0, 35, 2, 0, 0, 0, 0]
)";

    std::string config_path = createConfigFile(config_content);
    ConfigParser parser;

    EXPECT_THROW(parser.parseFromFile(config_path), std::runtime_error);
}

// 测试文件不存在
TEST_F(ConfigParserTest, FileNotFound) {
    ConfigParser parser;
    EXPECT_FALSE(parser.parseFromFile("/nonexistent/file.yaml"));
}

// 测试无效的YAML语法
TEST_F(ConfigParserTest, InvalidYamlSyntax) {
    std::string config_content = R"(
canbus_interfaces:
  canbus0:
    type: BUSMUST_A
    name: "Test Device"
    nbitrate: 500000
    dbitrate: 1000000
    nsamplepos: 75
    dsamplepos: 75
    enable: true
  invalid_yaml: [unclosed bracket
)";

    std::string config_path = createConfigFile(config_content);
    ConfigParser parser;

    EXPECT_FALSE(parser.parseFromFile(config_path));
}

// 测试空的配置文件
TEST_F(ConfigParserTest, EmptyConfigFile) {
    std::string config_content = "";
    std::string config_path = createConfigFile(config_content);
    ConfigParser parser;

    EXPECT_THROW(parser.parseFromFile(config_path), std::runtime_error);
}

// 测试设备参数类型转换
TEST_F(ConfigParserTest, DeviceParamTypeConversion) {
    std::string config_content = R"(
canbus_interfaces:
  canbus0:
    type: BUSMUST_A
    name: "Test Device"
    nbitrate: 500000
    dbitrate: 1000000
    nsamplepos: 75
    dsamplepos: 75
    enable: true

canbus0_devices:
  - name: motor_01
    class: motor
    device_id: 1
    params: [1.5, 2.7, 3.14]  # 浮点数参数
)";

    std::string config_path = createConfigFile(config_content);
    ConfigParser parser;

    EXPECT_TRUE(parser.parseFromFile(config_path));

    auto devices = parser.getDevices("canbus0");
    ASSERT_EQ(devices.size(), 1);
    EXPECT_EQ(devices[0].motor_params.size(), 3);
    EXPECT_FLOAT_EQ(devices[0].motor_params[0], 1.5f);
    EXPECT_FLOAT_EQ(devices[0].motor_params[1], 2.7f);
    EXPECT_FLOAT_EQ(devices[0].motor_params[2], 3.14f);
}