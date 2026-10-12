#include <gtest/gtest.h>
#include <lejusdk-utils/robot_version.hpp>
#include <stdexcept>
#include <cstdlib>

using namespace leju;

class RobotVersionTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Clear environment variable before each test
        unsetenv("ROBOT_VERSION");
    }

    void TearDown() override {
        // Clean up after each test
        unsetenv("ROBOT_VERSION");
    }
};

// Test default constructor
TEST_F(RobotVersionTest, DefaultConstructor) {
    RobotVersion version;
    EXPECT_EQ(version.major(), 0);
    EXPECT_EQ(version.minor(), 0);
    EXPECT_EQ(version.patch(), 0);
    EXPECT_EQ(version.version_name(), "0.0.0");
    EXPECT_EQ(version.version_name_short(), "0.0");
}

// Test parameterized constructor
TEST_F(RobotVersionTest, ParameterizedConstructor) {
    RobotVersion v1(4, 5);
    EXPECT_EQ(v1.major(), 4);
    EXPECT_EQ(v1.minor(), 5);
    EXPECT_EQ(v1.patch(), 0);

    RobotVersion v2(10, 20, 30);
    EXPECT_EQ(v2.major(), 10);
    EXPECT_EQ(v2.minor(), 20);
    EXPECT_EQ(v2.patch(), 30);
}

// Test constructor with invalid values
TEST_F(RobotVersionTest, ConstructorWithInvalidValues) {
    EXPECT_THROW(RobotVersion(10000, 0, 0), std::invalid_argument);
    EXPECT_THROW(RobotVersion(0, 10000, 0), std::invalid_argument);
    EXPECT_THROW(RobotVersion(0, 0, 10000), std::invalid_argument);
}

// Test create from big number
TEST_F(RobotVersionTest, CreateFromBigNumber) {
    // Test PPPPMMMMN format: PPPP*100000 + MMMM*10 + N
    // where: PPPP=patch, MMMM=major, N=minor
    // Example: 40000045 -> PPPP=400, MMMM=4, N=5 -> version 4.5.400

    RobotVersion v1 = RobotVersion::create(45);        // 45 -> major=4, minor=5, patch=0
    EXPECT_EQ(v1.major(), 4);
    EXPECT_EQ(v1.minor(), 5);
    EXPECT_EQ(v1.patch(), 0);
    EXPECT_EQ(v1.version_name(), "4.5.0");

    RobotVersion v2 = RobotVersion::create(100045);    // 100045 -> major=4, minor=5, patch=1
    EXPECT_EQ(v2.major(), 4);
    EXPECT_EQ(v2.minor(), 5);
    EXPECT_EQ(v2.patch(), 1);
    EXPECT_EQ(v2.version_name(), "4.5.1");

    RobotVersion v3 = RobotVersion::create(4000045);   // 4000045 -> major=4, minor=5, patch=40
    EXPECT_EQ(v3.major(), 4);
    EXPECT_EQ(v3.minor(), 5);
    EXPECT_EQ(v3.patch(), 40);
    EXPECT_EQ(v3.version_name(), "4.5.40");

    RobotVersion v4 = RobotVersion::create(2000105);   // 2000105 -> major=10, minor=5, patch=20
    EXPECT_EQ(v4.major(), 10);
    EXPECT_EQ(v4.minor(), 5);
    EXPECT_EQ(v4.patch(), 20);
    EXPECT_EQ(v4.version_name(), "10.5.20");

    // More examples
    RobotVersion v5 = RobotVersion::create(105);       // 105 -> major=10, minor=5, patch=0
    EXPECT_EQ(v5.major(), 10);
    EXPECT_EQ(v5.minor(), 5);
    EXPECT_EQ(v5.patch(), 0);
    EXPECT_EQ(v5.version_name(), "10.5.0");

    RobotVersion v6 = RobotVersion::create(12345678);  // 12345678 -> major=4567, minor=8, patch=123
    EXPECT_EQ(v6.major(), 4567);
    EXPECT_EQ(v6.minor(), 8);
    EXPECT_EQ(v6.patch(), 123);
    EXPECT_EQ(v6.version_name(), "4567.8.123");
}

// Test create from invalid big number
TEST_F(RobotVersionTest, CreateFromInvalidBigNumber) {
    EXPECT_THROW(RobotVersion::create(0), std::invalid_argument);
    EXPECT_THROW(RobotVersion::create(-1), std::invalid_argument);
    EXPECT_THROW(RobotVersion::create(1000000000), std::invalid_argument);
}

// Test is_valid static method
TEST_F(RobotVersionTest, IsValid) {
    EXPECT_TRUE(RobotVersion::is_valid(1));
    EXPECT_TRUE(RobotVersion::is_valid(45));
    EXPECT_TRUE(RobotVersion::is_valid(999999999));
    EXPECT_FALSE(RobotVersion::is_valid(0));
    EXPECT_FALSE(RobotVersion::is_valid(-1));
    EXPECT_FALSE(RobotVersion::is_valid(1000000000));
}

// Test from_env method
TEST_F(RobotVersionTest, FromEnv) {
    // Test with valid environment variable
    setenv("ROBOT_VERSION", "45", 1);
    RobotVersion v1 = RobotVersion::from_env();
    EXPECT_EQ(v1.major(), 4);
    EXPECT_EQ(v1.minor(), 5);
    EXPECT_EQ(v1.patch(), 0);
    EXPECT_EQ(v1.version_name(), "4.5.0");

    // Test with another valid value
    setenv("ROBOT_VERSION", "2000105", 1);
    RobotVersion v2 = RobotVersion::from_env();
    EXPECT_EQ(v2.major(), 10);
    EXPECT_EQ(v2.minor(), 5);
    EXPECT_EQ(v2.patch(), 20);
    EXPECT_EQ(v2.version_name(), "10.5.20");

    // Test with 4000045 -> 4.5.40
    setenv("ROBOT_VERSION", "4000045", 1);
    RobotVersion v3 = RobotVersion::from_env();
    EXPECT_EQ(v3.major(), 4);
    EXPECT_EQ(v3.minor(), 5);
    EXPECT_EQ(v3.patch(), 40);
    EXPECT_EQ(v3.version_name(), "4.5.40");
}

// Test from_env with invalid environment
TEST_F(RobotVersionTest, FromEnvInvalid) {
    // Test without environment variable
    EXPECT_THROW(RobotVersion::from_env(), std::invalid_argument);

    // Test with invalid environment variable
    setenv("ROBOT_VERSION", "invalid", 1);
    EXPECT_THROW(RobotVersion::from_env(), std::invalid_argument);

    setenv("ROBOT_VERSION", "0", 1);
    EXPECT_THROW(RobotVersion::from_env(), std::invalid_argument);
}

// Test start_with methods
TEST_F(RobotVersionTest, StartWith) {
    RobotVersion v(4, 5, 0);

    EXPECT_TRUE(v.start_with(4));
    EXPECT_FALSE(v.start_with(5));

    EXPECT_TRUE(v.start_with(4, 5));
    EXPECT_FALSE(v.start_with(4, 6));
    EXPECT_FALSE(v.start_with(5, 5));

    // Test with versions that have same prefix
    RobotVersion v2(4, 5, 1);
    EXPECT_TRUE(v2.start_with(4));
    EXPECT_TRUE(v2.start_with(4, 5));
}

// Test version_name method
TEST_F(RobotVersionTest, VersionName) {
    RobotVersion v1(4, 5, 0);
    EXPECT_EQ(v1.version_name(), "4.5.0");

    RobotVersion v2(10, 20, 30);
    EXPECT_EQ(v2.version_name(), "10.20.30");

    RobotVersion v3(0, 0, 0);
    EXPECT_EQ(v3.version_name(), "0.0.0");
}

// Test version_name_short method
TEST_F(RobotVersionTest, VersionNameShort) {
    RobotVersion v1(4, 5, 0);
    EXPECT_EQ(v1.version_name_short(), "4.5");

    RobotVersion v2(4, 5, 1);
    EXPECT_EQ(v2.version_name_short(), "4.5.1");

    RobotVersion v3(10, 0, 0);
    EXPECT_EQ(v3.version_name_short(), "10.0");

    RobotVersion v4(0, 0, 1);
    EXPECT_EQ(v4.version_name_short(), "0.0.1");
}

// Test comparison operators
TEST_F(RobotVersionTest, ComparisonOperators) {
    RobotVersion v1(4, 5, 0);
    RobotVersion v2(4, 5, 1);
    RobotVersion v3(4, 6, 0);
    RobotVersion v4(5, 0, 0);
    RobotVersion v5(4, 5, 0);

    // Test equality
    EXPECT_TRUE(v1 == v5);
    EXPECT_FALSE(v1 == v2);

    // Test inequality
    EXPECT_TRUE(v1 != v2);
    EXPECT_FALSE(v1 != v5);

    // Test less than
    EXPECT_TRUE(v1 < v2);
    EXPECT_TRUE(v2 < v3);
    EXPECT_TRUE(v3 < v4);
    EXPECT_FALSE(v1 < v5);
    EXPECT_FALSE(v2 < v1);

    // Test less than or equal
    EXPECT_TRUE(v1 <= v2);
    EXPECT_TRUE(v1 <= v5);
    EXPECT_FALSE(v2 <= v1);

    // Test greater than
    EXPECT_TRUE(v2 > v1);
    EXPECT_TRUE(v3 > v2);
    EXPECT_TRUE(v4 > v3);
    EXPECT_FALSE(v1 > v5);
    EXPECT_FALSE(v1 > v2);

    // Test greater than or equal
    EXPECT_TRUE(v2 >= v1);
    EXPECT_TRUE(v1 >= v5);
    EXPECT_FALSE(v1 >= v2);
}

// Test predefined versions
TEST_F(RobotVersionTest, PredefinedVersions) {
    using namespace RobotVersions;

    EXPECT_EQ(KUAVO4_BASE.major(), 4);
    EXPECT_EQ(KUAVO4_BASE.minor(), 2);
    EXPECT_EQ(KUAVO4_BASE.patch(), 0);
    EXPECT_EQ(KUAVO4_BASE.version_name(), "4.2.0");

    EXPECT_EQ(KUAVO4_PRO_DEXHAND.major(), 4);
    EXPECT_EQ(KUAVO4_PRO_DEXHAND.minor(), 5);
    EXPECT_EQ(KUAVO4_PRO_DEXHAND.patch(), 0);
    EXPECT_EQ(KUAVO4_PRO_DEXHAND.version_name(), "4.5.0");

    EXPECT_EQ(KUAVO4_PRO_GRIPPER.major(), 4);
    EXPECT_EQ(KUAVO4_PRO_GRIPPER.minor(), 7);
    EXPECT_EQ(KUAVO4_PRO_GRIPPER.patch(), 0);
    EXPECT_EQ(KUAVO4_PRO_GRIPPER.version_name(), "4.7.0");

    EXPECT_EQ(KUAVO4_EDU.major(), 4);
    EXPECT_EQ(KUAVO4_EDU.minor(), 9);
    EXPECT_EQ(KUAVO4_EDU.patch(), 0);
    EXPECT_EQ(KUAVO4_EDU.version_name(), "4.9.0");

    EXPECT_EQ(KUAVO5_BASE.major(), 5);
    EXPECT_EQ(KUAVO5_BASE.minor(), 0);
    EXPECT_EQ(KUAVO5_BASE.patch(), 0);
    EXPECT_EQ(KUAVO5_BASE.version_name(), "5.0.0");

    EXPECT_EQ(ROBAN2_BASE.major(), 1);
    EXPECT_EQ(ROBAN2_BASE.minor(), 4);
    EXPECT_EQ(ROBAN2_BASE.patch(), 0);
    EXPECT_EQ(ROBAN2_BASE.version_name(), "1.4.0");
}

// Test ostream operator
TEST_F(RobotVersionTest, OstreamOperator) {
    RobotVersion v(4, 5, 0);
    std::ostringstream oss;
    oss << v;
    EXPECT_EQ(oss.str(), "4.5.0");

    RobotVersion v2(10, 20, 30);
    oss.str("");
    oss.clear();
    oss << v2;
    EXPECT_EQ(oss.str(), "10.20.30");
}

// Test copy constructor and assignment
TEST_F(RobotVersionTest, CopyAndAssignment) {
    RobotVersion v1(4, 5, 6);
    RobotVersion v2(v1);  // Copy constructor

    EXPECT_EQ(v2.major(), 4);
    EXPECT_EQ(v2.minor(), 5);
    EXPECT_EQ(v2.patch(), 6);
    EXPECT_TRUE(v1 == v2);

    RobotVersion v3;
    v3 = v1;  // Assignment operator

    EXPECT_EQ(v3.major(), 4);
    EXPECT_EQ(v3.minor(), 5);
    EXPECT_EQ(v3.patch(), 6);
    EXPECT_TRUE(v1 == v3);
}

// Test constexpr support
TEST_F(RobotVersionTest, ConstexprSupport) {
    // Test that the constructor can be used in constexpr context
    constexpr RobotVersion v(4, 5, 6);
    EXPECT_EQ(v.major(), 4);
    EXPECT_EQ(v.minor(), 5);
    EXPECT_EQ(v.patch(), 6);

    // Test that create from big_number works at runtime
    RobotVersion v2 = RobotVersion::create(2000105);
    EXPECT_EQ(v2.major(), 10);
    EXPECT_EQ(v2.minor(), 5);
    EXPECT_EQ(v2.patch(), 20);

    RobotVersion v3 = RobotVersion::create(4000045);
    EXPECT_EQ(v3.major(), 4);
    EXPECT_EQ(v3.minor(), 5);
    EXPECT_EQ(v3.patch(), 40);

    // Test is_valid can be used in constexpr context
    constexpr bool valid = RobotVersion::is_valid(45);
    EXPECT_TRUE(valid);

    SUCCEED();
}

// Test robot type macros
TEST_F(RobotVersionTest, RobotTypeMacros) {
    // Test IS_KUAVO_LEGGED macro
    RobotVersion kuavo4(4, 0, 0);
    RobotVersion kuavo5(5, 0, 0);
    RobotVersion kuavo4_pro(4, 5, 0);
    RobotVersion kuavo4_pro2(4, 7, 0);
    RobotVersion non_kuavo(3, 0, 0);

    EXPECT_TRUE(IS_KUAVO_LEGGED(kuavo4));
    EXPECT_TRUE(IS_KUAVO_LEGGED(kuavo5));
    EXPECT_FALSE(IS_KUAVO_LEGGED(non_kuavo));

    // Test IS_KUAVO4_LEGGED macro
    EXPECT_TRUE(IS_KUAVO4_LEGGED(kuavo4));
    EXPECT_TRUE(IS_KUAVO4_LEGGED(kuavo4_pro));
    EXPECT_TRUE(IS_KUAVO4_LEGGED(kuavo4_pro2));
    EXPECT_FALSE(IS_KUAVO4_LEGGED(kuavo5));
    EXPECT_FALSE(IS_KUAVO4_LEGGED(non_kuavo));

    // Test IS_KUAVO5_LEGGED macro
    EXPECT_FALSE(IS_KUAVO5_LEGGED(kuavo4));
    EXPECT_TRUE(IS_KUAVO5_LEGGED(kuavo5));
    EXPECT_FALSE(IS_KUAVO5_LEGGED(non_kuavo));

    // Test IS_KUAVO4PRO_LEGGED macro
    EXPECT_FALSE(IS_KUAVO4PRO_LEGGED(kuavo4));  // 4.0 is not 4pro
    EXPECT_TRUE(IS_KUAVO4PRO_LEGGED(kuavo4_pro));  // 4.5 is 4pro
    EXPECT_TRUE(IS_KUAVO4PRO_LEGGED(kuavo4_pro2));  // 4.7 is 4pro
    EXPECT_FALSE(IS_KUAVO4PRO_LEGGED(kuavo5));
    EXPECT_FALSE(IS_KUAVO4PRO_LEGGED(non_kuavo));

    // Test IS_ROBAN_LEGGED and IS_ROBAN2_LEGGED macros
    RobotVersion roban1(1, 0, 0);
    RobotVersion roban14(1, 4, 0);
    RobotVersion roban2(2, 0, 0);

    EXPECT_TRUE(IS_ROBAN_LEGGED(roban1));
    EXPECT_TRUE(IS_ROBAN_LEGGED(roban14));
    EXPECT_TRUE(IS_ROBAN2_LEGGED(roban1));
    EXPECT_TRUE(IS_ROBAN2_LEGGED(roban14));
    EXPECT_FALSE(IS_ROBAN_LEGGED(roban2));
    EXPECT_FALSE(IS_ROBAN2_LEGGED(roban2));

    // Test IS_KUAVO_WHEELED macro
    RobotVersion kuavo_wheeled(6, 0, 0);
    EXPECT_FALSE(IS_KUAVO_WHEELED(kuavo4));
    EXPECT_FALSE(IS_KUAVO_WHEELED(kuavo5));
    EXPECT_TRUE(IS_KUAVO_WHEELED(kuavo_wheeled));
    EXPECT_FALSE(IS_KUAVO_WHEELED(non_kuavo));
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}