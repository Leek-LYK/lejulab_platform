/**
 * @file test_rpc_types.cpp
 * @brief rpc_types.hpp 工具函数单元测试
 */

#include <gtest/gtest.h>
#include <set>
#include <unordered_set>

#include "dds_rpc/rpc_types.hpp"
#include "rpc_idl/calculator_service.hpp"

using namespace dds_rpc;

//=============================================================================
// GUID 生成测试
//=============================================================================

TEST(RpcTypesTest, GenerateGuid_NonZero) {
    std::array<uint8_t, 16> guid{};
    generate_guid(guid);

    // GUID 不应全为 0
    bool all_zero = true;
    for (const auto& byte : guid) {
        if (byte != 0) {
            all_zero = false;
            break;
        }
    }
    EXPECT_FALSE(all_zero) << "Generated GUID should not be all zeros";
}

TEST(RpcTypesTest, GenerateGuid_Uniqueness) {
    const int NUM_GUIDS = 100;
    std::set<std::array<uint8_t, 16>> guids;

    for (int i = 0; i < NUM_GUIDS; ++i) {
        std::array<uint8_t, 16> guid{};
        generate_guid(guid);
        guids.insert(guid);
    }

    // 所有 GUID 应该都是唯一的
    EXPECT_EQ(guids.size(), NUM_GUIDS)
        << "All generated GUIDs should be unique";
}

//=============================================================================
// Topic 名称测试
//=============================================================================

TEST(RpcTypesTest, RequestTopicName_Format) {
    std::string name = request_topic_name("MyService");
    EXPECT_EQ(name, "rq/MyService/Request");
}

TEST(RpcTypesTest, ReplyTopicName_Format) {
    std::string name = reply_topic_name("MyService");
    EXPECT_EQ(name, "rr/MyService/Reply");
}

TEST(RpcTypesTest, TopicName_EmptyService) {
    std::string req_name = request_topic_name("");
    std::string rep_name = reply_topic_name("");

    EXPECT_EQ(req_name, "rq//Request");
    EXPECT_EQ(rep_name, "rr//Reply");
}

TEST(RpcTypesTest, TopicName_SpecialCharacters) {
    std::string service = "My_Service-123";
    std::string req_name = request_topic_name(service);
    std::string rep_name = reply_topic_name(service);

    EXPECT_EQ(req_name, "rq/My_Service-123/Request");
    EXPECT_EQ(rep_name, "rr/My_Service-123/Reply");
}

//=============================================================================
// SampleIdentity 哈希测试
//=============================================================================

TEST(RpcTypesTest, SampleIdentityHash_Consistency) {
    SampleIdentity id1;
    std::array<uint8_t, 16> guid{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
    id1.writer_guid(guid);
    id1.sequence_number(42);

    SampleIdentityHash hasher;
    std::size_t hash1 = hasher(id1);
    std::size_t hash2 = hasher(id1);

    // 同一对象的哈希应该一致
    EXPECT_EQ(hash1, hash2);
}

TEST(RpcTypesTest, SampleIdentityHash_DifferentIds) {
    SampleIdentity id1, id2;

    std::array<uint8_t, 16> guid1{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
    std::array<uint8_t, 16> guid2{16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1};

    id1.writer_guid(guid1);
    id1.sequence_number(42);

    id2.writer_guid(guid2);
    id2.sequence_number(42);

    SampleIdentityHash hasher;
    // 不同的 GUID 应该产生不同的哈希（极大概率）
    EXPECT_NE(hasher(id1), hasher(id2));
}

TEST(RpcTypesTest, SampleIdentityHash_DifferentSequence) {
    SampleIdentity id1, id2;

    std::array<uint8_t, 16> guid{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
    id1.writer_guid(guid);
    id1.sequence_number(42);

    id2.writer_guid(guid);
    id2.sequence_number(43);

    SampleIdentityHash hasher;
    // 不同的序列号应该产生不同的哈希
    EXPECT_NE(hasher(id1), hasher(id2));
}

TEST(RpcTypesTest, SampleIdentity_UnorderedMap) {
    std::unordered_map<SampleIdentity, int, SampleIdentityHash> map;

    SampleIdentity id1, id2;
    std::array<uint8_t, 16> guid1{}, guid2{};
    generate_guid(guid1);
    generate_guid(guid2);

    id1.writer_guid(guid1);
    id1.sequence_number(1);
    id2.writer_guid(guid2);
    id2.sequence_number(2);

    map[id1] = 100;
    map[id2] = 200;

    EXPECT_EQ(map.size(), 2u);
    EXPECT_EQ(map[id1], 100);
    EXPECT_EQ(map[id2], 200);
}

//=============================================================================
// 类型约束 (Type Traits) 测试
//=============================================================================

TEST(RpcTypesTest, TypeTraits_ValidRequest) {
    // CalculatorRequest 应该是有效的 Request 类型
    EXPECT_TRUE(traits::has_header_v<example::CalculatorRequest>);
    EXPECT_TRUE(traits::has_request_id_v<example::CalculatorRequest>);
    EXPECT_TRUE(traits::is_valid_request_v<example::CalculatorRequest>);
}

TEST(RpcTypesTest, TypeTraits_ValidReply) {
    // CalculatorReply 应该是有效的 Reply 类型
    EXPECT_TRUE(traits::has_header_v<example::CalculatorReply>);
    EXPECT_TRUE(traits::has_related_request_id_v<example::CalculatorReply>);
    EXPECT_TRUE(traits::has_remote_ex_v<example::CalculatorReply>);
    EXPECT_TRUE(traits::is_valid_reply_v<example::CalculatorReply>);
}

// 定义一个无效的类型用于测试
struct InvalidType {
    int value;
};

TEST(RpcTypesTest, TypeTraits_InvalidType) {
    // InvalidType 不应该满足类型约束
    EXPECT_FALSE(traits::has_header_v<InvalidType>);
    EXPECT_FALSE(traits::is_valid_request_v<InvalidType>);
    EXPECT_FALSE(traits::is_valid_reply_v<InvalidType>);
}

//=============================================================================
// QoS 配置测试
//=============================================================================

TEST(RpcTypesTest, RpcQosConfig_Defaults) {
    RpcQosConfig config;

    EXPECT_TRUE(config.reliable);
    EXPECT_EQ(config.history_depth, 100);
    EXPECT_EQ(config.deadline, DDS_INFINITY);
    EXPECT_EQ(config.lifespan, DDS_SECS(60));
    EXPECT_FALSE(config.allow_duplicate_service);
    EXPECT_EQ(config.discovery_wait_ms, 300);
}

TEST(RpcTypesTest, RpcQosConfig_CreateWriterQos) {
    RpcQosConfig config;
    dds_qos_t* qos = config.create_writer_qos();

    ASSERT_NE(qos, nullptr);

    // 清理
    dds_delete_qos(qos);
}

TEST(RpcTypesTest, RpcQosConfig_CreateReaderQos) {
    RpcQosConfig config;
    dds_qos_t* qos = config.create_reader_qos();

    ASSERT_NE(qos, nullptr);

    // 清理
    dds_delete_qos(qos);
}

//=============================================================================
// 异常测试
//=============================================================================

TEST(RpcTypesTest, RpcException_Construction) {
    RpcException ex("Test error", RemoteExceptionCode::REMOTE_EX_INVALID_ARGUMENT);

    EXPECT_STREQ(ex.what(), "Test error");
    EXPECT_EQ(ex.code(), RemoteExceptionCode::REMOTE_EX_INVALID_ARGUMENT);
}

TEST(RpcTypesTest, RpcException_DefaultCode) {
    RpcException ex("Test error");

    EXPECT_STREQ(ex.what(), "Test error");
    EXPECT_EQ(ex.code(), RemoteExceptionCode::REMOTE_EX_UNKNOWN_EXCEPTION);
}

TEST(RpcTypesTest, TimeoutException_Construction) {
    TimeoutException ex;

    EXPECT_STREQ(ex.what(), "Request timeout");
    EXPECT_EQ(ex.code(), RemoteExceptionCode::REMOTE_EX_UNKNOWN_EXCEPTION);
}

TEST(RpcTypesTest, ServiceNotFoundException_Construction) {
    ServiceNotFoundException ex("TestService");

    std::string msg = ex.what();
    EXPECT_TRUE(msg.find("TestService") != std::string::npos);
    EXPECT_EQ(ex.code(), RemoteExceptionCode::REMOTE_EX_UNKNOWN_OPERATION);
}

TEST(RpcTypesTest, DuplicateServiceException_Construction) {
    DuplicateServiceException ex("TestService");

    std::string msg = ex.what();
    EXPECT_TRUE(msg.find("TestService") != std::string::npos);
    EXPECT_TRUE(msg.find("already exists") != std::string::npos);
    EXPECT_EQ(ex.code(), RemoteExceptionCode::REMOTE_EX_UNSUPPORTED);
}

//=============================================================================
// SampleIdentity 相等性测试
//=============================================================================

TEST(RpcTypesTest, SampleIdentity_Equality) {
    SampleIdentity id1, id2;
    std::array<uint8_t, 16> guid{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};

    id1.writer_guid(guid);
    id1.sequence_number(42);

    id2.writer_guid(guid);
    id2.sequence_number(42);

    EXPECT_EQ(id1, id2);
}

TEST(RpcTypesTest, SampleIdentity_Inequality) {
    SampleIdentity id1, id2;
    std::array<uint8_t, 16> guid{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};

    id1.writer_guid(guid);
    id1.sequence_number(42);

    id2.writer_guid(guid);
    id2.sequence_number(43);

    EXPECT_NE(id1, id2);
}
