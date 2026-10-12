#include <gtest/gtest.h>
#include <thread>
#include <chrono>
#include <atomic>
#include <vector>

#include <dds/dds.hpp>
#include "lejusdk-lowlevel/topic_logger.h"
#include "lejusdk-topic-pubsub/topic_subscriber.hpp"
#include "lejusdk-dds-idl/Float64Types.hpp"

using namespace leju;

class TopicLoggerTest : public ::testing::Test {
protected:
    void SetUp() override {
        logger_ = TopicLogger::create(0);
        // 为端到端测试创建独立的 participant
        subscriber_participant_ = dds::domain::DomainParticipant(0);
    }

    void TearDown() override {
        logger_.reset();
        subscriber_participant_ = dds::core::null;
    }

    std::unique_ptr<TopicLogger> logger_;
    dds::domain::DomainParticipant subscriber_participant_{dds::core::null};
};

// 测试基本构造
TEST_F(TopicLoggerTest, Construction) {
    EXPECT_EQ(logger_->vectorTopicCount(), 0);
    EXPECT_EQ(logger_->valueTopicCount(), 0);
}

// 测试发布向量数据
TEST_F(TopicLoggerTest, PublishVector) {
    std::vector<double> data = {1.0, 2.0, 3.0, 4.0, 5.0};

    bool result = logger_->publishVector("/test/vector_topic", data);
    EXPECT_TRUE(result);
    EXPECT_EQ(logger_->vectorTopicCount(), 1);

    // 再次发布到同一 topic（复用 publisher）
    result = logger_->publishVector("/test/vector_topic", data);
    EXPECT_TRUE(result);
    EXPECT_EQ(logger_->vectorTopicCount(), 1);  // 仍然是 1

    // 发布到不同 topic
    result = logger_->publishVector("/test/another_vector", data);
    EXPECT_TRUE(result);
    EXPECT_EQ(logger_->vectorTopicCount(), 2);
}

// 测试发布标量数据
TEST_F(TopicLoggerTest, PublishValue) {
    bool result = logger_->publishValue("/test/value_topic", 3.14159);
    EXPECT_TRUE(result);
    EXPECT_EQ(logger_->valueTopicCount(), 1);

    // 再次发布到同一 topic
    result = logger_->publishValue("/test/value_topic", 2.71828);
    EXPECT_TRUE(result);
    EXPECT_EQ(logger_->valueTopicCount(), 1);

    // 发布到不同 topic
    result = logger_->publishValue("/test/another_value", 1.41421);
    EXPECT_TRUE(result);
    EXPECT_EQ(logger_->valueTopicCount(), 2);
}

// 测试发布空向量
TEST_F(TopicLoggerTest, PublishEmptyVector) {
    std::vector<double> empty_data;

    bool result = logger_->publishVector("/test/empty_vector", empty_data);
    EXPECT_TRUE(result);
    EXPECT_EQ(logger_->vectorTopicCount(), 1);
}

// 测试发布大向量
TEST_F(TopicLoggerTest, PublishLargeVector) {
    std::vector<double> large_data(1000);
    for (size_t i = 0; i < large_data.size(); ++i) {
        large_data[i] = static_cast<double>(i) * 0.001;
    }

    bool result = logger_->publishVector("/test/large_vector", large_data);
    EXPECT_TRUE(result);
}

// 测试多个不同 topic
TEST_F(TopicLoggerTest, MultipleTopics) {
    std::vector<double> vec_data = {1.0, 2.0, 3.0};

    // 创建多个 vector topics
    for (int i = 0; i < 10; ++i) {
        std::string topic_name = "/test/multi_vec_" + std::to_string(i);
        EXPECT_TRUE(logger_->publishVector(topic_name, vec_data));
    }
    EXPECT_EQ(logger_->vectorTopicCount(), 10);

    // 创建多个 value topics
    for (int i = 0; i < 5; ++i) {
        std::string topic_name = "/test/multi_val_" + std::to_string(i);
        EXPECT_TRUE(logger_->publishValue(topic_name, static_cast<double>(i)));
    }
    EXPECT_EQ(logger_->valueTopicCount(), 5);
}

// 测试数据接收（端到端测试）
TEST_F(TopicLoggerTest, EndToEndVector) {
    const std::string topic_name = "/test/e2e_vector";
    std::vector<double> sent_data = {1.1, 2.2, 3.3, 4.4};
    std::vector<double> received_data;
    std::atomic<bool> data_received{false};

    // 创建订阅者（使用独立的 participant）
    dds_common::TopicSubscriber<msgs::Float64Array> subscriber(
        subscriber_participant_,
        topic_name,
        [&](const msgs::Float64Array& msg) {
            received_data = msg.data();
            data_received = true;
        }
    );

    // 等待订阅者就绪
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // 发布数据
    EXPECT_TRUE(logger_->publishVector(topic_name, sent_data));

    // 等待接收
    auto start = std::chrono::steady_clock::now();
    while (!data_received &&
           std::chrono::steady_clock::now() - start < std::chrono::seconds(2)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    EXPECT_TRUE(data_received);
    EXPECT_EQ(received_data.size(), sent_data.size());
    for (size_t i = 0; i < sent_data.size(); ++i) {
        EXPECT_DOUBLE_EQ(received_data[i], sent_data[i]);
    }
}

// 测试数据接收（标量端到端测试）
TEST_F(TopicLoggerTest, EndToEndValue) {
    const std::string topic_name = "/test/e2e_value";
    double sent_value = 123.456;
    double received_value = 0.0;
    std::atomic<bool> data_received{false};

    // 创建订阅者（使用独立的 participant）
    dds_common::TopicSubscriber<msgs::Float64> subscriber(
        subscriber_participant_,
        topic_name,
        [&](const msgs::Float64& msg) {
            received_value = msg.data();
            data_received = true;
        }
    );

    // 等待订阅者就绪
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // 发布数据
    EXPECT_TRUE(logger_->publishValue(topic_name, sent_value));

    // 等待接收
    auto start = std::chrono::steady_clock::now();
    while (!data_received &&
           std::chrono::steady_clock::now() - start < std::chrono::seconds(2)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    EXPECT_TRUE(data_received);
    EXPECT_DOUBLE_EQ(received_value, sent_value);
}

// 测试多线程并发发布
TEST_F(TopicLoggerTest, ConcurrentPublish) {
    const int num_threads = 4;
    const int publishes_per_thread = 100;
    std::vector<std::thread> threads;
    std::atomic<int> success_count{0};

    for (int t = 0; t < num_threads; ++t) {
        threads.emplace_back([&, t]() {
            std::vector<double> data = {static_cast<double>(t), 1.0, 2.0};
            for (int i = 0; i < publishes_per_thread; ++i) {
                // 每个线程发布到自己的 topic 和共享 topic
                std::string own_topic = "/test/thread_" + std::to_string(t);
                std::string shared_topic = "/test/shared";

                if (logger_->publishVector(own_topic, data)) {
                    success_count++;
                }
                if (logger_->publishValue(shared_topic, static_cast<double>(i))) {
                    success_count++;
                }
            }
        });
    }

    for (auto& t : threads) {
        t.join();
    }

    // 所有发布都应该成功
    EXPECT_EQ(success_count.load(), num_threads * publishes_per_thread * 2);

    // 应该有 num_threads 个独立 vector topic + 1 个共享 value topic
    EXPECT_EQ(logger_->vectorTopicCount(), num_threads);
    EXPECT_EQ(logger_->valueTopicCount(), 1);
}

// 测试时间戳
TEST_F(TopicLoggerTest, TimestampValid) {
    const std::string topic_name = "/test/timestamp";
    int32_t received_sec = 0;
    uint32_t received_nanosec = 0;
    std::atomic<bool> data_received{false};

    // 记录发送前的时间
    auto before = std::chrono::system_clock::now();

    // 创建订阅者（使用独立的 participant）
    dds_common::TopicSubscriber<msgs::Float64> subscriber(
        subscriber_participant_,
        topic_name,
        [&](const msgs::Float64& msg) {
            received_sec = msg.header_sec();
            received_nanosec = msg.header_nanosec();
            data_received = true;
        }
    );

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // 发布数据
    EXPECT_TRUE(logger_->publishValue(topic_name, 1.0));

    // 记录发送后的时间
    auto after = std::chrono::system_clock::now();

    // 等待接收
    auto start = std::chrono::steady_clock::now();
    while (!data_received &&
           std::chrono::steady_clock::now() - start < std::chrono::seconds(2)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    EXPECT_TRUE(data_received);

    // 验证时间戳在合理范围内
    auto before_sec = std::chrono::duration_cast<std::chrono::seconds>(
        before.time_since_epoch()).count();
    auto after_sec = std::chrono::duration_cast<std::chrono::seconds>(
        after.time_since_epoch()).count();

    EXPECT_GE(received_sec, before_sec);
    EXPECT_LE(received_sec, after_sec + 1);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
