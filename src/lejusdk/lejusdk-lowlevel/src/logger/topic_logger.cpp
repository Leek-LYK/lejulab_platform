#include "lejusdk-lowlevel/topic_logger.h"
#include "lejusdk-utils/time_utils.hpp"

#include <unordered_map>
#include <shared_mutex>
#include <iostream>

#include <dds/dds.hpp>
#include <lejusdk-dds-idl/Float64Types.hpp>
#include <lejusdk-topic-pubsub/topic_publisher.hpp>

namespace leju {

namespace {
// 传感器/日志数据使用 BEST_EFFORT QoS，避免 write 阻塞等待 ACK
dds::pub::qos::DataWriterQos makeBestEffortQos() {
    dds::pub::qos::DataWriterQos qos;
    qos << dds::core::policy::Reliability::BestEffort();
    return qos;
}
} // anonymous namespace

// PIMPL 实现类
class TopicLogger::Impl {
public:
    // 类型别名
    using VectorPublisher = dds_common::TopicPublisher<msgs::Float64Array>;
    using ValuePublisher = dds_common::TopicPublisher<msgs::Float64>;
    using VectorPublisherMap = std::unordered_map<std::string, std::unique_ptr<VectorPublisher>>;
    using ValuePublisherMap = std::unordered_map<std::string, std::unique_ptr<ValuePublisher>>;

    explicit Impl(int domain_id)
        : participant_(domain_id)
    {
    }

    bool publishVector(const std::string& topic_name, const std::vector<double>& data)
    {
        auto* publisher = getOrCreateVectorPublisher(topic_name);
        if (publisher == nullptr) {
            return false;
        }

        int32_t sec;
        uint32_t nanosec;
        common::GetUnixTimestampSecNsec(sec, nanosec);
        msgs::Float64Array msg(sec, nanosec, data);
        return publisher->publish(msg);
    }

    bool publishValue(const std::string& topic_name, double value)
    {
        auto* publisher = getOrCreateValuePublisher(topic_name);
        if (publisher == nullptr) {
            return false;
        }

        int32_t sec;
        uint32_t nanosec;
        common::GetUnixTimestampSecNsec(sec, nanosec);
        msgs::Float64 msg(sec, nanosec, value);
        return publisher->publish(msg);
    }

    size_t vectorTopicCount() const
    {
        std::shared_lock<std::shared_mutex> lock(vector_mutex_);
        return vector_publishers_.size();
    }

    size_t valueTopicCount() const
    {
        std::shared_lock<std::shared_mutex> lock(value_mutex_);
        return value_publishers_.size();
    }

private:
    VectorPublisher* getOrCreateVectorPublisher(const std::string& topic_name)
    {
        // 1. 先用共享锁查找（多线程可并发读）
        {
            std::shared_lock<std::shared_mutex> lock(vector_mutex_);
            auto it = vector_publishers_.find(topic_name);
            if (it != vector_publishers_.end()) {
                return it->second.get();
            }
        }

        // 2. 不存在则用独占锁创建
        {
            std::unique_lock<std::shared_mutex> lock(vector_mutex_);
            // 再次检查（避免重复创建）
            auto it = vector_publishers_.find(topic_name);
            if (it != vector_publishers_.end()) {
                return it->second.get();
            }

            try {
                auto new_pub = std::make_unique<VectorPublisher>(participant_, topic_name, makeBestEffortQos());

                if (!new_pub->is_valid()) {
                    std::cerr << "TopicLogger: Failed to create vector publisher for topic: "
                              << topic_name << std::endl;
                    return nullptr;
                }

                auto* ptr = new_pub.get();
                vector_publishers_[topic_name] = std::move(new_pub);
                return ptr;
            } catch (const std::exception& e) {
                std::cerr << "TopicLogger: Exception creating vector publisher for topic: "
                          << topic_name << " - " << e.what() << std::endl;
                return nullptr;
            }
        }
    }

    ValuePublisher* getOrCreateValuePublisher(const std::string& topic_name)
    {
        // 1. 先用共享锁查找（多线程可并发读）
        {
            std::shared_lock<std::shared_mutex> lock(value_mutex_);
            auto it = value_publishers_.find(topic_name);
            if (it != value_publishers_.end()) {
                return it->second.get();
            }
        }

        // 2. 不存在则用独占锁创建
        {
            std::unique_lock<std::shared_mutex> lock(value_mutex_);
            // 再次检查（避免重复创建）
            auto it = value_publishers_.find(topic_name);
            if (it != value_publishers_.end()) {
                return it->second.get();
            }

            try {
                auto new_pub = std::make_unique<ValuePublisher>(participant_, topic_name, makeBestEffortQos());

                if (!new_pub->is_valid()) {
                    std::cerr << "TopicLogger: Failed to create value publisher for topic: "
                              << topic_name << std::endl;
                    return nullptr;
                }

                auto* ptr = new_pub.get();
                value_publishers_[topic_name] = std::move(new_pub);
                return ptr;
            } catch (const std::exception& e) {
                std::cerr << "TopicLogger: Exception creating value publisher for topic: "
                          << topic_name << " - " << e.what() << std::endl;
                return nullptr;
            }
        }
    }

private:
    dds::domain::DomainParticipant participant_;

    // 向量 Topic Publishers
    VectorPublisherMap vector_publishers_;
    mutable std::shared_mutex vector_mutex_;

    // 标量 Topic Publishers
    ValuePublisherMap value_publishers_;
    mutable std::shared_mutex value_mutex_;
};

// TopicLogger 公共接口实现
TopicLogger::TopicLogger() = default;

TopicLogger::~TopicLogger() = default;

std::unique_ptr<TopicLogger> TopicLogger::create(int domain_id)
{
    auto logger = std::unique_ptr<TopicLogger>(new TopicLogger());
    logger->impl_ = std::make_unique<Impl>(domain_id);
    return logger;
}

bool TopicLogger::publishVector(const std::string& topic_name, const std::vector<double>& data)
{
    return impl_->publishVector(topic_name, data);
}

bool TopicLogger::publishValue(const std::string& topic_name, double value)
{
    return impl_->publishValue(topic_name, value);
}

size_t TopicLogger::vectorTopicCount() const
{
    return impl_->vectorTopicCount();
}

size_t TopicLogger::valueTopicCount() const
{
    return impl_->valueTopicCount();
}

} // namespace leju
