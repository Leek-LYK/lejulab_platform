#pragma once

#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <memory>
#include <cstring>

// Log levels - kept for backward compatibility
enum RecorderLogLevel {
    RECORDER_LOG_LEVEL_TRACE = 0,
    RECORDER_LOG_LEVEL_DEBUG = 1,
    RECORDER_LOG_LEVEL_INFO = 2,
    RECORDER_LOG_LEVEL_WARN = 3,
    RECORDER_LOG_LEVEL_ERROR = 4,
    RECORDER_LOG_LEVEL_CRITICAL = 5,
    RECORDER_LOG_LEVEL_OFF = 6
};

// Internal function to get the spdlog logger
inline std::shared_ptr<spdlog::logger>& _recorder_logger() {
    static std::shared_ptr<spdlog::logger> logger;
    static bool initialized = false;
    if (!logger) {
        // Create sink with explicit color mode
        auto sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
        sink->set_color_mode(spdlog::color_mode::always);
        logger = std::make_shared<spdlog::logger>("recorder", sink);
    }
    if (!initialized) {
        logger->set_level(spdlog::level::info);
        // %l = lowercase level name, %^/%$ = color range
        logger->set_pattern("%^[%5l]%$ %v");
        initialized = true;
    }
    return logger;
}

// Set log level (backward compatible)
inline void recorder_set_log_level(RecorderLogLevel level) {
    switch (level) {
        case RECORDER_LOG_LEVEL_TRACE:
            _recorder_logger()->set_level(spdlog::level::trace);
            break;
        case RECORDER_LOG_LEVEL_DEBUG:
            _recorder_logger()->set_level(spdlog::level::debug);
            break;
        case RECORDER_LOG_LEVEL_INFO:
            _recorder_logger()->set_level(spdlog::level::info);
            break;
        case RECORDER_LOG_LEVEL_WARN:
            _recorder_logger()->set_level(spdlog::level::warn);
            break;
        case RECORDER_LOG_LEVEL_ERROR:
            _recorder_logger()->set_level(spdlog::level::err);
            break;
        case RECORDER_LOG_LEVEL_CRITICAL:
            _recorder_logger()->set_level(spdlog::level::critical);
            break;
        case RECORDER_LOG_LEVEL_OFF:
            _recorder_logger()->set_level(spdlog::level::off);
            break;
    }
}

// Backward compatible function to get/set log level (used by existing code)
inline RecorderLogLevel& _recorder_log_level() {
    static RecorderLogLevel level = RECORDER_LOG_LEVEL_INFO;
    return level;
}

// Logging macros using spdlog with fmt-style formatting
#define LOG_TRACE(...)    _recorder_logger()->trace(__VA_ARGS__)
#define LOG_DEBUG(...)    _recorder_logger()->debug(__VA_ARGS__)
#define LOG_INFO(...)     _recorder_logger()->info(__VA_ARGS__)
#define LOG_WARN(...)     _recorder_logger()->warn(__VA_ARGS__)
#define LOG_ERROR(...)    _recorder_logger()->error(__VA_ARGS__)
#define LOG_CRITICAL(...) _recorder_logger()->critical(__VA_ARGS__)

// LOG_SUCCESS is mapped to LOG_INFO
#define LOG_SUCCESS(...)  _recorder_logger()->info(__VA_ARGS__)
