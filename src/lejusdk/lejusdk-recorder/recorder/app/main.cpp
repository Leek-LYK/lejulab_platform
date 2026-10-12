#include "recorder/DdsRecorder.hpp"
#include "recorder/Configuration.hpp"
#include "common/Log.hpp"

#include <optionparser.h>
#include <iostream>
#include <csignal>
#include <atomic>
#include <cstring>

using namespace cyclone_record_replay;

// Global recorder pointer for signal handler
static recorder::DdsRecorder* g_recorder = nullptr;
static std::atomic<bool> g_running{true};

void signal_handler(int signal) {
    std::cout << "\nReceived signal " << signal << ", stopping recording..." << std::endl;
    g_running.store(false);
    if (g_recorder) {
        g_recorder->stop();
    }
}

enum OptionIndex {
    UNKNOWN,
    HELP,
    CONFIG,
    ALL,
    REGEX,
    EXCLUDE,
    OUTPUT,
    OUTPUT_NAME,
    DOMAIN,
    SPLIT,
    MAX_SPLITS,
    SIZE,
    DURATION,
    MIN_SPACE,
    ZSTD,
    LZ4,
    NO_COMPRESS,
    QUIET,
    VERBOSE
};

const option::Descriptor usage[] = {
    {UNKNOWN, 0, "", "", option::Arg::None,
        "Usage: lejusdk_recorder [options] [TOPIC1 TOPIC2 ...]\n\n"
        "Record DDS topics to MCAP file.\n\n"
        "Options:"},
    {HELP, 0, "h", "help", option::Arg::None,
        "  -h, --help            show this help message and exit"},
    {CONFIG, 0, "c", "config", option::Arg::Optional,
        "  -c, --config=FILE     configuration file path (YAML)"},
    {ALL, 0, "a", "all", option::Arg::None,
        "  -a, --all             record all topics"},
    {REGEX, 0, "e", "regex", option::Arg::None,
        "  -e, --regex           match topics using regular expressions"},
    {EXCLUDE, 0, "x", "exclude", option::Arg::Optional,
        "  -x EXCLUDE_REGEX, --exclude=EXCLUDE_REGEX\n"
        "                        exclude topics matching the follow regular expression\n"
        "                        (subtracts from -a or regex)"},
    {QUIET, 0, "q", "quiet", option::Arg::None,
        "  -q, --quiet           suppress console output"},
    {OUTPUT, 0, "o", "output-prefix", option::Arg::Optional,
        "  -o PREFIX, --output-prefix=PREFIX\n"
        "                        prepend PREFIX to beginning of bag name (name will\n"
        "                        always end with date stamp)"},
    {OUTPUT_NAME, 0, "O", "output-name", option::Arg::Optional,
        "  -O NAME, --output-name=NAME\n"
        "                        record to file with name NAME.mcap"},
    {DOMAIN, 0, "d", "domain", option::Arg::Optional,
        "  -d, --domain=ID       DDS domain ID (default: 0)"},
    {SPLIT, 0, "", "split", option::Arg::None,
        "  --split               split the file when maximum size or duration is reached"},
    {MAX_SPLITS, 0, "", "max-splits", option::Arg::Optional,
        "  --max-splits=MAX_SPLITS\n"
        "                        Keep a maximum of N files, when reaching the\n"
        "                        maximum erase the oldest one to keep a constant number\n"
        "                        of files."},
    {SIZE, 0, "", "size", option::Arg::Optional,
        "  --size=SIZE           record a file of maximum size SIZE MB. (Default:\n"
        "                        infinite)"},
    {DURATION, 0, "", "duration", option::Arg::Optional,
        "  --duration=DURATION   record a file of maximum duration DURATION in seconds,\n"
        "                        unless 'm', or 'h' is appended."},
    {MIN_SPACE, 0, "L", "min-space", option::Arg::Optional,
        "  -L MIN_SPACE, --min-space=MIN_SPACE\n"
        "                        Minimum allowed space on recording device (use G,M,k\n"
        "                        multipliers)"},
    {ZSTD, 0, "", "zstd", option::Arg::None,
        "  --zstd                use zstd compression (default)"},
    {LZ4, 0, "", "lz4", option::Arg::None,
        "  --lz4                 use LZ4 compression"},
    {NO_COMPRESS, 0, "", "no-compression", option::Arg::None,
        "  --no-compression      disable compression"},
    {VERBOSE, 0, "v", "verbose", option::Arg::None,
        "  -v, --verbose         enable verbose logging"},
    {0, 0, nullptr, nullptr, nullptr, nullptr}
};

// Parse size string with optional G/M/k suffix, returns bytes
// e.g. "1G" -> 1073741824, "500M" -> 524288000, "512k" -> 524288, "100" -> 100 MB
static uint64_t parse_size(const char* str) {
    char* end = nullptr;
    double val = std::strtod(str, &end);
    if (end && *end) {
        switch (*end) {
            case 'G': case 'g': return static_cast<uint64_t>(val * 1024 * 1024 * 1024);
            case 'M': case 'm': return static_cast<uint64_t>(val * 1024 * 1024);
            case 'k': case 'K': return static_cast<uint64_t>(val * 1024);
            default: break;
        }
    }
    // No suffix: default to MB
    return static_cast<uint64_t>(val * 1024 * 1024);
}

// Convert string log level to RecorderLogLevel
static RecorderLogLevel parse_log_level(const std::string& level) {
    if (level == "trace") return RECORDER_LOG_LEVEL_TRACE;
    if (level == "debug") return RECORDER_LOG_LEVEL_DEBUG;
    if (level == "info") return RECORDER_LOG_LEVEL_INFO;
    if (level == "warning" || level == "warn") return RECORDER_LOG_LEVEL_WARN;
    if (level == "error") return RECORDER_LOG_LEVEL_ERROR;
    if (level == "critical") return RECORDER_LOG_LEVEL_CRITICAL;
    return RECORDER_LOG_LEVEL_INFO; // default
}

// Parse duration string with optional m/h suffix, returns seconds
// e.g. "60" -> 60, "5m" -> 300, "2h" -> 7200
static double parse_duration(const char* str) {
    char* end = nullptr;
    double val = std::strtod(str, &end);
    if (end && *end) {
        switch (*end) {
            case 'm': return val * 60.0;
            case 'h': return val * 3600.0;
            default: break;
        }
    }
    return val; // default: seconds
}

int main(int argc, char* argv[]) {
    // Skip program name for option parsing
    argc -= (argc > 0);
    argv += (argc > 0);

    option::Stats opt_stats(usage, argc, argv);
    std::vector<option::Option> options(opt_stats.options_max);
    std::vector<option::Option> buffer(opt_stats.buffer_max);
    option::Parser parse(usage, argc, argv, options.data(), buffer.data());

    if (parse.error()) {
        return 1;
    }

    if (options[HELP]) {
        option::printUsage(std::cout, usage);
        return 0;
    }

    // Load configuration (YAML as base, CLI overrides)
    recorder::RecorderConfiguration config;

    if (options[CONFIG] && options[CONFIG].arg) {
        try {
            config.load_from_file(options[CONFIG].arg);
            LOG_INFO("Loaded configuration from '{}'", options[CONFIG].arg);
        } catch (const common::ConfigException& e) {
            LOG_ERROR("Failed to load config: {}", e.what());
            return 1;
        }
    }

    // Apply log level from configuration
    recorder_set_log_level(parse_log_level(config.logging_config().level));

    // Collect non-option arguments (topics) from original argv
    // optionparser doesn't expose non-options, so scan original argv
    {
        // Restore original argc/argv (we decremented above)
        int orig_argc = argc + 1;
        char** orig_argv = argv - 1;

        std::vector<std::string> cli_topics;
        for (int i = 1; i < orig_argc; ++i) {
            const char* a = orig_argv[i];
            // Skip options and their arguments
            if (a[0] == '-') {
                // If this option takes an argument, skip next too
                // Options with '=' don't need to skip next
                if (std::strchr(a, '=') == nullptr) {
                    // Check if it's a flag-only option (no arg)
                    bool is_flag = false;
                    const char* flags[] = {"-h", "--help", "-a", "--all", "-e", "--regex",
                        "--split", "--zstd", "--lz4", "--no-compression", "-q", "--quiet",
                        "-v", "--verbose", nullptr};
                    for (const char** f = flags; *f; ++f) {
                        if (std::strcmp(a, *f) == 0) { is_flag = true; break; }
                    }
                    if (!is_flag) { ++i; } // skip argument value
                }
                continue;
            }
            // Skip ROS remapping arguments (__name:=, __log:=, etc.)
            if (std::strncmp(a, "__", 2) == 0 && std::strchr(a, ':') != nullptr) {
                continue;
            }
            cli_topics.push_back(a);
        }

        if (options[ALL]) {
            // --all: clear topic filter to record everything
            common::TopicFilter filter;
            config.set_topic_filter(filter);
        } else if (!cli_topics.empty()) {
            // CLI topics override YAML topics
            common::TopicFilter filter;
            filter.topics = std::move(cli_topics);
            filter.use_regex = !!options[REGEX];
            config.set_topic_filter(filter);
        }

        // -e without CLI topics: enable regex on YAML topics
        if (options[REGEX] && cli_topics.empty() && !options[ALL]) {
            auto filter = config.dds_config().topic_filter;
            filter.use_regex = true;
            config.set_topic_filter(filter);
        }
    }

    // Handle -x exclude
    if (options[EXCLUDE] && options[EXCLUDE].arg) {
        auto filter = config.dds_config().topic_filter;
        filter.exclude_regex = options[EXCLUDE].arg;
        config.set_topic_filter(filter);
    }

    // Override with command line options
    if (options[OUTPUT_NAME] && options[OUTPUT_NAME].arg) {
        // -O: use exact name (without date stamp)
        config.set_output_prefix(options[OUTPUT_NAME].arg);
        config.mutable_output_config().append_date = false;
    } else if (options[OUTPUT] && options[OUTPUT].arg) {
        config.set_output_prefix(options[OUTPUT].arg);
    }

    if (options[DOMAIN] && options[DOMAIN].arg) {
        config.set_domain_id(std::stoi(options[DOMAIN].arg));
    }

    if (options[DURATION] && options[DURATION].arg) {
        config.set_max_duration(parse_duration(options[DURATION].arg));
    }

    if (options[SIZE] && options[SIZE].arg) {
        config.set_max_file_size(static_cast<size_t>(parse_size(options[SIZE].arg)));
    }

    if (options[SPLIT]) {
        config.set_split(true);
    }

    if (options[MAX_SPLITS] && options[MAX_SPLITS].arg) {
        config.set_max_splits(
            static_cast<uint32_t>(std::stoul(options[MAX_SPLITS].arg)));
    }

    if (options[MIN_SPACE] && options[MIN_SPACE].arg) {
        config.set_min_space(parse_size(options[MIN_SPACE].arg));
    }

    if (options[ZSTD]) {
        config.set_compression(true);
        config.set_compression_type("zstd");
    }

    if (options[LZ4]) {
        config.set_compression(true);
        config.set_compression_type("lz4");
    }

    if (options[NO_COMPRESS]) {
        config.set_compression(false);
    }

    // Handle -v verbose flag (override config file log level)
    if (options[VERBOSE]) {
        recorder_set_log_level(RECORDER_LOG_LEVEL_DEBUG);
    }

    // Setup signal handlers
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    // Use a block scope to ensure DdsRecorder is destroyed before exit
    {
        recorder::DdsRecorder dds_recorder(config);
        g_recorder = &dds_recorder;

        if (!dds_recorder.init()) {
            LOG_ERROR("Failed to initialize recorder");
            return 1;
        }

        if (!dds_recorder.start()) {
            LOG_ERROR("Failed to start recording");
            return 1;
        }

        LOG_INFO("Recording to '{}' - Press Ctrl+C to stop", dds_recorder.output_path());

        // Wait for completion or signal
        while (g_running.load() && dds_recorder.is_recording()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        // Ensure recorder is stopped
        if (dds_recorder.is_recording()) {
            dds_recorder.stop();
        }

        // Print statistics
        const auto& stats = dds_recorder.stats();
        std::cout << "\n=== Recording Statistics ===" << std::endl;
        std::cout << "Topics discovered: " << stats.topics_discovered.load() << std::endl;
        std::cout << "Messages received: " << stats.messages_received.load() << std::endl;
        std::cout << "Messages recorded: " << stats.messages_recorded.load() << std::endl;
        std::cout << "Bytes recorded: " << stats.bytes_recorded.load() << std::endl;
        std::cout << "Duration: " << stats.duration_seconds() << " seconds" << std::endl;
        std::cout << "Output file: " << dds_recorder.output_path() << std::endl;

        g_recorder = nullptr;
    }

    return 0;
}
