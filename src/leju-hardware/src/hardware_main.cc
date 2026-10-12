#include <iostream>
#include <memory>
#include <cstring>
#include <signal.h>
#include "hw_log.h"
#include "lejusdk-utils/leju_log.h"
#include "hardware_node.h"

namespace {
    leju::HardwareNode* g_hardware_node = nullptr;

    void signalHandler(int signal) {
        switch (signal) {
            case SIGINT:
                std::cout << "\n\033[33m[HardwareNode] Received Ctrl+C (SIGINT), shutting down...\033[0m" << std::endl;
                break;
            case SIGTERM:
                std::cout << "\n\033[33m[HardwareNode] Received termination signal, shutting down...\033[0m" << std::endl;
                break;
            default:
                std::cout << "\n\033[33m[HardwareNode] Received signal " << signal << ", shutting down...\033[0m" << std::endl;
                break;
        }

        if (g_hardware_node) {
            g_hardware_node->setStopReceived();
        }
    }
}

int main(int argc, char** argv)
{
    LOG_BANNER("Hardware Node Start");

    // Check if config directory argument is provided
    if (argc < 2) {
        std::cerr << "\033[31mUsage: " << argv[0] << " <config_dir>\033[0m" << std::endl;
        std::cerr << "Example: " << argv[0] << " ~/lejulab_platform/src/leju-hardware/config" << std::endl;
        return -1;
    }

    std::string config_dir(argv[1]);
    bool bypass_ankle_solver = false;
    for (int i = 2; i < argc; ++i) {
        if (strcmp(argv[i], "--bypass-ankle-solver") == 0) {
            bypass_ankle_solver = true;
        }
    }
    std::cout << "\033[32m[HardwareNode] Using config directory: " << config_dir << "\033[0m" << std::endl;

    // Setup signal handlers
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    leju::HardwareNode hardware_node(config_dir, bypass_ankle_solver);
    g_hardware_node = &hardware_node;

    if (!hardware_node.init()) {
        HWLOG_FAILURE("Hardware Node Init Failed.... exit");
        hardware_node.close();
        return -1;
    }

    std::cout << "\033[32m[HardwareNode] Node initialized successfully" << std::endl;
    std::cout << "[HardwareNode] Press Ctrl+C to stop\033[0m" << std::endl;

    // keep main thread alive
    // quit:
    //  - publish msg to DDS topic `leju:dds_topics:kHardwareStop`
    //  - or hardware_node.close()
    //  - or Ctrl+C (handled by signal handler)
    hardware_node.run();

    hardware_node.close();
    g_hardware_node = nullptr;

    LOG_BANNER("Hardware Node Exit");
    
    return 0;
}