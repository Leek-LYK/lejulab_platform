#include "stark_dexhand.h"
#include <iostream>
#include <thread>

using namespace dexhand;

int main() {
    std::unique_ptr<DexHandBase> right_hand = StarkDexhand::Connect("/dev/stark_serial_R");
    std::unique_ptr<DexHandBase> left_hand = StarkDexhand::Connect("/dev/stark_serial_L");

    if (right_hand == nullptr || left_hand == nullptr) {
        printf("Error: Failed to connect to hands\n");
        // return -1;
    }

    DeviceInfo_t right_dev_info, left_dev_info;
    if (right_hand) {
        right_hand->getDeviceInfo(right_dev_info);
        std::cout << "right_dev_info: " << right_dev_info << std::endl;
    }
    if (left_hand) {
        left_hand->getDeviceInfo(left_dev_info);
        std::cout << "left_dev_info: " << left_dev_info << std::endl;
    }

    if (right_hand) {
        auto force_level = right_hand->getGripForce();
        std::cout << "right_hand force_level: " << static_cast<int>(force_level) << std::endl;
        right_hand->setGripForce(dexhand::GripForce::FORCE_LEVEL_NORMAL);
    }
    if (left_hand) {
        auto force_level = left_hand->getGripForce();
        std::cout << "left_hand force_level: " << static_cast<int>(force_level) << std::endl;
    }

    // Create thread to continuously get and print finger positions
    bool running{true};
    std::thread status_thread([&]() {
        while(running) {
            dexhand::FingerStatus right_status, left_status;
            if (right_hand) {
                right_hand->getFingerStatus(right_status);
                std::cout << "right_status: " << right_status << std::endl;
            }
            if (left_hand) {
                left_hand->getFingerStatus(left_status);
                std::cout << "left_status: " << left_status << std::endl;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    });

    UnsignedFingerArray positions = {50, 50, 50, 50, 50, 50};
    positions = {0, 0, 0, 0, 0, 0};
    if (right_hand) right_hand->setFingerPositions(positions);
    if (left_hand) left_hand->setFingerPositions(positions);
    std::this_thread::sleep_for(std::chrono::seconds(3));
    positions = {50, 50, 50, 50, 50, 50};
    if (right_hand) right_hand->setFingerPositions(positions);
    if (left_hand) left_hand->setFingerPositions(positions);
    std::this_thread::sleep_for(std::chrono::seconds(3));

    if (left_hand) left_hand->setFingerSpeeds({20, 20, 20, 20, 20, 20});
    if (right_hand) right_hand->setFingerSpeeds({20, 20, 20, 20, 20, 20});
    std::this_thread::sleep_for(std::chrono::seconds(3));
    if (left_hand) left_hand->setFingerSpeeds({-20, -20, -20, -20, -20, -20});
    if (right_hand) right_hand->setFingerSpeeds({-20, -20, -20, -20, -20, -20});
    running = false;
    status_thread.join();

    // right_hand->runActionSequence(ActionSequenceId_t::ACTION_SEQUENCE_ID_DEFAULT_GESTURE_PINCH_THREE);
    // std::this_thread::sleep_for(std::chrono::seconds(3));
    return 0;
}