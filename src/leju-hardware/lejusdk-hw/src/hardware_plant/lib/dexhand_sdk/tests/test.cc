#include "dexhand_base.h"
#include "revo2_hand.h"
#include "revo1_hand.h"
#include "touch_dexhand.h"
#include "stark_dexhand.h"

// 定义宏来排除 revo2_can_test.cpp 中的 main 函数
#define REVO2_CAN_TEST_NO_MAIN
#include "revo2_can_test.cpp"
#define REVO1_CAN_TEST_NO_MAIN
#include "revo1_can_test.cpp"

#include <iostream>
#include <sys/stat.h>
#include <chrono>
#include <thread>
#include <array>
#include <vector>
#include <string>
#include <dirent.h>
#include <algorithm>
#include <cerrno>   
#include <cstring>
#include <map>

using namespace dexhand;
using namespace canbus_sdk;

enum class TestDexHandType {
    Revo1Normal = 0,
    Revo1Touch = 1,
    Revo2 = 2,
    Revo1Can = 3,
    Revo2Can = 4
};

std::unique_ptr<DexHandBase> left_hand = nullptr;
std::unique_ptr<DexHandBase> right_hand = nullptr;
bool running = true;
//////////////////// Global ////////////////////

void tips_check_hwconnect(TestDexHandType hand_type) {
    std::cout << "\033[32m\nTips: 以下命令可用于帮助：\n";
    std::cout << "     0. 请检查硬件设备是否已连接 \n";
    std::cout << "     1. 检查设备: ls -lh /dev/ttyUSB* \n";
    if(hand_type == TestDexHandType::Revo1Touch) {
        std::cout << "     2. 检查规则: ls -lh /dev/stark_serial_touch_*  或 ls /etc/udev/rules.d/stark_serial_touch_* \n";
    }
    else {
        std::cout << "     2. 检查规则: ls -lh /dev/stark_serial_*  或 ls /etc/udev/rules.d/stark_serial_* \n";
    }
    std::cout << "        2.1 规则创建脚本: tools/check_tool/gen_dexhand_serial_rules.sh \n";
    std::cout << "     3. 使用上位机连接灵巧手, 查看设备ID是否正确设置 \n";
    std::cout << "\033[0m\n";
}

#include <cerrno>   // 需要包含 errno 的定义
#include <cstring>  // 需要包含 strerror 函数

bool check_exist_ttyUSB_rules(int hand_side, TestDexHandType hand_type) {

    std::array<bool, 2> hand_side_flag = {true, true};
    
    if (hand_side == 0 || hand_side == 2) {
        // 检查左手
        const char* path = "/dev/stark_serial_L";
        if(hand_type == TestDexHandType::Revo1Touch) {
            path = "/dev/stark_serial_touch_L";
        }
        struct stat sb; // 需保留结构体作用域
        int ret = stat(path, &sb);        
        if (ret != 0) {
            int saved_errno = errno; // 立即保存错误码
            std::cout << "\033[33mError: 左灵巧手检测失败 (" << path 
                      << ") | 错误码: " << saved_errno 
                      << " | 错误信息: " << strerror(saved_errno) 
                      << "\033[0m\n";
            tips_check_hwconnect(hand_type);
            hand_side_flag[0] = false;
        }
    }

    if (hand_side == 1 || hand_side == 2) {
        // 检查右手
        const char* path = "/dev/stark_serial_R";
        if(hand_type == TestDexHandType::Revo1Touch) {
            path = "/dev/stark_serial_touch_R";
        }
        struct stat sb; // 需保留结构体作用域
        int ret = stat(path, &sb);        
        if (ret != 0) {
            int saved_errno = errno; // 立即保存错误码
            std::cout << "\033[33mError: 右灵巧手检测失败 (" << path 
                      << ") | 错误码: " << saved_errno 
                      << " | 错误信息: " << strerror(saved_errno) 
                      << "\033[0m\n";
            tips_check_hwconnect(hand_type);
            hand_side_flag[1] = false;
        }
    }

    return std::any_of(hand_side_flag.begin(), hand_side_flag.end(), [](bool x) { return x; });
}

// 识别灵巧手设备的ttyUSB
void discovery_dexhand_ttyUSB(TestDexHandType hand_type) {
    std::cout << "**************************************\n";
    std::cout << "*     Leju 灵巧手扫描识别工具           *\n";
    std::cout << "**************************************\n";

    if(hand_type == TestDexHandType::Revo1Can) {
        std::cout << "\033[33mError: Revo1Can 协议暂不支持扫描功能\033[0m\n";
        return;
    }

    if(hand_type == TestDexHandType::Revo2Can) {
        std::cout << "\033[33mError: Revo2Can 协议暂不支持扫描功能\033[0m\n";
        return;
    }


    const std::string path = "/dev/";
    const std::string prefix = "ttyUSB";
    struct dirent *entry;
    DIR *dp = opendir(path.c_str());
    if (dp == nullptr) {
        std::cerr << "Error: Could not open /dev directory." << std::endl;
        return;
    }

    std::cout << "当前所有的 ttyUSB 设备:" << std::endl;
    std::vector<std::string> devs;
    while ((entry = readdir(dp))) {
        std::string filename(entry->d_name);
        if (filename.find(prefix) == 0) {
            devs.push_back(filename);
            std::cout << "\033[32m" << filename << "\033[0m\n";
        }
    }
    closedir(dp);

    std::array<std::string, 2> dexhand_ttyusb {"not found", "not found"};
    std::array<bool, 2> found {false, false};
    for (auto &dev : devs) {
        if(found[0] && found[1]) break;
        std::string port = path + dev;
        std::cout << "\n----------------------------------\n";
        std::cout << "\033[32m尝试控制设备: " << port << " 运动, 请观察左手或者右手是否运动，如果运动则该手为 " << port << "\033[0m\n";

        std::unique_ptr<DexHandBase> hand = nullptr;

        if(hand_type == TestDexHandType::Revo1Touch) {
            hand = TouchDexhand::Connect(port, 1, 115200);
        }
        else if (hand_type == TestDexHandType::Revo2) {
            hand = Revo2Dexhand::Connect(port, 0x7e, 460800);
            if(hand == nullptr) {
                hand = Revo2Dexhand::Connect(port, 0x7f, 460800);
            }
        }
        else {
            hand = StarkDexhand::Connect(port);
        }

        if(hand == nullptr) {
            std::cout << "\033[33m连接失败，跳过该设备\033[0m\n";
            continue;
        }
        
        DeviceInfo_t dev_info;
        bool info_success = hand->getDeviceInfo(dev_info);
        
        if(!info_success) {
            std::cout << "\033[33m无法获取设备信息，跳过该设备\033[0m\n";
            hand = nullptr;
            continue;
        }

        // 根据设备类型判断左右手
        if(dev_info.sku_type == dexhand::DexHandType::SKU_TYPE_SMALL_RIGHT
        || dev_info.sku_type == dexhand::DexHandType::SKU_TYPE_MEDIUM_RIGHT) {
            dexhand_ttyusb[1] = port;    
            std::cout << "\033[32m识别到右手: " << port << "\033[0m\n";
            std::cout << "设备信息: " << dev_info << "\n";
            found[1] = true;
        }
        else if(dev_info.sku_type == dexhand::DexHandType::SKU_TYPE_SMALL_LEFT
        || dev_info.sku_type == dexhand::DexHandType::SKU_TYPE_MEDIUM_LEFT) {
            dexhand_ttyusb[0] = port;    
            std::cout << "\033[32m识别到左手: " << port << "\033[0m\n";
            std::cout << "设备信息: " << dev_info << "\n";
            found[0] = true;
        }
        else {
            std::cout << "\033[33m未识别的设备类型 " << port << " 跳过\033[0m\n";
            hand = nullptr;
            continue;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(20));

        std::cout << "控制 ---> 打开\n";
        hand->setFingerPositions(kOpenFingerPositions);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        
        std::cout << "控制 ---> 关闭\n";
        hand->setFingerPositions(kCloseFingerPositions);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        
        std::cout << "控制 ---> 打开\n";
        hand->setFingerPositions(kOpenFingerPositions);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        // 释放连接
        hand = nullptr;
    }

    std::cout << "\033[33m";
    std::cout << "**************************************\n";
    std::cout << "扫描结果:\n";
    std::cout << "左灵巧手:" << dexhand_ttyusb[0] << "\n";
    std::cout << "右灵巧手:" << dexhand_ttyusb[1] << "\n";
    std::cout << "**************************************\n";
    std::cout << "\033[0m";
}

void print_usage(std::string arg0) {
    std::cout << "Usage: " << arg0 << " --touch|--normal|--revo2|--revo2can [--test [round]] [--scan]\n";
    std::cout << "Options:\n";
    std::cout << "  --touch: Kuavo Revo1 触觉手测试模式\n";
    std::cout << "  --normal: Kuavo Revo1 普通手测试模式\n";
    std::cout << "  --revo2: Roban2 Revo2 普通灵巧手测试模式\n";
    std::cout << "  --revo1can: Roban2 Revo1 Can协议灵巧手测试模式\n";
    std::cout << "  --revo2can: Roban2 Revo2 Can协议灵巧手测试模式\n";
    std::cout << "  --test [round]: 测试灵巧手运动, round 为测试次数，默认为 5 次\n";
    std::cout << "  --scan: 扫描设备(revo2can不支持), 识别 ttyUSB 设备, 并尝试控制设备运动\n";
    std::cout <<"例如:\n";
    std::cout << "Kuavo Revo1 触觉手扫描: " << arg0 << " --touch --scan \n";
    std::cout << "Kuavo Revo1 触觉手测试: " << arg0 << " --touch --test \n";
    std::cout << "Kuavo Revo1 普通手扫描: " << arg0 << " --normal --scan \n";
    std::cout << "Kuavo Revo1 普通手测试: " << arg0 << " --normal --test \n";
    std::cout << "Roban2 Revo2 普通手扫描: " << arg0 << " --revo2 --scan \n";
    std::cout << "Roban2 Revo2 普通手测试: " << arg0 << " --revo2 --test \n";
    std::cout << "Roban2 Revo1 Can协议灵巧手测试: " << arg0 << " --revo1can --test \n";
    std::cout << "Roban2 Revo2 Can协议灵巧手测试: " << arg0 << " --revo2can --test \n";
}

int main(int argc, char **argv) {
    if (argc < 3) {
        print_usage(argv[0]);
        return -1;
    }

    // 使用 map 映射参数到对应的枚举
    std::map<std::string, TestDexHandType> hand_config_map = {
        {"--touch", TestDexHandType::Revo1Touch},
        {"--normal", TestDexHandType::Revo1Normal},
        {"--revo2", TestDexHandType::Revo2},
        {"--revo1can", TestDexHandType::Revo1Can},
        {"--revo2can", TestDexHandType::Revo2Can}
    };

    auto it = hand_config_map.find(std::string(argv[1]));
    if (it == hand_config_map.end()) {
        print_usage(argv[0]);
        return -1;
    }

    // 从 map 中获取枚举类型
    TestDexHandType dexhand_type = it->second;

    // 扫描模式
    if (argc == 3 && std::string(argv[2]) == "--scan") {
        discovery_dexhand_ttyUSB(dexhand_type);
        return 0;
    }

    // 测试模式
    int round = 5;
    if (std::string(argv[2]) != "--test") {
        print_usage(argv[0]);
        return 0;
    }

    // round
    if (argc == 4) {
        round = std::max(std::stoi(argv[3]), 1);
    }

    std::cout << "**************************************\n";
    if(dexhand_type == TestDexHandType::Revo1Touch) {
        std::cout << "* Leju Kuavo Revo1触觉手测试工具*\n";
    }
    else if (dexhand_type == TestDexHandType::Revo2) {
        std::cout << "* Leju Roban2 Revo2 手测试工具*\n";
    }
    else if (dexhand_type == TestDexHandType::Revo1Can) {
        std::cout << "* Leju Kuavo Revo1 手测试工具*\n";
        std::cout << "**************************************\n";

        Revo1CanTest revo1_can_test;
        revo1_can_test.run_test();

        return 0;
    }
    else if (dexhand_type == TestDexHandType::Revo2Can) {
        std::cout << "* Leju Roban2 Revo2 手测试工具*\n";
        std::cout << "**************************************\n";

        // 使用 Revo2CanTest 类进行测试
        Revo2CanTest revo2_can_test;
        revo2_can_test.run_test();

        return 0;
    }
    else {
        std::cout << "* Leju Kuavo Revo1 普通手测试工具*\n";
    }
    std::cout << "**************************************\n";
    
    // scan ttyUSB
    if(!check_exist_ttyUSB_rules(2, dexhand_type)) {
        std::cout << "\033[33mError: 未发现灵巧手设备, 请检查设备是否已连接!\033[0m\n";
        return -1;
    }

    if(dexhand_type == TestDexHandType::Revo1Touch) {
        left_hand = TouchDexhand::Connect("/dev/stark_serial_touch_L", 1, 115200);
        right_hand = TouchDexhand::Connect("/dev/stark_serial_touch_R", 1, 115200);
    }
    else if (dexhand_type == TestDexHandType::Revo2) {
        left_hand = Revo2Dexhand::Connect("/dev/stark_serial_L", 0x7e, 460800);
        right_hand = Revo2Dexhand::Connect("/dev/stark_serial_R", 0x7f, 460800);
    }
    else {
        left_hand = StarkDexhand::Connect("/dev/stark_serial_L");
        right_hand = StarkDexhand::Connect("/dev/stark_serial_R");
    }

    if(left_hand == nullptr && right_hand == nullptr) {
        return -1;
    }

    // print haraware fw info
    if (left_hand != nullptr) {
        DeviceInfo_t l_dev_info;
        bool left_info_result = left_hand->getDeviceInfo(l_dev_info);
        if(!left_info_result || l_dev_info.sku_type == SKU_TYPE_NONE) {
            std::cout << "\033[33mError: 获取 <左灵巧手> 设备信息失败或超时，如果灵巧手可动，请忽略!\033[0m\n";
        }
        else {
            std::cout << "<左灵巧手>固件信息：" << l_dev_info;
            std::cout << "\033[32mSuccess: <左灵巧手> 已连接! \033[0m\n";
            if(dexhand_type == TestDexHandType::Revo1Touch) {
                if(auto hand = dynamic_cast<TouchDexhand*>(left_hand.get())) {
                    hand->enableTouchSensor(0xFF);
                }
            }
            
        }
    }
    if (right_hand != nullptr) {
        DeviceInfo_t r_dev_info;
        bool right_info_result = right_hand->getDeviceInfo(r_dev_info);
        if (!right_info_result || r_dev_info.sku_type == SKU_TYPE_NONE) {
            std::cout << "\033[33mError: 获取 <右灵巧手> 设备信息失败或超时，如果灵巧手可动，请忽略!\033[0m\n";
        }
        else {
            std::cout << "<右灵巧手>固件信息：" << r_dev_info;
            std::cout << "\033[32mSuccess: <右灵巧手> 已连接! \033[0m\n";
            if(dexhand_type == TestDexHandType::Revo1Touch) {
                if(auto hand = dynamic_cast<TouchDexhand*>(right_hand.get())) {
                    hand->enableTouchSensor(0xFF);
                }
            }
        }
    }

    auto controlHand = [](std::unique_ptr<dexhand::DexHandBase> &hand, const std::string& hand_name) {
        // 打印绿色标题
        std::cout << "\n\033[32m*****************************\033[0m\n";    
        std::cout << "\033[32m* 现在控制" << hand_name << "运动\033[0m\n";
        std::cout << "\033[32m*****************************\033[0m\n";
        
        // 运动控制逻辑
        int count = 3;
        while(count > 0) {
            if (hand != nullptr) {
                hand->setFingerPositions(kOpenFingerPositions);
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                hand->setFingerPositions(kCloseFingerPositions);
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            }
            count --;
        }
    };

    controlHand(left_hand, "左手");
    controlHand(right_hand, "右手");

    // 2. 位置控制
    std::cout << "\n\033[32m*****************************\033[0m\n";    
    std::cout << "\033[32m* 现在开始位置控制测试(先左手后右手): " << round << " 轮 \033[0m\n";
    std::cout << "\033[32m*****************************\033[0m\n";    

    auto position_round_test = [&](std::unique_ptr<DexHandBase> &hand) {
        UnsignedFingerArray finger_positions = {0, 0, 0, 0, 0, 0};
        int j = 1;
        while (j < 6 && running) {
            finger_positions[j] = 90;
            if(hand) {
                hand->setFingerPositions(finger_positions);
                std::this_thread::sleep_for(std::chrono::milliseconds(600));
            }
            j++;
        }

        while (j >= 1 && running) {
            finger_positions[j] = 0;
            if(hand) {
                hand->setFingerPositions(finger_positions);
                std::this_thread::sleep_for(std::chrono::milliseconds(600));
            }
            j--;
        }
    };
    
    int i = 0;
    while (i<round && running) {
        position_round_test(left_hand);
        position_round_test(right_hand);   
        i++;
    }
    
    // 3. 速度控制测试
    std::cout << "\n\033[32m*****************************\033[0m\n";    
    std::cout << "\033[32m* 速度控制测试: " << round << " 轮 \033[0m\n";
    std::cout << "\033[32m*****************************\033[0m\n";    

    auto speed_round_test = [&](std::unique_ptr<DexHandBase> &hand, const std::string& hand_name, bool open_option) {
        FingerArray finger_vels = {30, 10, 30, 30, 30, 30};
        if (open_option) {
            finger_vels = {30, 10, 30, 30, 30, 30};
        } else {
            finger_vels = {-30, -10, -30, -30, -30, -30};
        }

        if(hand) {
            hand->setFingerSpeeds(finger_vels);
            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        }
    };

    i = 0;
    FingerArray finger_vels = {30, 10, 30, 30, 30, 30};
    while (i < round && running) {
        speed_round_test(left_hand, "左手", false);
        speed_round_test(left_hand, "左手", true);
        speed_round_test(right_hand, "右手", false);
        speed_round_test(right_hand, "右手", true);
        i++;
    }

    if (left_hand != nullptr) 
        left_hand->setFingerPositions(kOpenFingerPositions);
    if (right_hand != nullptr)
        right_hand->setFingerPositions(kOpenFingerPositions);

    // 4. 触觉传感器测试
    if(dexhand_type == TestDexHandType::Revo1Touch) {
        if(left_hand) {
            std::cout << "\n\033[32m左手触觉传感器测试:\033[0m\n";
            std::cout << "Tips: 请用手靠近食指，并按压表面观测一下传感数据变化.\n";
            int j = 0;
            while(j < 100 && running) {
                if(auto hand = dynamic_cast<TouchDexhand*>(left_hand.get())) {
                    auto touch_status = hand->getTouchStatus();
                    std::cout << "Touch Status: " << touch_status.at(1) << std::endl;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                j++;
            }
        }
        if(right_hand){
            std::cout << "\n\033[32m右手触觉传感器测试:\033[0m\n";
            std::cout << "Tips: 请用手靠近食指，并按压表面观测一下传感数据变化.\n";
            int j = 0;
            while(j < 100 && running) {
                if(auto hand = dynamic_cast<TouchDexhand*>(right_hand.get())) {
                    auto touch_status = hand->getTouchStatus();
                    std::cout << "Touch Status: " << touch_status.at(1) << std::endl;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                j++;
            }
        }
    }

    right_hand = nullptr;
    left_hand = nullptr;
    return 0;
}