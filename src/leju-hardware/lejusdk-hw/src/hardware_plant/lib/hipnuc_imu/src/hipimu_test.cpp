#include <iostream>
#include <thread>   // 添加这个头文件
#include <chrono>   // 添加这个头文件
#include "hipnuc_imu_receiver.h"

using namespace HIPNUC_IMU;
int main(int argc, char** argv)
{
    // 初始化 IMU
    if (imu_init() != 0) {
        std::cout << "Failed to initialize IMU." << std::endl;
        return -1;
    }

    std::cout << "IMU initialized successfully." << std::endl;

    Eigen::Vector3d acc;
    Eigen::Vector3d gyro;
    Eigen::Quaterniond quat;

    // 循环获取 IMU 数据
    for (int i = 0; i < 10; ++i) { // 获取10次数据
        if (getImuDataFrame(acc, gyro, quat)) {
            std::cout << "IMU Data Frame:" << std::endl;
            std::cout << "Acceleration: x = " << acc.x() << ", y = " << acc.y() << ", z = " << acc.z() << std::endl;
            std::cout << "Gyroscope: x = " << gyro.x() << ", y = " << gyro.y() << ", z = " << gyro.z() << std::endl;
            std::cout << "Quaternion: w = " << quat.w() << ", x = " << quat.x() << ", y = " << quat.y() << ", z = " << quat.z() << std::endl;
        } else {
            std::cout << "Failed to get IMU data frame." << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1)); // 每毫秒获取一次数据
    }

    // 停止 IMU
    imu_stop();
    std::cout << "IMU stopped." << std::endl;

    return 0;
}
