
//  Copyright (c) 2003-2021 Xsens Technologies B.V. or subsidiaries worldwide.
//  All rights reserved.
//
//  Redistribution and use in source and binary forms, with or without modification,
//  are permitted provided that the following conditions are met:
//
//  1.	Redistributions of source code must retain the above copyright notice,
//  	this list of conditions, and the following disclaimer.
//
//  2.	Redistributions in binary form must reproduce the above copyright notice,
//  	this list of conditions, and the following disclaimer in the documentation
//  	and/or other materials provided with the distribution.
//
//  3.	Neither the names of the copyright holders nor the names of their contributors
//  	may be used to endorse or promote products derived from this software without
//  	specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
//  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
//  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
//  THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
//  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
//  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
//  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.THE LAWS OF THE NETHERLANDS
//  SHALL BE EXCLUSIVELY APPLICABLE AND ANY DISPUTES SHALL BE FINALLY SETTLED UNDER THE RULES
//  OF ARBITRATION OF THE INTERNATIONAL CHAMBER OF COMMERCE IN THE HAGUE BY ONE OR MORE
//  ARBITRATORS APPOINTED IN ACCORDANCE WITH SAID RULES.
//
// #include "xdainterface.h"
#include "imu_receiver.h"

#include <iostream>
#include <stdexcept>
#include <string>
#include <pthread.h>
#include <unistd.h>
#include <iostream>
#include <iomanip>
#include <lcm/lcm-cpp.hpp>
#include "lcm_std_msgs/Float64MultiArray.hpp"

using std::chrono::milliseconds;

void stableQuatToEuler(const Eigen::Quaterniond &quat, Eigen::Vector3d &e)
{
    // q << w,x,y,z
    Eigen::Vector4d q(quat.w(), quat.x(), quat.y(), quat.z());

    double R[3][3];
    R[0][0] = 1 - 2 * (q[2] * q[2] + q[3] * q[3]);
    R[0][1] = 2 * (q[1] * q[2] - q[0] * q[3]);
    R[0][2] = 2 * (q[1] * q[3] + q[0] * q[2]);
    R[1][0] = 2 * (q[1] * q[2] + q[0] * q[3]);
    R[1][1] = 1 - 2 * (q[1] * q[1] + q[3] * q[3]);
    R[1][2] = 2 * (q[2] * q[3] - q[0] * q[1]);
    R[2][0] = 2 * (q[1] * q[3] - q[0] * q[2]);
    R[2][1] = 2 * (q[2] * q[3] + q[0] * q[1]);
    R[2][2] = 1 - 2 * (q[1] * q[1] + q[2] * q[2]);

    // 计算欧拉角
    double sy = sqrt(R[0][0] * R[0][0] + R[1][0] * R[1][0]);
    bool singular = sy < 1e-6;
    double x, y, z;
    if (!singular)
    {
        x = atan2(R[2][1], R[2][2]);
        y = atan2(-R[2][0], sy);
        z = atan2(R[1][0], R[0][0]);
    }
    else
    {
        x = atan2(-R[1][2], R[1][1]);
        y = atan2(-R[2][0], sy);
        z = 0;
    }

    // 将欧拉角转换为连续形式
    if (y < -M_PI / 2 + 1e-6)
    {
        y = -M_PI - y;
        x += M_PI;
        z += M_PI;
    }
    else if (y > M_PI / 2 - 1e-6)
    {
        y = M_PI - y;
        x += M_PI;
        z += M_PI;
    }
    e << x, y, z;
}
int main(int argc, char *argv[])
{
    Eigen::Vector3d accel, gyrosco;
    struct timespec time;
    struct timespec acc_time;
    struct timespec gyro_time;
    struct timespec quat_time;
    Eigen::Quaterniond quaternion;
    xsens_IMU::imu_init();
    bool readflag = xsens_IMU::getImuDataFrame(accel, gyrosco, quaternion,time,acc_time,gyro_time,quat_time);
    std::cout << "acc: " << readflag << " " << accel.transpose() << std::endl;
    if (readflag)
    {
        std::cout << "imu init success!!\r\n";
    }
    lcm::LCM lcm;

    // lcm.publish("IMU_DATA", &lcm_std_msgs::Float64);
    // 定义目标循环周期
    long period_ns = 1000000; // 1kHz
    // 获取当前时间
    struct timespec current_time;
    clock_gettime(CLOCK_MONOTONIC, &current_time);
    while (true)
    {
        bool readflag = xsens_IMU::getImuDataFrame(accel, gyrosco, quaternion,time,acc_time,gyro_time,quat_time);
        // usleep(900);
        if (readflag)
        {

            lcm_std_msgs::Float64MultiArray float64msg;
            float64msg.size = 9;
            float64msg.data.resize(float64msg.size);
            for (int i = 0; i < 9; i++)
            {
                if (i < 3)
                {
                    float64msg.data[i] = accel(i);
                }
                else if (i < 6)
                {
                    float64msg.data[i] = gyrosco(i - 3);
                }
                else
                {
                    Eigen::Matrix3d rotation_matrix = quaternion.toRotationMatrix();
                    Eigen::Vector3d euler_angles_degrees;
                    stableQuatToEuler(quaternion, euler_angles_degrees);
                    euler_angles_degrees = euler_angles_degrees * 180 / M_PI;
                    float64msg.data[i] = euler_angles_degrees(i - 6);
                }
            }
            lcm.publish("IMU_DATA", &float64msg);
        }
        else
        {
            std::cout << "read imu failed!" << std::endl;
            // break;
        }
        // 计算下一个时间点
        current_time.tv_nsec += period_ns;
        if (current_time.tv_nsec >= 1000000000)
        {
            current_time.tv_sec += 1;
            current_time.tv_nsec -= 1000000000;
        }

        // 休眠到下一个时间点
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &current_time, NULL);
    }

    return 0;
}

