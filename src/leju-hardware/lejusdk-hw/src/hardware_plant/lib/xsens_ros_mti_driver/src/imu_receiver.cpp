
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

#include "imu_receiver.h"

#include <iostream>
#include <stdexcept>
#include <string>
#include <pthread.h>
#include <unistd.h>
#include <iostream>
#include <iomanip>
#include <sched.h>

Journaller *gJournal = 0;

namespace xsens_IMU
{
using std::chrono::milliseconds;


bool imu_start_flag = false;
bool imu_running_flag = true;
XdaInterface *xdaInterface = new XdaInterface();
pthread_t thread_imu;

void *imu_thread(void *arg)
{
	int32_t ret;
	struct sched_param param;
	param.sched_priority = 80; // SCHED_RR
	ret = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
	if (ret != 0)
	{
		printf("Failed to set rt of thread %d. %s\n", int(pthread_self()), strerror(ret));
	}
	// xdaInterface->registerPublishers();
	if (!xdaInterface->connectDevice())
	{
		std::cout << "imu no connect !!" << std::endl;
		exit(1);
	}
	std::cout << "imu connect success!!" << std::endl;
	if (!xdaInterface->prepare())
	{
		std::cout << "imu prepare failed !!" << std::endl;
		exit(1);
	}
	imu_running_flag = true;
	while (imu_running_flag)
	{
		xdaInterface->spinFor(milliseconds(100));
		imu_start_flag = true;
		// std::cout << "thread: "<< imu_start_flag << std::endl;
	}
	delete xdaInterface;
	std::cout << "thread imu exited!" << std::endl;
	return nullptr;
}
bool getImuDataFrame(Eigen::Vector3d &acc, Eigen::Vector3d &gyro, Eigen::Quaterniond &quat)
{
    struct timespec time;
    struct timespec acc_time;
    struct timespec gyro_time;
    struct timespec quat_time;
    return getImuDataFrame(acc, gyro, quat, time, acc_time, gyro_time, quat_time);
};
// false: data length less than expert, <0 or <n
bool getImuDataFrame(Eigen::Vector3d &acc, Eigen::Vector3d &gyro, Eigen::Quaterniond &quat,struct timespec &timestamp,struct timespec &acc_timestamp,struct timespec &gyro_timestamp,struct timespec &quat_timestamp)
{
    IMUData mdata;
    mdata.acc_raw_vector = std::vector<std::pair<struct timespec, Eigen::Vector3d>>();
    mdata.acc_vector = std::vector<std::pair<struct timespec, Eigen::Vector3d>>();
    mdata.gyro_vector = std::vector<std::pair<struct timespec, Eigen::Vector3d>>();
    mdata.quat_vector = std::vector<std::pair<struct timespec, Eigen::Quaterniond>>();
    mdata.last_live_data_time_stamp = {0, 0};

	int readcout = 0;
	while (1)
	{
		if (xdaInterface->getNewestData(mdata))
		{
			break;
		}

		if (mdata.acc_vector.size() > 0 && mdata.gyro_vector.size() > 0 && mdata.quat_vector.size() > 0)
		{
			break;
		}
		else
		{
			// std::cout << (mdata.acc_vector.size() > 0 ? "" : "lost acc;") << (mdata.gyro_vector.size() > 0 ? "" : "lost gyro_vector;") << (mdata.quat_vector.size() > 0 ? "" : "lost quat_vector;\n");
		}
		usleep(200); // 每0.2ms读取一次

		readcout++;
		if (readcout > 10) // 读取10次都没有完全获得三种数据
		{
			std::cout << "get imu Data failed!!" << readcout << std::endl;
			return false;
		}
		// std::cout << "get imu Data failed!! readcout: " << readcout << std::endl;
	}

	acc = Eigen::Vector3d::Zero();
	for (size_t i = 0; i < mdata.acc_vector.size(); i++)
	{
		acc += mdata.acc_vector.at(i).second;
	}
	acc /= mdata.acc_vector.size();

	gyro = mdata.gyro_vector.front().second;
	quat = mdata.quat_vector.front().second;
	timestamp = mdata.last_live_data_time_stamp;
	acc_timestamp = mdata.acc_vector.front().first;
	gyro_timestamp = mdata.gyro_vector.front().first;
	quat_timestamp = mdata.quat_vector.front().first;
	return true;
}

int imu_init()
{
	int ret = 0;
	ret = pthread_create(&thread_imu, NULL, imu_thread, NULL);
	if (ret != 0)
	{
		printf("Create imu_thread failed! %s\n", strerror(ret));
		return -1;
	}

	cpu_set_t cpuset;
	CPU_ZERO(&cpuset);
	CPU_SET(5, &cpuset);
	ret = pthread_setaffinity_np(thread_imu, sizeof(cpu_set_t), &cpuset);
	if (ret != 0)
	{
		printf("pthread_setaffinity_np failed! %s\n", strerror(ret));
		return -1;
	}

	while (!xdaInterface->imu_start_flag)
	{
		usleep(1000);
	}
	return 0;
}

void imu_stop()
{
	imu_running_flag = false;
}
} // namespace xsens_IMU
