
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

#include "xdainterface.h"
#include <xscontroller/xsscanner.h>
#include <xscontroller/xscontrol_def.h>
#include <xscontroller/xsdevice_def.h>
#include <xstypes/xsdatapacket.h>
#include <iostream>
#include <unistd.h>
#include <iomanip>


#define TIME_DIFF_IMU_T1_TO_T0_IN_MS(t0, t1) ((t1.tv_sec + t1.tv_nsec * 1e-9) - (t0.tv_sec + t0.tv_nsec * 1e-9)) * 1000
logger_callback_vector g_logger_callback_vector=nullptr;
void set_logger_callback(logger_callback_vector callback){
	g_logger_callback_vector=callback;
}
void log_vector(const std::string& name, const Eigen::VectorXd& vec)
{
	if (g_logger_callback_vector!=nullptr)
	{
		g_logger_callback_vector(name, vec);
	}
}
void calibrate_timespec(struct timespec *init_time, double d_init_time)
{
	init_time->tv_sec -= (time_t)d_init_time;
	init_time->tv_nsec -= (long)((d_init_time - (time_t)d_init_time) * 1e9);

	if (init_time->tv_nsec >= 1000000000)
	{
		init_time->tv_sec++;
		init_time->tv_nsec -= 1000000000;
	}
	else if (init_time->tv_nsec < 0)
	{
		init_time->tv_sec--;
		init_time->tv_nsec += 1000000000;
	}
}
double timespec_to_double(const timespec &ts)
{
	return static_cast<double>(ts.tv_sec) + static_cast<double>(ts.tv_nsec) * 1e-9;
}
timespec double_to_timespec(double time)
{
	timespec ts;
	ts.tv_sec = static_cast<time_t>(time);
	ts.tv_nsec = static_cast<long>((time - ts.tv_sec) * 1e9);
	return ts;
}

std::pair<timespec, Eigen::Quaterniond> slerp(const Eigen::Quaterniond &q1, const Eigen::Quaterniond &q2, const double t1, const double t2, const double t)
{
	const double alpha = (t - t1) / (t2 - t1);
	timespec ts = double_to_timespec(t);
	return std::make_pair(ts, q1.slerp(alpha, q2));
}

std::vector<std::pair<timespec, Eigen::Quaterniond>> slerpFrequency(const Eigen::Quaterniond &q1, const Eigen::Quaterniond &q2, const timespec time1, const timespec time2, const double frequency = 1200.0)
{
	std::vector<std::pair<timespec, Eigen::Quaterniond>> result;
	const double t1 = timespec_to_double(time1);
	const double t2 = timespec_to_double(time2);
	const double dt = 1.0 / frequency;

	// 如果两个时间戳之间的时间间隔小于等于插值周期，则直接返回一个包含第二个四元数的向量
	if (t2 - t1 <= dt)
	{
		timespec ts = double_to_timespec(t2);
		result.push_back(std::make_pair(ts, q2));
		return result;
	}

	// 计算需要进行插值的时间戳
	double interpTimestamp = t1 + dt;

	// 进行球面线性插值
	while (interpTimestamp < t2)
	{
		result.push_back(slerp(q1, q2, t1, t2, interpTimestamp));
		interpTimestamp += dt;
	}

	return result;
}

XdaInterface::XdaInterface()
		: m_device(nullptr)
{
	("Creating XsControl object...");
	m_control = XsControl::construct();
	assert(m_control != 0);
	acc_read_sequence_flag = false;
	imu_start_flag = false;
	mdata_.acc_vector.resize(100);
	mdata_.gyro_vector.resize(100);
	mdata_.quat_vector.resize(100);
	last_data_.acc_vector.resize(1);
	last_data_.gyro_vector.resize(1);
	last_data_.quat_vector.resize(1);
	
}

XdaInterface::~XdaInterface()
{
	close();
	m_control->destruct();
}
bool XdaInterface::getNewestData(IMUData &data)
{
	std::lock_guard<std::mutex> lock(last_data_mtx);
	if (!last_data_.is_new_data)
	{
		reuse_new_data_cnt++;
		if (reuse_new_data_cnt > 50)
		{
			std::cout << "[XdaInterface]imu no new data for 50 times, exit" << std::endl;
			return false;
		}
		// std::cout << "using last data, times:"<< reuse_new_data_cnt << std::endl;
	}else
	{
		reuse_new_data_cnt = 0;
	}
	data = last_data_;
	last_data_.is_new_data = false;
	return true;
}
bool XdaInterface::getData(IMUData &data)
{
	bool updated = true;
	mtx_imu.lock();
	static struct timespec now, delay;
	clock_gettime(CLOCK_MONOTONIC, &now);
	delay.tv_sec = 0;
	delay.tv_nsec = 5000000;
    double timestamp_acc = (now.tv_sec + now.tv_nsec * 1e-9) - (delay.tv_sec + delay.tv_nsec * 1e-9) - (init_time[IMU_ACC_INDEX].tv_sec + init_time[IMU_ACC_INDEX].tv_nsec * 1e-9);
    double timestamp_gyrohr = (now.tv_sec + now.tv_nsec * 1e-9) - (delay.tv_sec + delay.tv_nsec * 1e-9) - (init_time[IMU_GYROHR_INDEX].tv_sec + init_time[IMU_GYROHR_INDEX].tv_nsec * 1e-9);

	// double cpu_t3 = (now.tv_sec + now.tv_nsec * 1e-9) - (delay.tv_sec + delay.tv_nsec * 1e-9) - (init_time[IMU_GYROHR_INDEX].tv_sec + init_time[IMU_GYROHR_INDEX].tv_nsec * 1e-9);
    double d_init_time;
    {
        std::lock_guard<std::mutex> lock(acc_time_mutex);
        d_init_time = acc_time - timestamp_acc - (delay.tv_sec + delay.tv_nsec * 1e-9) - dynamic_adjust_time_offset;
    }
	int gyro_quat_count = int(timestamp_gyrohr / (gyro_quat_time / quat_cnt)) % 100;
	// int gyro_count = int(timestamp_gyrohr / (gyro_quat_time/quat_cnt)) % 100;
	// int quat_count = int(cpu_t3 / (gyro_quat_time/quat_cnt)) % 100;
	data.last_live_data_time_stamp = mdata_.last_live_data_time_stamp;
	if ((now.tv_sec + now.tv_nsec * 1e-9) - (delay.tv_sec + delay.tv_nsec * 1e-9) < (data_time[IMU_ACC_INDEX].tv_sec + data_time[IMU_ACC_INDEX].tv_nsec * 1e-9))
	{
		if (std::abs(d_init_time) >= 0.003)
		{ // diff(cpu time,acc time)>= 3ms
			if (sync_reset_count >= 50)
            {
                std::cout << "difff(cpu time,acc time) >= 3ms, reset init_time; d_init_time: " << d_init_time << std::endl;
                std::cout << "before update init: " << init_time[IMU_ACC_INDEX].tv_nsec << "\n";
								Eigen::VectorXd vec(1);
                                vec << d_init_time;
								log_vector("sensor_data/imu_sync_reset", vec);
                int iter = 0;
                bool find_valid_new_init_time = false;
                dynamic_adjust_time_offset = 0.0;
                do {
                    calibrate_timespec(&init_time[IMU_GYROHR_INDEX], d_init_time);
                    timestamp_gyrohr = (now.tv_sec + now.tv_nsec * 1e-9) - (delay.tv_sec + delay.tv_nsec * 1e-9) - (init_time[IMU_GYROHR_INDEX].tv_sec + init_time[IMU_GYROHR_INDEX].tv_nsec * 1e-9);
                    gyro_quat_count = int(timestamp_gyrohr / (gyro_quat_time / quat_cnt)) % 100;
                    {
                        std::lock_guard<std::mutex> lock(index_mtx);
                        int diff = (index[IMU_QUAT_INDEX] - gyro_quat_count + 100) % 100;
                        find_valid_new_init_time = (diff >= 3 && diff < 5);
                    }
                    d_init_time -= 0.001; // decrement by 1ms (0.001 seconds)
                    dynamic_adjust_time_offset += 0.001;
                    iter++;
                } while (find_valid_new_init_time && iter < 100);

                calibrate_timespec(&init_time[IMU_ACC_INDEX], d_init_time);
                calibrate_timespec(&init_time[IMU_GYROHR_INDEX], d_init_time);
                calibrate_timespec(&init_time[IMU_QUAT_INDEX], d_init_time);
                timestamp_acc = (now.tv_sec + now.tv_nsec * 1e-9) - (delay.tv_sec + delay.tv_nsec * 1e-9) - (init_time[IMU_ACC_INDEX].tv_sec + init_time[IMU_ACC_INDEX].tv_nsec * 1e-9);
                timestamp_gyrohr = (now.tv_sec + now.tv_nsec * 1e-9) - (delay.tv_sec + delay.tv_nsec * 1e-9) - (init_time[IMU_GYROHR_INDEX].tv_sec + init_time[IMU_GYROHR_INDEX].tv_nsec * 1e-9);
                gyro_quat_count = int(timestamp_gyrohr / (gyro_quat_time / quat_cnt)) % 100;
                std::cout << "after update init:" << init_time[IMU_ACC_INDEX].tv_nsec << "\n";
            }
			sync_reset_count++;
		}
		else
		{
			sync_reset_count = 0;
		}
		int acc_count = int(timestamp_acc / (acc_time / acc_cnt)) % 100;

		int window = 0;
		Eigen::Vector3d smoothed_acc = Eigen::Vector3d::Zero();
		if (prev_acc_cnt < acc_count)
		{
			window = (acc_count - prev_acc_cnt);
			for (size_t i = prev_acc_cnt + 1; i <= acc_count; i++)
			{
				smoothed_acc += mdata_.acc_vector.at(i).second;
			}
		}
		else
		{
			for (size_t i = prev_acc_cnt + 1; i < 100; i++)
			{
				smoothed_acc += mdata_.acc_vector.at(i).second;
				window++;
			}
			for (size_t i = 0; i <= acc_count; i++)
			{
				smoothed_acc += mdata_.acc_vector.at(i).second;
				window++;
			}
		}
		smoothed_acc /= window;
		// std::cout<<"window "<<window<<std::endl;
		prev_acc_cnt = acc_count;
		data.acc_vector.push_back(std::make_pair(now, smoothed_acc));
	}
	else
	{
		if (data.acc_vector.size() > 0) // 这次没有获取到,但是前面已经获取到数据
		{
		}
		else
		{
			// std::cout << "acc not update\n";
			// data.acc_vector.push_back(std::make_pair(now, mdata_.acc_vector[index[IMU_ACC_INDEX]].second));
			updated = false;
		}
	}
	if ((now.tv_sec + now.tv_nsec * 1e-9) - (delay.tv_sec + delay.tv_nsec * 1e-9) < (data_time[IMU_GYROHR_INDEX].tv_sec + data_time[IMU_GYROHR_INDEX].tv_nsec * 1e-9))
	{
		int window = 0;
		Eigen::Vector3d smoothed_gyro = Eigen::Vector3d::Zero();
		if (prev_gyro_cnt < gyro_quat_count)
		{
			window = (gyro_quat_count - prev_gyro_cnt);
			for (size_t i = prev_gyro_cnt + 1; i <= gyro_quat_count; i++)
			{
				smoothed_gyro += mdata_.gyro_vector.at(i).second;
			}
		}
		else
		{
			for (size_t i = prev_gyro_cnt + 1; i < 100; i++)
			{
				smoothed_gyro += mdata_.gyro_vector.at(i).second;
				window++;
			}
			for (size_t i = 0; i <= gyro_quat_count; i++)
			{
				smoothed_gyro += mdata_.gyro_vector.at(i).second;
				window++;
			}
		}
		smoothed_gyro /= window;
		prev_gyro_cnt = gyro_quat_count;


		data.gyro_vector.push_back(std::make_pair(now, smoothed_gyro));
	}
	else
	{
		if (data.gyro_vector.size() > 0)
		{
		}
		else
		{
			// std::cout << "gyro not update\n";
			// data.gyro_vector.push_back(mdata_.gyro_vector[index[IMU_QUAT_INDEX]]);
			updated = false;
		}
	}
	if ((now.tv_sec + now.tv_nsec * 1e-9) - (delay.tv_sec + delay.tv_nsec * 1e-9) < (data_time[IMU_GYROHR_INDEX].tv_sec + data_time[IMU_GYROHR_INDEX].tv_nsec * 1e-9))
	{

        if(!imuQuatCachedQueue.pop(last_used_quat)){
            reuse_quat_because_no_new_data_cnt++;
            if(reuse_quat_because_no_new_data_cnt > 50){
                std::cerr << "Warning: Reusing quaternion due to lack of new data for more than 50 iterations. Possible issue with IMU." << std::endl;
            }
        }else{
            reuse_quat_because_no_new_data_cnt = 0;
        }

        data.quat_vector.push_back(std::make_pair(now, last_used_quat));
	}
	else
	{
		if (data.quat_vector.size() > 0)
		{
		}
		else
		{
			// std::cout << "quat not update\n";
			// data.quat_vector.push_back(mdata_.quat_vector[index[IMU_GYROHR_INDEX]]);
			updated = false;
		}
	}

	mtx_imu.unlock();

	return updated;
}

void XdaInterface::spinFor(std::chrono::milliseconds timeout)
{
    RosXsDataPacket rosPacket = m_xdaCallback.next(timeout);
    if (rosPacket.second.empty())
    {
        auto currentTime = std::chrono::system_clock::now();
        auto currentTimeMs = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime.time_since_epoch()).count();
        std::cout << "[" << currentTimeMs << "] IMU 轮询线程没收到数据，请检查 IMU 连接或者 IMU 硬件" << std::endl;
        return;
    }

    XsDataPacket packet = rosPacket.second;

    bool quaternion_available = packet.containsOrientation();
    // bool gyro_available = packet.containsCalibratedGyroscopeData();
    bool gyroHR_available = packet.containsRateOfTurnHR();
    bool accelHR_available = packet.containsAccelerationHR();
    // TODO： 长度检测
    static int accpacket_count, gyropacket_count, quatpacket_count;
    int t_fine = packet.sampleTimeFine();
    struct timespec now;
    now.tv_nsec = (t_fine % 10000) * 1e5;
    now.tv_sec = t_fine / 10000;
    mdata_.last_live_data_time_stamp = now;


    if (accelHR_available)
    {
        accpacket_count++;
        static struct timespec t0_sol, t1_sol;
        clock_gettime(CLOCK_MONOTONIC, &t1_sol);
        if (TIME_DIFF_IMU_T1_TO_T0_IN_MS(t0_sol, t1_sol) > 5 && TIME_DIFF_IMU_T1_TO_T0_IN_MS(t0_sol, t1_sol) < 1e5)
        {
            printf("IMU acc HR period(ms) err: %f\n",
                            TIME_DIFF_IMU_T1_TO_T0_IN_MS(t0_sol, t1_sol));
        }
        clock_gettime(CLOCK_MONOTONIC, &t0_sol);
        data_time[IMU_ACC_INDEX] = t0_sol;

        static bool flag1 = true;
        if (flag1)
        {
            flag1 = false;
            init_imu_time[IMU_ACC_INDEX].tv_nsec = (t_fine % 10000) * 1e5;
            init_imu_time[IMU_ACC_INDEX].tv_sec = t_fine / 10000;
            clock_gettime(CLOCK_MONOTONIC, &init_time[IMU_ACC_INDEX]);
        }

        double new_acc_time = ((now.tv_sec * 1e9 + now.tv_nsec) - (init_imu_time[IMU_ACC_INDEX].tv_sec * 1e9 + init_imu_time[IMU_ACC_INDEX].tv_nsec)) * 1e-9;
        static struct timespec cpu_now;
        clock_gettime(CLOCK_MONOTONIC, &cpu_now);
        double new_now_time = cpu_now.tv_sec + cpu_now.tv_nsec * 1e-9 - (init_time[IMU_ACC_INDEX].tv_sec + init_time[IMU_ACC_INDEX].tv_nsec * 1e-9);
        if (0)
        {

            std::cout << std::setprecision(6) << std::fixed;
            std::cout << std::setw(10) << std::setfill(' ') << new_acc_time << "|";
            std::cout << std::setw(10) << std::setfill(' ') << new_now_time << "|";
            std::cout << std::setw(10) << std::setfill(' ') << new_acc_time - new_now_time << " dt:";
            std::cout << std::setw(10) << std::setfill(' ') << new_now_time - now_time << " dt_samp:";
            std::cout << std::setw(10) << std::setfill(' ') << new_acc_time - acc_time << std::endl;
        }
        now_time = new_now_time;
        {
            std::lock_guard<std::mutex> lock(acc_time_mutex);
            acc_time = new_acc_time;
        }
        int smooth_window = 6;
        Eigen::Vector3d smoothed_acc = Eigen::Vector3d::Zero();

        XsVector accel = packet.accelerationHR();
        Eigen::Vector3d acc(accel[0], accel[1], accel[2]);



        mtx_imu.lock();
        mdata_.acc_raw_vector.push_back(std::make_pair(now, acc));
        if (mdata_.acc_raw_vector.size() > smooth_window)
        {
            mdata_.acc_raw_vector.erase(mdata_.acc_raw_vector.begin(), mdata_.acc_raw_vector.end() - smooth_window);
            for (size_t i = 0; i < smooth_window; i++)
            {
                smoothed_acc += mdata_.acc_raw_vector.at(i).second;
            }
            smoothed_acc /= smooth_window;
        }
        else
        {
            smoothed_acc = acc;
        }

        {
            std::lock_guard<std::mutex> lock(index_mtx);
            acc_cnt++;
            mdata_.acc_vector[index[IMU_ACC_INDEX]++] = std::make_pair(now, smoothed_acc);
            if (index[IMU_ACC_INDEX] >= 100)
            {
                index[IMU_ACC_INDEX] = 0;
            }
        }
        mtx_imu.unlock();
		{
			std::lock_guard<std::mutex> lock(last_data_mtx);
			last_data_.acc_vector[0] = std::make_pair(now, smoothed_acc);
			last_data_.is_new_data = true;
		}
    }

    if (quaternion_available)
    {

        static struct timespec t0_sol_q, t1_sol_q;
        clock_gettime(CLOCK_MONOTONIC, &t1_sol_q);
        if (TIME_DIFF_IMU_T1_TO_T0_IN_MS(t0_sol_q, t1_sol_q) > 8 && TIME_DIFF_IMU_T1_TO_T0_IN_MS(t0_sol_q, t1_sol_q) < 1e5)
        {
            printf("quat period(ms) err: %f\n",
                            TIME_DIFF_IMU_T1_TO_T0_IN_MS(t0_sol_q, t1_sol_q));
        }
        clock_gettime(CLOCK_MONOTONIC, &t0_sol_q);

        static bool flag2 = true;
        if (flag2)
        {
            flag2 = false;
            init_imu_time[IMU_QUAT_INDEX].tv_nsec = (t_fine % 10000) * 1e5;
            init_imu_time[IMU_QUAT_INDEX].tv_sec = t_fine / 10000;
            clock_gettime(CLOCK_MONOTONIC, &init_time[IMU_QUAT_INDEX]);
        }

        XsQuaternion q = packet.orientationQuaternion();
        Eigen::Quaterniond target_quat;
        target_quat.w() = q.w();
        target_quat.x() = q.x();
        target_quat.y() = q.y();
        target_quat.z() = q.z();
        last_quat_ = std::make_pair(now, target_quat);
        


        clock_gettime(CLOCK_MONOTONIC, &data_time[IMU_QUAT_INDEX]);
        int last_index = (index[IMU_QUAT_INDEX] - 1 < 0) ? 99 : index[IMU_QUAT_INDEX] - 1;
        mdata_.quat_vector[last_index] = std::make_pair(now, target_quat);
        imuReceivedQuatQueue.push_auto_pop(target_quat);
    }

    if (gyroHR_available)
    {

        static struct timespec t0_sol_g, t1_sol_g;
        clock_gettime(CLOCK_MONOTONIC, &t1_sol_g);
        if (TIME_DIFF_IMU_T1_TO_T0_IN_MS(t0_sol_g, t1_sol_g) > 5 && TIME_DIFF_IMU_T1_TO_T0_IN_MS(t0_sol_g, t1_sol_g) < 1e5)
        {
            printf("quat period(ms) err: %f\n",
                            TIME_DIFF_IMU_T1_TO_T0_IN_MS(t0_sol_g, t1_sol_g));
        }
        clock_gettime(CLOCK_MONOTONIC, &t0_sol_g);

        gyropacket_count++;

        clock_gettime(CLOCK_MONOTONIC, &data_time[IMU_GYROHR_INDEX]);
        static bool flag3 = true;
        if (flag3)
        {
            flag3 = false;
            init_imu_time[IMU_GYROHR_INDEX].tv_nsec = (t_fine % 10000) * 1e5;
            init_imu_time[IMU_GYROHR_INDEX].tv_sec = t_fine / 10000;

            clock_gettime(CLOCK_MONOTONIC, &init_time[IMU_GYROHR_INDEX]);
        }

        XsVector g = packet.rateOfTurnHR();
        Eigen::Vector3d gyro(g[0], g[1], g[2]);


        gyro_quat_time = ((now.tv_sec * 1e9 + now.tv_nsec) - (init_imu_time[IMU_GYROHR_INDEX].tv_sec * 1e9 + init_imu_time[IMU_GYROHR_INDEX].tv_nsec)) * 1e-9;
        mtx_imu.lock();
        {
            std::lock_guard<std::mutex> lock(index_mtx);
            mdata_.gyro_vector[index[IMU_GYROHR_INDEX]++] = std::make_pair(now, gyro);
            if (index[IMU_GYROHR_INDEX] >= 100)
            {
                index[IMU_GYROHR_INDEX] = 0;
            }
        }
        mtx_imu.unlock();

        timespec last_quat_time = last_quat_.first; // 上次接收到quat数据的时间戳
        timespec q_pred_time;
        Eigen::Quaterniond q_pred;
        Eigen::Quaterniond received_quat;
        if(imuReceivedQuatQueue.pop(received_quat)){
            q_pred = last_quat_.second;
            q_pred_time = last_quat_time;
            imuQuatCachedQueue.push_auto_pop(received_quat);
        }else
        {
            int last_index = (index[IMU_QUAT_INDEX] - 1 < 0) ? 99 : index[IMU_QUAT_INDEX] - 1;
            Eigen::Quaterniond last_quat = mdata_.quat_vector[last_index].second;
            timespec quat_vector_time = mdata_.quat_vector[last_index].first;
            double d_sampleTime = TIME_DIFF_IMU_T1_TO_T0_IN_MS(quat_vector_time, now) * 1e-3;
            // 使用四元数微分方程和当前角速度数据对当前四元数状态进行预测
            Eigen::Quaterniond dq(1.0, gyro.x() * d_sampleTime / 2, gyro.y() * d_sampleTime / 2, gyro.z() * d_sampleTime / 2);
            // 计算角速度向量的增量矩阵
            q_pred = last_quat * dq;
            // std::cout << "dq: " << dq.coeffs() << std::endl;
            q_pred_time = now;

            imuQuatCachedQueue.push_auto_pop(q_pred);
            // std::cout << "Quaternion: " << last_quat.coeffs() << std::endl;
            // std::cout << "TIME_DIFF_IMU_T1_TO_T0_IN_MS(last_quat_time, now) > 0 " << q_pred.w() << "|" << q_pred.x() << "|" << last_quat.w() << std::endl;
        }

        q_pred.normalize();



        mtx_imu.lock();
        {
            std::lock_guard<std::mutex> lock(index_mtx);
            quat_cnt++;

            mdata_.quat_vector[index[IMU_QUAT_INDEX]++] = std::make_pair(q_pred_time, q_pred);
            if (index[IMU_QUAT_INDEX] >= 100)
            {
                index[IMU_QUAT_INDEX] = 0;
            }
        }
        mtx_imu.unlock();

		{
			std::lock_guard<std::mutex> lock(last_data_mtx);
			last_data_.quat_vector[0] = std::make_pair(q_pred_time, q_pred);
			last_data_.gyro_vector[0] = std::make_pair(now, gyro);
			last_data_.is_new_data = true;
		}
    }

    if (!imu_start_flag) {
        if (accpacket_count > 100 && gyropacket_count > 100 && quat_cnt > 1000) {
            imu_start_flag = true;
        }
    }
}


void XdaInterface::registerPublishers()
{
	// registerCallback(new AccelerationHRPublisher());
	// registerCallback(new ImuPublisher());
}

bool XdaInterface::connectDevice()
{
	// Read baudrate parameter if set
	XsBaudRate baudrate = XBR_Invalid;
	// if (ros::param::has("~baudrate"))
	// {
	// 	int baudrateParam = 0;
	// 	ros::param::get("~baudrate", baudrateParam);
	// 	ROS_INFO("Found baudrate parameter: %d", baudrateParam);
	// 	baudrate = XsBaud::numericToRate(baudrateParam);
	// }
	// // Read device ID parameter
	bool checkDeviceID = false;
	std::string deviceId;
	// if (ros::param::has("~device_id"))
	// {
	// 	ros::param::get("~device_id", deviceId);
	// 	checkDeviceID = true;
	// 	ROS_INFO("Found device ID parameter: %s.",deviceId.c_str());

	// }
	// // Read port parameter if set
	XsPortInfo mtPort;
	// if (ros::param::has("~port"))
	// {
	// 	std::string portName;
	// 	ros::param::get("~port", portName);
	// 	ROS_INFO("Found port name parameter: %s", portName.c_str());
	// 	mtPort = XsPortInfo(portName, baudrate);
	// 	ROS_INFO("Scanning port %s ...", portName.c_str());
	// 	if (!XsScanner::scanPort(mtPort, baudrate))
	// 		return handleError("No MTi device found. Verify port and baudrate.");
	// 	if (checkDeviceID && mtPort.deviceId().toString().c_str() != deviceId)
	// 		return handleError("No MTi device found with matching device ID.");

	// }
	// else
	{
		// ROS_INFO("Scanning for devices...");
		XsPortInfoArray portInfoArray = XsScanner::scanPorts(baudrate);

		for (auto const &portInfo : portInfoArray)
		{
			if (portInfo.deviceId().isMti() || portInfo.deviceId().isMtig())
			{
				if (checkDeviceID)
				{
					if (portInfo.deviceId().toString().c_str() == deviceId)
					{
						mtPort = portInfo;
						break;
					}
				}
				else
				{
					mtPort = portInfo;
					break;
				}
			}
		}
	}

	if (mtPort.empty())
		return handleError("No MTi device found.");
	std::string s(mtPort.portName().c_str());

	if (!m_control->openPort(mtPort))
		return handleError("Could not open port");
	std::cout << "m_portName" << s << std::endl;
	m_device = m_control->device(mtPort.deviceId());
	assert(m_device != 0);

	m_device->addCallbackHandler(&m_xdaCallback);

	return true;
}

bool XdaInterface::prepare()
{
	assert(m_device != 0);

	if (!m_device->gotoConfig())
		return handleError("Could not go to config");

	// read EMTS and device config stored in .mtb file header.
	if (!m_device->readEmtsAndDeviceConfiguration())
		return handleError("Could not read device configuration");

	if (!m_device->gotoMeasurement())
		return handleError("Could not put device into measurement mode");

	// std::string logFile;
	// if (ros::param::get("~log_file", logFile))
	// {
	// 	if (m_device->createLogFile(logFile) != XRV_OK)
	// 		return handleError("Failed to create a log file! (" + logFile + ")");
	// 	else
	// 		ROS_INFO("Created a log file: %s", logFile.c_str());

	// 	ROS_INFO("Recording to %s ...", logFile.c_str());
	// 	if (!m_device->startRecording())
	// 		return handleError("Could not start recording");
	// }

	return true;
}

void XdaInterface::close()
{
	if (m_device != nullptr)
	{
		m_device->stopRecording();
		m_device->closeLogFile();
		m_device->removeCallbackHandler(&m_xdaCallback);
	}
	m_control->closePort(m_port);
}

void XdaInterface::registerCallback(PacketCallback *cb)
{
	m_callbacks.push_back(cb);
}

bool XdaInterface::handleError(std::string error)
{
	close();
	return false;
}
