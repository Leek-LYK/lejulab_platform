#include "hipnuc.h"
#include "hipnuc_imu_receiver.h"

#include <stdexcept>
#include <string>
#include <pthread.h>
#include <unistd.h>
#include <iostream>
#include <iomanip>
#include <sched.h>
#include <cmath>
#include <fstream>
#include <sys/stat.h>  // 替换filesystem

namespace HIPNUC_IMU
{
	std::string imu_port;
	int serial_baud;
	std::string frame_id;
	std::string imu_topic;
	int fd = 0;
	struct pollfd p;
	Eigen::Vector3d euler_offset = Eigen::Vector3d::Zero();  // 初始化全局offset变量

	bool imu_start_flag = false;
	bool imu_running_flag = true;
	pthread_t thread_imu;
	static uint8_t buf[2048];
	static int frame_rate_0x91, frame_rate_0x92;
	static int frame_count_0x91, frame_count_0x92;

	std::mutex imu_data_mutex;
	std::queue<hipnuc_raw_t> imu_data_queue;
	hipnuc_raw_t last_data;
	bool last_data_valid = false;

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
		imu_port = "/dev/HIPNUC_IMU";
		serial_baud = 921600;

		fd = open_port(imu_port, serial_baud);
		if (fd == 0) 
		{
			std::cout << "imu no connect !!" << std::endl;
			exit(1);
		}
		std::cout << "imu connect success !!" << std::endl;
		p.fd = fd;
		p.events = POLLIN;

		int is_imuMsg_get = 0;

		imu_running_flag = true;
		while (imu_running_flag)
		{
			if (read_hipnuc_imu(fd, &p))
			{
				std::lock_guard<std::mutex> lock(imu_data_mutex);
				imu_data_queue.push(raw); // Store raw data in the buffer
			}
		}
		
		return 0;
	}

	// 添加读取offset的函数
	void readEulerOffsetFromFile()
	{
		std::string home_dir = getenv("HOME");
		std::string config_path = home_dir + "/.config/lejuconfig/hipimuEulerOffset.csv";
		
		struct stat buffer;
		if (stat(config_path.c_str(), &buffer) == 0)  // 检查文件是否存在
		{
			std::ifstream file(config_path);
			if (file.is_open())
			{
				std::string line;
				if (std::getline(file, line))
				{
					std::stringstream ss(line);
					std::string value;
					int i = 0;
					while (std::getline(ss, value, ',') && i < 3)
					{
						euler_offset[i++] = std::stod(value) * M_PI / 180.0; // 将角度转换为弧度
					}
				}
				file.close();
				std::cout << "[hipnuc_imu_receiver.cpp] 成功读取IMU欧拉角偏移: " 
					<< euler_offset[0] * 180.0 / M_PI << ", " 
					<< euler_offset[1] * 180.0 / M_PI << ", " 
					<< euler_offset[2] * 180.0 / M_PI << " 度" << std::endl;
			}
			else
			{
				std::cout << "[hipnuc_imu_receiver.cpp] 无法打开配置文件: " << config_path << std::endl;
			}
		}
		else
		{
			std::cout << "[hipnuc_imu_receiver.cpp] 配置文件不存在: " << config_path << std::endl;
			std::cout << "[hipnuc_imu_receiver.cpp] 自动创建配置文件并写入默认值: 0,0,0" << std::endl;
			// 创建父目录
			std::string config_dir = home_dir + "/.config/lejuconfig";
			struct stat dir_buffer;
			if (stat(config_dir.c_str(), &dir_buffer) != 0) {
				std::string mkdir_cmd = "mkdir -p " + config_dir;
				system(mkdir_cmd.c_str());
			}
			// 写入默认值
			std::ofstream outfile(config_path);
			if (outfile.is_open()) {
				outfile << "0,0,0" << std::endl;
				outfile.close();
			} else {
				std::cout << "[hipnuc_imu_receiver.cpp] 创建配置文件失败: " << config_path << std::endl;
			}
			// 默认偏移为0
			euler_offset = Eigen::Vector3d::Zero();
		}
	}

	bool getImuDataFrame(Eigen::Vector3d &acc, Eigen::Vector3d &gyro, Eigen::Quaterniond &quat)
	{
		int readcount = 0;
		auto start_time = std::chrono::steady_clock::now(); // 获取当前时间
		hipnuc_raw_t data;
		bool use_last_data = false;

		while (true)
		{
			std::lock_guard<std::mutex> lock(imu_data_mutex);
			// std::cout << "IMU DATA QUEUE SIZE :   " << imu_data_queue.size() << std::endl;

			if (!imu_data_queue.empty())
			{
				// 从队列中获取数据
				data = imu_data_queue.front();
				imu_data_queue.pop();

				// 更新 last_data 并设置标志为有效
				last_data = data;
				last_data_valid = true;
				// std::cout << "Using current imu data ! GOOD !!!!" << std::endl;
				while (!imu_data_queue.empty())
                    {
                        imu_data_queue.pop();
                    }
				break;
			}
			else
			{
				// std::cout << "'imu_data_queue' is empty " << std::endl;

				// 如果没有数据，检查 last_data 是否有效
				if (last_data_valid)
				{
					// 使用上一次的数据
					data = last_data;
					use_last_data = true;
					break;
				}
			}

			usleep(100); // 等待200微秒
			readcount++;

			if (readcount > 10) // 重试10次仍未成功
			{
				std::cout << "get imu Data failed after 10 attempts!" << std::endl;
				if (last_data_valid)
				{
					data = last_data;
					use_last_data = true;
					break;
				}
				return false; // 如果没有上一次的数据，也没有新数据，则返回失败
			}

			// 设置超时时间，保证循环一定退出
			auto current_time = std::chrono::steady_clock::now();
			auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time);
			if (elapsed_time.count() > 1) // 如果超过1ms还未获取到数据
			{
				std::cout << "get imu Data timed out after " << elapsed_time.count() << " milliseconds." << std::endl;

				// 超时后使用上一次的数据
				if (last_data_valid)
				{
					data = last_data;
					use_last_data = true;
					break;
				}
				return false; // 如果没有上一次的数据，则返回失败
			}
		}

		// 填充加速度和陀螺仪数据
		acc = Eigen::Vector3d::Zero();
		gyro = Eigen::Vector3d::Zero();
		for (size_t i = 0; i < acc.size(); i++)
		{
			acc[i] = data.hi91.acc[i] * GRA_ACC;
			gyro[i] = data.hi91.gyr[i] * DEG_TO_RAD;
		}
		quat = Eigen::Quaterniond(data.hi91.quat[0],
								data.hi91.quat[1],
								data.hi91.quat[2],
								data.hi91.quat[3]);


		Eigen::Vector3d euler = quat.toRotationMatrix().eulerAngles(2, 1, 0);
		euler += euler_offset;  // 使用全局offset变量

		// 将欧拉角转回四元数
		Eigen::AngleAxisd rollAngle(euler[2], Eigen::Vector3d::UnitX());
		Eigen::AngleAxisd pitchAngle(euler[1], Eigen::Vector3d::UnitY());
		Eigen::AngleAxisd yawAngle(euler[0], Eigen::Vector3d::UnitZ());
		quat = yawAngle * pitchAngle * rollAngle;
		quat.normalize();  // 归一化四元数

		// 只使用offset对应的旋转矩阵来旋转加速度和角速度
		Eigen::AngleAxisd rollOffset(euler_offset[2], Eigen::Vector3d::UnitX());
		Eigen::AngleAxisd pitchOffset(euler_offset[1], Eigen::Vector3d::UnitY());
		Eigen::AngleAxisd yawOffset(euler_offset[0], Eigen::Vector3d::UnitZ());
		Eigen::Matrix3d offset_rotation = (yawOffset * pitchOffset * rollOffset).toRotationMatrix();
		acc = offset_rotation * acc;
		gyro = offset_rotation * gyro;

		// // 如果使用的是上一次的数据，打印提示
		// if (use_last_data)
		// {
		// 	std::cout << "Using last IMU data." << std::endl;
		// }

		return true;
	}


	int read_hipnuc_imu(int fd, struct pollfd *p)
	{
		int n = 0; 
		int rev = 0;
		int rpoll = poll(p, 1, 5);

		if(rpoll == 0)
			return 0;
		n = read(fd, buf, sizeof(buf));

		if(n > 0)
		{
			for (int i = 0; i < n; i++)
			{
				rev = hipnuc_input(&raw, buf[i]);

			}
			return 1;
		}
		return 0;
	}

	int open_port(std::string port_device, int baud)
	{
		const char* port_device1 = port_device.c_str();
		int fd = open(port_device1, O_RDWR | O_NOCTTY | O_NDELAY);

		if (fd == -1)
		{
			perror("open_port: Unable to open SerialPort");
			puts("Please check the usb port name!!!");
			exit(0);
		}

		if(fcntl(fd, F_SETFL, O_NONBLOCK) < 0)
			printf("fcntl failed\n");
		else
			fcntl(fd, F_SETFL, O_NONBLOCK);
	
		if(isatty(STDIN_FILENO) == 0)
			printf("standard input is not a terminal device\n");
		else 
			printf("isatty success!\n");

		struct termios options;
		tcgetattr(fd, &options);
		
		std::cout << baud <<std::endl;
		switch(baud)
		{
			case 115200:
				cfsetispeed(&options, B115200);
				cfsetospeed(&options, B115200);
			break;
			case 460800:
				cfsetispeed(&options, B460800);
				cfsetospeed(&options, B460800);
			break;
			case 921600:
				cfsetispeed(&options, B921600);
				cfsetospeed(&options, B921600);
			break;
			default:
			std::cout << "SERIAL PORT BAUD RATE ERROR" << std::endl;
		}

		options.c_cflag &= ~PARENB; 
		options.c_cflag &= ~CSTOPB; 
		options.c_cflag &= ~CSIZE;  
		options.c_cflag |= HUPCL;   
		options.c_cflag |= CS8;     
		options.c_cflag &= ~CRTSCTS; 
		options.c_cflag |= CREAD | CLOCAL; 

		options.c_iflag &= ~(IXON | IXOFF | IXANY); 
		options.c_iflag &= ~(INLCR|ICRNL); 

		options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

		options.c_oflag &= ~OPOST; 
		options.c_oflag &= ~(ONLCR|OCRNL); 

		options.c_cc[VMIN] = 0;  
		options.c_cc[VTIME] = 0; 

		tcsetattr(fd, TCSANOW, &options);
		return (fd);
	}

	int imu_init()
	{
		// 首先读取offset配置
		readEulerOffsetFromFile();

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
		usleep(500000); // 500ms: nohz_full 核心上 poll() 唤醒较慢，需要更长等待
		return 0;
	}

	void imu_stop()
	{
		imu_running_flag = false;
	}

	bool get_imu_running_flag(){
		return imu_running_flag;
	}

}
