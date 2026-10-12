#include "ruiwo_actuator.h"
#include <cstdlib>
#include <thread>
#include <vector>
#include <iostream>
#include <experimental/filesystem>
#include <csignal>
#include <cmath>
#include <unistd.h>

namespace fs = std::experimental::filesystem;

RuiWoActuator actuator;
std::vector<std::vector<float>> stateList;
std::vector<double> poseList;
std::vector<double> torList;
std::vector<double> velList;

void close_ruiwo()
{
    stateList = actuator.get_joint_state();
    poseList = actuator.get_positions();
    torList = actuator.get_torque();
    velList = actuator.get_velocity();
    std::cout << "-------------------------------" << std::endl;
    std::cout << "[RUIWO motor]:Joint state: " << std::endl;
    for (const auto &row : stateList)
    {
        for (const auto &element : row)
        {
            std::cout << element << " ";
        }
        std::cout << std::endl;
    }
    std::cout << "-------------------------------" << std::endl;
    std::cout << "[RUIWO motor]:Joint positions: ";
    for (size_t i = 0; i < poseList.size(); i++)
    {
        std::cout << poseList[i] << " ";
    }
    std::cout << std::endl;
    std::cout << "[RUIWO motor]:Joint torque: ";
    for (size_t i = 0; i < torList.size(); i++)
    {
        std::cout << torList[i] << " ";
    }
    std::cout << std::endl;
    std::cout << "[RUIWO motor]:Joint velocity: ";
    for (size_t i = 0; i < velList.size(); i++)
    {
        std::cout << velList[i] << " ";
    }
    std::cout << std::endl;
    actuator.close();
}

void signalHandler(int signum)
{
    std::cout << "Interrupt signal (" << signum << ") received.\n";
    close_ruiwo();
    exit(signum);
}

int main()
{
    signal(SIGINT, signalHandler);
    if(0 != actuator.initialize()) {
        std::cout << "Initialize failed!" << std::endl;
        return -1;
    }
    long i = 0;

    for(int index = 0; index < 14; ++index){
        i = 0;
        std::cout << index << std::endl;
        while (true)
        {
            i++;
            // double positions = 0;// * std::sin(i * 3.14 / 180.0);
            double torque = 0.5;// * std::cos(i * 3.14 / 180.0);
            double velocity = 0.0;// * std::sin(i * 3.14 / 180.0);
            std::vector<double> position;
            position.assign(14, 0);
            position[index] = 20;
            if (index >= 6 && index < 12) position[index] = -position[index];
            actuator.set_positions({0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13},
                position,
                                {torque, torque, torque, torque, torque, torque,
                                    -torque, -torque, -torque, -torque, -torque, -torque,
                                    torque, torque},
                                {velocity, velocity, velocity, velocity, velocity, velocity,
                                    velocity, velocity, velocity, velocity, velocity, velocity,
                                    velocity, velocity});
            usleep(10000);
            std::vector<double> pose_double = actuator.get_positions();

            if (i % 99 == 0){
                std::cout << "pose_double";
                for(int k = 0; k < pose_double.size(); ++k)
                    std::cout << pose_double[k] << " ";
                std::cout << std::endl;
            }
            
            std::vector<double> get_torque = actuator.get_torque();

            if (i % 99 == 0){
                std::cout << "get_torque";
                for(int k = 0; k < get_torque.size(); ++k)
                    std::cout << get_torque[k] << " ";
                std::cout << std::endl;
            }
            std::vector<double> get_velocity = actuator.get_velocity();
            if (i % 99 == 0){
                std::cout << "get_velocity";
                for(int k = 0; k < get_velocity.size(); ++k)
                    std::cout << get_velocity[k] << " ";
                std::cout << std::endl;
            }

            if(i > 100) break;
        }
    }
    return 0;
}