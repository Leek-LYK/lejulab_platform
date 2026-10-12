#include "ruiwo_actuator.h"
#include <cstdlib>
#include <thread>
#include <vector>
#include <iostream>
#include <csignal>
#include <cmath>
#include <unistd.h>

RuiWoActuator actuator;
void signalHandler(int signum)
{
    std::cout << "Interrupt signal (" << signum << ") received.\n";
    actuator.close();
    exit(signum);
}

int main()
{
    signal(SIGINT, signalHandler);
    if(0 != actuator.initialize()) {
        std::cout << "Initialize failed!" << std::endl;
        return -1;
    }
    auto i = 0;
    
    actuator.enable();
     
    std::vector<uint8_t> ids = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0X0A, 0X0B, 0X0C, 0X0D};
    std::vector<double> pos_deg = {5,5,5,5,5,  5,5,5,5,5, 5,5,5,5};
    std::vector<double> taus_deg = {5,5,5,5,5,  5,5,5,5,5, 5,5,5,5};
    std::vector<double> vels_deg = {5,5,5,5,5,  5,5,5,5,5, 5,5,5,5};

    auto degree_to_radian = [](double degree) -> double {
        return degree * (M_PI / 180.0);
    };

    int motor_id = 6;
    // for(int motor_id = 0; motor_id < 0X0D; motor_id++) {

        // degree to radian
        auto pos = pos_deg;

        // 关节正向 +15°
        pos[motor_id] += 15.0;
        actuator.set_positions(ids, pos, taus_deg, vels_deg);
        std::cout << "motor: " << motor_id+1 << ", set position: " << pos[motor_id] << "\n";
        sleep(1);


        // 关节反向 +15°
        pos[motor_id] -= 15.0*2;
        actuator.set_positions(ids, pos, taus_deg, vels_deg);
        std::cout << "motor: " << motor_id+1 << ", set position: " << pos_deg[motor_id] << "\n";
        sleep(1);


        // // 归位
        actuator.set_positions(ids, pos_deg, taus_deg, vels_deg);
        std::cout << "motor: " << motor_id+1 << ", set position: " << pos_deg[motor_id] << "\n";
        sleep(1);

    // }

    sleep(10);

    return 0;
}